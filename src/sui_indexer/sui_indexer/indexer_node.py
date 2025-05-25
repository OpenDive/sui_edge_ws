#!/usr/bin/env python3
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional, Tuple
import json
import asyncio
from threading import Thread
import os

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from sui_indexer_msgs.msg import SuiEvent, IndexerStatus
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from ament_index_python.packages import get_package_share_directory

from prisma import Prisma
from pysui.sui.sui_clients.sync_client import SuiClient
from pysui.sui.sui_config import SuiConfig
from pysui.sui.sui_constants import (
    DEVNET_SUI_URL,
    DEVNET_SOCKET_URL,
    TESTNET_SUI_URL,
    TESTNET_SOCKET_URL,
    MAINNET_SUI_URL,
    MAINNET_SOCKET_URL
)
from pysui.sui.sui_builders.get_builders import QueryEvents
from pysui.sui.sui_types.collections import EventID
from pysui.sui.sui_types.event_filter import MoveEventModuleQuery

@dataclass
class EventTracker:
    """Tracks events for a specific module."""
    type: str
    filter: MoveEventModuleQuery
    callback: Callable
    cursor: Optional[EventID] = None

class SuiIndexerNode(Node):
    """ROS2 node for indexing Sui blockchain events."""
    
    def __init__(self):
        super().__init__('sui_indexer')
        
        # Event loop management
        self.event_loop: Optional[asyncio.AbstractEventLoop] = None
        self.event_thread: Optional[Thread] = None
        
        # Declare parameters with proper types
        self.declare_parameter('package_id', '', descriptor=ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description='The Sui package ID to index',
            read_only=True,
            additional_constraints='Required. Must be a valid Sui package ID'
        ))
        
        self.declare_parameter('network', 'testnet', descriptor=ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description='Sui network to connect to',
            additional_constraints='Must be one of: testnet, mainnet, devnet'
        ))
        
        self.declare_parameter('polling_interval_ms', 1000, descriptor=ParameterDescriptor(
            type=ParameterType.PARAMETER_INTEGER,
            description='Polling interval in milliseconds',
            additional_constraints='Must be > 0'
        ))
        
        self.declare_parameter('default_limit', 50, descriptor=ParameterDescriptor(
            type=ParameterType.PARAMETER_INTEGER,
            description='Default limit for event queries',
            additional_constraints='Must be between 1 and 100'
        ))
        
        self.declare_parameter('database_url', '', descriptor=ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description='Database URL for the indexer. Use absolute path for production deployments.'
        ))
        
        # Validate parameters
        self._validate_parameters()
        
        # Log all settings
        self.get_logger().info("Indexer Settings:")
        self.get_logger().info(f"  Network: {self.get_parameter('network').value}")
        self.get_logger().info(f"  Package ID: {self.get_parameter('package_id').value}")
        self.get_logger().info(f"  Polling Interval: {self.get_parameter('polling_interval_ms').value}ms")
        self.get_logger().info(f"  Default Limit: {self.get_parameter('default_limit').value}")
        self.get_logger().info(f"  Database URL: {self.get_parameter('database_url').value}")
        
        # Publishers
        self.event_pub = self.create_publisher(SuiEvent, 'sui_events', 10)
        self.status_pub = self.create_publisher(IndexerStatus, 'indexer_status', 10)
        
        # Initialize clients
        self.db = None  # Prisma client
        self.sui_client = None  # Sui client
        
        # Event trackers
        self.event_trackers: List[EventTracker] = []
        
        # Get database URL from parameter or use default in package share
        database_url = self.get_parameter('database_url').value
        if not database_url or database_url == 'file:sui_indexer.db':
            # Use package share directory for default database location
            pkg_share = get_package_share_directory('sui_indexer')
            data_dir = os.path.join(pkg_share, 'data')
            db_path = os.path.join(data_dir, 'sui_indexer.db')
            database_url = f'file:{db_path}'
        
        self.get_logger().info(f"Working directory: {os.getcwd()}")
        self.get_logger().info(f"Using database URL: {database_url}")
        
        # Initialize everything
        self.initialize(database_url)
        
        # Create timer for polling
        polling_interval = self.get_parameter('polling_interval_ms').value / 1000.0  # Convert to seconds
        self.polling_timer = self.create_timer(polling_interval, self.poll_events)
        
    def _validate_parameters(self):
        """Validate all parameters."""
        # Validate package_id (required)
        package_id = self.get_parameter('package_id').value
        if not package_id:
            self.get_logger().error("No package_id provided. This parameter is required.")
            rclpy.shutdown()
            return
            
        # Validate network
        network = self.get_parameter('network').value
        if network not in ['testnet', 'mainnet', 'devnet']:
            self.get_logger().error(f"Invalid network value: {network}. Must be one of: testnet, mainnet, devnet")
            rclpy.shutdown()
            return
            
        # Validate polling_interval_ms
        polling_interval = self.get_parameter('polling_interval_ms').value
        if polling_interval <= 0:
            self.get_logger().error(f"Invalid polling_interval_ms: {polling_interval}. Must be > 0")
            rclpy.shutdown()
            return
            
        # Validate default_limit
        default_limit = self.get_parameter('default_limit').value
        if not (1 <= default_limit <= 100):
            self.get_logger().error(f"Invalid default_limit: {default_limit}. Must be between 1 and 100")
            rclpy.shutdown()
            return
    
    def start_event_loop(self):
        """Start asyncio event loop in separate thread."""
        def run_event_loop():
            self.event_loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.event_loop)
            self.event_loop.run_forever()

        self.event_thread = Thread(target=run_event_loop, daemon=True)
        self.event_thread.start()

    def initialize(self, database_url):
        """Initialize clients and set up event tracking."""
        try:
            # Start event loop
            self.start_event_loop()
            
            # Initialize Prisma with database URL from parameter
            self.db = Prisma(
                datasource={
                    "url": database_url
                }
            )
            
            future = asyncio.run_coroutine_threadsafe(
                self.db.connect(),
                self.event_loop
            )
            future.result()  # Wait for connection
            
            # Initialize Sui client with proper network configuration
            network = self.get_parameter('network').value
            network_urls = {
                'devnet': (DEVNET_SUI_URL, DEVNET_SOCKET_URL),
                'testnet': (TESTNET_SUI_URL, TESTNET_SOCKET_URL),
                'mainnet': (MAINNET_SUI_URL, MAINNET_SOCKET_URL)
            }
            
            if network not in network_urls:
                raise ValueError(f"Invalid network {network}. Must be one of: devnet, testnet, mainnet")
                
            rpc_url, ws_url = network_urls[network]
            config = SuiConfig.user_config(
                rpc_url=rpc_url,
                ws_url=ws_url
            )
            self.sui_client = SuiClient(config)
            
            package_id = self.get_parameter('package_id').value
            
            # Set up event trackers with proper filter objects
            self.event_trackers = [
                EventTracker(
                    type=f"{package_id}::lock",
                    filter=MoveEventModuleQuery(
                        module="lock",
                        package_id=package_id
                    ),
                    callback=self.handle_lock_objects
                ),
                EventTracker(
                    type=f"{package_id}::shared",
                    filter=MoveEventModuleQuery(
                        module="shared",
                        package_id=package_id
                    ),
                    callback=self.handle_escrow_objects
                )
            ]
            
            # Load initial cursors
            for tracker in self.event_trackers:
                tracker.cursor = self.get_latest_cursor(tracker)
            
        except Exception as e:
            self.get_logger().error(f"Initialization error: {str(e)}")
            rclpy.shutdown()
    
    async def execute_event_job(self, tracker: EventTracker) -> Tuple[Optional[dict], bool]:
        """Execute a single event job, matching TypeScript implementation pattern."""
        try:
            self.get_logger().info(f"Current tracker cursor type: {type(tracker.cursor)}, value: {tracker.cursor}")
            
            # Convert dict cursor to EventID for Sui query
            query_cursor = None
            if tracker.cursor:
                query_cursor = EventID(
                    event_seq=str(tracker.cursor["eventSeq"]),
                    tx_seq=tracker.cursor["txDigest"]
                )
            
            # Query events from the chain
            query_builder = QueryEvents(
                query=tracker.filter,
                cursor=query_cursor,
                limit=self.get_parameter('default_limit').value,
                descending_order=False
            )
            
            result = self.sui_client.execute(query_builder)
            if not result.is_ok():
                self.get_logger().error(f"Failed to query events: {result.result_string}")
                return tracker.cursor, False
            
            # Extract data from the result
            events_data = result.result_data
            data = events_data.data
            has_next_page = events_data.has_next_page
            next_cursor = events_data.next_cursor
            
            self.get_logger().info(f"Query result - Events: {len(data) if data else 0}, HasNextPage: {has_next_page}")
            if next_cursor:
                self.get_logger().info(f"Next cursor: {next_cursor}")
            
            # Process events and update cursor only if we have both new data and next cursor
            if next_cursor and data and len(data) > 0:
                self.get_logger().info(f"Processing {len(data)} events")
                try:
                    # Directly await the callback since we're already in the async context
                    await tracker.callback(data, tracker.type)
                    
                    # Publish events to ROS topics
                    for event in data:
                        msg = SuiEvent()
                        msg.event_type = event.event_type
                        msg.tx_digest = event.event_id['txDigest']
                        msg.event_seq = int(event.event_id['eventSeq'])
                        msg.event_data = json.dumps(event.parsed_json)
                        msg.module_name = event.transaction_module
                        msg.package_id = event.package_id
                        
                        timestamp = Time()
                        ms = int(event.timestamp_ms)
                        timestamp.sec = ms // 1000
                        timestamp.nanosec = (ms % 1000) * 1000000
                        msg.timestamp = timestamp

                        self.get_logger().info(f"Publishing event: {msg}")
                        
                        self.event_pub.publish(msg)
                    
                    # Update cursor after successful processing
                    await self.save_latest_cursor(tracker, next_cursor)
                    return next_cursor, has_next_page
                    
                except Exception as e:
                    self.get_logger().error(f"Error processing events: {str(e)}")
                    return tracker.cursor, False
            else:
                self.get_logger().info("No new events to process or no next cursor")
                return tracker.cursor, False
            
        except Exception as e:
            self.get_logger().error(f"Error in execute_event_job: {str(e)}")
            return tracker.cursor, False
    
    def poll_events(self):
        """Timer callback to poll for events."""
        for tracker in self.event_trackers:
            try:
                # Execute event job and get result
                cursor, has_next_page = asyncio.run_coroutine_threadsafe(
                    self.execute_event_job(tracker),
                    self.event_loop
                ).result()
                
                # Update tracker with new cursor
                tracker.cursor = cursor
                
                # Use timer reset for proper ROS2 event loop handling
                if has_next_page:
                    self.get_logger().info("More events available, polling with minimal delay")
                    self.polling_timer.timer_period_ns = 1000000  # 1ms in nanoseconds
                    self.polling_timer.reset()
                else:
                    interval_ms = self.get_parameter('polling_interval_ms').value
                    self.get_logger().info(f"No more events, waiting {interval_ms}ms")
                    self.polling_timer.timer_period_ns = interval_ms * 1000000  # Convert ms to ns
                    self.polling_timer.reset()
                
            except Exception as e:
                self.get_logger().error(f"Error in poll_events: {str(e)}")
                # On error, keep current cursor and wait normal interval
                interval_ms = self.get_parameter('polling_interval_ms').value
                self.polling_timer.timer_period_ns = interval_ms * 1000000
                self.polling_timer.reset()
    
    def get_latest_cursor(self, tracker: EventTracker) -> Optional[dict]:
        """Get the latest cursor for an event tracker."""
        try:
            future = asyncio.run_coroutine_threadsafe(
                self.db.cursor.find_unique(
                    where={
                        "id": tracker.type
                    }
                ),
                self.event_loop
            )
            cursor_data = future.result()
            self.get_logger().info(f"Retrieved cursor from DB: {cursor_data}")
            if cursor_data is None:
                return None
                
            # Return cursor as dictionary
            return {
                "txDigest": cursor_data.txDigest,
                "eventSeq": cursor_data.eventSeq
            }
        except Exception as e:
            self.get_logger().error(f"Error getting cursor: {str(e)}")
            return None
    
    def save_latest_cursor(self, tracker: EventTracker, cursor: dict):
        """Save the latest cursor for an event tracker."""
        self.get_logger().info(f"Saving cursor type: {type(cursor)}, value: {cursor}")
        data = {
            "id": tracker.type,
            "txDigest": cursor["txDigest"],
            "eventSeq": cursor["eventSeq"]
        }
        try:
            future = asyncio.run_coroutine_threadsafe(
                self.db.cursor.upsert(
                    where={
                        "id": tracker.type
                    },
                    data={
                        "create": data,
                        "update": {
                            "txDigest": cursor["txDigest"],
                            "eventSeq": cursor["eventSeq"]
                        }
                    }
                ),
                self.event_loop
            )
            future.result()  # Wait for the operation to complete
        except Exception as e:
            self.get_logger().error(f"Error saving cursor: {str(e)}")
    
    async def handle_lock_objects(self, events: List[Dict], type_: str):
        """Handle lock object events."""
        updates = {}
        
        for event in events:
            if not event.event_type.startswith(type_):
                raise ValueError('Invalid event module origin')
                
            data = event.parsed_json
            is_deletion_event = 'key_id' not in data
            
            if data['lock_id'] not in updates:
                updates[data['lock_id']] = {
                    'objectId': data['lock_id']
                }
            
            if is_deletion_event:
                updates[data['lock_id']]['deleted'] = True
                continue
            
            updates[data['lock_id']].update({
                'keyId': data['key_id'],
                'creator': data['creator'],
                'itemId': data['item_id']
            })
        
        # Update database
        for update in updates.values():
            try:
                await self.db.locked.upsert(
                    where={'objectId': update['objectId']},
                    data={
                        'create': update,
                        'update': update
                    }
                )
            except Exception as e:
                self.get_logger().error(f"Error updating locked object: {str(e)}")
    
    async def handle_escrow_objects(self, events: List[Dict], type_: str):
        """Handle escrow object events."""
        updates = {}
        
        for event in events:
            if not event.event_type.startswith(type_):
                raise ValueError('Invalid event module origin')
                
            data = event.parsed_json
            
            if data['escrow_id'] not in updates:
                updates[data['escrow_id']] = {
                    'objectId': data['escrow_id']
                }
            
            if event.event_type.endswith('::EscrowCancelled'):
                updates[data['escrow_id']]['cancelled'] = True
                continue
                
            if event.event_type.endswith('::EscrowSwapped'):
                updates[data['escrow_id']]['swapped'] = True
                continue
            
            updates[data['escrow_id']].update({
                'sender': data['sender'],
                'recipient': data['recipient'],
                'keyId': data['key_id'],
                'itemId': data['item_id']
            })
        
        # Update database
        for update in updates.values():
            try:
                await self.db.escrow.upsert(
                    where={'objectId': update['objectId']},
                    data={
                        'create': update,
                        'update': update
                    }
                )
            except Exception as e:
                self.get_logger().error(f"Error updating escrow object: {str(e)}")

    def destroy_node(self):
        """Clean up node resources."""
        if self.event_loop:
            # Close database connection
            if self.db:
                future = asyncio.run_coroutine_threadsafe(
                    self.db.disconnect(),
                    self.event_loop
                )
                try:
                    future.result(timeout=1.0)
                except Exception as e:
                    self.get_logger().error(f"Error disconnecting from database: {str(e)}")
            
            # Stop event loop
            self.event_loop.call_soon_threadsafe(self.event_loop.stop)
            
        if self.event_thread:
            self.event_thread.join(timeout=1.0)
            
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SuiIndexerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
