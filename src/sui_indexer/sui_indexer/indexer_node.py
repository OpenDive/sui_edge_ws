#!/usr/bin/env python3
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional
import json
import asyncio
from threading import Thread

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from sui_indexer_msgs.msg import SuiEvent, IndexerStatus
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

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
        
        self.declare_parameter('database_url', 'file:sui_indexer.db', descriptor=ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description='Database URL for the indexer'
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
        
        # Initialize everything
        self.initialize()
        
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

    def initialize(self):
        """Initialize clients and set up event tracking."""
        try:
            # Start event loop
            self.start_event_loop()
            
            # Initialize Prisma with database URL from parameter
            database_url = self.get_parameter('database_url').value
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
    
    def poll_events(self):
        """Timer callback to poll for events."""
        for tracker in self.event_trackers:
            try:
                # Query events from the chain using the proper builder
                query_builder = QueryEvents(
                    query=tracker.filter,  # Pass the MoveEventModuleQuery directly
                    cursor=tracker.cursor,
                    limit=self.get_parameter('default_limit').value,
                    descending_order=False  # We want ascending order like the TypeScript version
                )
                
                result = self.sui_client.execute(query_builder)
                if not result.is_ok():
                    self.get_logger().error(f"Failed to query events: {result.result_string}")
                    continue
                
                # Extract data from the result
                events_data = result.result_data
                data = events_data.data
                has_next_page = events_data.has_next_page
                next_cursor = events_data.next_cursor
                
                # Debug logging
                self.get_logger().info(f"Events data type: {type(events_data)}")
                self.get_logger().info(f"Data type: {type(data)}")
                if data:
                    self.get_logger().info(f"First event type: {type(data[0])}")
                    self.get_logger().info(f"First event dir: {dir(data[0])}")
                    self.get_logger().info(f"First event content: {data[0]}")
                
                # Process events
                if data:
                    self.get_logger().info(f"Found {len(data)} events")
                    tracker.callback(data, tracker.type)
                    
                    # Publish events to ROS topics
                    for event in data:
                        msg = SuiEvent()
                        msg.event_type = event.event_type
                        msg.tx_digest = event.event_id['txDigest']
                        msg.event_seq = int(event.event_id['eventSeq'])  # Convert to uint64
                        msg.event_data = json.dumps(event.parsed_json)
                        msg.module_name = event.transaction_module
                        msg.package_id = event.package_id
                        
                        # Use event timestamp instead of current time
                        timestamp = Time()
                        # Convert milliseconds to seconds and nanoseconds
                        ms = int(event.timestamp_ms)
                        timestamp.sec = ms // 1000
                        timestamp.nanosec = (ms % 1000) * 1000000
                        msg.timestamp = timestamp
                        
                        self.event_pub.publish(msg)
                
                # Update cursor if we have new data
                if next_cursor and data:
                    self.save_latest_cursor(tracker, next_cursor)
                    tracker.cursor = next_cursor
                
                # Adjust polling interval if needed
                if has_next_page:
                    self.polling_timer.timer_period_ns = 0  # Poll immediately
                else:
                    self.polling_timer.timer_period_ns = self.get_parameter('polling_interval_ms').value * 1000000  # Convert ms to ns
                
            except Exception as e:
                self.get_logger().error(f"Error polling events: {str(e)}")
                continue
    
    def get_latest_cursor(self, tracker: EventTracker) -> Optional[EventID]:
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
            cursor = future.result()
            if cursor is None:
                return None
            return EventID(
                tx_digest=cursor.txDigest,
                event_seq=cursor.eventSeq,
                timestamp=cursor.timestamp
            )
        except Exception as e:
            self.get_logger().error(f"Error getting cursor: {str(e)}")
            return None
    
    def save_latest_cursor(self, tracker: EventTracker, cursor: EventID):
        """Save the latest cursor for an event tracker."""
        data = {
            "id": tracker.type,
            "txDigest": cursor.event_id['txDigest'],
            "eventSeq": cursor.event_id['eventSeq'],
            "timestamp": cursor.timestamp_ms
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
                            "txDigest": cursor.event_id['txDigest'],
                            "eventSeq": cursor.event_id['eventSeq'],
                            "timestamp": cursor.timestamp_ms
                        }
                    }
                ),
                self.event_loop
            )
            future.result()  # Wait for the operation to complete
        except Exception as e:
            self.get_logger().error(f"Error saving cursor: {str(e)}")
    
    def handle_lock_objects(self, events: List[Dict], type_: str):
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
            self.db.locked.upsert(
                where={'objectId': update['objectId']},
                data={
                    'create': update,
                    'update': update
                }
            )
    
    def handle_escrow_objects(self, events: List[Dict], type_: str):
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
            self.db.escrow.upsert(
                where={'objectId': update['objectId']},
                data={
                    'create': update,
                    'update': update
                }
            )

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
