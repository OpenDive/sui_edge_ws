#!/usr/bin/env python3
import asyncio
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional
import json

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

@dataclass
class EventTracker:
    """Tracks events for a specific module."""
    type: str
    filter: Dict[str, Any]
    callback: Callable

class SuiIndexerNode(Node):
    """ROS2 node for indexing Sui blockchain events."""
    
    def __init__(self):
        super().__init__('sui_indexer')
        
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
        self.tracker_tasks = {}
        
        # Setup async loop
        self.event_loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.event_loop)
        
        # Initialize everything
        self.event_loop.create_task(self.initialize())
    
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
    
    async def initialize(self):
        """Initialize clients and start event tracking."""
        try:
            # Initialize Prisma
            self.db = Prisma()
            await self.db.connect()
            
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
            
            # Set up event trackers
            self.event_trackers = [
                EventTracker(
                    type=f"{self.get_parameter('package_id').value}::lock",
                    filter={
                        "MoveEventModule": {
                            "module": "lock",
                            "package": self.get_parameter('package_id').value
                        }
                    },
                    callback=self.handle_lock_objects
                ),
                EventTracker(
                    type=f"{self.get_parameter('package_id').value}::shared",
                    filter={
                        "MoveEventModule": {
                            "module": "shared",
                            "package": self.get_parameter('package_id').value
                        }
                    },
                    callback=self.handle_escrow_objects
                )
            ]
            
            # Start tracking
            await self.setup_listeners()
            
        except Exception as e:
            self.get_logger().error(f"Initialization error: {str(e)}")
            rclpy.shutdown()
    
    async def setup_listeners(self):
        """Set up event listeners for each tracker."""
        for event in self.event_trackers:
            self.tracker_tasks[event.type] = self.event_loop.create_task(
                self.run_event_job(event, await self.get_latest_cursor(event))
            )
    
    async def execute_event_job(self, tracker: EventTracker, cursor: Optional[Dict]) -> Dict:
        """Execute a single event job."""
        try:
            # Query events from the chain
            response = await self.sui_client.queryEvents({
                "query": tracker.filter,
                "cursor": cursor,
                "order": "ascending"
            })
            
            data = response.get('data', [])
            has_next_page = response.get('hasNextPage', False)
            next_cursor = response.get('nextCursor')
            
            # Process events
            if data:
                self.get_logger().info("Found events:" + str(data))
                await tracker.callback(data, tracker.type)
                
                # Publish events to ROS topics
                for event in data:
                    msg = SuiEvent()
                    msg.event_type = event.get('type', '')
                    msg.transaction_digest = event.get('txDigest', '')
                    msg.event_sequence = str(event.get('eventSeq', ''))
                    msg.module_name = tracker.filter['MoveEventModule']['module']
                    msg.package_id = tracker.filter['MoveEventModule']['package']
                    msg.parsed_json = json.dumps(event.get('parsedJson', {}))
                    # Set timestamp
                    now = self.get_clock().now().to_msg()
                    msg.timestamp = now
                    self.event_pub.publish(msg)
            
            # Update cursor if we have new data
            if next_cursor and data:
                await self.save_latest_cursor(tracker, next_cursor)
                return {
                    'cursor': next_cursor,
                    'hasNextPage': has_next_page
                }
                
        except Exception as e:
            self.get_logger().error(f"Error in execute_event_job: {str(e)}")
        
        return {
            'cursor': cursor,
            'hasNextPage': False
        }
    
    async def run_event_job(self, tracker: EventTracker, cursor: Optional[Dict]):
        """Run event job continuously."""
        while rclpy.ok():
            result = await self.execute_event_job(tracker, cursor)
            cursor = result['cursor']
            
            # Wait appropriate interval
            await asyncio.sleep(
                0 if result['hasNextPage'] else 
                self.get_parameter('polling_interval_ms').value / 1000.0
            )
    
    async def get_latest_cursor(self, tracker: EventTracker) -> Optional[Dict]:
        """Get the latest cursor for an event tracker."""
        cursor = await self.db.cursor.find_unique(
            where={
                'id': tracker.type
            }
        )
        return cursor
    
    async def save_latest_cursor(self, tracker: EventTracker, cursor: Dict):
        """Save the latest cursor for an event tracker."""
        data = {
            'eventSeq': cursor['eventSeq'],
            'txDigest': cursor['txDigest']
        }
        
        await self.db.cursor.upsert(
            where={
                'id': tracker.type
            },
            data={
                'create': {'id': tracker.type, **data},
                'update': data
            }
        )
    
    async def handle_lock_objects(self, events: List[Dict], type_: str):
        """Handle lock object events."""
        updates = {}
        
        for event in events:
            if not event['type'].startswith(type_):
                raise ValueError('Invalid event module origin')
                
            data = event['parsedJson']
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
            await self.db.locked.upsert(
                where={'objectId': update['objectId']},
                data={
                    'create': update,
                    'update': update
                }
            )
    
    async def handle_escrow_objects(self, events: List[Dict], type_: str):
        """Handle escrow object events."""
        updates = {}
        
        for event in events:
            if not event['type'].startswith(type_):
                raise ValueError('Invalid event module origin')
                
            data = event['parsedJson']
            
            if data['escrow_id'] not in updates:
                updates[data['escrow_id']] = {
                    'objectId': data['escrow_id']
                }
            
            if event['type'].endswith('::EscrowCancelled'):
                updates[data['escrow_id']]['cancelled'] = True
                continue
                
            if event['type'].endswith('::EscrowSwapped'):
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
            await self.db.escrow.upsert(
                where={'objectId': update['objectId']},
                data={
                    'create': update,
                    'update': update
                }
            )

    def __del__(self):
        """Cleanup when the node is destroyed."""
        if self.event_loop:
            # Cancel all running tasks
            for task in asyncio.all_tasks(self.event_loop):
                task.cancel()
            # Run the event loop one last time to process cancellations
            self.event_loop.run_until_complete(asyncio.gather(*asyncio.all_tasks(self.event_loop), return_exceptions=True))
            self.event_loop.close()
            
    def destroy_node(self):
        """Clean up the node."""
        # Cancel all running tasks
        if self.event_loop:
            for task in asyncio.all_tasks(self.event_loop):
                task.cancel()
            # Run the event loop one last time to process cancellations
            self.event_loop.run_until_complete(asyncio.gather(*asyncio.all_tasks(self.event_loop), return_exceptions=True))
            self.event_loop.close()
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
