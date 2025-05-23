#!/usr/bin/env python3
import asyncio
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional
import json

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from sui_indexer_msgs.msg import SuiEvent, IndexerStatus

from prisma import Prisma
from pysui.sui.sui_clients import SuiClient
from pysui.sui.sui_config import SuiConfig

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
        
        # Initialize parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('polling_interval_ms', 1000),
                ('network', 'testnet'),
                ('default_limit', 50),
                ('database_url', 'file:sui_indexer.db'),
                ('package_id', '')
            ]
        )
        
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
        self.event_loop = asyncio.get_event_loop()
        
        # Initialize everything
        self.event_loop.create_task(self.initialize())
    
    async def initialize(self):
        """Initialize clients and start event tracking."""
        try:
            # Initialize Prisma
            self.db = Prisma()
            await self.db.connect()
            
            # Initialize Sui client
            config = SuiConfig.default()
            self.sui_client = SuiClient(config)
            
            # Set up event trackers
            package_id = self.get_parameter('package_id').value
            self.event_trackers = [
                EventTracker(
                    type=f"{package_id}::lock",
                    filter={
                        "MoveEventModule": {
                            "module": "lock",
                            "package": package_id
                        }
                    },
                    callback=self.handle_lock_objects
                ),
                EventTracker(
                    type=f"{package_id}::shared",
                    filter={
                        "MoveEventModule": {
                            "module": "shared",
                            "package": package_id
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
