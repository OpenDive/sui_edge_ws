#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import asyncio
from threading import Thread
from typing import Optional
import json

from sui_edge.srv import SubmitTransaction, GetEvents, GetObject, GetStatus
from .sui_client import SuiBlockchainClient
from .config import BridgeConfig

class SuiServiceNode(Node):
    """ROS2 Service node for Sui blockchain interactions."""
    
    def __init__(self):
        super().__init__('sui_service_node')
        
        # Load configuration
        self.config = BridgeConfig()
        
        # Initialize blockchain client
        self.sui_client = SuiBlockchainClient(
            network=self.config.sui_network,
            rpc_url=self.config.sui_rpc_url,
            ws_url=self.config.sui_ws_url
        )
        
        # Initialize event loop for async operations
        self.event_loop: Optional[asyncio.AbstractEventLoop] = None
        self.event_thread: Optional[Thread] = None
        
        # Create services
        self.submit_tx_srv = self.create_service(
            SubmitTransaction,
            'sui/submit_transaction',
            self.handle_submit_transaction
        )
        
        self.get_events_srv = self.create_service(
            GetEvents,
            'sui/get_events',
            self.handle_get_events
        )
        
        self.get_object_srv = self.create_service(
            GetObject,
            'sui/get_object',
            self.handle_get_object
        )
        
        self.get_status_srv = self.create_service(
            GetStatus,
            'sui/get_status',
            self.handle_get_status
        )
        
        # Start async event loop
        self.start_async_loop()
        
    def start_async_loop(self):
        """Start asyncio event loop in separate thread."""
        def run_event_loop():
            self.event_loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.event_loop)
            
            # Connect to Sui network
            self.event_loop.run_until_complete(self.sui_client.connect())
            
            self.event_loop.run_forever()

        self.event_thread = Thread(target=run_event_loop, daemon=True)
        self.event_thread.start()

    def handle_submit_transaction(self, request, response):
        """Handle transaction submission service calls."""
        future = asyncio.run_coroutine_threadsafe(
            self.sui_client.submit_transaction(
                request.tx_bytes,
                request.signature
            ),
            self.event_loop
        )
        
        try:
            result = future.result(timeout=self.config.transaction_timeout)
            response.success = result['success']
            response.status = result.get('status', '')
            response.digest = result.get('digest', '')
            response.error_message = result.get('error', '')
            response.effects = json.dumps(result.get('effects', {}))
        except Exception as e:
            response.success = False
            response.error_message = str(e)
        
        return response

    def handle_get_events(self, request, response):
        """Handle event query service calls."""
        future = asyncio.run_coroutine_threadsafe(
            self.sui_client.get_events(
                cursor=request.cursor if request.cursor else None,
                limit=request.limit
            ),
            self.event_loop
        )
        
        try:
            result = future.result(timeout=self.config.transaction_timeout)
            response.success = result['success']
            response.events = [json.dumps(event) for event in result.get('events', [])]
            response.next_cursor = result.get('next_cursor', '')
            response.error_message = result.get('error', '')
        except Exception as e:
            response.success = False
            response.error_message = str(e)
        
        return response

    def handle_get_object(self, request, response):
        """Handle object query service calls."""
        future = asyncio.run_coroutine_threadsafe(
            self.sui_client.get_object(request.object_id),
            self.event_loop
        )
        
        try:
            result = future.result(timeout=self.config.transaction_timeout)
            response.success = result['success']
            response.object_info = json.dumps(result.get('object_info', {}))
            response.error_message = result.get('error', '')
        except Exception as e:
            response.success = False
            response.error_message = str(e)
        
        return response

    def handle_get_status(self, request, response):
        """Handle status query service calls."""
        future = asyncio.run_coroutine_threadsafe(
            self.sui_client.get_status(),
            self.event_loop
        )
        
        try:
            result = future.result(timeout=self.config.transaction_timeout)
            response.success = True
            response.network = result['network']
            response.latest_checkpoint = result['latest_checkpoint']
            response.error_message = ''
        except Exception as e:
            response.success = False
            response.error_message = str(e)
        
        return response

    def destroy_node(self):
        """Clean up node resources."""
        if self.event_loop:
            future = asyncio.run_coroutine_threadsafe(
                self.sui_client.close(),
                self.event_loop
            )
            future.result(timeout=1.0)
            self.event_loop.call_soon_threadsafe(self.event_loop.stop)
        if self.event_thread:
            self.event_thread.join(timeout=1.0)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SuiServiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 