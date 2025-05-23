#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import asyncio
from threading import Thread
from typing import Optional
import json

from std_srvs.srv import Trigger
from rcl_interfaces.srv import GetParameters, ListParameters, SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

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
            SetParameters,
            'sui/submit_transaction',
            self.handle_submit_transaction
        )
        
        self.get_events_srv = self.create_service(
            ListParameters,
            'sui/get_events',
            self.handle_get_events
        )
        
        self.get_object_srv = self.create_service(
            GetParameters,
            'sui/get_object',
            self.handle_get_object
        )
        
        self.get_status_srv = self.create_service(
            Trigger,
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
        # Extract transaction data from parameters
        tx_bytes = None
        signature = None
        for param in request.parameters:
            if param.name == 'tx_bytes':
                tx_bytes = param.value.string_value
            elif param.name == 'signature':
                signature = param.value.string_value
        
        if not tx_bytes or not signature:
            response.results = []
            return response
            
        future = asyncio.run_coroutine_threadsafe(
            self.sui_client.submit_transaction(tx_bytes, signature),
            self.event_loop
        )
        
        try:
            result = future.result(timeout=self.config.transaction_timeout)
            # Convert result to parameter
            param = Parameter()
            param.name = 'transaction_result'
            param.value.type = ParameterType.STRING
            param.value.string_value = json.dumps(result)
            response.results.append(param)
        except Exception as e:
            param = Parameter()
            param.name = 'error'
            param.value.type = ParameterType.STRING
            param.value.string_value = str(e)
            response.results.append(param)
        
        return response

    def handle_get_events(self, request, response):
        """Handle event query service calls."""
        # Extract cursor and limit from prefix if provided
        cursor = None
        limit = 50  # default
        if request.prefixes:
            try:
                cursor = request.prefixes[0]
                if len(request.prefixes) > 1:
                    limit = int(request.prefixes[1])
            except (IndexError, ValueError):
                pass
                
        future = asyncio.run_coroutine_threadsafe(
            self.sui_client.get_events(cursor=cursor, limit=limit),
            self.event_loop
        )
        
        try:
            result = future.result(timeout=self.config.transaction_timeout)
            response.result = json.dumps(result)
        except Exception as e:
            response.result = json.dumps({'error': str(e)})
        
        return response

    def handle_get_object(self, request, response):
        """Handle object query service calls."""
        if not request.names:
            response.values = []
            return response
            
        object_id = request.names[0]  # Get first object ID
        future = asyncio.run_coroutine_threadsafe(
            self.sui_client.get_object(object_id),
            self.event_loop
        )
        
        try:
            result = future.result(timeout=self.config.transaction_timeout)
            value = ParameterValue()
            value.type = ParameterType.STRING
            value.string_value = json.dumps(result)
            response.values.append(value)
        except Exception as e:
            value = ParameterValue()
            value.type = ParameterType.STRING
            value.string_value = json.dumps({'error': str(e)})
            response.values.append(value)
        
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
            response.message = json.dumps(result)
        except Exception as e:
            response.success = False
            response.message = str(e)
        
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