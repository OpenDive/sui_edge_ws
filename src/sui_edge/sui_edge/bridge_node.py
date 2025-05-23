import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import asyncio
from threading import Thread
from typing import Optional
import json

from .config import BridgeConfig
from .sui_client import SuiBlockchainClient
from .message_translator import MessageTranslator

class SuiBridgeNode(Node):
    """ROS2 node for bridging with Sui blockchain."""
    
    def __init__(self):
        self.config = BridgeConfig()
        super().__init__(self.config.ros_node_name)
        
        # Initialize components
        self.translator = MessageTranslator(self.get_logger())
        self.sui_client = SuiBridgeClient(self.config)
        
        # Set up publishers
        self.tx_status_pub = self.create_publisher(
            String,
            self.config.ros_topics['transaction_status'],
            10
        )
        self.events_pub = self.create_publisher(
            String,
            self.config.ros_topics['events'],
            10
        )
        
        # Set up subscribers
        self.create_subscription(
            String,
            '/sui/submit_transaction',
            self.handle_transaction_submission,
            10
        )
        
        # Initialize event loop for async operations
        self.event_loop: Optional[asyncio.AbstractEventLoop] = None
        self.event_thread: Optional[Thread] = None
        
        # Start async event loop
        self.start_async_loop()

    def start_async_loop(self):
        """Start asyncio event loop in separate thread."""
        def run_event_loop():
            self.event_loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.event_loop)
            self.event_loop.run_forever()

        self.event_thread = Thread(target=run_event_loop, daemon=True)
        self.event_thread.start()
        
        # Initialize Sui client
        future = asyncio.run_coroutine_threadsafe(
            self.sui_client.connect(),
            self.event_loop
        )
        future.result()  # Wait for connection
        
        # Start event polling
        self.create_timer(
            self.config.event_polling_interval,
            self.poll_events
        )

    def handle_transaction_submission(self, msg: String):
        """Handle incoming transaction submission requests."""
        try:
            tx_data = self.translator.ros_msg_to_sui_tx(msg)
            
            # Submit transaction asynchronously
            future = asyncio.run_coroutine_threadsafe(
                self.sui_client.submit_transaction(
                    tx_data['tx_bytes'],
                    tx_data['signature']
                ),
                self.event_loop
            )
            
            # Handle response
            response = future.result(timeout=self.config.transaction_timeout)
            response_msg = self.translator.sui_tx_response_to_ros_msg(response)
            self.tx_status_pub.publish(response_msg)
            
        except Exception as e:
            self.get_logger().error(f'Transaction submission failed: {str(e)}')
            # Publish error status
            error_msg = String(data=json.dumps({
                'status': 'error',
                'message': str(e)
            }))
            self.tx_status_pub.publish(error_msg)

    def poll_events(self):
        """Poll for new events from Sui network."""
        try:
            future = asyncio.run_coroutine_threadsafe(
                self.sui_client.get_events(limit=self.config.max_batch_size),
                self.event_loop
            )
            events = future.result(timeout=self.config.transaction_timeout)
            
            for event in events:
                event_msg = self.translator.sui_event_to_ros_msg(event)
                self.events_pub.publish(event_msg)
                
        except Exception as e:
            self.get_logger().error(f'Event polling failed: {str(e)}')

    def destroy_node(self):
        """Clean up node resources."""
        if self.event_loop:
            self.event_loop.call_soon_threadsafe(self.sui_client.close)
            self.event_loop.call_soon_threadsafe(self.event_loop.stop)
        if self.event_thread:
            self.event_thread.join(timeout=1.0)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SuiBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
