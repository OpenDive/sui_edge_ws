from typing import Dict, Any
import json
from std_msgs.msg import String
from rclpy.impl.rcutils_logger import RcutilsLogger

class MessageTranslator:
    """Handles translation between ROS2 messages and Sui data structures."""
    
    def __init__(self, logger: RcutilsLogger):
        self.logger = logger

    def sui_event_to_ros_msg(self, event: Dict[str, Any]) -> String:
        """Convert a Sui event to a ROS2 message."""
        try:
            return String(data=json.dumps(event))
        except Exception as e:
            self.logger.error(f'Failed to convert Sui event to ROS message: {str(e)}')
            raise

    def ros_msg_to_sui_tx(self, msg: String) -> Dict[str, Any]:
        """Convert a ROS2 message to Sui transaction data."""
        try:
            return json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.logger.error(f'Failed to parse ROS message as JSON: {str(e)}')
            raise
        except Exception as e:
            self.logger.error(f'Failed to convert ROS message to Sui transaction: {str(e)}')
            raise

    def sui_tx_response_to_ros_msg(self, response: Dict[str, Any]) -> String:
        """Convert a Sui transaction response to a ROS2 message."""
        try:
            return String(data=json.dumps(response))
        except Exception as e:
            self.logger.error(f'Failed to convert Sui response to ROS message: {str(e)}')
            raise
