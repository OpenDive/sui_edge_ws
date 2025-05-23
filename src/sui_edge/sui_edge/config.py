import os
from typing import Dict, Any
import yaml
import rclpy.logging

class BridgeConfig:
    """Configuration manager for the Sui-ROS2 bridge."""
    
    def __init__(self, config_path: str = None):
        self.logger = rclpy.logging.get_logger('bridge_config')
        self._config: Dict[str, Any] = {}
        
        # Default configuration
        self._config = {
            'sui': {
                'network': 'devnet',  # or 'testnet', 'mainnet'
                'rpc_url': None,  # Will use default from SDK if None
                'ws_url': None,   # WebSocket URL for events
                'keypair_path': os.path.expanduser('~/.sui/sui_config/sui.keystore'),
            },
            'ros': {
                'node_name': 'sui_bridge',
                'namespace': '',
                'topics': {
                    # Default topics configuration
                    'transaction_status': '/sui/transaction_status',
                    'events': '/sui/events',
                }
            },
            'bridge': {
                'event_polling_interval': 1.0,  # seconds
                'transaction_timeout': 30.0,    # seconds
                'max_batch_size': 50,          # maximum number of transactions to batch
            }
        }
        
        if config_path:
            self.load_config(config_path)

    def load_config(self, config_path: str) -> None:
        """Load configuration from a YAML file."""
        try:
            with open(config_path, 'r') as f:
                yaml_config = yaml.safe_load(f)
                self._update_config(yaml_config)
                self.logger.info(f'Loaded configuration from {config_path}')
        except Exception as e:
            self.logger.error(f'Error loading config from {config_path}: {str(e)}')
            raise

    def _update_config(self, new_config: Dict[str, Any]) -> None:
        """Recursively update configuration dictionary."""
        for key, value in new_config.items():
            if key in self._config:
                if isinstance(value, dict) and isinstance(self._config[key], dict):
                    self._config[key].update(value)
                else:
                    self._config[key] = value

    @property
    def sui_network(self) -> str:
        return self._config['sui']['network']

    @property
    def sui_rpc_url(self) -> str:
        return self._config['sui']['rpc_url']

    @property
    def sui_ws_url(self) -> str:
        return self._config['sui']['ws_url']

    @property
    def sui_keypair_path(self) -> str:
        return self._config['sui']['keypair_path']

    @property
    def ros_node_name(self) -> str:
        return self._config['ros']['node_name']

    @property
    def ros_namespace(self) -> str:
        return self._config['ros']['namespace']

    @property
    def ros_topics(self) -> Dict[str, str]:
        return self._config['ros']['topics']

    @property
    def event_polling_interval(self) -> float:
        return self._config['bridge']['event_polling_interval']

    @property
    def transaction_timeout(self) -> float:
        return self._config['bridge']['transaction_timeout']

    @property
    def max_batch_size(self) -> int:
        return self._config['bridge']['max_batch_size']
