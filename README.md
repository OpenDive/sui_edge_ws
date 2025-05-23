# Sui Edge - ROS2 Blockchain Bridge

A ROS2 bridge for integrating robots and smart devices with the Sui blockchain. This package enables bidirectional communication between ROS2 nodes and the Sui blockchain, allowing for blockchain-based control, monitoring, and data verification in robotics applications.

## Features

- Bidirectional communication between ROS2 and Sui blockchain
- Real-time event monitoring from the Sui network
- Transaction submission and status tracking
- Configurable polling intervals and batch sizes
- Support for multiple Sui networks (devnet, testnet, mainnet)
- Asynchronous operation for improved performance

## Prerequisites

- ROS2 (Humble or later recommended)
- Python 3.8+
- Sui SDK and CLI tools

## Installation

1. Set up a Python virtual environment:
```bash
# Create and activate virtual environment
python3 -m venv .venv
source .venv/bin/activate

# Update pip
pip install --upgrade pip
```

2. Install ROS2 build dependencies in the virtual environment:
```bash
# Core dependencies
pip install pysui>=0.85.0 pyyaml

# ROS2 build dependencies
pip install empy catkin_pkg numpy
```

3. Clone the repository:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/yourusername/sui_edge_ws.git
cd sui_edge_ws
```

4. Build the ROS2 package:
```bash
cd ~/ros2_ws
colcon build --packages-select sui_edge
source install/setup.bash
```

## Configuration

The bridge can be configured through the `config/bridge_config.yaml` file. Key configuration options include:

```yaml
sui:
  network: "devnet"        # Sui network to connect to
  rpc_url: null           # Custom RPC URL (optional)
  ws_url: null            # Custom WebSocket URL (optional)
  keypair_path: "~/.sui/sui_config/sui.keystore"

ros:
  node_name: "sui_bridge"  # Name of the ROS2 node
  topics:                  # ROS2 topic configuration
    transaction_status: "/sui/transaction_status"
    events: "/sui/events"

bridge:
  event_polling_interval: 1.0  # Event polling frequency (seconds)
  transaction_timeout: 30.0    # Transaction timeout (seconds)
  max_batch_size: 50          # Maximum events per batch
```

## Usage

1. Launch the bridge:
```bash
ros2 launch sui_edge bridge_launch.py
```

2. Submit transactions to the Sui blockchain:
```bash
# Publish transaction data to the submission topic
ros2 topic pub /sui/submit_transaction std_msgs/String "data: '{\"tx_bytes\": \"...\", \"signature\": \"...\"}'"
```

3. Monitor transaction status:
```bash
ros2 topic echo /sui/transaction_status
```

4. Monitor Sui events:
```bash
ros2 topic echo /sui/events
```

## ROS2 Topics

### Publishers
- `/sui/transaction_status` (std_msgs/String)
  - Publishes transaction execution results and status updates
- `/sui/events` (std_msgs/String)
  - Publishes events from the Sui blockchain

### Subscribers
- `/sui/submit_transaction` (std_msgs/String)
  - Accepts transaction data for submission to the Sui blockchain

## Message Formats

### Transaction Submission
```json
{
    "tx_bytes": "base64_encoded_transaction",
    "signature": "base64_encoded_signature"
}
```

### Transaction Status
```json
{
    "status": "success|error",
    "digest": "transaction_digest",
    "effects": {
        // Transaction effects
    }
}
```

### Sui Events
```json
{
    "id": "event_id",
    "type": "event_type",
    "data": {
        // Event specific data
    }
}
```

## Development

### Project Structure

sui_edge/
├── sui_edge/
│ ├── init.py
│ ├── bridge_node.py # Main ROS2 node implementation
│ ├── sui_client.py # Sui blockchain interaction
│ ├── message_translator.py # Message conversion utilities
│ └── config.py # Configuration management
├── launch/
│ └── bridge_launch.py # Launch file
├── config/
│ └── bridge_config.yaml # Configuration file
└── test/
└── test_bridge.py # Unit tests

## Adding New Features

1. Message Translation:
   - Extend `message_translator.py` to support new message types
   - Add corresponding ROS2 message definitions if needed

2. Blockchain Integration:
   - Add new methods to `sui_client.py` for additional blockchain interactions
   - Update the bridge node to handle new functionality

3. Configuration:
   - Add new configuration options to `bridge_config.yaml`
   - Update `config.py` to support new options

## Testing

Run the tests using:
```bash
colcon test --packages-select sui_edge
```

## Contributing

1. Fork the repository
2. Create a feature branch
3. Commit your changes
4. Push to the branch
5. Create a Pull Request

## License

Apache License 2.0

## Support

For issues and feature requests, please use the GitHub issue tracker.
