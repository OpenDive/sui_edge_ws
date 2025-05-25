# SUI Edge - ROS2 Integration

A ROS2 package that provides integration with the Sui blockchain through ROS2 services and event indexing.

## Features

- Submit transactions to Sui blockchain
- Query Sui events
- Get object information
- Get network status
- Blockchain event indexing and tracking
- Persistent event cursor storage
- Event publishing via ROS2 topics

## Installation

1. Create a ROS2 workspace:
```bash
mkdir -p ~/sui_edge_ws/src
cd ~/sui_edge_ws/src
```

2. Clone this repository:
```bash
git clone https://github.com/yourusername/sui_edge.git
```

3. Set up the Python virtual environment and install dependencies:
```bash
cd ~/sui_edge_ws

# Set up virtual environment and install dependencies
source scripts/setup-venv.sh

# Install ROS2 dependencies
rosdep install --from-paths src --ignore-src -r -y
```

4. Build the workspace:
```bash
# Source ROS2
source scripts/ros-source.sh

# Build the workspace (the virtual environment settings will be used automatically)
colcon build
```

5. Source the workspace:
```bash
source install/setup.bash
```

## Development

When working on the project, always ensure you're using the virtual environment:

1. Activate the environment:
```bash
cd ~/sui_edge_ws
source setup.local  # This loads the virtual environment settings
source install/setup.bash
```

2. After making changes, rebuild:
```bash
colcon build
source install/setup.bash
```

## Troubleshooting

If you encounter Python-related build issues:

1. Ensure you're using the virtual environment:
```bash
which python3  # Should point to your venv
echo $VIRTUAL_ENV  # Should show your venv path
```

2. If needed, clean and rebuild:
```bash
rm -rf build/ install/ log/
source setup.local
colcon build
```

## Usage

1. Source the workspace:
```bash
source ~/sui_edge_ws/install/setup.bash
```

2. Launch the Sui service node:
```bash
ros2 launch sui_edge service_launch.py
```

3. Launch the Sui indexer (in a new terminal):
```bash
# First source the workspace
source ~/sui_edge_ws/install/setup.bash

# Launch the indexer with your package ID
ros2 launch sui_indexer indexer.launch.py "package_id:='0xfe09cf0b3d77678b99250572624bf74fe3b12af915c5db95f0ed5d755612eb68'"

# Or with custom parameters
ros2 launch sui_indexer indexer.launch.py \
    "package_id:='0xfe09cf0b3d77678b99250572624bf74fe3b12af915c5db95f0ed5d755612eb68'" \
    "network:='mainnet'" \
    "polling_interval_ms:=2000"
```

4. Monitor indexed events:
```bash
# Monitor blockchain events
ros2 topic echo /sui_events

# Monitor indexer status
ros2 topic echo /indexer_status
```

## Project Structure

```
sui_edge_ws/
├── src/
│   ├── sui_edge/
│   │   ├── sui_edge/
│   │   │   ├── __init__.py
│   │   │   ├── sui_client.py      # Sui blockchain client
│   │   │   ├── sui_service_node.py # ROS2 service node
│   │   │   ├── message_translator.py
│   │   │   └── config.py
│   │   ├── launch/
│   │   │   └── service_launch.py  # Launch file
│   │   ├── config/
│   │   │   └── config.yaml       # Configuration file
│   │   ├── package.xml
│   │   └── setup.py
│   ├── sui_indexer/              # Blockchain event indexer
│   │   ├── sui_indexer/
│   │   │   ├── __init__.py
│   │   │   ├── indexer_node.py   # Main indexer node
│   │   │   └── scripts/
│   │   │       └── setup_prisma.py # Database setup
│   │   ├── prisma/
│   │   │   └── schema.prisma     # Database schema
│   │   ├── launch/
│   │   │   ├── setup.launch.py   # Database setup launch
│   │   │   └── indexer.launch.py # Main indexer launch
│   │   ├── msg/
│   │   │   ├── SuiEvent.msg     # Event message definition
│   │   │   └── IndexerStatus.msg # Status message definition
│   │   ├── package.xml
│   │   └── setup.py
│   └── sui_edge_msgs/            # Message definitions
```

## Services

- `/sui/submit_transaction` - Submit a transaction to the Sui blockchain
- `/sui/get_events` - Query events from the Sui blockchain
- `/sui/get_object` - Get object information
- `/sui/get_status` - Get network status

## Topics

- `/sui_events` - Published blockchain events
- `/indexer_status` - Indexer node status updates

## Configuration

### Service Node
Edit `config/config.yaml` to configure:
- Network (testnet/devnet/mainnet)
- RPC URL
- WebSocket URL
- Timeouts and other parameters

### Indexer Node
Configure via launch file parameters:
- `polling_interval_ms` (default: 1000) - Event polling interval
- `network` (default: testnet) - Sui network to connect to
- `default_limit` (default: 50) - Maximum events per query
- `package_id` (required) - Sui package ID to monitor
- `database_url` (default: file:sui_indexer.db) - SQLite database location

## Database Schema

The indexer maintains three tables:
- `Cursor` - Tracks event processing progress
- `Escrow` - Stores escrow-related events
- `Locked` - Stores lock-related events

Access the database directly using SQLite tools or via the Prisma client.
