from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('sui_indexer')
    
    return LaunchDescription([
        # First include the setup launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    pkg_share,
                    'launch',
                    'setup.launch.py'
                ])
            ])
        ),
        
        # Then start the indexer node
        Node(
            package='sui_indexer',
            executable='indexer_node',
            name='sui_indexer',
            output='screen',
            parameters=[{
                'polling_interval_ms': 1000,
                'network': 'testnet',
                'default_limit': 50,
                'database_url': 'file:sui_indexer.db'
            }]
        )
    ]) 