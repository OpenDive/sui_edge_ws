from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('sui_edge')
    
    config_path = os.path.join(pkg_share, 'config', 'bridge_config.yaml')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=config_path,
            description='Path to configuration file'
        ),
        
        # Sui Service Node
        Node(
            package='sui_edge',
            executable='sui_service_node',
            name='sui_service_node',
            parameters=[
                {'config_file': LaunchConfiguration('config_file')}
            ],
            output='screen'
        ),
        
        # Bridge Node (if still needed for other functionality)
        Node(
            package='sui_edge',
            executable='bridge_node',
            name='sui_bridge',
            parameters=[
                {'config_file': LaunchConfiguration('config_file')}
            ],
            output='screen'
        )
    ])
