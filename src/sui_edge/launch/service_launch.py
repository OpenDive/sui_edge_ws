from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sui_edge',
            executable='sui_service_node',
            name='sui_service_node',
            output='screen',
            emulate_tty=True,
        )
    ]) 