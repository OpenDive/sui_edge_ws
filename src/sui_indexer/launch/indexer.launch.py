from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, RegisterEventHandler
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    pkg_share = FindPackageShare('sui_indexer')
    
    # First run Prisma setup
    setup_prisma = ExecuteProcess(
        cmd=['ros2', 'run', 'sui_indexer', 'prisma_setup'],
        name='prisma_setup',
        output='screen'
    )
    
    # Then start the indexer node
    indexer_node = Node(
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
    
    # Create a timer action that starts the indexer node only after prisma setup completes
    start_indexer = TimerAction(
        period=0.0,  # Start immediately after the condition is met
        actions=[indexer_node]
    )
    
    return LaunchDescription([
        setup_prisma,
        # Only start the indexer after prisma_setup exits successfully
        RegisterEventHandler(
            OnProcessExit(
                target_action=setup_prisma,
                on_exit=[start_indexer]
            )
        )
    ]) 