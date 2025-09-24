from ament_index_python.packages import get_package_share_directory
import launch
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Path to world file
    world_path = os.path.join(get_package_share_directory('tortoisebot_gazebo'), 'worlds/room2.sdf')

    # Launch description
    return launch.LaunchDescription([
        # Declare use_sim_time argument
        launch.actions.DeclareLaunchArgument(
            name='use_sim_time',
            default_value='True',
            description='Flag to enable use_sim_time'
        ),

        # Gazebo node
        Node(
            package='ros_gz_sim',
            executable='gz_sim',
            output='screen',
            arguments=['-r', '-v', '4', world_path],
            parameters=[{'use_sim_time': use_sim_time}]
        )
    ])

