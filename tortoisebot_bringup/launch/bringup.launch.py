import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Package directories
    pkg_share = get_package_share_directory('tortoisebot_description')
    rviz_launch_dir = os.path.join(pkg_share, 'launch')
    gazebo_launch_dir = os.path.join(get_package_share_directory('tortoisebot_gazebo'), 'launch')
    rplidar_launch_dir = os.path.join(get_package_share_directory('rplidar_ros'), 'launch')

    # Paths
    default_model_path = os.path.join(pkg_share, 'models/urdf/tortoisebot_simple.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/tortoisebot_sensor_display.rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Nodes
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': ParameterValue(
                Command(['xacro ', LaunchConfiguration('model')]),
                value_type=str
            )
        }]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Differential drive node (real hardware only)
    differential_drive_node = Node(
        package='tortoisebot_firmware',
        executable='differential.py',  # ✅ fixed executable name
        name='differential_drive_publisher',
        condition=launch.conditions.UnlessCondition(use_sim_time),
    )

    # Static transform for LIDAR
    static_tf_lidar_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_static_tf',
        arguments=['0','0','0','0','0','0','base_link','laser'],
        output='screen'
    )


    # Raspberry Pi Camera node (check package/executable names in your system)
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',  # ⚠️ verify this is correct
        name='camera',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=launch.conditions.UnlessCondition(use_sim_time),
    )

    # RPLIDAR node (real hardware only)
    # RPLIDAR node (real hardware only)
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',   # ✅ same as view_rplidar_a1_launch.py
        name='rplidar_node',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200,
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Sensitivity'
        }],
        output='screen',
        condition=launch.conditions.UnlessCondition(use_sim_time),
    )






    # Gazebo (simulation only)
    gazebo_node = Node(
        package='ros_gz_sim',
        executable='create',               # Modern replacement for gz_sim
        output='screen',
        arguments=['-v', '4', '-r', 
               os.path.join(get_package_share_directory('tortoisebot_gazebo'), 'worlds/room2.sdf')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=launch.conditions.IfCondition(use_sim_time),
     )	

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        DeclareLaunchArgument('use_sim_time', default_value='False',
                              description='Use simulation clock if true'),
        DeclareLaunchArgument('model', default_value=default_model_path,
                              description='Absolute path to robot urdf file'),
        DeclareLaunchArgument('rvizconfig', default_value=default_rviz_config_path,
                              description='Absolute path to rviz config file'),

        rviz_node,
        robot_state_publisher_node,
        joint_state_publisher_node,
        differential_drive_node,
        camera_node,
        rplidar_node,
        static_tf_lidar_node,
        gazebo_node,
    ])

