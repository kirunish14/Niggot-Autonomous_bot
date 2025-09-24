import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Package paths
    pkg_share = get_package_share_directory('tortoisebot_description')
    navigation_dir = os.path.join(get_package_share_directory('tortoisebot_navigation'), 'launch')
    gazebo_launch_dir = os.path.join(get_package_share_directory('tortoisebot_gazebo'), 'launch')
    rplidar_launch_dir = os.path.join(get_package_share_directory('rplidar_ros'), 'launch')
    cartographer_launch_dir = os.path.join(get_package_share_directory('tortoisebot_slam'), 'launch')

    default_model_path = os.path.join(pkg_share, 'models/urdf/tortoisebot_simple.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/tortoisebot_sensor_display.rviz')
    params_file_robot = os.path.join(get_package_share_directory('tortoisebot_navigation'), 'config', 'nav2_params_robot.yaml')
    map_file = os.path.join(get_package_share_directory('tortoisebot_bringup'), 'maps', 'room2.yaml')

    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    exploration = LaunchConfiguration('exploration')

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Robot state publisher
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



    # Joint state publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Differential drive (hardware only)
    differential_drive_node = Node(
        package='tortoisebot_firmware',
        executable='differential.py',
        name='differential_drive_publisher',
        condition=UnlessCondition(use_sim_time),
    )

    # Pi camera (hardware only)
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camera_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=UnlessCondition(use_sim_time),
    )

    # RPLIDAR A1 (hardware only)
    rplidar_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rplidar_launch_dir, 'rplidar_a1_launch.py')
        ),
        condition=UnlessCondition(use_sim_time),
    )
    
    
        # Static transform: base_link -> laser
    static_tf_laser_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0.2', '0', '0', '0', 'base_link', 'laser'],
        condition=UnlessCondition(use_sim_time),   # only for real robot
    )


    # Gazebo (simulation only)
    gazebo_node = Node(
        package='ros_gz_sim',
        executable='create',  
        output='screen',
        arguments=['-v', '4', '-r', 
                   os.path.join(get_package_share_directory('tortoisebot_gazebo'), 'worlds/room2.sdf')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_sim_time),
    )

    # Navigation (Nav2)
    navigation_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_dir, 'navigation.launch.py')
        ),
        launch_arguments={'params_file': params_file_robot}.items()
    )

    # Cartographer SLAM
    cartographer_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(cartographer_launch_dir, 'cartographer.launch.py')
        ),
        launch_arguments={
            'params_file': params_file_robot,
            'exploration': exploration,
            'use_sim_time': use_sim_time
        }.items()
    )

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        DeclareLaunchArgument('use_sim_time', default_value='False', description='Use simulation clock if true'),
        DeclareLaunchArgument('exploration', default_value='True', description='Enable exploration mode'),
        DeclareLaunchArgument('model', default_value=default_model_path, description='Absolute path to robot urdf file'),
        DeclareLaunchArgument('map', default_value=map_file, description='Map to be used'),
        DeclareLaunchArgument('rvizconfig', default_value=default_rviz_config_path, description='Absolute path to rviz config file'),

        rviz_node,
        robot_state_publisher_node,
        joint_state_publisher_node,
        differential_drive_node,
        camera_node,
        rplidar_launch_cmd,
        static_tf_laser_node,
        gazebo_node,
        navigation_launch_cmd,
        cartographer_launch_cmd
    ])

