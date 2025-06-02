import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

from launch_ros.actions import Node

def generate_launch_description():

    package_name = 'main'

    declare_use_sim_time = DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true')
    use_sim_time = LaunchConfiguration('use_sim_time')
    no_sim = LaunchConfiguration('no_sim')


    config_ekf= os.path.join(get_package_share_directory(package_name),'config','ekf_params.yaml')
    # config_ekf2= os.path.join(get_package_share_directory(package_name),'config','ekf_params2.yaml')
    
    ekf = Node(
        package = 'robot_localization',
        name = 'ekf_filter_node_map',
        executable = 'ekf_node',
        output="screen",
        parameters=[config_ekf, {'use_sim_time': use_sim_time}],
    )
    ekf2 = Node(
        package = 'robot_localization',
        name = 'ekf_filter_node_odom',
        executable = 'ekf_node',
        output="screen",
        parameters=[config_ekf, {'use_sim_time': use_sim_time}],
        remappings=[('/odometry/filtered', '/odometry/raw')],
    )
    indicator_node = Node(
        package = 'core',
        name = 'indicator_node',
        executable = 'indicator_node',
        output='screen',
    )
    goal_checker_node = Node(
        package = 'core',
        name = 'goal_checker_node',
        executable = 'goal_checker_node',
        output='screen',
    )
    goal_sender_node = Node(
        package = 'core',
        name = 'goal_sender_node',
        executable = 'goal_sender_node',
        output='screen',
    )
    uart_node = Node(
        package = 'interfaces',
        name = 'uart_node',
        executable = 'uart_node',
        output='screen',
        condition=IfCondition(no_sim),
    )
    mavlink_node = Node(
        package = 'interfaces',
        name = 'mavlink_node',
        executable = 'mamba_mavlink',
        output='screen',
        condition=IfCondition(no_sim),
    )
    
    gps_filter = Node(
        package='robot_localization',
        name = 'gps_filter_node',
        executable='navsat_transform_node',
        output='screen',
        parameters=[config_ekf],
        remappings=[('/odometry/filtered', '/odometry/filtered')]
    )
    
    rgbd_odometry_node = Node(
        package='rtabmap_odom', executable='rgbd_odometry', output="screen",
        parameters=[config_ekf],
        remappings=[
            ("depth/image", '/depth_camera/image'),
            ("rgb/camera_info", '/camera/camera_info'),
            ("rgb/image", '/camera/image'),
            ("odom", '/odometry/rgbd'),],
        arguments=["--delete_db_on_start", ''],
        prefix='',
        namespace='rtabmap'
    )
    
    nav_params = os.path.join(get_package_share_directory(package_name),'config','nav2_params.yaml')
    # nav_params = os.path.join(get_package_share_directory(package_name),'config','nav2_params_amcl.yaml')
    start_localization = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','localization.launch.py'
                    # get_package_share_directory(package_name),'launch','localization_launch_amcl.py'
                )]), 
                # condition=IfCondition( is_localization ), 
                launch_arguments={ 'use_sim_time': use_sim_time, 'params_file': nav_params}.items()
    )
    
    start_navigation = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','navigation.launch.py'
                )]), 
                # condition=IfCondition( is_navigation ), 
                launch_arguments={'use_sim_time': use_sim_time, 'map_subscribe_transient_local': 'true', 'params_file': nav_params}.items()
    )

    mapviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'mapviz.launch.py'
        )]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    
    gps_fixer = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="swri_transform",
            arguments=["0", "0", "0", "0", "0", "0", "base_link", "gps"]
        )
    lidar_fixer = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="swri_transform",
            arguments=["0", "0", "0.7", "0", "0.05", "0", "base_link", "lidar_link"]
        )
    #TODO!!!! Create/find and launch RealSense node !!!!     

    return LaunchDescription([
        declare_use_sim_time,
        ekf,
        ekf2,
        gps_fixer,
        lidar_fixer,
        gps_filter,
        # indicator_node,
        # goal_checker_node,
        # goal_sender_node,
        # uart_node,
        # mavlink_node,
        # start_localization,
        # start_navigation,
        # mapviz,
        TimerAction(
            period=5.0,  # Delay in seconds
            actions=[
                # rgbd_odometry_node
            ]
        )
    ])