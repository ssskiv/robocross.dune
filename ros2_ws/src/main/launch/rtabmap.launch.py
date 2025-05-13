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
            default_value='true',
            description='Use sim time if true')
    use_sim_time = LaunchConfiguration('use_sim_time')
    no_sim = LaunchConfiguration('no_sim')


    config_ekf= os.path.join(get_package_share_directory(package_name),'config','ekf_params.yaml')
    
    rtabmap_odom = Node(
        package='rtabmap_odom', executable='rgbd_odometry', output="screen",
        parameters=[config_ekf],
        remappings=[
            ("depth/image", '/depth_camera/image'),
            ("rgb/camera_info", '/camera/camera_info'),
            ("rgb/image", '/camera/image'),
            ("odom", '/odometry/rgbd'),],
        arguments=["--delete_db_on_start", ''],
        prefix='',
        namespace='',
    )
    
    rtabmap_viz = Node(
        package='rtabmap_viz', executable='rtabmap_viz', output="screen",
            parameters=[{
                'subscribe_depth': True,
                
                }],
            remappings=[
                ("depth/image", '/depth_camera/image'),
                ("rgb/camera_info", '/camera/camera_info'),
                ("rgb/image", '/camera/image'),
                ("odom", '/odometry/rgbd'),],
            arguments=["--delete_db_on_start", ''],
            prefix='',
            namespace=''
    )
    
    return LaunchDescription([
        rtabmap_odom,
        # rtabmap_viz,
    ])
