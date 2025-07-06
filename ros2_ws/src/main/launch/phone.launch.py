import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
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


    imu = Node(
            package="interfaces",
            executable="imu_publisher_node",
            name="imu_node",
            output = 'screen',
            respawn = True,
            # arguments=['-d'+os.path.join(get_package_share_directory(package_name), 'config', 'urdf_core.rviz')]
    )

    gps = Node(
            package="interfaces",
            executable="gps_publisher_node",
            name="gps_node",
            output = 'screen',
            respawn = True,
            # arguments=['-d'+os.path.join(get_package_share_directory(package_name), 'config', 'urdf_core.rviz')]
    )
    
    
    #TODO!!!! Create/find and launch RealSense node !!!!     

    return LaunchDescription([
        # ExecuteProcess(cmd=['adb', 'kill-server']),
        ExecuteProcess(cmd=['adb', 'start-server']),
        ExecuteProcess(cmd=['adb', 'forward', 'tcp:45123', 'tcp:45123']),
        imu,
        gps,
    ])
