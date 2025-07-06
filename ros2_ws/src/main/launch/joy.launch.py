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


    joy = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output = 'screen',
        respawn = True,
        # arguments=['-d'+os.path.join(get_package_share_directory(package_name), 'config', 'urdf_core.rviz')]
    )

    # teleop = Node(
    #     package="teleop_twist_joy",
    #     executable="teleop_node",
    #     name="teleop",
    #     output = 'screen',
    #     respawn = True,
    #     # arguments=['-d'+os.path.join(get_package_share_directory(package_name), 'config', 'urdf_core.rviz')]
    #     parameters = [{'joy_config':'/home/developer/robocross.dune/ros2_ws/src/main/config/f310.config.yam'}]

    # )
    
    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('teleop_twist_joy'), 'launch', 'teleop-launch.py'
        )]),
        launch_arguments={'joy_config': 'f310', 'joy_vel': 'cmd_vel_planned'}.items(),
        
    )

    #TODO!!!! Create/find and launch RealSense node !!!!     

    return LaunchDescription([
        ExecuteProcess(cmd=['adb', 'kill-server']),
        joy,
        teleop,
    ])
