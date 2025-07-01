import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.event_handlers import OnShutdown
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, RegisterEventHandler

from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


import xacro
import pathlib


def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')
    package_name='main'
    package_dir = get_package_share_directory(package_name)
    
    params = {'use_sim_time': use_sim_time}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        # namespace="main_bot",
        parameters=[params]
    )

    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        # namespace="main_bot",
        parameters=[params]
    )
    robot_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','robot.launch.py'
                )]), 
                # condition=IfCondition( is_navigation ), 
                launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    urdf = os.path.join(
        package_dir,
        os.path.join(package_dir, pathlib.Path(os.path.join(package_dir, 'resource', 'robot.urdf'))))
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            arguments=[urdf])
    
    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        state_publisher_node,
        # joint_state_publisher_gui, 
        # node_robot_state_publisher,
        # node_joint_state_publisher,
        # spawn_entity,
        robot_launch,
    ])