import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'main'
    default_world = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'obstacle.world'
    )
    world_arg = DeclareLaunchArgument(
        'world',
        default_value = default_world,
        description = 'World to load'
    )
    gazebo = ExecuteProcess(
            cmd=['gazebo','--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', default_world], # Replace with your command and arguments
            shell=True, # Optional: Set to True if you need shell features like wildcards or pipes
            name='gazebo', # Optional: Name for the process
            output='screen' # Optional: 'screen' to display output in the terminal, 'log' to save to a log file, or 'none' to discard
        )
    return LaunchDescription([
        world_arg,
        gazebo,
    ])