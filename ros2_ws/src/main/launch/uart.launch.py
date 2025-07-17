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
    no_sim = LaunchConfiguration('no_sim', default=True)


    config_ekf= os.path.join(get_package_share_directory(package_name),'config','ekf_params.yaml')


    uart_node = Node(
        package = 'uart_drive',
        name = 'uart_node',
        executable = 'uart_drive',
        output='screen',
        condition=IfCondition(no_sim),
    )
    
    nav_params = os.path.join(get_package_share_directory(package_name),'config','nav2_params.yaml')

    return LaunchDescription([
        ExecuteProcess(cmd=['ros2','run','topic_tools','throttle','messages','/indication_planned','10','/indication']),
        ExecuteProcess(cmd=['ros2','run','topic_tools','throttle','messages','/cmd_vel_planned','10','/cmd_vel']),
        declare_use_sim_time,
        uart_node,
        
    ])
