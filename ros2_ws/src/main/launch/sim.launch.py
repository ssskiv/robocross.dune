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

    default_world = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'obstacle.world'
    )
    world = LaunchConfiguration('world')
    world_arg = DeclareLaunchArgument(
        'world',
        default_value = default_world,
        description = 'World to load'
    )
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
            launch_arguments = {'gz_args' : ['-r  -v4 ', world], 'on_exit_shutdown' : 'true'}.items()
        )

    bridge_params = os.path.join(get_package_share_directory(package_name),'config','gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )

    spawn_entity = Node(package = 'ros_gz_sim', executable = 'create',
    arguments = ['-topic', 'robot_description',
    '-name', 'my_bot', '-x', '1.0', '-y', '1.0', '-z', '1.0'],
    output = 'screen')
    

    return LaunchDescription([
        world_arg,
        gazebo,
        rsp,
        ros_gz_bridge,
        spawn_entity,
    ])