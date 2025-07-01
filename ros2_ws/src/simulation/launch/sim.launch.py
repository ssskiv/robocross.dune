import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

from launch_ros.actions import Node

def generate_launch_description():

    main_name = 'main'
    package_name = 'simulation'

    default_world = os.path.join(
        get_package_share_directory(main_name),
        'worlds',
        'obstacle.world'
    )
    world = LaunchConfiguration('world')
    world_arg = DeclareLaunchArgument(
        'world',
        default_value = default_world,
        description = 'World to load'
    )
    
    webots = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'webots.launch.py')]),
        )
    
    rviz2 = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', [os.path.join(get_package_share_directory(main_name), 'config', 'urdf_core.rviz')]],
        parameters = [{'use_sim_time': False}]
    )
    
    rsp_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(main_name),'launch','rsp.launch.py'
                )]),
    )

    return LaunchDescription([
        world_arg,
        webots,
        rviz2,
        rsp_launch,
    ])