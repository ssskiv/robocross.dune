import os

from ament_index_python.packages import get_package_share_directory

import launch
import pathlib
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

from launch_ros.actions import Node

from webots_ros2_driver.webots_launcher import WebotsLauncher, Ros2SupervisorLauncher
from webots_ros2_driver.utils import controller_url_prefix
from webots_ros2_driver.webots_controller import WebotsController

def generate_launch_description():
    package_name = 'simulation'
    package_dir = get_package_share_directory(package_name)
    # sim_pkg_dir = get_package_share_directory('simulation')
    
    robot_description = os.path.join(package_dir, pathlib.Path(os.path.join(package_dir, 'resource', 'robot.urdf')))
    
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
    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'world.wbt'), ros2_supervisor=True, stream=True
    )
    
    # ros2_supervisor = Ros2SupervisorLauncher()

    # my_robot_driver = Node(
    #     package='webots_ros2_driver',
    #     executable='driver',
    #     output='screen',
    #     additional_env={'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'my_robot'},
    #     parameters=[
    #         {'robot_description': robot_description},
    #     ]
    # )
    my_robot_driver = WebotsController(
        robot_name='my_robot',
        parameters=[
            {'robot_description': robot_description}
        ],
        respawn=True
    )
    
    return LaunchDescription([
        world_arg,
        webots,
        webots._supervisor,
        # ros2_supervisor,
        my_robot_driver,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
    ])