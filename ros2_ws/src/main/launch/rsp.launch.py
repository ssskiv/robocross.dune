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


def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')
    package_name='main'
    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory(package_name))
    xacro_file = os.path.join(pkg_path,'urdf','rover.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config.toxml(),  'use_sim_time': use_sim_time}


    # joint_state_publisher_gui = Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     name='joint_state_publisher_gui',
    #     arguments=[],
    #     output=['screen']
    # )
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

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-entity', 'rover', '-topic', 'robot_description', '-z', '4.5'],
                        output='screen')



    delete_model_command = [
        'ros2', 'service', 'call',
        '/delete_entity',
        'gazebo_msgs/srv/DeleteEntity',
        '"{name: \'' + 'rover' + '\'}"' # JSON string for the request
    ]

    delete_model_action = ExecuteProcess(
        cmd=delete_model_command,
        name='delete_model_service_call',
        output='screen',
        shell=True, # Required because we're passing a complex command string
        # Do not wait for this process to finish, as the launch system might be shutting down
        # This is more of a "fire and forget" if you don't need its success/failure for further launch logic
        # You can add OnExecutionComplete if you want to handle the service call's completion
    )

    # 3. Register an event handler to trigger despawn when the robot node exits
    on_robot_node_exit_handler = RegisterEventHandler(
        OnShutdown(
            # target_action=node_robot_state_publisher,
            on_shutdown=[
                # ExecuteProcess(
                #     cmd=['echo', f'Node {node_robot_state_publisher.name} exited. Attempting to despawn rover in Gazebo Classic...'],
                #     name='despawn_log_message',
                #     output='screen'
                # ),
                # delete_model_action
                os.system("ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity '{name: 'rover'}'"),
            ]
        )
    )

    robot_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','robot.launch.py'
                )]), 
                # condition=IfCondition( is_navigation ), 
                launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    
    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use sim time if true'),
        # joint_state_publisher_gui, 
        node_robot_state_publisher,
        node_joint_state_publisher,
        spawn_entity,
        robot_launch,
        on_robot_node_exit_handler,
    ])