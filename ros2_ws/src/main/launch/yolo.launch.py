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

    rs1 = Node(
        package='core',
        executable='yolo_detect',
        output='screen',
        parameters=[{
            'input_topic':'/rs1/rs1/infra1/image_rect_raw',
            'base_frame':'rs1_link',
            'outtopic_img':'/yolo_detect_image_rs1',
            'outtopic_scan':'/yolo_scan1',
            }]
    )
        
    rs2 = Node(
        package='core',
        executable='yolo_detect',
        output='screen',
        parameters=[{
            'input_topic':'/rs2/rs2/infra1/image_rect_raw',
            'base_frame':'rs2_link',
            'outtopic_img':'/yolo_detect_image_rs2',
            'outtopic_scan':'/yolo_scan2',
            }]
    )


    return LaunchDescription([
        rs1,
        #rs2,
    ])
