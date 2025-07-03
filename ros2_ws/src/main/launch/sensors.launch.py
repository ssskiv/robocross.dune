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

    rs1 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory("realsense2_camera")+'/launch', '/rs_launch.py']),
            launch_arguments={
                # 'enable_rgbd':'True',
                'enable_sync':'True',
                # 'align_depth.enable':'True',
                'enable_color':'false',
                'enable_depth':'True',
                'pointcloud.enable':'True',
                'camera_namespace':'rs1',
                # 'base_frame_id':'cam1_link',
                # 'usb_port_id':'2-2',
                'serial_no': '_938422071383',
                'camera_name':'rs1',
                'depth_module.depth_profile': '640x360x6',
                'depth_module.infra_profile': '424x240x6',
                'rgb_camera.color_profile': '320x180x6',
                'rgb_camera.color_format': 'Y8',
                'clip_distance':'3.0',
                'enable_infra1':'true',
                # 'json_file_path':str(get_package_share_directory('main')+'/config/settings.json'),
            }.items(),
        )
    rs2 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory("realsense2_camera")+'/launch', '/rs_launch.py']),
            launch_arguments={
                # 'enable_rgbd':'True',
                'enable_sync':'True',
                # 'align_depth.enable':'True',
                'enable_color':'false',
                'enable_depth':'True',
                'pointcloud.enable':'True',
                'camera_namespace':'rs2',
                # 'base_frame_id':'cam2_link',
                # 'usb_port_id':'2-1',
                'serial_no':'_944122071765',
                'camera_name':'rs2',
                'depth_module.depth_profile': '640x360x6',
                'depth_module.infra_profile': '424x240x6',
                'rgb_camera.color_profile': '320x180x6',
                'rgb_camera.color_format': 'Y8',
                'clip_distance':'3.0',
                # 'json_file_path':str(get_package_share_directory('main')+'/config/settings.json'),
            }.items(),
        )

    #lidar = IncludeLaunchDescription(
    #    PythonLaunchDescriptionSource([get_package_share_directory("rplidar_ros")+'/launch', '/rplidar_a3_launch.py']),
    #    launch_arguments={
    #        'frame_id':'lidar_link',
    #        'serial_baudrate':'256000',
    #    }.items(),
    #)

    return LaunchDescription([
        rs1,
        rs2,
        #lidar,
    ])
