# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes, SetParameter
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get the launch directory
    package_name='main'    
    bringup_dir = get_package_share_directory(package_name)

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    container_name_full = (namespace, '/', container_name)
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    map_yaml_file = LaunchConfiguration('map_yaml_file')
    
    map_yaml_file = LaunchConfiguration(
        'map_yaml_file',
        default=PathJoinSubstitution(
            [
                FindPackageShare(package_name),
                'maps',
                'map.yaml'
            ]
        )
    )
    # topology_map_yaml_file = LaunchConfiguration(
    #     'topology_map_yaml_file',
    #     default=PathJoinSubstitution(
    #         [
    #             FindPackageShare('rdsim_nav2'),
    #             'map',
    #             'topology.yaml'
    #         ]
    #     )
    # )

    params_file = LaunchConfiguration(
        'params_file',
        default=PathJoinSubstitution(
            [
                FindPackageShare(package_name),
                'config',
                'nav2_params.yaml'
            ]
        )
    )
    
    nav2_launch_file_dir = PathJoinSubstitution(
        [
            FindPackageShare(package_name),
            'launch',
        ]
    )
    
    

    nav2_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_yaml_file,
                # 'topology_map': topology_map_yaml_file,
                'use_sim_time': 'False',
                'params_file': params_file,
                # 'default_bt_xml_filename': default_bt_xml_filename,
                'autostart': 'True',
                'use_composition': 'False',
                'use_respawn': 'True',
                'use_localization':'False',
            }.items(),
        )
    return LaunchDescription([
        nav2_launch
    ])