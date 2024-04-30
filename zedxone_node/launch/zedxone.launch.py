# Copyright 2024 Stereolabs
#
# Licensed under the Apache License, Version 2.0 (the 'License');
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an 'AS IS' BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    SetEnvironmentVariable
)
from launch.substitutions import (
    LaunchConfiguration,
    TextSubstitution
)
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

# ROS distribution
distro = os.environ['ROS_DISTRO']

# ZED X One Configurations to be loaded by ZED X One Node
foxy=''
if(distro=='foxy'):
     foxy='_foxy'

default_config_path = os.path.join(
    get_package_share_directory('zedxone_node'),
    'config',
    'zedxone'+foxy+'.yaml'
)

def launch_setup(context, *args, **kwargs):

    camera_name = LaunchConfiguration('camera_name')
    camera_model = LaunchConfiguration('camera_model')
    camera_idx = LaunchConfiguration('idx')

    node_name = LaunchConfiguration('node_name')

    config_path = LaunchConfiguration('config_path')

    camera_name_val = camera_name.perform(context)
    camera_model_val = camera_model.perform(context)
    config_path_val = config_path.perform(context)
    node_name_val = node_name.perform(context)

    if (camera_name_val == ''):
        camera_name_val = 'zedxone'

    print("*** Launch arguments ***")
    print(" * Node name: %s" % node_name_val)
    print(" * Camera name: %s" % camera_name_val)
    print(" * Camera model: %s" % camera_model_val)
    print(" * Config file: %s" % config_path_val)

    # ZED X One component
    zedxone_component = ComposableNode(
        package='zedxone_components',
        namespace=camera_name_val,
        plugin='stereolabs::ZedXOneCamera',
        name=node_name_val,
        parameters=[
            # YAML files
            config_path_val,  # Common parameters
            # Overriding
            {
                'camera.camera_name': camera_name_val,
                'camera.camera_model': camera_model_val,
                'camera.idx': camera_idx
            }
        ],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    # ROS 2 Component Container
    container = ComposableNodeContainer(
            name='zedxone_container',
            namespace=camera_name_val,
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                zedxone_component
            ],
            output='screen'
    )

    return [
        container
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            SetEnvironmentVariable(name='RCUTILS_COLORIZED_OUTPUT', value='1'),
            DeclareLaunchArgument(
                'camera_name',
                default_value=TextSubstitution(text='zedxone'),
                description='The name of the camera. It can be different from the camera model and it will be used as node `namespace`.'),
            DeclareLaunchArgument(
                'camera_model',
                description='[REQUIRED] The model of the camera. Using a wrong camera model can disable camera features.',
                choices=['GS', '4K']),
            DeclareLaunchArgument(
                'node_name',
                default_value='zedxone_node',
                description='The name of the zed_wrapper node. All the topic will have the same prefix: `/<camera_name>/<node_name>/`'),
            DeclareLaunchArgument(
                'config_path',
                default_value=TextSubstitution(text=default_config_path),
                description='Path to the YAML configuration file for the camera.'),
            DeclareLaunchArgument(
                'idx',
                default_value='0',
                description='The index number of the camera to be opened.'),
            OpaqueFunction(function=launch_setup)
        ]
    )
