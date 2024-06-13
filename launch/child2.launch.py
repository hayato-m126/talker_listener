# Copyright (c) 2022 TIER IV.inc
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


from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_arg = []
    launch_arg.append(
        DeclareLaunchArgument("param_file", description="param file path")
    )
    override_param = {"prefix_msg": "pre_dict"}
    return LaunchDescription(
        [
            *launch_arg,
            Node(
                package="talker_listener",
                namespace="talker_listener",
                executable="talker_node.py",
                output="screen",
                name="talker2",
                parameters=[LaunchConfiguration("param_file"), override_param],
            ),
            Node(
                package="talker_listener",
                namespace="talker_listener",
                executable="listener_node.py",
                output="screen",
                name="listener2",
                # parameters=[],
            ),
        ]
    )
