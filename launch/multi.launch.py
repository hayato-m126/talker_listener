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

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import AnyLaunchDescriptionSource


def set_multi_launch(context: LaunchContext):
    param_file = Path(
        get_package_share_directory("talker_listener"), "config", "params.yaml"
    ).as_posix()
    target_dir = get_package_share_directory("talker_listener") + "/launch/"
    child_arg = {"param_file": param_file}

    multi_launch = []
    launch_names_str = context.launch_configurations["launch_files"]
    launch_names_list: list = eval(launch_names_str)
    for launch_name in launch_names_list:
        multi_launch.append(LogInfo(msg=launch_name))
        multi_launch.append(
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    [
                        target_dir,
                        launch_name,
                        ".launch.py",
                    ]  # 文字列結合は配列
                ),
                launch_arguments=child_arg.items(),
            )
        )
    return multi_launch


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "launch_files",
                description="launch files",
                default_value="['child1', 'child2']",
            ),
            OpaqueFunction(function=set_multi_launch),
        ]
    )
