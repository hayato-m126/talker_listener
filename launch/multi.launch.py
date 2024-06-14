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
from typing import Any

from ament_index_python.packages import get_package_share_directory

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def set_multi_launch(context: LaunchContext, *args: Any, **kwargs: Any):
    param_file = Path(
        get_package_share_directory("talker_listener"), "config", "params.yaml"
    ).as_posix()
    target_dir = get_package_share_directory("talker_listener") + "/launch/"
    child_arg = {"param_file": param_file}

    multi_launch = []
    for arg in args:
        launch_name = context.perform_substitution(arg)
        multi_launch.append(LogInfo(msg=launch_name))

    # for launch_name in launch_files:
    #     multi_launch.append(LogInfo(msg=launch_name))
    #     multi_launch.append(
    #         launch.actions.IncludeLaunchDescription(
    #             launch.launch_description_sources.AnyLaunchDescriptionSource(
    #                 [
    #                     target_dir,
    #                     launch_name,
    #                     ".launch.py",
    #                 ]  # 文字列結合は配列
    #             ),
    #             launch_arguments=child_arg.items(),
    #         )
    #     )
    return multi_launch


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "launch_files",
                description="launch files",
                default_value=["chatter1", "chatter2"],
            ),
            OpaqueFunction(
                function=set_multi_launch,
                args=[
                    LaunchConfiguration("launch_files"),
                ],
            ),
        ]
    )
