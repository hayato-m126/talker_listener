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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction


def set_multi_launch(context, *args, **kwargs):
    param_file = Path(
        get_package_share_directory("talker_listener"), "config", "params.yaml"
    ).as_posix()
    target_dir = get_package_share_directory("talker_listener") + "/launch/"
    child_arg = {"param_file": param_file}

    multi_launch = []
    launch_files = context.launch_configurations["launch_files"]
    multi_launch.append(LogInfo(msg=f"{launch_files=}"))
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
    for launch_name in launch_files:
        multi_launch.append(LogInfo(msg=launch_name))
    return multi_launch


"""
❯ ros2 launch talker_listener multi.launch.py launch_files:=[a, b, c]
[INFO] [launch]: All log files can be found below /home/hyt/.ros/log/2024-06-14-13-49-46-892771-dpc2405001-1635528
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [launch.user]: launch_files='[a, b, c]'
[INFO] [launch.user]: [
[INFO] [launch.user]: a
[INFO] [launch.user]: ,
[INFO] [launch.user]:
[INFO] [launch.user]: b
[INFO] [launch.user]: ,
[INFO] [launch.user]:
[INFO] [launch.user]: c
[INFO] [launch.user]: ]
"""


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "launch_files",
                description="launch files",
                default_value=["chatter1", "chatter2"],
            ),
            OpaqueFunction(function=set_multi_launch),
        ]
    )
