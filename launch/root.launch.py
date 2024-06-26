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

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_arg = []
    launch_arg.append(
        DeclareLaunchArgument("launch_file_name", description="launch file name")
    )

    param_file = Path(
        get_package_share_directory("talker_listener"), "config", "params.yaml"
    ).as_posix()

    target_dir = get_package_share_directory("talker_listener") + "/launch/"

    child_arg = {"param_file": param_file}

    child_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.AnyLaunchDescriptionSource(
            [
                target_dir,
                LaunchConfiguration("launch_file_name"),
                ".launch.py",
            ]  # 文字列結合は配列
        ),
        launch_arguments=child_arg.items(),
    )

    return LaunchDescription(
        [*launch_arg, child_launch],
    )
