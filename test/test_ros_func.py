#!/usr/bin/env python3

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

# https://answers.ros.org/question/396345/ros2-launch-file-how-to-convert-launchargument-to-string/

from pathlib import Path

from launch import LaunchContext, LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

robot_description_path = Path("destination.urdf")


def render_xacro(context: LaunchContext, *args, **kwargs):
    support_package_str = context.perform_substitution(args[0])
    # render xacro... just dumping the support_package value in there for example.
    robot_description_config = support_package_str
    robot_description_path.write_text(robot_description_config)
    print(f"wrote robot description to {robot_description_path}")


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("support_package", default_value="hello"),
            OpaqueFunction(
                function=render_xacro, args=[LaunchConfiguration("support_package")]
            ),
            ExecuteProcess(cmd=["cat", str(robot_description_path)], output="screen"),
        ]
    )


if __name__ == "__main__":
    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())
    ls.run()
