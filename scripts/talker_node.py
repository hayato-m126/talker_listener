#!/usr/bin/env python3

# Copyright 2017 Open Source Robotics Foundation, Inc.
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

import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class Talker(Node):
    """
    A node with a single publisher.

    This class creates a node which regularly publishes messages on a topic. Creating a node by
    inheriting from Node is recommended because it allows it to be imported and used by
    other scripts.
    """

    def __init__(self):
        # Calls Node.__init__('talker')
        super().__init__("talker")
        self.declare_parameter("prefix_msg", "default pre")
        self._prefix_msg = (
            self.get_parameter("prefix_msg").get_parameter_value().string_value
        )
        self.declare_parameter("postfix_msg", "default post")
        self._postfix_msg = (
            self.get_parameter("postfix_msg").get_parameter_value().string_value
        )
        self.declare_parameter("array_value", ["default_v0", "default_v1"])
        self._array_value = (
            self.get_parameter("array_value").get_parameter_value().string_array_value
        )

        for v in self._array_value:
            self.get_logger().info(f"{v=}")

        self.i = 0
        self.pub = self.create_publisher(String, "chatter", 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = f"{self._prefix_msg}-{self.i}-{self._postfix_msg}"
        self.i += 1
        # self.get_logger().info('Publishing: "{0}"'.format(msg.data))
        self.pub.publish(msg)


def main(args=None):
    """
    Run a Talker node standalone.

    This function is called directly when using an entrypoint. Entrypoints are configured in
    setup.py. This along with the script installation in setup.cfg allows a talker node to be run
    with the command `ros2 run examples_rclpy_executors talker`.

    :param args: Arguments passed in from the command line.
    """
    # Run standalone
    rclpy.init(args=args)
    try:
        talker = Talker()
        rclpy.spin(talker)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)


if __name__ == "__main__":
    # Runs a talker node when this script is run directly (not through an entrypoint)
    main()
