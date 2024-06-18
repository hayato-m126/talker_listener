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


class Listener(Node):
    """
    A node with a single subscriber.

    This class creates a node which prints messages it receives on a topic. Creating a node by
    inheriting from Node is recommended because it allows it to be imported and used by
    other scripts.
    """

    def __init__(self):
        # Calls Node.__init__('listener')
        super().__init__("listener")
        self._prefix_msg = (
            self.get_parameter("prefix_msg").get_parameter_value().string_value
        )
        self.declare_parameter("postfix_msg", "default post")
        self._postfix_msg = (
            self.get_parameter("postfix_msg").get_parameter_value().string_value
        )
        self.sub = self.create_subscription(
            String, "chatter", self.chatter_callback, 10
        )

    def chatter_callback(self, msg):
        self.get_logger().info(f"{self._prefix_msg}-{msg.data}-{self._postfix_msg}")


def main(args=None):
    """
    Run a Listener node standalone.

    This function is called directly when using an entrypoint. Entrypoints are configured in
    setup.py. This along with the script installation in setup.cfg allows a listener node to be run
    with the command `ros2 run examples_rclpy_executors listener`.

    :param args: Arguments passed in from the command line.
    """
    rclpy.init(args=args)
    try:
        listener = Listener()
        rclpy.spin(listener)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)


if __name__ == "__main__":
    # Runs a listener node when this script is run directly (not through an entrypoint)
    main()
