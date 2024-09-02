# Copyright 2016 Open Source Robotics Foundation, Inc.
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

# native ROS libraries
import sys

# ROS specific libraries
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

# message types
from std_msgs.msg import String, Empty
from interface_wsg.msg import GripperState, MoveFingers

# wsg50 TCP/IP interface
from .submodules.wsg50 import wsg50


class WsgCommander(Node):

    def __init__(self):
        super().__init__('wsg_command_interface')

        # subscribers
        self.sub_grasp = self.create_subscription(
            MoveFingers, '/wsg_50/grasp', self.grasp_callback, 1)
        self.sub_preposition = self.create_subscription(
            MoveFingers, '/wsg_50/preposition', self.preposition_callback, 1)
        self.sub_release = self.create_subscription(
            MoveFingers, '/wsg_50/release', self.release_callback, 1)
        self.sub_release = self.create_subscription(
            Empty, '/wsg_50/disconnect', self.disconnect_callback, 1)
        self.sub_release = self.create_subscription(
            Empty, '/wsg_50/connect', self.connect_callback, 1)

        # publishers
        self.publisher_ = self.create_publisher(
            GripperState, '/wsg_50/gripper_state', 1)

        # every time period run callback function to publish the gripper state
        timer_period = 10  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        # start wsg50 instance
        self.wsg_instance = wsg50()

    def grasp_callback(self, msg):
        self.get_logger().info('%s, %s' % (msg.width, msg.speed))
        self.wsg_instance.grasp_part(msg.width, msg.speed)

    def preposition_callback(self, msg):
        self.get_logger().info('%s, %s' % (msg.width, msg.speed))
        self.wsg_instance.preposition_gripper(msg.width, msg.speed)

    def release_callback(self, msg):
        self.get_logger().info('%s, %s' % (msg.width, msg.speed))
        self.wsg_instance.release_part(msg.width, msg.speed)

    def timer_callback(self):
        #state_flag_list = self.wsg_instance.gripper_state()
        state_flag_list = ['SF_REFERENCED', 'SF_MOVING', 'SF_BLOCKED_MINUS']
        if state_flag_list != None:
            msg = GripperState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.state_flags = state_flag_list # e.g. ['SF_REFERENCED', 'SF_MOVING', 'SF_BLOCKED_MINUS']
            self.publisher_.publish(msg)
            #self.get_logger().info('Published the following message: {0}'.format(msg))

    def connect_callback(self):
        self.get_logger().info('Establishing connection to gripper')
        self.wsg_instance.start_connection()

    def disconnect_callback(self):
        self.get_logger().info('Closing connection to gripper')
        self.wsg_instance.end_connection()


def main(args=None):
    rclpy.init(args=args)

    node = WsgCommander()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.end_connection()
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
