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

from time import sleep

import rclpy

#from std_msgs.msg import String
from sensor_msgs.msg import JointState
# We do not recommend this style as ROS 2 provides timers for this purpose,
# and it is recommended that all nodes call a variation of spin.
# This example is only included for completeness because it is similar to examples in ROS 1.
# For periodic publication please see the other examples using timers.


def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('minimal_publisher')

    publisher_right = node.create_publisher(JointState, '/right_driver/kinova_arm_joint', 10)
    publisher_left = node.create_publisher(JointState, '/left_driver/kinova_arm_joint', 10)
    publisher_finger_right = node.create_publisher(JointState, '/right_driver/kinova_finger_joint', 10)
    publisher_finger_left = node.create_publisher(JointState, '/left_driver/kinova_finger_joint', 10)

    msg = JointState()

    i = 0
    sleep(1.0)
    while rclpy.ok():
        msg.velocity = [ float(0) for i in range(7)]
        msg.velocity[0] = 3
        # msg.position = [
        #     274.99,
        #     262.30,
        #     257.18,
        #     152.32,
        #     190.78,
        #     162.09,
        #     317.55
        # ]
        #msg.position[0] = 100
        msg.position = [ float(6400) for i in range(7)]
        publisher_finger_right.publish(msg)
        publisher_finger_left.publish(msg)
        i += 1
        #node.get_logger().info('Publishing: "%s"' % msg.velocity)
        node.get_logger().info('Publishing: "%s"' % msg.position)
        #publisher_right.publish(msg)
        #publisher_left.publish(msg)
        sleep(0.5)  # seconds

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
