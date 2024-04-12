#!/usr/bin/env python3

# Copyright (c) 2020-2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, Vector3
import numpy as np
import time


class TestROS2Bridge(Node):
    def __init__(self):

        super().__init__("test_ros2bridge")

        # Create the publisher. This publisher will publish a JointState message to the /joint_command topic.
        # self.joint_state_publisher = self.create_publisher(JointState, "jackal_joint_command", 10)
        self.joint_twist_publisher = self.create_publisher(Twist, "jackal_cmd_vel", 10)

        # self.joint_state = JointState()
        self.twist = Twist()

        self.twist.angular = Vector3()
        self.twist.linear = Vector3()

        # set linear and angular vectors to zero
        self.twist.linear.x, self.twist.linear.y, self.twist.linear.z = 10.0, 0.0, 0.0
        self.twist.angular.x, self.twist.angular.y, self.twist.angular.z = 0.0, 0.0, 0.0
     
        self.joint_twist_publisher.publish(self.twist)

        # send robot a linear speed of 10 m/s for 5 seconds using timer callback function
        self.time_start = time.time()
        timer_period = 0.05
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        self.twist.linear.x = -1 * self.twist.linear.x
        self.joint_twist_publisher.publish(self.twist)
        time.sleep(5)

def main(args=None):
    rclpy.init(args=args)

    ros2_publisher = TestROS2Bridge()

    rclpy.spin(ros2_publisher)

    # Destroy the node explicitly
    ros2_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
