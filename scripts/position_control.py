#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import time


class TestROS2Bridge(Node):
    def __init__(self):

        super().__init__("test_ros2bridge")

        # publish a JointState message to the /joint_command topic.
        self.publisher_ = self.create_publisher(JointState, "franka_joint_command", 10)
        
        self.joint_state = JointState()

        self.joint_state.name = [
            "panda_joint1",
            "panda_joint2",
            "panda_joint3",
            "panda_joint4",
            "panda_joint5",
            "panda_joint6",
            "panda_joint7",
            "panda_finger_joint1",
            "panda_finger_joint2",
        ]

        num_joints = len(self.joint_state.name)

        self.joint_state.position = np.array([0.0] * num_joints, dtype=np.float64).tolist()
        self.default_joints = [0.0, -1.16, -0.0, -2.3, -0.0, 1.6, 1.1, 0.4, 0.4]

        # limiting the movements to a smaller range
        self.max_joints = np.array(self.default_joints) + 0.5
        self.min_joints = np.array(self.default_joints) - 0.5

        # position control the robot
        self.time_start = time.time()

        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        self.joint_state.header.stamp = self.get_clock().now().to_msg()

        joint_position = (
            np.sin(time.time() - self.time_start) * (self.max_joints - self.min_joints) * 0.5 + self.default_joints
        )
        self.joint_state.position = joint_position.tolist()

        self.publisher_.publish(self.joint_state)

def main(args=None):
    rclpy.init(args=args)

    position_control = TestROS2Bridge()

    rclpy.spin(position_control)

    position_control.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
