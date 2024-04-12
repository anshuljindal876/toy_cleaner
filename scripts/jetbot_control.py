#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3
import numpy as np
import math
from tf_transformations import euler_from_quaternion
import time

class MoveToCoordinate(Node):
    def __init__(self):
        super().__init__("move_to_coordinate")
        self.velocity_publisher = self.create_publisher(Twist, "jetbot_cmd_vel", 10)
        self.odometry_subscriber = self.create_subscription(Odometry, "odom", self.odom_callback, 10)

        self.twist = Twist()
        self.twist.angular = Vector3()
        self.twist.linear = Vector3()

        0.22098

        self.waypoints = [(2.0, 2.0), (-2.0, 2.0), (2.0, -2.0), (-2.0, -2.0)]
        self.num_waypoints = len(self.waypoints)
        self.target_index = 0

        self.target_x, self.target_y = None, None

        self.linear_speed = 5.0  # m/s
        self.angular_speed = 0.5  # m/s

        self.distance_tolerance = 0.1  # meters
        self.orientation_tolerance = 0.1  # radians

        # Initialize current position and orientation
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0  # In radians

        # Frequency of execution in Hz
        self.loop_rate = 10
        self.timer = self.create_timer(1.0 / self.loop_rate, self.move_to_target)

    def odom_callback(self, msg):
        # Update position
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Update orientation
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, self.current_yaw) = euler_from_quaternion(orientation_list)

    def move_to_target(self):
        if self.target_index < self.num_waypoints:
            # Print logs
            self.get_logger().info(f"Current position: ({self.current_x:.2f}, {self.current_y:.2f}), Target position: ({self.target_x}, {self.target_y}), Linear speed: {self.twist.linear.x}, Angular speed: {self.twist.angular.z}")

            self.target_x, self.target_y = self.waypoints[self.target_index]

            distance = math.sqrt((self.target_x - self.current_x) ** 2 + (self.target_y - self.current_y) ** 2)
            angle_to_target = math.atan2(self.target_y - self.current_y, self.target_x - self.current_x)
            
            if distance > self.distance_tolerance:  # If not close enough to the target
                angle_difference = angle_to_target - self.current_yaw
                # Normalize the angle difference
                angle_difference = (angle_difference + np.pi) % (2 * np.pi) - np.pi

                if abs(angle_difference) > self.orientation_tolerance:  # If not pointing to the target
                    self.twist.linear.x = 0.0
                    self.twist.angular.z = 10*self.angular_speed if angle_difference > 0 else -10*self.angular_speed
                else:  # Move forward
                    self.twist.linear.x = distance
                    self.twist.angular.z = 0.0

            # if distance > self.distance_tolerance:  # If not close enough to the target
            #     angle_difference = angle_to_target - self.current_yaw
            #     # Normalize the angle difference
            #     angle_difference = (angle_difference + np.pi) % (2 * np.pi) - np.pi

            #     self.twist.linear.x = distance
            #     self.twist.angular.z = angle_difference*self.angular_speed

            else:
                self.twist.linear.x = 0.0
                self.twist.linear.y = 0.0
                self.twist.linear.z = 0.0

                self.twist.angular.x = 0.0
                self.twist.angular.y = 0.0
                self.twist.angular.z = 0.0

                self.target_index += 1
                self.get_logger().info(f"############### Reached waypoint #{self.target_index} ###############")

            self.velocity_publisher.publish(self.twist)

        else:
            self.target_index = 0
            self.get_logger().info(f"#############################################")
            self.get_logger().info(f"############### STARTING OVER ###############")
            self.get_logger().info(f"#############################################")

def main(args=None):
    rclpy.init(args=args)
    move_to_coordinate_node = MoveToCoordinate()
    rclpy.spin(move_to_coordinate_node)

    move_to_coordinate_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
