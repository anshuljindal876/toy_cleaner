#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
import time
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray

class CoordinateFinder(Node):
    def __init__(self):
        super().__init__('coordinate_finder')
        self.subscription = self.create_subscription(
            CameraInfo,
            '/realsense/depth/camera_info',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        #self.publisher = self.create_publisher(String,"/camera_reading/color", 10)
        #self.publisher2 = self.create_publisher(Int32MultiArray,"/camera_reading/pixel_coordinates", 10)

    def listener_callback(self, data):
        print("In callback")
        
        time.sleep(30)

def main(args=None):
        rclpy.init(args=args)
        coordinate_finder = CoordinateFinder()
        rclpy.spin(coordinate_finder)
        coordinate_finder.destroy_node()
       
        rclpy.shutdown()

if __name__ == '__main__':
    main()
