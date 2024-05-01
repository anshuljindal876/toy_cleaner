#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
import cv2 as cv
from cv_bridge import CvBridge
import time
import numpy as np
#from std_msgs.msg import String
#from std_msgs.msg import Int32MultiArray
import image_geometry
from std_msgs.msg import Float64MultiArray
import math
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import TransformListener, Buffer
import tf_transformations

hasRed = False;
hasBlue = False;
hasGreen = False;
redObjectPixelCenter = (-1, -1)
blueObjectPixelCenter = (-1, -1)
greenObjectPixelCenter = (-1, -1)
item1Location = (0.0, 0.0, 0.0)
item2Location = (0.0, 0.0, 0.0)
item3Location = (0.0, 0.0, 0.0)
hasPoints = False
hasTF = False
TF = np.arange(1, 17).reshape(4,4);
#cv.namedWindow('Vision', cv.WINDOW_NORMAL)
resized = False
redLocation = (-1.0, -1.0, -1.0)
blueLocation = (-1.0, -1.0, -1.0)
greenLocation = (-1.0, -1.0, -1.0)

class CameraReading(Node):
    def __init__(self):
        super().__init__('camera_reading')
        self.subscription = self.create_subscription(
            Image,
            '/realsense/image_raw',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.subscription2 = self.create_subscription(
            PointCloud2,
            '/clustered_point_cloud',
            self.listener_callback2,
            10)
        self.subscription2  # prevent unused variable warning
        self.subscription3 = self.create_subscription(
            CameraInfo,
            '/realsense/camera_info',
            self.listener_callback3,
            10)
        self.subscription4 = self.create_subscription(
            Image,
            '/realsense/image_raw',
            self.listener_callback4,
            10)
        self.subscription4  # prevent unused variable warning
        self.publisher1 = self.create_publisher(Float64MultiArray,"/camera_reading/red_center", 10)
        self.publisher2 = self.create_publisher(Float64MultiArray,"/camera_reading/blue_center", 10)
        self.publisher3 = self.create_publisher(Float64MultiArray,"/camera_reading/green_center", 10)
        self.tf_buffer = Buffer()
        self.tf_transformer = TransformListener(self.tf_buffer, self)

    def listener_callback(self, data):
        print("In callback")
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        #cv.imshow('camera image', img)
        #cv.waitKey(0)
        isRed, redCenter, imgRed = check_red(img)
        isBlue, blueCenter, imgBlue = check_blue(img)
        isGreen, greenCenter, imgGreen = check_green(img)
        global hasRed, hasBlue, hasGreen, redObjectPixelCenter, blueObjectPixelCenter, greenObjectPixelCenter
        if isRed == True:
                print("Has red")
                hasRed = True
                redObjectPixelCenter = redCenter
        if isBlue == True:
                print("Has blue")
                hasBlue = True
                blueObjectPixelCenter = blueCenter
        if isGreen == True:
                print("has green")
                hasGreen = True
                greenObjectPixelCenter = greenCenter
        finalImg = np.maximum(np.maximum(imgRed, imgBlue), imgGreen)
        finalImg = cv.rotate(finalImg, cv.ROTATE_180)
        global resized
        if resized == False:
                height, width, _ = finalImg.shape
                cv.namedWindow('Vision', cv.WINDOW_NORMAL)
                cv.resizeWindow('Vision', width, height)
                resized = True
        font = cv.FONT_HERSHEY_COMPLEX 
        font_scale = 0.75
        thickness = 2
        if isRed == True:
                global redLocation
                font_color = (0, 0, 255)
                cv.putText(finalImg, "Red Center: (" + '{:.2f}'.format(redLocation[0]) + "m, " + '{:.2f}'.format(redLocation[1]) + "m, " + '{:.2f}'.format(redLocation[2]) + "m)", (15, 30), font, font_scale, font_color, thickness)
        if isBlue == True:
                global blueLocation
                font_color = (255, 80, 0)
                cv.putText(finalImg, "Blue Center: (" + '{:.2f}'.format(blueLocation[0]) + "m, " + '{:.2f}'.format(blueLocation[1]) + "m, " + '{:.2f}'.format(blueLocation[2]) + "m)", (15, 60), font, font_scale, font_color, thickness)
        if isGreen == True:
                global greenLocation
                font_color = (0, 255, 0)
                cv.putText(finalImg, "Green Center: (" + '{:.2f}'.format(greenLocation[0]) + "m, " + '{:.2f}'.format(greenLocation[1]) + "m, " + '{:.2f}'.format(greenLocation[2]) + "m)", (15, 90), font, font_scale, font_color, thickness)
        cv.imshow('Vision', finalImg)
        cv.waitKey(1)
        #cv.imshow('camera image', finalImg)
        #cv.waitKey(0)
        #colorData = String()
        #colorData.data = color
        #self.publisher.publish(colorData)
        #pixelData = Int32MultiArray()
        #pixelData.data = center
        #self.publisher2.publish(pixelData)
        #time.sleep(30)

    def listener_callback2(self, data):
        print("In callback2")
        global hasPoints, hasRed, hasBlue, hasGreen
        if hasRed == True or (hasBlue == True or hasGreen == True):
            global item1Location, item2Location, item3Location
            points = point_cloud2.read_points_numpy(data)
            points = points[:, :4]
            rows = points.shape[0]
            total1 = 0
            total2 = 0
            total3 = 0
            for i in range(rows):
                if points[i][3] == 80:
            	    item1Location = (item1Location[0] + points[i][0], item1Location[1] + points[i][1], item1Location[2] + points[i][2])
            	    total1 = total1 + 1
                elif points[i][3] == 160:
            	    item2Location = (item2Location[0] + points[i][0], item2Location[1] + points[i][1], item2Location[2] + points[i][2])
            	    total2 = total2 + 1
                elif points[i][3] == 240:
            	    item3Location = (item3Location[0] + points[i][0], item3Location[1] + points[i][1], item3Location[2] + points[i][2])
            	    total3 = total3 + 1
            hasPoints = True
            if total1 != 0:
                item1Location = (item1Location[0]/total1, item1Location[1]/total1, item1Location[2]/total1)
            if total2 != 0:
                item2Location = (item2Location[0]/total2, item2Location[1]/total2, item2Location[2]/total2)
            if total3 != 0:
                item3Location = (item3Location[0]/total3, item3Location[1]/total3, item3Location[2]/total3)
            print(item1Location)
            print(item2Location)
            print(item3Location)

    def listener_callback3(self, data):
        print("In callback3")
        global hasPoints, hasTF
        if hasPoints == True and hasTF == True:
            camera = image_geometry.PinholeCameraModel()
            camera.fromCameraInfo(data)
            global hasRed, hasBlue, hasGreen, item1Location, item2Location, item3Location, TF
            item1 = np.arange(1, 5).reshape(4,1)
            item1 = np.array([[item1Location[0]],
                               [item1Location[1]],
                               [item1Location[2]],
                               [1]])
            item2 = np.arange(1, 5).reshape(4,1)
            item2 = np.array([[item2Location[0]],
                               [item2Location[1]],
                               [item2Location[2]],
                               [1]])
            item3 = np.arange(1, 5).reshape(4,1)
            item3 = np.array([[item3Location[0]],
                               [item3Location[1]],
                               [item3Location[2]],
                               [1]])
            print(TF)
            item1 = np.dot(TF, item1)
            item2 = np.dot(TF, item2)
            item3 = np.dot(TF, item3)
            item1CameraLocation = (item1[0][0]/item1[2][0], item1[1][0]/item1[2][0], item1[2][0])
            item2CameraLocation = (item2[0][0]/item2[2][0], item2[1][0]/item2[2][0], item2[2][0])
            item3CameraLocation = (item3[0][0]/item3[2][0], item3[1][0]/item3[2][0], item3[2][0])
            print(item1Location)
            print(item1CameraLocation)
            #item1CameraLocation = (item1CameraLocation[0]/item1CameraLocation[2], item1CameraLocation[1]/item1CameraLocation[2], item1CameraLocation[2]/item1CameraLocation[2])
            #print(item1CameraLocation)
            item1PixelCenter = camera.project3dToPixel(item1CameraLocation)
            item2PixelCenter = camera.project3dToPixel(item2CameraLocation)
            item3PixelCenter = camera.project3dToPixel(item3CameraLocation)
            print("checking centers")
            item1PixelCenter = (item1PixelCenter[0], item1PixelCenter[1])
            item2PixelCenter = (item2PixelCenter[0], item2PixelCenter[1])
            item3PixelCenter = (item3PixelCenter[0], item3PixelCenter[1])
            print(item1PixelCenter)
            print(item2PixelCenter)
            print(item3PixelCenter)
            print(redObjectPixelCenter)
            print(blueObjectPixelCenter)
            print(greenObjectPixelCenter)
            if hasRed == True:           
                item1Test = math.sqrt((redObjectPixelCenter[0]-item1PixelCenter[0])**2 + (redObjectPixelCenter[1]-item1PixelCenter[1])**2)
                item2Test = math.sqrt((redObjectPixelCenter[0]-item2PixelCenter[0])**2 + (redObjectPixelCenter[1]-item2PixelCenter[1])**2)
                item3Test = math.sqrt((redObjectPixelCenter[0]-item3PixelCenter[0])**2 + (redObjectPixelCenter[1]-item3PixelCenter[1])**2)
                lowest = item1Test
                location = item1Location
                if lowest > item2Test:
                    lowest = item2Test
                    location = item2Location
                if lowest > item3Test:
                    lowest = item3Test
                    location = item3Location
                mat1 = Float64MultiArray()
                mat1.data = [location[0], location[1], location[2]]
                self.publisher1.publish(mat1)
                global redLocation
                redLocation = location
            else:
                redLocation = (-1.0, -1.0, -1.0)
            if hasBlue == True:           
                item1Test = math.sqrt((blueObjectPixelCenter[0]-item1PixelCenter[0])**2 + (blueObjectPixelCenter[1]-item1PixelCenter[1])**2)
                item2Test = math.sqrt((blueObjectPixelCenter[0]-item2PixelCenter[0])**2 + (blueObjectPixelCenter[1]-item2PixelCenter[1])**2)
                item3Test = math.sqrt((blueObjectPixelCenter[0]-item3PixelCenter[0])**2 + (blueObjectPixelCenter[1]-item3PixelCenter[1])**2)
                lowest = item1Test
                location = item1Location
                if lowest > item2Test:
                    lowest = item2Test
                    location = item2Location
                if lowest > item3Test:
                    lowest = item3Test
                    location = item3Location
                mat2 = Float64MultiArray()
                mat2.data = [location[0], location[1], location[2]]
                self.publisher2.publish(mat2)
                global blueLocation
                blueLocation = location
            else:
                blueLocation = (-1.0, -1.0, -1.0)
            if hasGreen == True:           
                item1Test = math.sqrt((greenObjectPixelCenter[0]-item1PixelCenter[0])**2 + (greenObjectPixelCenter[1]-item1PixelCenter[1])**2)
                item2Test = math.sqrt((greenObjectPixelCenter[0]-item2PixelCenter[0])**2 + (greenObjectPixelCenter[1]-item2PixelCenter[1])**2)
                item3Test = math.sqrt((greenObjectPixelCenter[0]-item3PixelCenter[0])**2 + (greenObjectPixelCenter[1]-item3PixelCenter[1])**2)
                lowest = item1Test
                location = item1Location
                if lowest > item2Test:
                    lowest = item2Test
                    location = item2Location
                if lowest > item3Test:
                    lowest = item3Test
                    location = item3Location
                mat3 = Float64MultiArray()
                mat3.data = [location[0], location[1], location[2]]
                self.publisher3.publish(mat3)
                global greenLocation
                greenLocation = location
            else:
                greenLocation = (-1.0, -1.0, -1.0)
        else:
            print("No points yets")

    def listener_callback4(self, data):
        print("In callback4")
        global hasTF, TF
        t = self.tf_buffer.lookup_transform('world', 'camera_link_optical', rclpy.time.Time())    
        tf_rot = tf_transformations.quaternion_matrix([t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w])
        tf_rot[:3, 3] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
        TF = inverse_matrix = np.linalg.inv(tf_rot)
        print(TF)
        hasTF = True
        #time.sleep(100)

def check_red(img):
        print("In red check")
        red = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        lower_limit=np.array([0, 100, 100]) # setting the blue lower limit 
        upper_limit=np.array([10, 255, 255])
        red = cv.inRange(red, lower_limit, upper_limit)
        red=cv.bitwise_and(img,img,mask=red)
        average = cv.mean(red)[0]
        print("average =",average)
        if average == 0:
                print("Not Red")
                return False, (-1, -1), (img * 0)
        else:
                print("Red")
                #cv.imshow('red', red)
                #cv.waitKey(0)
                check = cv.cvtColor(red, cv.COLOR_BGR2GRAY)
                ret,thresh = cv.threshold(check,50,255,0)
                contours,hierarchy = cv.findContours(thresh, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
                cnt = contours[0]
                (x,y),radius = cv.minEnclosingCircle(cnt)
                center = (int(x),int(y))
                radius = int(radius)
                circleImg = cv.circle(red,center,radius,(0,255,0),2)
                #cv.imshow('circle', circleImg)
                #cv.waitKey(0)
                print(center)
                return True, center, circleImg

def check_blue(img):
        print("In blue check")
        blue = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        lower_limit=np.array([98, 50, 50]) # setting the blue lower limit 
        upper_limit=np.array([139, 255, 255])
        blue = cv.inRange(blue, lower_limit, upper_limit)
        blue=cv.bitwise_and(img,img,mask=blue)
        average = cv.mean(blue)[0]
        print("average =",average)
        if average == 0:
                print("Not Blue")
                return False, (-1, -1), (img * 0)
        else:
                print("Blue")
                #cv.imshow('Blue', blue)
                #cv.waitKey(0)
                check = cv.cvtColor(blue, cv.COLOR_BGR2GRAY)
                ret,thresh = cv.threshold(check,10,255,0)
                contours,hierarchy = cv.findContours(thresh, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
                cnt = contours[0]
                (x,y),radius = cv.minEnclosingCircle(cnt)
                center = (int(x),int(y))
                radius = int(radius)
                circleImg = cv.circle(blue,center,radius,(0,255,0),2)
                #cv.imshow('circle', circleImg)
                #cv.waitKey(0)
                print(center)
                return True, center, circleImg

def check_green(img):
        print("In green check")
        green = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        lower_limit=np.array([40, 40, 40]) # setting the blue lower limit 
        upper_limit=np.array([70, 255, 255])
        green = cv.inRange(green, lower_limit, upper_limit)
        green=cv.bitwise_and(img,img,mask=green)
        average = cv.mean(green)[0]
        print("average =",average)
        if average == 0:
                print("Not Green")
                return False, (-1, -1), (img * 0)
        else:
                print("Green")
                #cv.imshow('Green', green)
                #cv.waitKey(0)
                check = cv.cvtColor(green, cv.COLOR_BGR2GRAY)
                ret,thresh = cv.threshold(check,20,255,0)
                contours,hierarchy = cv.findContours(thresh, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
                cnt = contours[0]
                (x,y),radius = cv.minEnclosingCircle(cnt)
                center = (int(x),int(y))
                radius = int(radius)
                circleImg = cv.circle(green,center,radius,(0,255,0),2)
                #cv.imshow('circle', circleImg)
                #cv.waitKey(0)
                print(center)
                return True, center, circleImg

def main(args=None):
        rclpy.init(args=args)
        camera_reading = CameraReading()
        rclpy.spin(camera_reading)
        camera_reading.destroy_node()
       
        rclpy.shutdown()

if __name__ == '__main__':
    main()
