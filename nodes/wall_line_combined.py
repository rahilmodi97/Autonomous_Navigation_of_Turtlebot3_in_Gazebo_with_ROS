#!/usr/bin/env python

import rospy
import time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from decimal import *
import numpy as np
import cv2

right_min = 0.3
left_min = 0.3
front_min = 10
d = 0

def callback_wall(data):
    global right_min, left_min, front_min
    left = data.ranges[90]
    right = data.ranges[270]
    front = data.ranges[0]
    right_cone = []
    left_cone = []
    front_cone = []
    if right == float("inf"): right_min = 0.3
    else: 
        for i in range(300,341):
            right_cone.append(data.ranges[i])

    if left == float("inf"): left_min = 0.3
    else: 
        for o in range(20,61):
            left_cone.append(data.ranges[o])

    for p in range(len(data.ranges)):
        if p<20 or p>340:
            front_cone.append(data.ranges[p])
        right_min = min(right_cone)
        left_min = min(left_cone)
        front_min = min (front_cone)

def camera_callback(data):
    global d

    # We select bgr8 because its the OpneCV encoding by default
    cv_image = bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")

        
    # We get image dimensions and crop the parts of the image we dont need
    height, width, channels = cv_image.shape
    crop_img = cv_image[(height/2)+230:(height/2)+240][1:width]
    
    # Convert from RGB to HSV
    hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
    
    # Define the Yellow Colour in HSV

    """
    To know which color to track in HSV use ColorZilla to get the color registered by the camera in BGR and convert to HSV. 
    """

    # Threshold the HSV image to get only yellow colors
    lower_yellow = np.array([20,100,100])
    upper_yellow = np.array([50,255,255])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    
    # Calculate centroid of the blob of binary image using ImageMoments
    m = cv2.moments(mask, False)
    d = 0
    try:
        cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
        d = 2
    except ZeroDivisionError:
        cx, cy = height/2, width/2
        d = 1

    
    # Draw the centroid in the resultut image
    # cv2.circle(img, center, radius, color[, thickness[, lineType[, shift]]]) 
    cv2.circle(mask,(int(cx), int(cy)), 10,(0,0,255),-1)

    cv2.imshow("Original", cv_image)
    cv2.imshow("MASK", mask)
    cv2.waitKey(1)
    
    """
    Enter controller here.
    """
    twist_object = Twist()
    twist_object.linear.x = 0.03
    diff = width/2 - cx
    twist_object.angular.z = diff/1200
    print(d)
    vel_pub.publish(twist_object)

    rospy.loginfo("ANGULAR VALUE SENT===>"+str(twist_object.angular.z))
    # Make it start turning
    # self.moveTurtlebot3_object.move_robot(twist_object)

if __name__ == '__main__':
    global right_min, left_min, front_min
    rospy.init_node('gazebo_scan_sub')
    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
    scan_sub = rospy.Subscriber('/scan', LaserScan, callback_wall)
    cam_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,camera_callback)
    rate = rospy.Rate(10)
    vel_msg = Twist()
    vel_msg.linear.x = 0.2
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0

    while not rospy.is_shutdown():

        error = (left_min - right_min)/2
        
        print("error: ", error)
        
        if front_min <= 0.4: 
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0.05
        if front_min > 0.2: vel_msg.linear.x = 0.12 

        if error!= float("inf") and error != float("-inf"):
            vel_msg.angular.z = error * 3
        if d !=2:
            vel_pub.publish(vel_msg)


        rate.sleep()
    rospy.spin()