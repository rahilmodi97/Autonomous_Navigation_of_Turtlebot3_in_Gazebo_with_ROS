#!/usr/bin/env python

import rospy
import time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from darknet_ros_msgs.msg import BoundingBoxes
from people_msgs.msg import PositionMeasurementArray
from nav_msgs.msg import Odometry
from decimal import *
import numpy as np
import cv2
import tf
import math
from geometry_msgs.msg import PoseArray
global d
right_min = 0.3
left_min = 0.3
front_min = 10
d = 0
det_flag = 0
bot_x = 0; bot_y = 0; bot_yaw = 0


class LineFollower(object):
    def __init__(self):
        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.camera_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.callback_wall)
        self.det_sub = rospy.Subscriber("/darknet_ros/bounding_boxes",BoundingBoxes, self.det_callback)
        self.leg_sub = rospy.Subscriber('/to_pose_array/leg_detector', PoseArray, self.leg_callback)
        #self.leg_sub = rospy.Subscriber('/people_tracker_measurements', PositionMeasurementArray, self.leg_callback)
        self.bot_sub = rospy.Subscriber('/odom', Odometry, self.bot_callback)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        # self.rosnet_sub = rospy.Subscriber("/darknet_ros/bounding_boxes",BoundingBoxes, self.callback)
        # self.moveTurtlebot3_object = MoveTurtlebot3()

        self.rate = rospy.Rate(225)

    def callback_wall(self, data):
        # print("in 1")
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
            if len(right_cone)!=0:
                right_min = min(right_cone)
            if len(left_cone)!= 0: 
                left_min = min(left_cone)

            front_min = min (front_cone)

    def camera_callback(self, data):
        global d, det_flag
        # print("in 2")
        # print("d: ", d, "det_flag: ", det_flag)
        if d ==3 and det_flag == 1: 
            print("line vala sleep liya")
            rospy.sleep(3)
            d = 4
            print(d)
        # We select bgr8 because its the OpneCV encoding by default
        cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")

            
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
        # d = 0
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
        twist_object.linear.x = 0.12
        diff = width/2 - cx
        twist_object.angular.z = diff/900

        if d==2:
            self.vel_pub.publish(twist_object)


    def det_callback(self,data):
        global d, det_flag
        #print("in 3")
        twist_obj = Twist()

        for i in range(len(data.bounding_boxes)):
            if data.bounding_boxes[i].Class == "stop sign":
                d = 3
                det_flag += 1
                print(d)
                print("dikhi to hai...")
                if det_flag ==1:
                    print("loop me bhi aya")
                    twist_obj.linear.x = 0
                    twist_obj.angular.z = 0
                    self.vel_pub.publish(twist_obj)
                    rospy.sleep(3)
                    rospy.loginfo("detected!!!!!!!!!")


    def wall_avoid(self):
        global right_min, left_min, front_min,d, det_flag
        print("in 4")
        vel_msg = Twist()
        vel_msg.linear.x = 0.1
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0


        while det_flag<1 and d<2:

            error = (left_min - right_min)/2
            
            # print("error: ", error)
            
            if front_min <= 0.2: 
                vel_msg.linear.x = 0
                vel_msg.angular.z = 0.1
            if front_min > 0.2: vel_msg.linear.x = 0.1

            if error!= float("inf") and error != float("-inf"):
                vel_msg.angular.z = error * 3
            if d!=2 and d!=3:
                self.vel_pub.publish(vel_msg)
            self.rate.sleep()

    def leg_callback(self, data):
        global d, det_flag, bot_x, bot_y, bot_yaw
        # print("in 5")
        # print("function entry done")
        # if det_flag == 1 and d == 3:
        #     print("tangdi vala sleep liya")
            # rospy.sleep(3)
        if det_flag>=1  and d ==1:
            print("ab leg karunga")
        # data extraction
        # pose array topic
            if len(data.poses) !=0:
                vel_msg = Twist()
                Kp_dist = 0.3 ; Kp_ang = 0.4
                leg_x = data.poses[0].position.x
                leg_y = abs(data.poses[0].position.y)
                theta_desired = math.atan2(leg_y - bot_y , leg_x - bot_x)
                distance_error = math.sqrt((leg_x - bot_x)**2 + (leg_y - bot_y)**2)
                rospy.loginfo("Bot x %f     Bot y %f    Bot yaw %f",bot_x,bot_y,bot_yaw)
                rospy.loginfo("leg x %f     leg y %f",leg_x,leg_y)
                rospy.loginfo("theta is %f",theta_desired)
                rospy.loginfo("distance error %f",distance_error)
                angular_error = theta_desired - bot_yaw
                if distance_error > 0.25:
                    vel_msg.linear.x = Kp_dist * distance_error
                    vel_msg.angular.z = Kp_ang * angular_error
                    self.vel_pub.publish(vel_msg)
                    self.rate.sleep()
                    # vel_msg.linear.x = 0
                    # vel_msg.angular.z = 0
                    # self.vel_pub.publish(vel_msg)
                



            # people_tracker topic
            # if len(data.people) !=0:
            #     self.rate_tracker = rospy.Rate(10)
            #     vel_msg = Twist()
            #     Kp_dist = 10 ; Kp_ang = 0.1
            #     leg_x = data.people[0].pos.x
            #     leg_y = data.people[0].pos.x
            #     theta_desired = math.atan2(leg_y - bot_y , leg_x - bot_x)
            #     distance_error = math.sqrt((leg_x - bot_x)**2 + (leg_y - bot_y)**2)
            #     rospy.loginfo("Bot x %f     Bot y %f    Bot yaw %f",bot_x,bot_y,bot_yaw)
            #     rospy.loginfo("leg x %f     leg y %f",leg_x,leg_y)
            #     rospy.loginfo("theta is %f",theta_desired)
            #     rospy.loginfo("distance error %f",distance_error)
            #     angular_error = theta_desired - bot_yaw
            #     if distance_error > 0.1:
            #         vel_msg.linear.x = Kp_dist * distance_error
            #         vel_msg.angular.z = Kp_ang * angular_error
            #         self.vel_pub.publish(vel_msg)
            #         self.rate.sleep()



        
        

    def bot_callback(self, msg):
        global bot_x, bot_y, bot_yaw
        pose = Twist()
        #print("in 6")
        quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        roll = euler[0]
        pitch = euler[1]
        bot_yaw = euler[2]
        bot_x = msg.pose.pose.position.x
        bot_y = msg.pose.pose.position.y
        
        

if __name__ == '__main__':
    # global right_min, left_min, front_min
    # global d, det_flag
    rospy.init_node('gazebo_scan_sub')
    tobj = LineFollower()
    tobj.wall_avoid()
    

    
    rospy.spin()