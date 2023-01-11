#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
from math import *
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
objPose = Pose()
objPose.position.x = 0
objPose.position.y = 0
objPose.position.z = 0

vel = Twist()
vel.linear.x = 0.0
vel.linear.y = 0.0
vel.linear.z = 0.0
vel.angular.x = 0.0
vel.angular.y = 0.0
vel.angular.z = 0.0


class Stop_color:
    def __init__(self):    
        #Subscribe Object Pose

        self.Pose_sub = rospy.Subscriber("object_detect_pose", Pose, self.poseCallback)
        #Publish Command
        self.vel_pub = rospy.Publisher('teleop_cmd_vel', Twist, queue_size=1)
	
	self.flag = False
	self.move()

    def poseCallback(self,Pose):

        X = Pose.position.x
        Y = Pose.position.y
        Z = Pose.position.z


        if Z >=100 :                            
            self.flag = True
        else:
            self.flag = False

    def move(self):	
	vel = Twist()
	
	if self.flag:
            print("stop")
	    vel.linear.x = 0
        else:
            print("forward")
	    vel.linear.x = 70
	self.flag = False
	self.vel_pub.publish(vel)
        rospy.loginfo(
                    "Publsh velocity command[{} m/s, {} rad/s]".format(
                        vel.linear.x, vel.angular.z))


if __name__ == '__main__':
    try:
        # Initialize ROS node
        rospy.init_node("Stop_color")
        rospy.loginfo("Starting")
        obj = Stop_color()
	rate = rospy.Rate(5)
	while not rospy.is_shutdown():
		obj.move()
        	rate.sleep()
    except KeyboardInterrupt:
        print "Shutting down follow_object node."
        cv2.destroyAllWindows()
