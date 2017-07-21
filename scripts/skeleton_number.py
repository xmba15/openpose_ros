#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import numpy as np
import cv_bridge
import message_filters
import 
from openpose_ros_msgs.msg import PeoplePoseArray as PDA
from sensor_msgs.msg import Image

def callback(data):
    rospy.loginfo("%d", len(data.poses))

def listener():
    rospy.init_node("listenser", anonymous = True)
    rospy.Subscriber("/openpose_ros/skeleton", PDA, callback)
    rospy.spin()

if __name__ == "__main__":
    listener()
