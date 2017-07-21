#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
from openpose_ros_msgs.msg import PeoplePoseArray as PDA

def callback(data):
    rospy.loginfo("%d", len(data.poses))

def listener():
    rospy.init_node("listenser", anonymous = True)
    rospy.Subscriber("/openpose_ros/skeleton", PDA, callback)
    rospy.spin()

if __name__ == "__main__":
    listener()
