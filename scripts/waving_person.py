#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
from openpose_ros_msgs.msg import PeoplePoseArray as PDA

import cv2
import numpy as np
import cv_bridge
import message_filters
from sensor_msgs.msg import Image

class WavingPerson:

    limb_names = [    
        "Nose",
        "Neck",
        "RShoulder",
        "RElbow",
        "RWrist",
        "LShoulder",
        "LElbow",
        "LWrist",
        "RHip",
        "RKnee",
        "RAnkle",
        "LHip",
        "LKnee",
        "LAnkle",
        "REye",
        "LEye",
        "REar",
        "LEar"]

    def __init__(self):
        self.image_topic = rospy.get_param("camera_topic", "/camera/rgb/image_rect_color")
        self.skeleton_topic = rospy.get_param("skeleton_topic", "/openpose_ros/skeleton")
        self.subscribe()
        self.img_pub_ = rospy.Publisher("/openpose_ros/waving_person/debug_img", Image, queue_size = 1)
        

    def subscribe(self):
        queue_size = rospy.get_param('~queue_size', 10)
        sub_img = message_filters.Subscriber(
            self.image_topic, Image, queue_size=1, buff_size=2**24)
        sub_pose = message_filters.Subscriber(
            self.skeleton_topic, PDA, queue_size=1, buff_size=2**24)
        self.subs = [sub_img, sub_pose]
        
        if rospy.get_param('~approximate_sync', False):
            slop = rospy.get_param('~slop', 0.1)
            sync = message_filters.ApproximateTimeSynchronizer(
                fs=self.subs, queue_size=queue_size, slop=slop)
        else:
            sync = message_filters.TimeSynchronizer(
                fs=self.subs, queue_size=queue_size)
        sync.registerCallback(self._cb)

    def unsubscribe(self):
        for sub in self.subs:
            sub.unregister()

    def _cb(self, img_msg, pda_msg):
        print "aaaaa"

        br = cv_bridge.CvBridge()
        img = br.imgmsg_to_cv2(img_msg, desired_encoding = "bgr8")
        
        people_pose = pda_msg.poses

        debug_img_msg = br.cv2_to_imgmsg(img, encoding = "bgr8")
        debug_img_msg.header = img_msg.header

        self.img_pub_.publish(debug_img_msg)

if __name__ == "__main__":
    rospy.init_node("waving_person_detection")
    WavingPerson()
    rospy.spin()
