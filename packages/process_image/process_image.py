#!/usr/bin/env python

import rospy
import math
import os
import numpy as np
import sys
from time import sleep
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage


def detector_yellow(data):
    cv_image = CvBridge().compressed_imgmsg_to_cv2(data, desired_encoding="passthrough")
    lower_boundery = np.array([17, 15, 100])
    upper_boundery = np.array([50,56,200])
    mask = cv2.inRange(cv_image,lower_boundery, upper_boundery)
    output = cv2.bitwise_and(cv_image, cv_image, mask = mask)
    print("publish")
    ros_img = CvBridge().cv2_to_compressed_imgmsg(output)
    contours, hier = cv2.findContours(threshed_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    pub.publish(ros_img)

def detector_red(data):
    #cv_image = CvBridge().compressed_imgmsg_to_cv2(data, desired_encoding="passthrough")
    print(" ")

vehicle_name = os.environ['VEHICLE_NAME']
color = (rospy.myargv(argv=sys.argv)[1])
topic = '/' + vehicle_name + '/camera_node/image/compressed'
rospy.init_node('colordetector' + color, anonymous=True)
if color == "yellow":
    print(color)
    pub = rospy.Publisher("/maskedimage/compressed", CompressedImage, queue_size=5)
    rospy.Subscriber(topic, CompressedImage,detector_yellow)
elif color == "red":
    print(color)
    rospy.Subscriber(topic, CompressedImage,detector_red)

rospy.spin()


