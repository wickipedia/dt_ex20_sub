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
    detect_yellow = rospy.get_param("/detector/yellow")
    detect_red = rospy.get_param("/detector/red")
    if not detect_yellow or detect_red:
        return
    cv_image = CvBridge().compressed_imgmsg_to_cv2(data, desired_encoding="passthrough")
    lower_boundery = np.array([0, 150, 0])
    upper_boundery = np.array([200, 250, 250])
    mask = cv2.inRange(cv_image,lower_boundery, upper_boundery)
    output = cv2.bitwise_and(cv_image, cv_image, mask = mask)
    _,contours, hier = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    print("publish")
    for c in contours:
        # get the bounding rect
        x, y, w, h = cv2.boundingRect(c)
        # draw a green rectangle to visualize the bounding rect
        cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
    ros_img = CvBridge().cv2_to_compressed_imgmsg(cv_image)
    pub.publish(ros_img)

def detector_red(data):
    detect_red = rospy.get_param("/detector/red")
    detect_yellow = rospy.get_param("/detector/yellow")
    if not detect_red or detect_yellow:
        return
    cv_image = CvBridge().compressed_imgmsg_to_cv2(data, desired_encoding="passthrough")
    lower_boundery = np.array([0, 0, 0])
    upper_boundery = np.array([50,56,250])
    mask = cv2.inRange(cv_image,lower_boundery, upper_boundery)
    output = cv2.bitwise_and(cv_image, cv_image, mask = mask)
    _,contours, hier = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    print("publish")
    for c in contours:
        # get the bounding rect
        x, y, w, h = cv2.boundingRect(c)
        # draw a green rectangle to visualize the bounding rect
        cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
    ros_img = CvBridge().cv2_to_compressed_imgmsg(cv_image)
    pub.publish(ros_img)

vehicle_name = os.environ['VEHICLE_NAME']
color = (rospy.myargv(argv=sys.argv)[1])
topic = '/' + vehicle_name + '/camera_node/image/compressed'
rospy.init_node('colordetector' + color, anonymous=True)
pub = rospy.Publisher("/maskedimage/compressed", CompressedImage, queue_size=5)
if color == "yellow":
    print(color)
    rospy.Subscriber(topic, CompressedImage,detector_yellow)
elif color == "red":
    print(color)
    rospy.Subscriber(topic, CompressedImage,detector_red)

rospy.spin()


