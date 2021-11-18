#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from sensor_msgs.msg import Image,CompressedImage 
from cv_bridge import CvBridge, CvBridgeError
from threading import Thread
import numpy as np
import argparse
import os
import cv2
import math
from time import sleep
from VideoGet import VideoGet
from VideoShow import VideoShow
from aruco import aruco_detect

node_name = "m5_opencv_camera"
topic_pub_images = "/armpap00/Images"
topic_cordinates = "/armpap00/cordinates"
wifi_camera = "http://192.168.0.144/capture"
window_name = "Armpap_camera"
path_destination = "/data/"
img_counter = 0  # !!! Initialize if you do not want to delete the previous captures

"""
    Program that captures the image of wifi cameras and send it Compressed by ROS.
    Also Save the capture in specified folder
    And some utils with aruco depending on the case
"""
def grayscale(in_frame):
    grayFrame = cv2.cvtColor(in_frame, cv2.COLOR_BGR2GRAY)
    return grayFrame

def save_images(out_frame):
    global img_counter, path_destination, window_name
    img_name = path_destination + window_name + str(img_counter) + ".png"
    cv2.imwrite(img_name, out_frame)
    img_counter += 1

def send_images(out_frame):
    global pub_image
    imgmsg = CompressedImage()
    imgmsg.header.stamp = rospy.Time.now()
    imgmsg.format ="jpeg"
    imgmsg.data = np.array(cv2.imencode('.jpg', out_frame)[1]).tostring()
    pub_image.publish(imgmsg)


if __name__ == '__main__':
    rospy.init_node(node_name, anonymous =True)
    rospy.loginfo("M5_camera cv_bridge")
    pub_image = rospy.Publisher(topic_pub_images, CompressedImage)
    imgmsg = CompressedImage()
    cmd = rospy.Publisher(topic_cordinates, Float32MultiArray, queue_size=1)
    msg = Float32MultiArray()
    msg.layout.dim.append(MultiArrayDimension())
    msg.layout.dim[0].label = 'cordinates'
    msg.layout.dim[0].size = 3
    msg.layout.dim[0].stride = 1
    msg.layout.data_offset = 0
    aruco = aruco_detect(cv2.aruco.DICT_4X4_100,20,297)
    video_getter = VideoGet(wifi_camera).start()
    video_shower =VideoShow(video_getter.frame, window_name).start()
    while not rospy.is_shutdown():
        if video_getter.stopped or video_shower.stopped:
            video_getter.stop()
            video_shower.stop()
            break
        frame = video_getter.frame
        if frame is not None:
            send_images(frame)
            frame,cordinates = aruco.detect_corde(frame)
            video_shower.frame = frame
            if all(x != y for x,y in zip(cordinates,(0.0,0.0,0.0))):
                msg.data = [cordinates[0],cordinates[1],cordinates[2]]
                cmd.publish(msg)
                
