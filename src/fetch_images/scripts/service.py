#!/usr/bin/env python

import math
import base64
import numpy as np

import cv2
from cv_bridge import CvBridge, CvBridgeError

import rospy


from std_srvs.srv import Trigger, TriggerResponse
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from fetch_images.srv import GetImages, GetImagesResponse
import message_filters

global rgb_data
global depth_data

bridge = CvBridge()


def callback(rgb_data_msg, depth_data_msg):
    global rgb_data
    global depth_data
    rgb_data = rgb_data_msg
    depth_data = depth_data_msg
    
    
    # np.save('test.npy', data)


def image_service(data):
    print('call')
    global rgb_data
    global depth_data
    response = GetImagesResponse()
    response.rgb_image = rgb_data
    response.depth_image = depth_data
    # response.rgb_image = bridge.imgmsg_to_cv2(rgb_data, 'bgr8')
    # response.depth_image = bridge.imgmsg_to_cv2(depth_data, '32FC1')
    
    return response

if __name__=="__main__":
    rospy.init_node('image_service', anonymous=True)
    rgb_sub = message_filters.Subscriber('/head_camera/rgb/image_raw', Image)
    depth_sub = message_filters.Subscriber('/head_camera/depth/image_raw', Image)

    fps = 100.
    delay = 1/fps*.5
    ts = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub], 10, delay)
    ts.registerCallback(callback)
    image_service = rospy.Service("/srv/image_service", GetImages, image_service)
    rospy.spin()