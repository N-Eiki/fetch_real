#!/usr/bin/env python

import rospy
from fetch_images.srv import GetImages,GetImagesResponse
from cv_bridge import CvBridge, CvBridgeError

import matplotlib.pyplot as plt

if __name__=="__main__":
    rospy.init_node('image_client')
    rospy.wait_for_service("/srv/image_service")
    image_service = rospy.ServiceProxy('/srv/image_service', GetImages)
    image_service()
    bridge = CvBridge()
    images = image_service()
    rgb, depth = images.rgb_image, images.depth_image
    # rgb, depth = image_service().rgb_image, image_service().depth_image
    rgb = bridge.imgmsg_to_cv2(rgb, 'bgr8')
    depth = bridge.imgmsg_to_cv2(depth, '32FC1')

    plt.imshow(rgb);plt.show()
    plt.imshow(depth);plt.show()