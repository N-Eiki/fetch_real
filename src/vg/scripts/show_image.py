import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

import numpy as np
import os

class Fetch:
    def __init__(self, ):
        rospy.init_node('fetch_camera_subscriber', anonymous=True)
        self.bridge = CvBridge()
        sub = rospy.Subscriber('/head_camera/rgb/image_raw', Image, self.callback)
        
        square_size = 2.
        self.pattern_size = (3, 3)
        self.pattern_points = np.zeros( (np.prod(self.pattern_size), 3), np.float32 )
        self.pattern_points[:,:2] = np.indices(self.pattern_size).T.reshape(-1, 2)
        self.pattern_points *= square_size
        self.objpoints = []
        self.imgpoints = []

        
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.capture = True
            rate.sleep()


    def callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            cv2.imshow("img", img)
            cv2.waitKey(1)

        except Exception as err:
            rospy.loginfo(err)


    
        

if __name__=='__main__':
    f = Fetch()

        