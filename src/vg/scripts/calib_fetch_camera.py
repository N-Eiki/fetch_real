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

        self.root = "/home/rl-user/repos/nagata/catkin_ws/src/calibrate"
        self.mtx, self.dist = None, None

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.capture = True
            rate.sleep()


    def callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            if self.capture:
                ret, corner = cv2.findChessboardCorners(gray, self.pattern_size)

                if ret == True :
                    term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.1)
                    cv2.cornerSubPix(gray, corner, (5,5), (-1,-1), term)
                    self.imgpoints.append(corner.reshape(-1, 2))
                    self.objpoints.append(self.pattern_points)
                    if len(self.objpoints)%10==0:
                        ret, self.mtx, self.dist, rvecs, tvecs = cv2.calibrateCamera(self.objpoints, self.imgpoints, gray.shape[::-1], None, None)
                        np.save(os.path.join(self.root, "params", "fetch_mtx.npy"), self.mtx)
                        np.save(os.path.join(self.root, "params", "fetch_dist.npy"), self.dist)
                        print('saved')
                self.capture = False
            if self.mtx is not None:
                dst = self.distortion(img)
                cv2.imshow("dst", dst)

            cv2.imshow("img", img)
            cv2.waitKey(1)

        except Exception as err:
            rospy.loginfo(err)

    def distortion(self, img):
        mtx, dist = self.mtx, self.dist
        h, w = img.shape[:2]
        # getOptimalNewCameraMatrix
        newmtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h,), 1,(w, h))
        dst = cv2.undistort(img, mtx, dist, None, newmtx)
        return dst


    
        

if __name__=='__main__':
    f = Fetch()

        