#!/usr/bin/env python

import os
import numpy as np

class config:
    root = "/home/rl-user/repos/nagata/catkin_ws/src"
    print(root)
    intrinsics = np.load(os.path.join(root, "calibrate/params/fetch_mtx.npy"))
    if os.path.exists(os.path.join(root, "calibrate/real", "camera_pose.txt")):
        cam_pose = np.loadtxt(os.path.join(root, "calibrate/real", "camera_pose.txt"), delimiter=" ")
        cam_depth_scale = np.loadtxt(os.path.join(root, "calibrate/real", "camera_depth_scale.txt"))
