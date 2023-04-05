#!/usr/bin/env python

import os
import numpy as np

class config:
    root = "/home/rl-user/repos/nagata/catkin_ws/src/"

    #camera
    intrinsics = np.load(os.path.join(root, 'calibrate/params/fetch_mtx.npy'))
    cam_pose = np.loadtxt(os.path.join(root,"calibrate/real", "camera_pose.txt"), delimiter=" ")
    cam_depth_scale = np.loadtxt(os.path.join(root, "calibrate/real", "camera_depth_scale.txt"))
    
    #robot env
    x_offset = 0.3
    y_offset = -0.05
    heightmap_resolution = 0.5 / 224
    workspace_limits = np.asarray([[0.2+x_offset, 0.7+x_offset], [-0.25 + y_offset, 0.25 + y_offset], [4., 1.5]])
    # workspace_limits = np.asarray([[0.6, 1.1], [-0.25, 0.25], [0., 2.]])

    