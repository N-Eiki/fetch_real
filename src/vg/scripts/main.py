#!/usr/bin/env python
import time
import matplotlib.pyplot as plt

import struct
import math
import numpy as np
import cv2
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.autograd import Variable

import rospy
import geometry_msgs.msg
from geometry_msgs.msg import Vector3, Quaternion
from geometry_msgs.msg import Pose

from robot import Robot
import utils

if __name__=='__main__':
    rospy.init_node('top_down_image_test')

    ## use for primitive_position
    x_offset = 0.0
    y_offset = 0.1

    heightmap_resolution = 0.5 / 224
    robot = Robot()
    workspace_limits = robot.config.workspace_limits
    # ps = robot.group.get_current_pose().pose
    # ps.position.x = np.random.uniform(0.4, 0.7)
    # ps.position.y = np.random.uniform(-0.25, 0.25)
    # ps.position.z= np.random.uniform(0.6, 0.8)
    # ps.orientation.x = 0.707352
    # ps.orientation.y = -0.706861
    # ps.orientation.z = -0.000439
    # ps.orientation.w = 0.000032
    # robot.move_manager(pose_requested=ps, joints_array_requested=None, movement_type_requested="TCP")
    time.sleep(1)
    
    color_img, depth_img = robot.get_camera_data()
    depth_img = depth_img * robot.config.cam_depth_scale

    color_heightmap, depth_heightmap = utils.get_heightmap(color_img, depth_img, robot.config.intrinsics, robot.config.cam_pose, workspace_limits, heightmap_resolution)
    valid_depth_heightmap = depth_heightmap.copy()
    valid_depth_heightmap[np.isnan(valid_depth_heightmap)] = 0

    best_pix_ind = np.unravel_index(np.argmax(valid_depth_heightmap), valid_depth_heightmap.shape)
    prediction_vis = cv2.cvtColor(color_heightmap, cv2.COLOR_BGR2RGB)
    prediction_vis = cv2.circle(prediction_vis, (int(best_pix_ind[1]), int(best_pix_ind[0])), 7, (255,0,0), 2)
    plt.imshow(prediction_vis.astype(np.uint8))
    plt.savefig('/home/rl-user/repos/nagata/catkin_ws/src/vg/tmp/tmp.png')
    robot.grasp(best_pix_ind, valid_depth_heightmap)
    
    
    
    best_pix_x, best_pix_y = best_pix_ind
    primitive_position = [best_pix_x * heightmap_resolution + workspace_limits[0][0] - x_offset, best_pix_y * heightmap_resolution + workspace_limits[1][0] - y_offset, max(valid_depth_heightmap[best_pix_y][best_pix_x] + workspace_limits[2][0], 0.6)]
    ps = geometry_msgs.msg.Pose()
    # ps = robot.group.get_current_pose().pose
    ps.position = Vector3(*primitive_position)
    ps.orientation = Quaternion(*[-0.5, 0.5, 0.5, 0.5])
    
    # print(ps)
    # robot.move_manager(pose_requested=ps, joints_array_requested=None, movement_type_requested="TCP")

    # plt.imshow(color_heightmap);plt.show()
    # plt.imshow(valid_depth_heightmap);plt.show()
    # robot.go_home_position()
    # import ipdb;ipdb.set_trace()
    print('done')