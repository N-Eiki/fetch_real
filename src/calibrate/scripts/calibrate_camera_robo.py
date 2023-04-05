#!/usr/bin/env python
import time
import os
import rospy
import sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import trajectory_msgs.msg


import copy
import actionlib
import rospy
from math import sin, cos
from moveit_python import (MoveGroupInterface,
                           PlanningSceneInterface,
                           PickPlaceInterface)
from moveit_python.geometry import rotate_pose_msg_by_euler_angles

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from control_msgs.msg import PointHeadAction, PointHeadGoal
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Vector3, Quaternion

import matplotlib.pyplot as plt
import numpy as np
import time
import cv2

from tqdm import tqdm

# from real.camera import Camera

from scipy import optimize  
from mpl_toolkits.mplot3d import Axes3D  

from robot import Robot
# Point the head using controller


# Estimate rigid transform with SVD (from Nghia Ho)
def get_rigid_transform(A, B):
    assert len(A) == len(B)
    N = A.shape[0]; # Total points
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)
    AA = A - np.tile(centroid_A, (N, 1)) # Centre the points
    BB = B - np.tile(centroid_B, (N, 1))
    H = np.dot(np.transpose(AA), BB) # Dot is matrix multiplication for array
    U, S, Vt = np.linalg.svd(H)
    R = np.dot(Vt.T, U.T)
    if np.linalg.det(R) < 0: # Special reflection case
       Vt[2,:] *= -1
       R = np.dot(Vt.T, U.T)
    t = np.dot(-R, centroid_A.T) + centroid_B.T
    return R, t

def get_rigid_transform_error(z_scale, ):
    global measured_pts, observed_pts, observed_pix, world2camera, robot #camera

    # Apply z offset and compute new observed points using camera intrinsics
    observed_z = observed_pts[:,2:] * z_scale
    observed_x = np.multiply(observed_pix[:,[0]]-robot.config.intrinsics[0][2],observed_z/robot.config.intrinsics[0][0])
    observed_y = np.multiply(observed_pix[:,[1]]-robot.config.intrinsics[1][2],observed_z/robot.config.intrinsics[1][1])
    new_observed_pts = np.concatenate((observed_x, observed_y, observed_z), axis=1)

    # Estimate rigid transform between measured points and new observed points
    R, t = get_rigid_transform(np.asarray(measured_pts), np.asarray(new_observed_pts))
    t.shape = (3,1)
    world2camera = np.concatenate((np.concatenate((R, t), axis=1),np.array([[0, 0, 0, 1]])), axis=0)

    # Compute rigid transform error
    registered_pts = np.dot(R,np.transpose(measured_pts)) + np.tile(t,(1,measured_pts.shape[0]))
    error = np.transpose(registered_pts) - new_observed_pts
    error = np.sum(np.multiply(error,error))
    rmse = np.sqrt(error/measured_pts.shape[0]);
    return rmse

def do_calibrate():
    '''
    camera
        0.6, 0, 0
    x : +front, -back 
        0.2~0.7
    y : + left, - right
        -0.25~0.25
    '''
    global robot
    robot = Robot()

    workspace_limits = np.asarray([
        # [0.2, 0.7], # x-axis
        # [-0.25, 0.25],# y-axis
        # [0.5, 0.7], # z-axis
        [0.2, 0.7], # x-axis
        [-0.25, 0.25],# y-axis
        [0.6, 0.7], # z-axis
    ])

    checkerboard_offset_from_tool = [0., 0.0, -0.035]
    # checkerboard_offset_from_tool = [-0.3, 0.0, 0]

    calib_grid_step = 0.05
    rate_obj = rospy.Rate(1)

    # robot.move_manager(pose_requested=[],
    #                             joints_array_requested=[0.6, 0.0, 0.0],
    #                             movement_type_requested="HEAD")
    # Construct 3D calibration grid across workspace
    gridspace_x = np.linspace(workspace_limits[0][0], workspace_limits[0][1], 1 + (workspace_limits[0][1] - workspace_limits[0][0])/calib_grid_step)
    gridspace_y = np.linspace(workspace_limits[1][0], workspace_limits[1][1], 1 + (workspace_limits[1][1] - workspace_limits[1][0])/calib_grid_step)
    gridspace_z = np.linspace(workspace_limits[2][0], workspace_limits[2][1], 1 + (workspace_limits[2][1] - workspace_limits[2][0])/calib_grid_step)
    calib_grid_x, calib_grid_y, calib_grid_z = np.meshgrid(gridspace_x, gridspace_y, gridspace_z)
    num_calib_grid_pts = calib_grid_x.shape[0]*calib_grid_x.shape[1]*calib_grid_x.shape[2]

    
    calib_grid_x.shape = (num_calib_grid_pts,1)
    calib_grid_y.shape = (num_calib_grid_pts,1)
    calib_grid_z.shape = (num_calib_grid_pts,1)
    calib_grid_pts = np.concatenate((calib_grid_x, calib_grid_y, calib_grid_z), axis=1)
    global measured_pts, observed_pts, observed_pix, world2camera
    measured_pts = []
    observed_pts = []
    observed_pix = []
    done_count = 0
    if os.path.exists(os.path.join(robot.config.root, "calibrate/data/done_count.npy")):
        measured_pts = np.load(os.path.join(robot.config.root, "calibrate/data/measured_pts.npy")).tolist()
        observed_pts =  np.load(os.path.join(robot.config.root, "calibrate/data/observed_pts.npy")).tolist()
        observed_pix = np.load(os.path.join(robot.config.root, "calibrate/data/observed_pix.npy")).tolist()
        done_count = np.load(os.path.join(robot.config.root, "calibrate/data/done_count.npy")).item()
    def check_error(target, curr):
        return np.sqrt((target.x-curr.x)**2 + (target.y - curr.y)**2 + (target.z-target.z)**2)
    robot.close_gripper()
    print('Collecting data...')
    replace_flg = False
    for calib_pt_idx in tqdm(range(done_count, num_calib_grid_pts)):
        # if calib_pt_idx<:
        #     continue
        
        if replace_flg:
            robot.go_home_position()
            replace_flg = False
        tool_position = calib_grid_pts[calib_pt_idx,:]
        text = "go to " + str(tool_position)
        tqdm.write(text)
        # ps = geometry_msgs.msg.Pose()
        ps = robot.group.get_current_pose().pose
        ps.position = Vector3(*tool_position)
        ps.orientation = Quaternion(x=0.707352, y=-0.706861, z=-0.000439, w=0.000032)
        robot.move_manager(pose_requested=ps, joints_array_requested=None, movement_type_requested="TCP")

        time.sleep(1)
        replace_flg = check_error(ps.position,  robot.group.get_current_pose().pose.position)>0.01 
        # if check_error(ps.position,  robot.group.get_current_pose().pose.position)>0.01:
        #     print('error bigger than 0.01, sontinue')
            # robot.go_home_position()

        checkerboard_size = (3,3)
        refine_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        bgr_color_data, camera_depth_img = robot.get_camera_data()
        gray_data = cv2.cvtColor(bgr_color_data, cv2.COLOR_BGR2GRAY)
        checkerboard_found, corners = cv2.findChessboardCorners(gray_data, checkerboard_size, None, cv2.CALIB_CB_ADAPTIVE_THRESH)
        np.save('/home/rl-user/repos/nagata/catkin_ws/src/calibrate/data/done_count.npy', calib_pt_idx)
        if checkerboard_found:
            tqdm.write('found')
            corners_refined = cv2.cornerSubPix(gray_data, corners, (3,3), (-1,-1), refine_criteria)

            # Get observed checkerboard center 3D point in camera space
            checkerboard_pix = np.round(corners_refined[4,0,:]).astype(int)
            checkerboard_z = camera_depth_img[checkerboard_pix[1]][checkerboard_pix[0]]
            checkerboard_x = np.multiply(checkerboard_pix[0]-robot.config.intrinsics[0][2],checkerboard_z/robot.config.intrinsics[0][0])
            checkerboard_y = np.multiply(checkerboard_pix[1]-robot.config.intrinsics[1][2],checkerboard_z/robot.config.intrinsics[1][1])
            if checkerboard_z == 0:
                continue
            observed_pts.append([checkerboard_x,checkerboard_y,checkerboard_z])
            curr_pose = robot.group.get_current_pose().pose.position
            tool_position = np.array([curr_pose.x, curr_pose.y, curr_pose.z])
            tool_position = tool_position + checkerboard_offset_from_tool
            measured_pts.append(tool_position)
            observed_pix.append(checkerboard_pix)
            vis = cv2.drawChessboardCorners(bgr_color_data, (1,1), corners_refined[4,:,:], checkerboard_found)
            # vis = cv2.circle(vis, (int(tool_position[0]), int(tool_position[1])), 7, (0,0,255), 2)
            print('================')
            print(corners_refined[4,:,:],)
            print(tool_position)
            print('================')
            # cv2.imwrite('%06d.png' % len(measured_pts), vis)
            cv2.imshow('Calibration',vis)
            cv2.waitKey(10)
            np.save('/home/rl-user/repos/nagata/catkin_ws/src/calibrate/data/measured_pts.npy', measured_pts)
            np.save('/home/rl-user/repos/nagata/catkin_ws/src/calibrate/data/observed_pts.npy', observed_pts)
            np.save('/home/rl-user/repos/nagata/catkin_ws/src/calibrate/data/observed_pix.npy', observed_pix)
        else:
            tqdm.write('not found')
    text = 'collected ' + str(len(measured_pts)) + " data!"
    rospy.loginfo(text)
    robot.go_home_position()

    measured_pts = np.asarray(measured_pts)
    observed_pts = np.asarray(observed_pts)
    observed_pix = np.asarray(observed_pix)
    world2camera = np.eye(4)
    
    # Optimize z scale w.r.t. rigid transform error
    print('Calibrating...')
    z_scale_init = 1
    optim_result = optimize.minimize(get_rigid_transform_error, np.asarray(z_scale_init), method='Nelder-Mead')
    camera_depth_offset = optim_result.x

    # Save camera optimized offset and camera pose
    print('Saving...')
    root = robot.config.root
    np.savetxt(os.path.join(root, "calibrate/", 'real/camera_depth_scale.txt'), camera_depth_offset, delimiter=' ')
    get_rigid_transform_error(camera_depth_offset)
    camera_pose = np.linalg.inv(world2camera)
    np.savetxt(os.path.join(root, "calibrate/", 'real/camera_pose.txt'), camera_pose, delimiter=' ')
    print('Done.')
    

   
   
if __name__ == '__main__':
    rospy.init_node('fetch_move_node', anonymous=True, log_level=rospy.DEBUG)
    # watch_gripper_test()
    # updown_test()
    # calibrate_test()
    do_calibrate()