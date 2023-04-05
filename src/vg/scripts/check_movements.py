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


def updown_test():
    # robot = MoveFetch()
    robot = Robot()
    # robot.group.set_end_effector_link('gripper_link')
    ps = robot.group.get_current_pose().pose
    # import ipdb;ipdb.set_trace();
    import copy
    arm_pose_positions_L = copy.deepcopy(ps)
    arm_pose_positions_R = copy.deepcopy(ps)
    # from utils import show_pose_values
    # show_pose_values( robot.group)
    # import sys
    # sys.exit()
    pose_dict = {
        "vertical":{
            "ori_x":-0.5,
            "ori_y":0.5,
            "ori_z":0.5,
            "ori_w":0.5,
            "low_h" : 0.5,
            "high_h":0.9
        },
        "test":{
            "ori_x":0.707352,
            "ori_y":-0.706861,
            "ori_z":-0.000439,
            "ori_w":0.000032,
            "low_h" : 0.5,
            "high_h":0.9
        },
        "watch_palm":{
            "ori_x":0.5,
            "ori_y":-0.5,
            "ori_z":-0.5,
            "ori_w":0.5,
            "low_h" : 0.5,
            "high_h":0.9
        }
    }
    #arm_pose_positions_L high vertical to body
    pose = pose_dict["test"]
    arm_pose_positions_L.position.x = 0.7
    arm_pose_positions_L.position.y = 0.0
    arm_pose_positions_L.position.z= 0.6
    arm_pose_positions_L.orientation.x= pose["ori_x"]
    arm_pose_positions_L.orientation.y= pose["ori_y"]
    arm_pose_positions_L.orientation.z= pose["ori_z"]
    arm_pose_positions_L.orientation.w= pose["ori_w"]

    arm_pose_positions_R.position.x= 0.2
    arm_pose_positions_R.position.y= 0.0
    arm_pose_positions_R.position.z= 0.6
    arm_pose_positions_R.orientation.x= pose["ori_x"]
    arm_pose_positions_R.orientation.y= pose["ori_y"]
    arm_pose_positions_R.orientation.z= pose["ori_z"]
    arm_pose_positions_R.orientation.w= pose["ori_w"]
    
    
    
    side = "LEFT"

    rate_obj = rospy.Rate(1)
    while not rospy.is_shutdown():
        if side == "LEFT":
            side = "RIGHT"
            robot.move_manager(pose_requested=arm_pose_positions_L, joints_array_requested=None, movement_type_requested="TCP")
            # robot.close_gripper()

        else:
            side = "LEFT"
            robot.move_manager(pose_requested=arm_pose_positions_R,joints_array_requested=None,movement_type_requested="TCP")
            # robot.open_gripper()
        # curr = robot.group.get_current_joint_values()
        # curr[-1] += np.deg2rad(90)

        rate_obj.sleep()
        ps = robot.group.get_current_pose().pose
        
def watch_gripper_test():
    arm_joint_positions_L = [-1.57, -0.9, 0, 0.9, 0.0, 1.57, 0.0]
    arm_joint_positions_R = [1.57, -0.9, 0, 0.9, 0.0, 1.57, 0.0]
    robot = Robot()

    side = "UP"

    rate_obj = rospy.Rate(3)
    while not rospy.is_shutdown():

        if side == "UP":
            side = "DOWN"
            robot.move_manager(pose_requested=[],
                                joints_array_requested=[0.6, 0.0, 0.1],
                                movement_type_requested="HEAD")
            rgb, depth = robot.get_camera_data()
            # print("CLOSE GRIPPER")
            # robot.close_gripper()
        else:
            side = "UP"
            robot.move_manager(pose_requested=[],
                                joints_array_requested=[0.6, 0.0, 0.99],
                                movement_type_requested="HEAD")
            rgb, depth = robot.get_camera_data()
            # print("OPEN GRIPPER")
            # robot.open_gripper()
        rate_obj.sleep()
        cv2.imshow(side, rgb)
        cv2.waitKey(1)
        # import ipdb;ipdb.set_trace()


def calibrate_test():
    # robot = MoveFetch()
    robot = Robot()
    # robot.group.set_end_effector_link('gripper_link')
    ps = robot.group.get_current_pose().pose

    import copy
    arm_pose_positions_L = copy.deepcopy(ps)
    arm_pose_positions_R = copy.deepcopy(ps)

    pose_dict = {
        "test":{
            "ori_x":0.707352,
            "ori_y":-0.706861,
            "ori_z":-0.000439,
            "ori_w":0.000032,
            "low_h" : 0.5,
            "high_h":0.9
        },
    }
    #arm_pose_positions_L high vertical to body
    pose = pose_dict["test"]
    '''
    camera
        0.6, 0, 0
    x : +front, -back 
        0.2~0.7
    y : + left, - right
        -0.25~0.25
    '''
    arm_pose_positions_L.position.x = 0.5
    arm_pose_positions_L.position.y = -0.25
    arm_pose_positions_L.position.z= 0.6
    arm_pose_positions_L.orientation.x= pose["ori_x"]
    arm_pose_positions_L.orientation.y= pose["ori_y"]
    arm_pose_positions_L.orientation.z= pose["ori_z"]
    arm_pose_positions_L.orientation.w= pose["ori_w"]

    arm_pose_positions_R.position.x= 0.5
    arm_pose_positions_R.position.y= 0.25
    arm_pose_positions_R.position.z= 0.6
    arm_pose_positions_R.orientation.x= pose["ori_x"]
    arm_pose_positions_R.orientation.y= pose["ori_y"]
    arm_pose_positions_R.orientation.z= pose["ori_z"]
    arm_pose_positions_R.orientation.w= pose["ori_w"]
    
    side = "LEFT"
    rate_obj = rospy.Rate(1)

    robot.move_manager(pose_requested=[],
                                joints_array_requested=[0.6, 0.0, 0.0],
                                movement_type_requested="HEAD")
    while not rospy.is_shutdown():
        if side == "LEFT":
            side = "RIGHT"
            robot.move_manager(pose_requested=arm_pose_positions_L, joints_array_requested=None, movement_type_requested="TCP")
            # robot.close_gripper()

        else:
            side = "LEFT"
            robot.move_manager(pose_requested=arm_pose_positions_R,joints_array_requested=None,movement_type_requested="TCP")
            # robot.open_gripper()
        # curr = robot.group.get_current_joint_values()
        # curr[-1] += np.deg2rad(90)

        rate_obj.sleep()
        ps = robot.group.get_current_pose().pose


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

def get_rigid_transform_error(z_scale):
    global measured_pts, observed_pts, observed_pix, world2camera, camera

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

def check_movements():
    '''
    camera
        0.6, 0, 0
    x : +front, -back 
        0.2~0.7
    y : + left, - right
        -0.25~0.25
    '''
    robot = Robot()

    workspace_limits = np.asarray([
        # [0.2, 0.7], # x-axis
        # [-0.25, 0.25],# y-axis
        # [0.6, 0.7], # z-axis
        [0.3, 0.7], # x-axis
        [-0.25, 0.25],# y-axis
        [0.5, 0.7], # z-axis
    ])

    checkerboard_offset_from_tool = [0.1, 0, 0]

    calib_grid_step = 0.05
    rate_obj = rospy.Rate(1)

    robot.move_manager(pose_requested=[],
                                joints_array_requested=[0.6, 0.0, 0.0],
                                movement_type_requested="HEAD")
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

    success_x, success_y, success_z = [], [], []
    fail_x, fail_y, fail_z = [], [], []
    print('Collecting data...')
    
    def check_error(target, curr):
        return np.sqrt((target.x-curr.x)**2 + (target.y - curr.y)**2 + (target.z-target.z)**2)

    fig = plt.figure(figsize = (8, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title("Helix", size = 20)
    ax.set_xlabel("x", size = 14)
    ax.set_ylabel("y", size = 14)
    ax.set_zlabel("z", size = 14)
    ax.set_xticks([-1.0, -.5, 0.0, .5, 1.0])
    ax.set_yticks([-1.0, -.5, 0.0, .5, 1.0])
    start = 0
    if os.path.exists('/home/rl-user/repos/nagata/catkin_ws/src/calibrate/data/success.npy'):
        start = len(np.load('/home/rl-user/repos/nagata/catkin_ws/src/calibrate/data/success.npy')[0]) + len(np.load('/home/rl-user/repos/nagata/catkin_ws/src/calibrate/data/fail.npy')[0])
    print('start from ', start)
    for calib_pt_idx in tqdm(range(num_calib_grid_pts)):
        if calib_pt_idx<start:
            continue
        tool_position = calib_grid_pts[calib_pt_idx,:]
        text = "go to " + str(tool_position)
        tqdm.write(text)
        ps = geometry_msgs.msg.Pose()
        ps.position = Vector3(*tool_position)
        ps.orientation = Quaternion(x=0.707352, y=-0.706861, z=-0.000439, w=0.000032)
        robot.move_manager(pose_requested=ps, joints_array_requested=None, movement_type_requested="TCP")
        time.sleep(1)
        if check_error(ps.position,  robot.group.get_current_pose().pose.position)<0.01:
            rospy.loginfo('succeed')
            success_x.append(ps.position.x)
            success_y.append(ps.position.y)
            success_z.append(ps.position.z)
        else:
            rospy.loginfo('failed')
            robot.go_home_position()
            fail_x.append(ps.position.x)
            fail_y.append(ps.position.y)
            fail_z.append(ps.position.z)
        np.save("/home/rl-user/repos/nagata/catkin_ws/src/calibrate/data/success.npy", [success_x, success_y, success_z])
        np.save("/home/rl-user/repos/nagata/catkin_ws/src/calibrate/data/fail.npy", [fail_x, fail_y, fail_z])

    np.save("/home/rl-user/repos/nagata/catkin_ws/src/calibrate/data/success.npy", [success_x, success_y, success_z])
    np.save("/home/rl-user/repos/nagata/catkin_ws/src/calibrate/data/fail.npy", [fail_x, fail_y, fail_z])
    ax.scatter(success_x, success_y, success_z, s = 40, c = "blue")
    ax.scatter(fail_x, fail_y, fail_z, s=40, c="red")
    plt.show()
    

    print('Done.')
    

   
   
if __name__ == '__main__':
    rospy.init_node('fetch_move_node', anonymous=True, log_level=rospy.DEBUG)
    # watch_gripper_test()
    # updown_test()
    # calibrate_test()
    check_movements()