#!/usr/bin/env python
import time
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

import matplotlib.pyplot as plt
import numpy as np
import time
import cv2

from tqdm import tqdm

from real.camera import Camera

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

    rate_obj = rospy.Rate(1)
    while not rospy.is_shutdown():

        if side == "UP":
            side = "DOWN"
            robot.move_manager(pose_requested=[],
                                joints_array_requested=[0.6, 0.0, 0.75],
                                movement_type_requested="HEAD")
            # print("CLOSE GRIPPER")
            # robot.close_gripper()
        else:
            side = "UP"
            robot.move_manager(pose_requested=[],
                                joints_array_requested=[0.6, 0.0, 0.1],
                                movement_type_requested="HEAD")
            # print("OPEN GRIPPER")
            # robot.open_gripper()
        rate_obj.sleep()
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



def do_calibrate():
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
        [0.2, 0.7], # x-axis
        [-0.25, 0.25],# y-axis
        [0.6, 0.7], # z-axis
    ])

    calib_grid_step = 0.05
    # Construct 3D calibration grid across workspace
    gridspace_x = np.linspace(workspace_limits[0][0], workspace_limits[0][1], 1 + (workspace_limits[0][1] - workspace_limits[0][0])/calib_grid_step)
    gridspace_y = np.linspace(workspace_limits[1][0], workspace_limits[1][1], 1 + (workspace_limits[1][1] - workspace_limits[1][0])/calib_grid_step)
    gridspace_z = np.linspace(workspace_limits[2][0], workspace_limits[2][1], 1 + (workspace_limits[2][1] - workspace_limits[2][0])/calib_grid_step)
    num_calib_grid_pts = calib_grid_x.shape[0]*calib_grid_x.shape[1]*calib_grid_x.shape[2]

    calib_grid_x, calib_grid_y, calib_grid_z = np.meshgrid(gridspace_x, gridspace_y, gridspace_z)
    calib_grid_x.shape = (num_calib_grid_pts,1)
    calib_grid_y.shape = (num_calib_grid_pts,1)
    calib_grid_z.shape = (num_calib_grid_pts,1)
    calib_grid_pts = np.concatenate((calib_grid_x, calib_grid_y, calib_grid_z), axis=1)

    measured_pts = []
    observed_pts = []
    observed_pix = []
    print('Collecting data...')
    for calib_pt_idx in tqdm(range(num_calib_grid_pts)):
        tool_position = calib_grid_pts[calib_pt_idx,:]
        time.sleep(1)
        checkerboard_size = (3,3)
        refine_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        camera_color_img, camera_depth_img = robot.get_camera_data()

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
    [ 0.7  -0.15  0.5 ]
    arm_pose_positions_L.position.x = 0.7
    arm_pose_positions_L.position.y = -0.15
    arm_pose_positions_L.position.z= 0.5
    
    # arm_pose_positions_L.position.x = 0.5
    # arm_pose_positions_L.position.y = -0.25
    # arm_pose_positions_L.position.z= 0.6
    # arm_pose_positions_L.orientation.x= pose["ori_x"]
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
if __name__ == '__main__':
    rospy.init_node('fetch_move_node', anonymous=True, log_level=rospy.DEBUG)
    # watch_gripper_test()
    # updown_test()
    # calibrate_test()
    do_calibrate()