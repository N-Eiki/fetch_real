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
import numpy as np
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

from robot import Robot
# Point the head using controller


def updown_test():
    # move_fetch_obj = MoveFetch()
    move_fetch_obj = Robot()
    # move_fetch_obj.group.set_end_effector_link('gripper_link')
    ps = move_fetch_obj.group.get_current_pose().pose
    # import ipdb;ipdb.set_trace();
    import copy
    arm_pose_positions_L = copy.deepcopy(ps)
    arm_pose_positions_R = copy.deepcopy(ps)
    # from utils import show_pose_values
    # show_pose_values( move_fetch_obj.group)
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
    arm_pose_positions_L.position.x = 0.5
    arm_pose_positions_L.position.y = 0.0
    arm_pose_positions_L.position.z= 0.8
    arm_pose_positions_L.orientation.x= pose["ori_x"]
    arm_pose_positions_L.orientation.y= pose["ori_y"]
    arm_pose_positions_L.orientation.z= pose["ori_z"]
    arm_pose_positions_L.orientation.w= pose["ori_w"]

    arm_pose_positions_R.position.x= 0.5
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
            move_fetch_obj.move_manager(pose_requested=arm_pose_positions_L, joints_array_requested=None, movement_type_requested="TCP")
            move_fetch_obj.close_gripper()

        else:
            side = "LEFT"
            move_fetch_obj.move_manager(pose_requested=arm_pose_positions_R,joints_array_requested=None,movement_type_requested="TCP")
            move_fetch_obj.open_gripper()
        # curr = move_fetch_obj.group.get_current_joint_values()
        # curr[-1] += np.deg2rad(90)

        rate_obj.sleep()
        ps = move_fetch_obj.group.get_current_pose().pose
        
def watch_gripper_test():
    arm_joint_positions_L = [-1.57, -0.9, 0, 0.9, 0.0, 1.57, 0.0]
    arm_joint_positions_R = [1.57, -0.9, 0, 0.9, 0.0, 1.57, 0.0]
    move_fetch_obj = Robot()

    side = "UP"

    rate_obj = rospy.Rate(1)
    while not rospy.is_shutdown():

        if side == "UP":
            side = "DOWN"
            move_fetch_obj.move_manager(pose_requested=[],
                                joints_array_requested=[0.6, 0.0, 0.75],
                                movement_type_requested="HEAD")
            # print("CLOSE GRIPPER")
            # move_fetch_obj.close_gripper()
        else:
            side = "UP"
            move_fetch_obj.move_manager(pose_requested=[],
                                joints_array_requested=[0.6, 0.0, 0.1],
                                movement_type_requested="HEAD")
            # print("OPEN GRIPPER")
            # move_fetch_obj.open_gripper()
        rate_obj.sleep()
        # import ipdb;ipdb.set_trace()



if __name__ == '__main__':
    rospy.init_node('fetch_move_node', anonymous=True, log_level=rospy.DEBUG)

    watch_gripper_test()
    # updown_test()