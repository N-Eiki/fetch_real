#!/usr/bin/env python
import time
import sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import trajectory_msgs.msg

from fetch_images.srv import GetImages,GetImagesResponse
from cv_bridge import CvBridge, CvBridgeError

import copy
import actionlib
import rospy
import numpy as np
import cv2
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

from config import config

class Robot:
    def __init__(self, eef='gripper_link', ):        
        rospy.loginfo("In Move Fetch Calss init...")
        # Init Torso Action
        self.torso_action = FollowTrajectoryClient(
            "torso_controller", ["torso_lift_joint"])
        # Gripper Action
        self.gripper_action = GripperActionClient()
        # Point Head action
        self.head_action = PointHeadClient()
        # MOVEIT
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.logdebug("moveit_commander initialised...")

        rospy.logdebug("Starting Robot Commander...")
        self.robot = moveit_commander.RobotCommander()
        rospy.logdebug("Starting Robot Commander...DONE")
        self.scene = moveit_commander.PlanningSceneInterface()
        rospy.logdebug("PlanningSceneInterface initialised...DONE")
        self.group = moveit_commander.MoveGroupCommander("arm")
        rospy.logdebug("MoveGroupCommander for arm initialised...DONE")
        rospy.logwarn("self.group TYPE==>"+str(type(self.group)))
        rospy.loginfo("FETCH ready to move!")
        self.group.set_end_effector_link(eef)
        self.gripper_group = moveit_commander.MoveGroupCommander("gripper")

        #add env object
        # rospy.loginfo('add table to env')
        # self.scene = moveit_commander.PlanningSceneInterface()
        # rospy.sleep(2)

        # p = geometry_msgs.msg.PoseStamped()
        # p.header.frame_id = "world"
        # p.pose.position.x = 0.5
        # p.pose.position.y = 0.0
        # p.pose.position.z = 0.7
        # # self.scene.add_box('table', p, size=(-0.5, 0.5, 0.72))
        # self.scene.add_box('table', p, size=(0.75, 0.1, 1.2))

        rospy.loginfo('another settings')
        
        self.bridge = CvBridge()
        self.config = config
        # self.group.stop()
        # rospy.sleep(2)
        self.go_home_position()
        self.open_gripper()
        # self.group.stop()
        # rospy.sleep(2)
    
    def grasp(self, best_pix_ind, valid_depth_heightmap):
        rot = 4
        if len(best_pix_ind)==2:
            best_pix_x, best_pix_y = best_pix_ind
        else:
            rot = best_pix_x, best_pix_y = best_pix_ind
        height = valid_depth_heightmap[best_pix_y][best_pix_x]
        self.open_gripper()
        quaternion = [-0.5, 0.5, 0.5, 0.5]
        quaternion = [-0.499219533259, 0.500511143052, 0.500218485425, 0.500049917642]
        rospy.loginfo(height)
        self.go_target_position(best_pix_x, best_pix_y, height + self.config.workspace_limits[2][0] + 0.05, quaternion)

        # [TODO]
        # implement rotation into quaternion
        jvs = self.group.get_current_joint_values()
        # jvs[-1] -= np.deg2rad(22.5*rot)
        jvs[-1] -= np.deg2rad(22.5*rot)
        rotate = False
        # while not rotate:
        # rotate = self.move_manager(pose_requested=None,joints_array_requested=jvs,movement_type_requested="JOINTS")

        rotates = {
            0:[-0.499219533259, 0.500511143052, 0.500218485425, 0.500049917642],
            4:[-0.707131561789, 0.000240113539969,  0.707081467691, 0.000833495785438]
        }

        quaternion = rotates[rot]
        self.go_target_position(best_pix_x, best_pix_y, height + self.config.workspace_limits[2][0] - 0.05, quaternion)
        
        rospy.loginfo('close_gripper')
        self.close_gripper()
        rospy.sleep(1)
        self.go_target_position(best_pix_x, best_pix_y, height + self.config.workspace_limits[2][0] + 0.05,quaternion)
        
        # self.open_gripper()
        success = self.check_grasp()
        return success

    def check_grasp(self):
        curr_width = self.gripper_group.get_current_joint_values()[0]
        return curr_width > 0.01

    def go_target_position(self, best_pix_x, best_pix_y, height, quaternion):
        primitive_position = [best_pix_x * self.config.heightmap_resolution + self.config.workspace_limits[0][0] , #-self.config.x_offset/2,
                                 best_pix_y * self.config.heightmap_resolution + self.config.workspace_limits[1][0] - self.config.y_offset,
                                  height + self.config.workspace_limits[2][0]]
        # primitive_position = [best_pix_y * self.config.heightmap_resolution + self.config.workspace_limits[1][0] - self.config.y_offset,\
        #                         best_pix_x * self.config.heightmap_resolution + self.config.workspace_limits[0][0] - self.config.x_offset,\
        #                           max(height + self.config.workspace_limits[2][0], 0.6)]
        ps = geometry_msgs.msg.Pose()
        ps.position = Vector3(*primitive_position)
        ps.orientation = Quaternion(*quaternion)
        result = False
        # while not result:
        result = self.move_manager(pose_requested=ps, joints_array_requested=None, movement_type_requested="TCP")


    def get_camera_data(self):
        rospy.wait_for_service("/srv/image_service")
        image_service = rospy.ServiceProxy('/srv/image_service', GetImages)

        time.sleep(3)
        images = image_service()
        rgb, depth = images.rgb_image, images.depth_image
        # rgb, depth = image_service().rgb_image, image_service().depth_image
        rgb = self.bridge.imgmsg_to_cv2(rgb, 'bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth, '32FC1')
        # rotate image 180 degree

        rgb = cv2.rotate(rgb, cv2.ROTATE_90_CLOCKWISE)
        depth = cv2.rotate(depth, cv2.ROTATE_90_CLOCKWISE)
        rgb = cv2.rotate(rgb, cv2.ROTATE_90_CLOCKWISE)
        depth = cv2.rotate(depth, cv2.ROTATE_90_CLOCKWISE)
        return rgb, depth

    def go_home_position(self):
        init = [1.57, -0.9, 0, 0.9, 0.0, 1.57, 0.0]
        done = False
        # while not done:
        done = self.move_manager(pose_requested=None,
            joints_array_requested=init,
            movement_type_requested="JOINTS")
        self.move_manager(pose_requested=[],
                                joints_array_requested=[0.6, 0.0, 0.],
                                movement_type_requested="HEAD")
        rospy.sleep(2)
        # self.group.stop()

    def open_gripper(self):
        rospy.loginfo('OPEN')
        # self.gripper_group.stop()
        result = False
        while not result:
            self.gripper_group.set_joint_value_target([0.045, 0.045])
            result = self.execute_trajectory(self.gripper_group)
            # result = self.gripper_group.go()
    
    def close_gripper(self):
        rospy.loginfo('CLOSE')
        # self.gripper_group.stop()
        result = False
        while not result:
            self.gripper_group.set_joint_value_target([0.0, 0.0])
            result = self.execute_trajectory(self.gripper_group)
            # result = self.gripper_group.go(wait=True)
        rospy.sleep(2)

    def move_manager(self, pose_requested, joints_array_requested, movement_type_requested):
        success = False
        # self.group.stop()
        rospy.sleep(1)
        if movement_type_requested == "TCP":
            print(pose_requested)
            success = self.ee_traj(pose_requested)
        elif movement_type_requested == "JOINTS":
            success = self.joint_traj(joints_array_requested)
        elif movement_type_requested == "TORSO":
            torso_height = joints_array_requested[0]
            success = self.move_torso(torso_height)
        elif movement_type_requested == "HEAD":
            XYZ = [joints_array_requested[0],
                   joints_array_requested[1],
                   joints_array_requested[2]]
            success = self.move_head_point(XYZ)
        elif movement_type_requested == "GRIPPER":
            gripper_x = joints_array_requested[0]
            max_effort = joints_array_requested[1]

            success = self.move_gripper(gripper_x, max_effort)
        else:
            rospy.logerr("Asked for non supported movement type==>" +
                         str(movement_type_requested))
        
        
        return success

    def ee_traj(self, pose):
        pose_frame = self.group.get_pose_reference_frame()
        if pose_frame != "base_link":
            new_reference_frame = "base_link"
            self.group.set_pose_reference_frame(new_reference_frame)
            pose_frame = self.group.get_pose_reference_frame()
        else:
            pass
        self.group.set_pose_target(pose)
        result = self.execute_trajectory(self.group)
        return result

    def joint_traj(self, positions_array):

        self.group_variable_values = self.group.get_current_joint_values()
        rospy.logdebug("Group Vars:")
        rospy.logdebug(self.group_variable_values)
        rospy.logdebug("Point:")
        rospy.logdebug(positions_array)
        self.group_variable_values[0] = positions_array[0]
        self.group_variable_values[1] = positions_array[1]
        self.group_variable_values[2] = positions_array[2]
        self.group_variable_values[3] = positions_array[3]
        self.group_variable_values[4] = positions_array[4]
        self.group_variable_values[5] = positions_array[5]
        self.group_variable_values[6] = positions_array[6]
        self.group.set_joint_value_target(self.group_variable_values)
        result = self.execute_trajectory(self.group)

        return result
    def move_torso(self, torso_height):
        """
        Changes the Fetch Torso height based on the height given.
        """
        result = self.torso_action.move_to([torso_height, ])

        return result

    def move_gripper(self, gripper_x, max_effort):
        """
        Moves the gripper to given pose
        """
        result = self.gripper_action.move_gripper(gripper_x, max_effort)

        return result

    def move_head_point(self, XYZ, frame="base_link"):
        """
        Changes the Fetch Torso height based on the height given.
        """
        result = self.head_action.look_at(XYZ[0], XYZ[1], XYZ[2], frame)

        return result

    def execute_trajectory(self, group):
        plan = group.plan()
        if plan.joint_trajectory.header.frame_id=='':
            return False
        # result = self.group.go()
        result = group.execute(plan, wait=True)
        # # self.group.stop()
       


        return result

    def ee_pose(self):

        gripper_pose = self.group.get_current_pose()

        rospy.logdebug("EE POSE==>"+str(gripper_pose))

        return gripper_pose

    def ee_rpy(self, request):

        gripper_rpy = self.group.get_current_rpy()

        return gripper_rpy


class PointHeadClient(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient(
            "head_controller/point_head", PointHeadAction)
        rospy.loginfo("Waiting for head_controller...")
        self.client.wait_for_server()

    def look_at(self, x, y, z, frame, duration=1.0):
        goal = PointHeadGoal()
        goal.target.header.stamp = rospy.Time.now()
        goal.target.header.frame_id = frame
        goal.target.point.x = x
        goal.target.point.y = y
        goal.target.point.z = z
        goal.min_duration = rospy.Duration(duration)
        self.client.send_goal(goal)
        self.client.wait_for_result()

# Move base using navigation stack


class MoveBaseClient(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base...")
        self.client.wait_for_server()

    def goto(self, x, y, theta, frame="map"):
        move_goal = MoveBaseGoal()
        move_goal.target_pose.pose.position.x = x
        move_goal.target_pose.pose.position.y = y
        move_goal.target_pose.pose.orientation.z = sin(theta/2.0)
        move_goal.target_pose.pose.orientation.w = cos(theta/2.0)
        move_goal.target_pose.header.frame_id = frame
        move_goal.target_pose.header.stamp = rospy.Time.now()

        # TODO wait for things to work
        self.client.send_goal(move_goal)
        self.client.wait_for_result()


class FollowTrajectoryClient(object):

    def __init__(self, name, joint_names):
        self.client = actionlib.SimpleActionClient("%s/follow_joint_trajectory" % name,
                                                   FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for %s..." % name)
        self.client.wait_for_server()
        self.joint_names = joint_names

    def move_to(self, positions, duration=5.0):
        if len(self.joint_names) != len(positions):
            print("Invalid trajectory position")
            return False
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        trajectory.points.append(JointTrajectoryPoint())
        trajectory.points[0].positions = positions
        trajectory.points[0].velocities = [0.0 for _ in positions]
        trajectory.points[0].accelerations = [0.0 for _ in positions]
        trajectory.points[0].time_from_start = rospy.Duration(duration)
        follow_goal = FollowJointTrajectoryGoal()
        follow_goal.trajectory = trajectory

        self.client.send_goal(follow_goal)
        result = self.client.wait_for_result()

        return result


class GripperActionClient(object):
    def __init__(self):

        rospy.loginfo("Waiting for gripper_controller...")
        self.gripper_client = actionlib.SimpleActionClient(
            "gripper_controller/gripper_action", GripperCommandAction)
        self.gripper_client.wait_for_server()
        rospy.loginfo("...connected.")

    def move_gripper(self, gripper_x, max_effort, timeout=5.0):

        gripper_goal = GripperCommandGoal()
        gripper_goal.command.max_effort = max_effort
        gripper_goal.command.position = gripper_x

        self.gripper_client.send_goal(gripper_goal)
        result = self.gripper_client.wait_for_result(rospy.Duration(timeout))

        return result