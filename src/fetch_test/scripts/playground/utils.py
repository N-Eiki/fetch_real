import os
import numpy as np
import rospy

def show_pose_values(mgc):
    ps = mgc.get_current_pose().pose
    pos = ps.position
    ori = ps.orientation
    rospy.loginfo('====================')
    rospy.loginfo('current pose values')
    rospy.loginfo('position x ; %f', pos.x)
    rospy.loginfo('position y ; %f', pos.y)
    rospy.loginfo('position z ; %f', pos.z)
    rospy.loginfo('orientation x ; %f'%ori.x)
    rospy.loginfo('orientation y ; %f'%ori.y)
    rospy.loginfo('orientation z ; %f', ori.z)
    rospy.loginfo('orientation w ; %f', ori.w)
    rospy.loginfo('====================')