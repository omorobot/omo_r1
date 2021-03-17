#!/usr/bin/env python

import rospy 
import time 
import numpy as np
import PID_pw as PID
from math import atan2, pi, sqrt, cos

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

from omo_r1_bringup.srv import ResetOdom, ResetOdomResponse
from ar_track_alvar_msgs.msg import AlvarMarkers

def sub_odom_callback(msg):
    global cur_pos
    
    rot_q = msg.pose.pose.orientation 
    roll, pitch, theta = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    cur_pos.x = msg.pose.pose.position.x
    cur_pos.y = msg.pose.pose.position.y
    cur_pos.theta = theta

def sub_markers_callback(msg):

    marker_id = []
    length = []
    x_pos = []
    y_pos = []
    marker_theta = []

    for each in msg.markers:
        marker_id.append(each.id)
        x_pos.append(each.pose.pose.position.x)
        y_pos.append(each.pose.pose.position.y)
        marker_theta.append(each.pose.pose.orientation.z)
        length.append(np.sqrt(x_pos**2, y_pos**2))

    idx = np.argmin(length)
    target_x = x_pos(idx)
    target_y = y_pos(idx)
    target_marker_theta = marker_theta(idx)
    target_id = marker_id(idx)

    print idx, target_x, target_y, np.min(length)



if __name__ == '__main__':
    rospy.init_node('vanilla_position_ctrl')

    cur_pos = PID.RobotState()
    speed = Twist()

    theta_PID = PID.PID()
    theta_PID.P = rospy.get_param("/rotational_PID/P")
    theta_PID.I = rospy.get_param("/rotational_PID/I")
    theta_PID.max_state = rospy.get_param("/rotational_PID/max_state")
    theta_PID.min_state = rospy.get_param("/rotational_PID/min_state")

    translation_PID = PID.PID()
    translation_PID.P = rospy.get_param("/translation_PID/P")
    translation_PID.I = rospy.get_param("/translation_PID/I")
    translation_PID.max_state = rospy.get_param("/translation_PID/max_state")
    translation_PID.min_state = rospy.get_param("/translation_PID/min_state")

    error_s_max = rospy.get_param("/marker_trace/error_s_max")

    dist_stop_condition = rospy.get_param("/robot_stop/dist_stop_condition")
    rotational_stop_condition = rospy.get_param("/robot_stop/rotational_stop_condition")
    
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

    sub_markers = rospy.Subscriber('ar_pose_marker', AlvarMarkers, sub_markers_callback)
    sub_odom = rospy.Subscriber("/odom", Odometry, sub_odom_callback)

    rospy.spin()