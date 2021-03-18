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

# trace, rotation, station, done
op_mode = 'done'

def sub_odom_callback(msg):
    global cur_pos
    
    rot_q = msg.pose.pose.orientation 
    roll, pitch, theta = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    cur_pos.x = msg.pose.pose.position.x
    cur_pos.y = msg.pose.pose.position.y
    cur_pos.theta = theta

def calc_errors(cur_pos, goal):
    delta_y_ref = goal.y - cur_pos.y
    delta_x_ref = goal.x - cur_pos.x

    if ((goal.x == 0.) and (goal.y == 0.)):
        e_s = 0.
    else:
        goal.theta = atan2(delta_y_ref, delta_x_ref)
        e_s = sqrt(delta_x_ref**2 + delta_y_ref**2) * cos( atan2(delta_y_ref, delta_x_ref) - cur_pos.theta )
    
    e_theta = goal.theta - cur_pos.theta

    return e_s, e_theta

def sub_markers_callback(msg):
    global operation_mode

    marker_id = []
    length = []

    x_pos = []
    y_pos = []
    z_pos = []
    ori_x = []
    ori_y = []
    ori_z = []

    for each in msg.markers:
        marker_id.append(each.id)

        x_pos.append(each.pose.pose.position.x)
        y_pos.append(each.pose.pose.position.y)
        z_pos.append(each.pose.pose.position.z)
        length.append(np.sqrt(x_pos**2, y_pos**2, z_pos**2))

        r, p, th = euler_from_quaternion([each.pose.pose.orientation.x, each.pose.pose.orientation.y, 
                                            each.pose.pose.orientation.z, each.pose.pose.orientation.w])

        ori_x.append(r)
        ori_y.append(p)
        ori_z.append(th)

    idx = np.argmin(length)

    target_marker = marker_is[idx]

    target_x = x_slope * x_pos[idx] + x_bias
    target_y = y_slope * y_pos[idx] + y_bias

    if op_mode == 'done':
        if target_marker == 0:
            op_mode = 'trace'

        elif target_marker == 1:
            op_mode = 'rotation'

        else:
            op_mode = 'station'

    if op_mode == 'trace':
        e_s, e_theta = calc_errors(cur_pos, goal)
        
        speed.angular.z = theta_PID.process(e_theta)
        speed.linear.x = translation_PID.process(e_s)

        pub.publish(speed)


if __name__ == '__main__':
    rospy.init_node('marker_tracker')

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

    x_slope = rospy.get_param("/linear_model_for_target_pos/x_slope")
    x_bias = rospy.get_param("/linear_model_for_target_pos/x_bias")
    y_slope = rospy.get_param("/linear_model_for_target_pos/y_slope")
    y_bias = rospy.get_param("/linear_model_for_target_pos/y_bias")

    dist_stop_condition = rospy.get_param("/robot_stop/dist_stop_condition")
    rotational_stop_condition = rospy.get_param("/robot_stop/rotational_stop_condition")
    
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
    sub_markers = rospy.Subscriber('ar_pose_marker', AlvarMarkers, sub_markers_callback)
    #sub_odom = rospy.Subscriber("/odom", Odometry, sub_odom_callback)

    rospy.spin()