#!/usr/bin/env python

import rospy
import time 
import PID_pw as PID
from math import atan2, pi, sqrt, cos
from copy import deepcopy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

import actionlib
import omo_simple_position_controller.msg
from omo_r1_bringup.srv import ResetOdom, ResetOdomResponse

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

def robot_odom_sub(msg):
    global cur_pos
    
    rot_q = msg.pose.pose.orientation 
    roll, pitch, theta = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    cur_pos.x = msg.pose.pose.position.x
    cur_pos.y = msg.pose.pose.position.y
    cur_pos.theta = theta

def call_reset_odom(x, y, th):
    try:
        rospy.wait_for_service('reset_odom')
        reset_odom = rospy.ServiceProxy('reset_odom', ResetOdom)
        reset_odom(x, y, th)
        rospy.loginfo('Robot Odom is reset to (x, y, theta) = (0, 0, 0)')
    except rospy.ServiceException as e:
        rospy.loginfo("Service call failed: %s"%e)

def execute_cb(goal):
    global cur_pos, pub, speed, dist_stop_condition

    rospy.loginfo('Start vanilla position control to (x, y) = (%1.2f, %1.2f)', goal.x, goal.y)

    rate = rospy.Rate(50)
    progress = True

    sub = rospy.Subscriber("/odom", Odometry, robot_odom_sub)

    while True:
        e_s, e_theta = calc_errors(cur_pos, goal)

        feedback.error_s = e_s
        feedback.error_theta = e_theta*180/pi
        robot_action.publish_feedback(feedback)

        speed.angular.z = theta_PID.process(e_theta)
        speed.linear.x = translation_PID.process(e_s)

        pub.publish(speed)

        if ((abs(e_s) < dist_stop_condition) and ((abs(e_theta)) < rotational_stop_condition)):
            rospy.loginfo('Complete vanilla position control to (x, y) = (%1.2f, %1.2f)', goal.x, goal.y)
            rospy.loginfo('Residue Error is (translation, rotation(deg)) = (%1.2f, %1.2f)',
                                                            feedback.error_s, feedback.error_theta)
            call_reset_odom(0, 0, 0)
            break

        if robot_action.is_preempt_requested():
            rospy.loginfo('Request Preempted.')
            robot_action.set_preempted()
            call_reset_odom(0, 0, 0)
            break

        rate.sleep() 

    result.x_result = cur_pos.x
    result.y_result = cur_pos.y
    result.theta_result = cur_pos.theta

    print "=== set_succeeded result" 

    robot_action.set_succeeded(result)

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

    dist_stop_condition = rospy.get_param("/robot_stop/dist_stop_condition")
    rotational_stop_condition = rospy.get_param("/robot_stop/rotational_stop_condition")
    
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

    feedback = omo_simple_position_controller.msg.VanillaPositionFeedback()
    result = omo_simple_position_controller.msg.VanillaPositionResult()

    robot_action = actionlib.SimpleActionServer(rospy.get_name(), 
                                      omo_simple_position_controller.msg.VanillaPositionAction, 
                                      execute_cb = execute_cb, auto_start = False)

    robot_action.start()

    rospy.spin()