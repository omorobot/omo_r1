#!/usr/bin/env python

# original source code -> https://github.com/omorobot/omoros
# modified by Bishop Pearson

import sys
import rospy
import math
from time import sleep
from omo_r1_packet_handler import PacketHandler
from omo_r1d2_packet_handler import PacketHandler2

from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, Point, Vector3, Quaternion
from tf.broadcaster import TransformBroadcaster
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from omo_r1_bringup.srv import ResetOdom, ResetOdomResponse

class OdomPose(object):
   x = 0.0
   y = 0.0
   theta = 0.0
   timestamp = 0
   pre_timestamp = 0

class OdomVel(object):
   x = 0.0
   y = 0.0
   w = 0.0

class Joint(object):
   joint_name = ['wheel_left_joint', 'wheel_right_joint']
   joint_pos = [0.0, 0.0]
   joint_vel = [0.0, 0.0]

class RobotConfig(object):
   body_circumference = 0        # circumference length of robot for spin in place
   wheel_separation = 0.0        # Default Vehicle width in mm
   wheel_radius = 0.0            # Wheel radius
   wheel_circumference = 0       # Wheel circumference
   max_lin_vel_wheel = 0.0       # Maximum speed can be applied to each wheel (mm/s)
   max_lin_vel_x = 0             # Speed limit for vehicle (m/s)
   max_ang_vel_z = 0             # Rotational Speed limit for vehicle (rad/s)

   encoder_gear_ratio = 0
   encoder_step = 0
   encoder_pulse_per_wheel_rev = 0
   encoder_pulse_per_gear_rev = 0
    
class OMOR1MotorNode:
   def __init__(self):
      self.odom_mode = rospy.get_param("~odom_mode", "wheel_only")
      self.model_name = rospy.get_param("~model_name", "r1")
      # Open serial port
      if self.model_name == 'r1':
         self.ph = PacketHandler()
         self.ph.incomming_info = ['QENCOD', 'QODO', 'QDIFFV']
         self.ph.set_periodic_info(50)
         sleep(0.1)
      elif self.model_name == 'r1d2':
         self.ph = PacketHandler2()
         self.ph.incomming_info = ['ODO', 'VW', "POSE", "GYRO"]
         self.use_gyro = rospy.get_param("/use_imu_during_odom_calc/use_imu")
         self.ph.set_periodic_info(100)
         sleep(0.1)
      else :
         rospy.loginfo('Entered model name:{} is not supported!'.format(self.model_name))
         sys.exit()

      
      # Storaging
      self.odom_pose = OdomPose()
      self.odom_vel = OdomVel()
      self.joint = Joint() 

      self.enc_left_tot_prev, self.enc_right_tot_prev = 0.0, 0.0   
      self.enc_offset_left, self.enc_offset_right = 0.0, 0.0

      self.is_enc_offset_set = False
      self.is_imu_offset_set = False
      self.orientation = [0.0, 0.0, 0.0, 0.0]
      self.last_theta = 0.0

      # Set vehicle specific configurations
      self.config = RobotConfig()
      self.config.wheel_separation = 0.591
      self.config.wheel_radius = 0.11
      self.config.max_lin_vel_wheel = 1200.0
      self.config.max_lin_vel_x = 1.2
      self.config.max_ang_vel_z = 1.0
      self.config.encoder_pulse_per_gear_rev = 1000
      self.config.encoder_gear_ratio = 15
      self.config.body_circumference = self.config.wheel_separation * math.pi
      self.config.wheel_circumference = self.config.wheel_radius * 2 * math.pi
      self.config.encoder_pulse_per_wheel_rev = self.config.encoder_pulse_per_gear_rev * self.config.encoder_gear_ratio * 4
      self.config.encoder_step = self.config.wheel_circumference / self.config.encoder_pulse_per_wheel_rev

      rospy.loginfo('Wheel Track:{:.2f}m, Radius:{:.3f}m'.format(self.config.wheel_separation, self.config.wheel_radius))
      rospy.loginfo('Platform Rotation arc length: {:04f}m'.format(self.config.body_circumference))
      rospy.loginfo('Wheel circumference: {:04f}m'.format(self.config.wheel_circumference))
      rospy.loginfo('Encoder step: {:04f}m/pulse'.format(self.config.encoder_step))
      rospy.loginfo('Encoder pulses per wheel rev: {:.2f} pulses/rev'.format(self.config.encoder_pulse_per_wheel_rev))
      #rospy.loginfo('Serial port: %s', self.port_handler.get_port_name())

      # subscriber
      rospy.Subscriber("cmd_vel", Twist, self.cbSubCmdVelTMsg, queue_size=1)        # command velocity data subscriber
      rospy.Subscriber("imu", Imu, self.cbSubIMUTMsg, queue_size=1)                 # imu data subscriber

      # publisher
      self.pub_joint_states = rospy.Publisher('joint_states', JointState, queue_size=10)
      self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=10)
      self.odom_broadcaster = TransformBroadcaster()
      self.pub_pose = rospy.Publisher("pose", Pose, queue_size=1000)

      rospy.Service('reset_odom', ResetOdom, self.reset_odom_handle)
      
      # timer
      rospy.Timer(rospy.Duration(0.01), self.cbTimerUpdateDriverData) # 10 hz update
      #self.odom_pose.timestamp = rospy.Time.now().to_nsec()
      self.odom_pose.timestamp = rospy.Time.now()
      self.odom_pose.pre_timestamp = rospy.Time.now()
      self.reset_odometry()
      

      rospy.on_shutdown(self.__del__)

   def reset_odometry(self):
      self.is_enc_offset_set = False
      self.is_imu_offset_set = False

      self.joint.joint_pos = [0.0, 0.0]
      self.joint.joint_vel = [0.0, 0.0]

   def update_odometry(self, odo_l, odo_r, trans_vel, orient_vel, vel_z):
      odo_l /= 1000.
      odo_r /= 1000.
      trans_vel /= 1000.
      orient_vel /= 1000.

      self.odom_pose.timestamp = rospy.Time.now()
      dt = (self.odom_pose.timestamp - self.odom_pose.pre_timestamp).to_sec()
      self.odom_pose.pre_timestamp = self.odom_pose.timestamp

      if self.use_gyro:
         self.calc_yaw.wheel_ang += orient_vel * dt
         self.odom_pose.theta = self.calc_yaw.calc_filter(vel_z*math.pi/180., dt)
         rospy.loginfo('R1mini state : whl pos %1.2f, %1.2f, gyro : %1.2f, whl odom : %1.2f, robot theta : %1.2f', 
                           odo_l, odo_r, vel_z,
                           self.calc_yaw.wheel_ang*180/math.pi, 
                           self.odom_pose.theta*180/math.pi )
      else:
         self.odom_pose.theta += orient_vel * dt
         #rospy.loginfo('R1mini state : wheel pos %s, %s, speed %s, %s',
         #                odo_l, odo_r, trans_vel, orient_vel)

      d_x = trans_vel * math.cos(self.odom_pose.theta) 
      d_y = trans_vel * math.sin(self.odom_pose.theta) 

      self.odom_pose.x += d_x * dt
      self.odom_pose.y += d_y * dt

      odom_orientation_quat = quaternion_from_euler(0, 0, self.odom_pose.theta)

      self.odom_vel.x = trans_vel
      self.odom_vel.y = 0.
      self.odom_vel.w = orient_vel

      odom = Odometry()
      odom.header.frame_id = "odom"
      odom.child_frame_id = "base_footprint"

      self.odom_broadcaster.sendTransform((self.odom_pose.x, self.odom_pose.y, 0.), 
                                             odom_orientation_quat, self.odom_pose.timestamp, 
                                             odom.child_frame_id, odom.header.frame_id)

      odom.header.stamp = rospy.Time.now()
      odom.pose.pose = Pose(Point(self.odom_pose.x, self.odom_pose.y, 0.), Quaternion(*odom_orientation_quat))
      odom.twist.twist = Twist(Vector3(self.odom_vel.x, self.odom_vel.y, 0), Vector3(0, 0, self.odom_vel.w))

      self.odom_pub.publish(odom)

   def updatePoseStates(self, roll, pitch, yaw):
      #Added to publish pose orientation of IMU
      pose = Pose()
      pose.orientation.x = roll
      pose.orientation.y = pitch
      pose.orientation.z = yaw
      self.pub_pose.publish(pose)

   def updateJointStates(self, odo_l, odo_r, trans_vel, orient_vel):
      odo_l /= 1000.
      odo_r /= 1000.

      wheel_ang_left = odo_l / self.wheel_radius
      wheel_ang_right = odo_r / self.wheel_radius

      wheel_ang_vel_left = (trans_vel - (self.wheel_base / 2.0) * orient_vel) / self.wheel_radius
      wheel_ang_vel_right = (trans_vel + (self.wheel_base / 2.0) * orient_vel) / self.wheel_radius

      self.joint.joint_pos = [wheel_ang_left, wheel_ang_right]
      self.joint.joint_vel = [wheel_ang_vel_left, wheel_ang_vel_right]

      joint_states = JointState()
      joint_states.header.frame_id = "base_link"
      joint_states.header.stamp = rospy.Time.now()
      joint_states.name = self.joint.joint_name
      joint_states.position = self.joint.joint_pos
      joint_states.velocity = self.joint.joint_vel
      joint_states.effort = []

      self.pub_joint_states.publish(joint_states)

   def cbTimerUpdateDriverData(self, event):
      #self.packet_read_handler.read_packet()
      self.ph.read_packet()
      if self.model_name == 'r1d2':
         odo_l = self.ph._wodom[0]
         odo_r = self.ph._wodom[1]
         trans_vel = self.ph._vel[0]
         orient_vel = self.ph._vel[1]
         vel_z = self.ph._gyro[2]
         roll_imu = self.ph._imu[0]
         pitch_imu = self.ph._imu[1]
         yaw_imu = self.ph._imu[2]

         self.update_odometry(odo_l, odo_r, trans_vel, orient_vel, vel_z)
         self.updateJointStates(odo_l, odo_r, trans_vel, orient_vel)
         self.updatePoseStates(roll_imu, pitch_imu, yaw_imu)
      else :   
         lin_vel_x = self.ph.get_base_velocity()[0]
         ang_vel_z = self.ph.get_base_velocity()[1]

         odom_left_wheel = float(self.ph.get_wheel_odom()[0])
         odom_right_wheel = float(self.ph.get_wheel_odom()[1])
         rospy.loginfo('V= {}, W= {}, odo_l: {} odo_r:{}'.format(lin_vel_x, ang_vel_z,odom_left_wheel, odom_right_wheel))
         rpm_left_wheel = int(self.ph.get_wheel_rpm()[0])
         rpm_right_wheel = int(self.ph.get_wheel_rpm()[1])

         lin_vel_left_wheel = int(self.ph.get_wheel_velocity()[0])
         lin_vel_right_wheel = int(self.ph.get_wheel_velocity()[1])

         enc_left_now = self.ph.get_wheel_encoder()[0]
         enc_right_now = self.ph.get_wheel_encoder()[1]
         #rospy.loginfo('enc_l: {} enc_r:{}'.format(enc_left_now, enc_right_now))

         if self.is_enc_offset_set == False:
            self.enc_offset_left = enc_left_now
            self.enc_offset_right = enc_right_now
            self.is_enc_offset_set = True
         enc_left_tot = enc_left_now - self.enc_offset_left
         enc_right_tot = enc_right_now - self.enc_offset_right

         if self.odom_mode == "wheel_only":
            self.updatePoseUsingWheel(enc_left_tot, enc_right_tot)

         if self.odom_mode == "using_imu":
            self.updatePoseUsingIMU(enc_left_tot, enc_right_tot)
         
         self.updateJointStates(enc_left_tot, enc_right_tot, lin_vel_left_wheel, lin_vel_right_wheel)

   def cbSubIMUTMsg(self, imu_msg):
      self.orientation[0] = imu_msg.orientation.x
      self.orientation[1] = imu_msg.orientation.y
      self.orientation[2] = imu_msg.orientation.z
      self.orientation[3] = imu_msg.orientation.w

   def cbSubCmdVelTMsg(self, cmd_vel_msg):
      lin_vel_x = cmd_vel_msg.linear.x
      ang_vel_z = cmd_vel_msg.angular.z

      lin_vel_x = max(-self.config.max_lin_vel_x, min(self.config.max_lin_vel_x, lin_vel_x))
      ang_vel_z = max(-self.config.max_ang_vel_z, min(self.config.max_ang_vel_z, ang_vel_z))

      angular_speed_left_wheel = (lin_vel_x - (self.config.wheel_separation / 2.0) * ang_vel_z) / self.config.wheel_radius
      angular_speed_right_wheel = (lin_vel_x + (self.config.wheel_separation / 2.0) * ang_vel_z) / self.config.wheel_radius

      #self.packet_write_handler.write_wheel_velocity(angular_speed_left_wheel * self.config.wheel_radius * 1000, angular_speed_right_wheel * self.config.wheel_radius * 1000)
      self.ph.write_wheel_velocity(angular_speed_left_wheel * self.config.wheel_radius * 1000, angular_speed_right_wheel * self.config.wheel_radius * 1000)
   
   def updatePoseUsingWheel(self, enc_left_tot, enc_right_tot):
      enc_left_diff = enc_left_tot - self.enc_left_tot_prev
      enc_right_diff = enc_right_tot - self.enc_right_tot_prev
      self.enc_left_tot_prev = enc_left_tot
      self.enc_right_tot_prev = enc_right_tot

      timestamp_now = rospy.Time.now()
      timestamp_now_nsec = timestamp_now.to_nsec()
      d_time = (timestamp_now_nsec - self.odom_pose.timestamp) / 1000000000.0
      self.odom_pose.timestamp = timestamp_now_nsec

      d_s = (enc_left_diff + enc_right_diff) * self.config.encoder_step / 2.0

      b_l = enc_left_diff * self.config.encoder_step
      b_r = enc_right_diff * self.config.encoder_step

      r = (b_r + b_l) / 2.0
      d_theta = (b_r - b_l) / self.config.wheel_separation

      self.odom_pose.theta += d_theta
      self.odom_pose.x += math.cos(self.odom_pose.theta) * r
      self.odom_pose.y += math.sin(self.odom_pose.theta) * r

      self.odom_vel.x = d_s / d_time
      self.odom_vel.y = 0.0
      self.odom_vel.w = d_theta / d_time

      parent_frame_id = "odom"
      child_frame_id = "base_footprint"

      odom_orientation_quat = quaternion_from_euler(0, 0, self.odom_pose.theta)
      self.odom_broadcaster.sendTransform((self.odom_pose.x, self.odom_pose.y, 0.), odom_orientation_quat, timestamp_now, child_frame_id, parent_frame_id)
      
      odom = Odometry()
      odom.header.stamp = timestamp_now
      odom.header.frame_id = parent_frame_id
      odom.child_frame_id = child_frame_id
      odom.pose.pose = Pose(Point(self.odom_pose.x, self.odom_pose.y, 0.), Quaternion(*odom_orientation_quat))
      odom.twist.twist = Twist(Vector3(self.odom_vel.x, self.odom_vel.y, 0), Vector3(0, 0, self.odom_vel.w))
      
      self.odom_pub.publish(odom)

   def updatePoseUsingIMU(self, enc_left_tot, enc_right_tot):
      enc_left_diff = enc_left_tot - self.enc_left_tot_prev
      enc_right_diff = enc_right_tot - self.enc_right_tot_prev
      self.enc_left_tot_prev = enc_left_tot
      self.enc_right_tot_prev = enc_right_tot

      timestamp_now = rospy.Time.now()
      timestamp_now_nsec = timestamp_now.to_nsec()
      d_time = (timestamp_now_nsec - self.odom_pose.timestamp) / 1000000000.0
      self.odom_pose.timestamp = timestamp_now_nsec

      d_s = (enc_left_diff + enc_right_diff) * self.config.encoder_step / 2.0

      euler = euler_from_quaternion((self.orientation[0], self.orientation[1], self.orientation[2], self.orientation[3]))
      theta = euler[2]

      if self.is_imu_offset_set == False:
         self.last_theta = theta
         self.is_imu_offset_set = True

      d_theta = theta - self.last_theta
      self.last_theta = theta

      self.odom_pose.x += d_s * math.cos(self.odom_pose.theta + (d_theta / 2.0))
      self.odom_pose.y += d_s * math.sin(self.odom_pose.theta + (d_theta / 2.0))
      self.odom_pose.theta += d_theta

      self.odom_vel.x = d_s / d_time
      self.odom_vel.y = 0.0
      self.odom_vel.w = d_theta / d_time

      parent_frame_id = "odom"
      child_frame_id = "base_footprint"

      odom_orientation_quat = quaternion_from_euler(0, 0, self.odom_pose.theta)
      self.odom_broadcaster.sendTransform((self.odom_pose.x, self.odom_pose.y, 0.), odom_orientation_quat, timestamp_now, child_frame_id, parent_frame_id)
      
      odom = Odometry()
      odom.header.stamp = timestamp_now
      odom.header.frame_id = parent_frame_id
      odom.child_frame_id = child_frame_id
      odom.pose.pose = Pose(Point(self.odom_pose.x, self.odom_pose.y, 0.), Quaternion(*odom_orientation_quat))
      odom.twist.twist = Twist(Vector3(self.odom_vel.x, self.odom_vel.y, 0), Vector3(0, 0, self.odom_vel.w))
      
      self.odom_pub.publish(odom)

   def updateJointStates(self, enc_left_tot, enc_right_tot, lin_vel_left_wheel, lin_vel_right_wheel):
      wheel_ang_left = enc_left_tot * self.config.encoder_step / self.config.wheel_radius
      wheel_ang_right = enc_right_tot * self.config.encoder_step / self.config.wheel_radius

      wheel_ang_vel_left = lin_vel_left_wheel * 0.001 / self.config.wheel_radius
      wheel_ang_vel_right = lin_vel_right_wheel * 0.001 / self.config.wheel_radius

      self.joint.joint_pos = [wheel_ang_left, wheel_ang_right]
      self.joint.joint_vel = [wheel_ang_vel_left, wheel_ang_vel_right]

      joint_states = JointState()
      joint_states.header.frame_id = "base_link"
      joint_states.header.stamp = rospy.Time.now()
      joint_states.name = self.joint.joint_name
      joint_states.position = self.joint.joint_pos
      joint_states.velocity = self.joint.joint_vel
      joint_states.effort = []

      self.pub_joint_states.publish(joint_states)

   def reset_odom_handle(self, req):
      self.odom_pose.x = req.x
      self.odom_pose.y = req.y
      self.odom_pose.theta = req.theta

      return ResetOdomResponse()

   def main(self):
      rospy.spin()

   def __del__(self):
      print("terminating omo_r1_motor_node")
      rospy.loginfo("Shutting down. velocity will be 0")
      self.ph.close_port()

if __name__ == '__main__':
    rospy.init_node('omo_r1_motor_node')
    node = OMOR1MotorNode()
    node.main()