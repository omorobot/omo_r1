#!/usr/bin/env python

# original source code -> https://github.com/omorobot/omoros
# modified by Bishop Pearson

import sys
import rospy
import serial
import io
import math
from time import sleep

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
   
class PacketWriteHandler:
   _ph = None

   def __init__(self, ph):
      self._ph = ph

   def __del__(self):
      self._ph = None

   def write_register(self, param1, param2):
      self.write_packet("$SREGI," + str(param1) + ',' + param2)
      sleep(0.05)

   def write_periodic_query_value(self, param):
      self.write_packet("$SPERI," + str(param))
      sleep(0.05)

   def write_periodic_query_enable(self, param):
      self.write_packet("$SPEEN," + str(param))
      sleep(0.05)

   def write_init_odometry(self):
      self.write_packet("$SODO")
      sleep(0.05)

   def write_wheel_velocity(self, wheel_l_lin_vel, wheel_r_lin_vel):
      self.write_packet('$CDIFFV,{:.0f},{:.0f}'.format(wheel_l_lin_vel, wheel_r_lin_vel))

   def write_base_velocity(self, lin_vel, ang_vel):
      # lin_vel : mm/s, ang_vel : mrad/s
      self.write_packet('$CVW,{:.0f},{:.0f}'.format(lin_vel, ang_vel))

   def write_packet(self, packet):
      if self._ph.get_port_state() == True:
         self._ph.write_port(packet)

   def stop_peen(self):
      self.write_packet("$SPEEN, 0")
      sleep(0.05)

   def stop_callback(self):
      self.write_packet("$SCBEN, 0")
      sleep(0.05)
      
class ReadLine:
   def __init__(self, s):
      self.buf = bytearray()
      self.s = s

   def readline(self):
      i = self.buf.find(b"\n")
      if i >= 0:
         r = self.buf[:i+1]
         self.buf = self.buf[i+1:]
         return r
      while True:
         i = max(1, min(2048, self.s.in_waiting))
         data = self.s.read(i)
         i = data.find(b"\n")
         if i >= 0:
            r = self.buf + data[:i+1]
            self.buf[0:] = data[i+1:]
            return r
         else:
            self.buf.extend(data)

class PacketReadHandler:
   _ph = None

   # velocity
   _vel = None
   # encoder
   _enc = None
   # odometry
   _odom = None
   # rpm
   _rpm = None
   # wheel velocity
   _wvel = None

   def __init__(self, ph):
      self._ph = ph

      self._vel = [0.0, 0.0]
      self._enc = [0.0, 0.0]
      self._wodom = [0.0, 0.0]
      self._rpm = [0.0, 0.0]
      self._wvel = [0.0, 0.0]

   def __del__(self):
      self._ph = None

      self._vel = None
      self._enc = None
      self._wodom = None
      self._rpm = None
      self._wvel = None

   def get_base_velocity(self):
      return self._vel
   
   def get_wheel_encoder(self):
      return self._enc

   def get_wheel_odom(self):
      return self._wodom

   def get_wheel_rpm(self):
      return self._rpm
   
   def get_wheel_velocity(self):
      return self._wvel

   def read_packet(self):
      if self._ph.get_port_state() == True:
         whole_packet = self._ph.read_port()
         if whole_packet:
            packet = whole_packet.split(",")
            try:
               header = packet[0].split("#")[1]
               
               if header.startswith('QVW'):
                  self._vel = [int(packet[1]), int(packet[2])]
               elif header.startswith('QENCOD'):
                  self._enc = [int(packet[1]), int(packet[2])]
               elif header.startswith('QODO'):
                  self._wodom = [int(packet[1]), int(packet[2])]
               elif header.startswith('QRPM'):
                  self._rpm = [int(packet[1]), int(packet[2])]
               elif header.startswith('QDIFFV'):
                  self._wvel = [int(packet[1]), int(packet[2])]
            except:
               pass

class PortHandler():
   _port_name = None
   _baud_rate = None
   _ser = None
   _ser_io = None
   _rl = None

   def __init__(self, port_name, baud_rate):
      self._port_name = port_name
      self._baud_rate = baud_rate

      self.set_port_handler(port_name, baud_rate)

   def __del__(self):
      self._ser.close()

   def set_port_handler(self, port_name, baud_rate):
      self._ser = serial.Serial(port_name, baud_rate)
      self._ser_io = io.TextIOWrapper(io.BufferedRWPair(self._ser, self._ser, 1), newline = '\r', line_buffering = True)
      self._rl = ReadLine(self._ser)
   def get_port_handler(self):
      return self._ser

   def get_port_name(self):
      return self._port_name

   def get_port_baud_rate(self):
      return self._baud_rate

   def get_port_state(self):
      return self._ser.isOpen()

   def write_port(self, buffer):
      self._ser.write(buffer + "\r\n")

   def read_port(self):
      return self._rl.readline()
      #try: 
      #   return (self._ser_io.readline().decode("utf-8")).strip('\r\n')
      #except UnicodeDecodeError:
      #   print('UnicodeDecodeError')
    
class OMOR1MotorNode:
   def __init__(self):
      # Open serial port
      port_name = rospy.get_param('~port', '/dev/ttyMotor')
      baud_rate = rospy.get_param('~baud', 115200)
      self.odom_mode = rospy.get_param("~odom_mode", "wheel_only")
      self.port_handler = PortHandler(port_name, baud_rate)
      
      # Set packet handler
      self.packet_write_handler = PacketWriteHandler(self.port_handler)
      self.packet_write_handler.stop_peen()
      self.packet_write_handler.stop_callback()
      self.packet_write_handler.write_init_odometry()
      self.packet_write_handler.write_register(0, 'QENCOD')
      self.packet_write_handler.write_register(1, 'QODO')
      self.packet_write_handler.write_register(2, 'QDIFFV')
      self.packet_write_handler.write_register(3, '0') # 'QVW'
      self.packet_write_handler.write_register(4, '0') # 'QRPM'
      self.packet_write_handler.write_periodic_query_value(20)
      self.packet_write_handler.write_periodic_query_enable(1)

      self.packet_read_handler = PacketReadHandler(self.port_handler)

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
      rospy.loginfo('Serial port: %s', self.port_handler.get_port_name())

      # subscriber
      rospy.Subscriber("cmd_vel", Twist, self.cbSubCmdVelTMsg, queue_size=1)        # command velocity data subscriber
      rospy.Subscriber("imu", Imu, self.cbSubIMUTMsg, queue_size=1)                 # imu data subscriber

      # publisher
      self.pub_joint_states = rospy.Publisher('joint_states', JointState, queue_size=10)
      self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=10)
      self.odom_broadcaster = TransformBroadcaster()

      rospy.Service('reset_odom', ResetOdom, self.reset_odom_handle)
      
      # timer
      rospy.Timer(rospy.Duration(0.01), self.cbTimerUpdateDriverData) # 10 hz update
      self.odom_pose.timestamp = rospy.Time.now().to_nsec()
      self.reset_odometry()

      rospy.on_shutdown(self.__del__)

   def reset_odometry(self):
      self.is_enc_offset_set = False
      self.is_imu_offset_set = False

      self.joint.joint_pos = [0.0, 0.0]
      self.joint.joint_vel = [0.0, 0.0]

   def cbTimerUpdateDriverData(self, event):
      self.packet_read_handler.read_packet()

      lin_vel_x = self.packet_read_handler.get_base_velocity()[0]
      ang_vel_z = self.packet_read_handler.get_base_velocity()[1]

      odom_left_wheel = float(self.packet_read_handler.get_wheel_odom()[0])
      odom_right_wheel = float(self.packet_read_handler.get_wheel_odom()[1])

      rpm_left_wheel = int(self.packet_read_handler.get_wheel_rpm()[0])
      rpm_right_wheel = int(self.packet_read_handler.get_wheel_rpm()[1])

      lin_vel_left_wheel = int(self.packet_read_handler.get_wheel_velocity()[0])
      lin_vel_right_wheel = int(self.packet_read_handler.get_wheel_velocity()[1])

      enc_left_now = self.packet_read_handler.get_wheel_encoder()[0]
      enc_right_now = self.packet_read_handler.get_wheel_encoder()[1]
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

      self.packet_write_handler.write_wheel_velocity(angular_speed_left_wheel * self.config.wheel_radius * 1000, angular_speed_right_wheel * self.config.wheel_radius * 1000)

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

if __name__ == '__main__':
    rospy.init_node('omo_r1_motor_node')
    node = OMOR1MotorNode()
    node.main()