#!/usr/bin/env python

# original source code -> https://github.com/omorobot/omoros
# modified by Bishop Pearson

import sys
import rospy
import serial
import io
import math
import os
from time import sleep

from omo_r1_bringup.msg import R1MotorStatusLR, R1MotorStatus
from std_msgs.msg import UInt8, Int8, Int16, Float64, Float32
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TwistWithCovariance, Pose, Point, Vector3, Quaternion
from tf.broadcaster import TransformBroadcaster
from tf.transformations import quaternion_from_euler, euler_from_quaternion

class OdomPose(object):
   x = 0.0
   y = 0.0
   theta = 0.0
   timestamp = 0

class OdomVel(object):
   x = 0.0
   y = 0.0
   w = 0.0

class Encoder(object):
   direction = 1.0
   PPR = 0
   GearRatio = 0
   Step = 0
   PPWheelRev = 0
   
class VehicleConfig(object):
   BodyCircumference = 0   # circumference length of robot for spin in place
   WheelCircumference = 0
   WIDTH = 0.0             # Default Vehicle width in mm
   WHEEL_R = 0.0           # Wheel radius
   WHEEL_MAXV = 0.0        # Maximum wheel speed (mm/s)
   V_Limit = 0             # Speed limit for vehicle (m/s)
   W_Limit = 0             # Rotational Speed limit for vehicle (rad/s)
   encoder = Encoder()
   
class Command:
   isAlive = False   # Set to True if subscrived command message has been received
   mode = 0          # Command mode (0:vel, rot) <--> (1:speed_left_wheel, speed_right_wheel)
   speed = 0.0       # Speed mm/s
   deg_sec = 0.0     # Rotational speed deg/s
   speed_left_wheel = 0.0      # Left Wheel speed mm/s
   speed_right_wheel = 0.0      # Right wheel speed mm/s

class MCUData:
   # velocity
   
   # encoder

   # odometry

   # rpm

   # wheel velocity
   def __init__(self):
      a = 0

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

   def read_base_velocity(self):
      return self._vel
   
   def read_wheel_encoder(self):
      return self._enc

   def read_wheel_odom(self):
      return self._wodom

   def read_wheel_rpm(self):
      return self._rpm
   
   def read_wheel_velocity(self):
      return self._wvel

   def read_packet(self):
      if self._ph.get_port_state() == True:
         whole_packet = self._ph.read_port()
         
         if whole_packet:
            packet = whole_packet.split(",")
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


class PortHandler():
   _port_name = None
   _baud_rate = None
   _ser = None
   _ser_io = None

   def __init__(self, port_name, baud_rate):
      self._port_name = port_name
      self._baud_rate = baud_rate

      self.set_port_handler(port_name, baud_rate)

   def __del__(self):
      self._ser.close()

   def set_port_handler(self, port_name, baud_rate):
      self._ser = serial.Serial(port_name, baud_rate)

      self._ser_io = io.TextIOWrapper(io.BufferedRWPair(self._ser, self._ser, 1), newline = '\r', line_buffering = True)

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
      return self._ser_io.readline()
    


class OMOR1MotorNode:
   def __init__(self):
      # Init data
      self.enc_L, self.enc_R = 0.0, 0.0          # Left / Right wheel encoder count from QENCOD message
      self.enc_left_prev, self.enc_right_prev = 0.0, 0.0   
      self.enc_offset_left, self.enc_offset_right = 0.0, 0.0
      self.odo_L, self.odo_R = 0.0, 0.0          # Left / Right wheel odometry returned from QODO message
      self.RPM_L, self.RPM_R = 0.0, 0.0          # Left / Right wheel RPM returned from QRPM message
      self.speed_left_wheel, self.speed_right_wheel = 0.0, 0.0         # Left / Right wheel speed returned from QDIFF message

      self.vel = 0.0            # Velocity returned from CVW command
      self.rot = 0.0            # Rotational speed returned from CVR command

      self.is_enc_offset_set = False


      self.config = VehicleConfig()
      self.odom_pose = OdomPose()
      self.odom_vel = OdomVel()     

      port_name = rospy.get_param('~port', '/dev/ttyMotor')
      baud_rate = rospy.get_param('~baud', 115200)


      # Open serial port
      self.port_handler = PortHandler(port_name, baud_rate)
      
      # Set packet handler
      self.packet_write_handler = PacketWriteHandler(self.port_handler)
      self.packet_read_handler = PacketReadHandler(self.port_handler)

      # Set vehicle specific configurations
      self.config.wheel_separation = 0.591        # Apply vehicle width for R1 version
      self.config.wheel_radius = 0.11       # Apply wheel radius for R1 version
      self.config.max_linear_speed_wheel = 1200.0  # Maximum speed can be applied to each wheel (mm/s)
      self.config.max_lin_vel_x = 1.2        # Limited speed (m/s)
      self.config.max_ang_vel_z = 1.0
      self.config.encoder.direction = 1.0
      self.config.encoder.PPR = 1000
      self.config.encoder.GearRatio = 15

      self.is_enc_offset_set = False
      self.is_imu_offset_set = False
      self.orientation = [0.0, 0.0, 0.0, 0.0]
      self.last_theta = 0.0

      self.config.BodyCircumference = self.config.wheel_separation * math.pi
      self.config.WheelCircumference = self.config.wheel_radius * 2 * math.pi
      self.config.encoder.Step = self.config.WheelCircumference / (self.config.encoder.PPR * self.config.encoder.GearRatio * 4)
      self.config.encoder.PPWheelRev = self.config.WheelCircumference / self.config.encoder.Step

      rospy.loginfo('Wheel Track:{:.2f}m, Radius:{:.3f}m'.format(self.config.wheel_separation, self.config.wheel_radius))
      rospy.loginfo('Platform Rotation arc length: {:04f}m'.format(self.config.BodyCircumference))
      rospy.loginfo('Wheel circumference: {:04f}m'.format(self.config.WheelCircumference))
      rospy.loginfo('Encoder step: {:04f}m/pulse'.format(self.config.encoder.Step))
      rospy.loginfo('Encoder pulses per wheel rev: {:.2f} pulses/rev'.format(self.config.encoder.PPWheelRev))
      rospy.loginfo('Serial port: %s', self.port_handler.get_port_name())

      # Configure data output
      self.packet_write_handler.write_init_odometry()
      self.packet_write_handler.write_register(0, 'QENCOD')
      self.packet_write_handler.write_register(1, 'QODO')
      self.packet_write_handler.write_register(2, 'QDIFFV')
      self.packet_write_handler.write_register(3, '0') # 'QVW'
      self.packet_write_handler.write_register(4, '0') # 'QRPM'
      self.packet_write_handler.write_periodic_query_enable(1)
      self.packet_write_handler.write_periodic_query_value(20)
         
      self.reset_odometry()

      self.count = 0

      # subscriber
      rospy.Subscriber("cmd_vel", Twist, self.cbSubCmdVelTMsg, queue_size=1)      # command velocity data subscriber
      rospy.Subscriber("imu", Imu, self.cbSubIMUTMsg, queue_size=1)              # imu data subscriber

      # publisher
      self.pub_motor_status = rospy.Publisher('motor/status', R1MotorStatusLR, queue_size=10)
      self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=10)
      self.odom_broadcaster = TransformBroadcaster()
      
      rate = rospy.Rate(rospy.get_param('~hz', 30)) # 30hz
      rospy.Timer(rospy.Duration(0.01), self.serReader)
      self.odom_pose.timestamp = rospy.Time.now().to_nsec()


      rospy.on_shutdown(self.__del__)

   def serReader(self, event):
      self.packet_read_handler.read_packet()

      self.vel = self.packet_read_handler.read_base_velocity()[0]
      self.rot = self.packet_read_handler.read_base_velocity()[1]

      enc_left_now = self.packet_read_handler.read_wheel_encoder()[0]
      enc_right_now = self.packet_read_handler.read_wheel_encoder()[1]
      if self.is_enc_offset_set == False:
         self.enc_offset_left = enc_left_now
         self.enc_offset_right = enc_right_now
         self.is_enc_offset_set = True
      enc_left_tot = enc_left_now * self.config.encoder.direction - self.enc_offset_left
      enc_right_tot = enc_right_now * self.config.encoder.direction - self.enc_offset_right

      self.odo_L = float(self.packet_read_handler.read_wheel_odom()[0]) * self.config.encoder.direction
      self.odo_R = float(self.packet_read_handler.read_wheel_odom()[1]) * self.config.encoder.direction

      self.RPM_L = int(self.packet_read_handler.read_wheel_rpm()[0])
      self.RPM_R = int(self.packet_read_handler.read_wheel_rpm()[1])

      self.speed_left_wheel = int(self.packet_read_handler.read_wheel_velocity()[0])
      self.speed_right_wheel = int(self.packet_read_handler.read_wheel_velocity()[1])

      self.updatePose(enc_left_tot, enc_right_tot)

         # status_left = R1MotorStatus(low_voltage = 0, overloaded = 0, power = 0,
         #                   encoder = self.enc_left, RPM = self.RPM_L, ODO = self.odo_L, speed = self.speed_left_wheel)
         # status_right = R1MotorStatus(low_voltage = 0, overloaded = 0, power = 0,
         #                   encoder = self.enc_right, RPM = self.RPM_R, ODO = self.odo_R, speed = self.speed_right_wheel)
         # self.pub_motor_status.publish(R1MotorStatusLR(header=Header(stamp=rospy.Time.now()),
         #                   Vspeed = self.vel, Vomega = self.rot,
         #                   left=status_left, right=status_right))        

   def cbSubIMUTMsg(self, imu_msg):
      self.orientation[0] = imu_msg.orientation.x
      self.orientation[1] = imu_msg.orientation.y
      self.orientation[2] = imu_msg.orientation.z
      self.orientation[3] = imu_msg.orientation.w

   def cbSubCmdVelTMsg(self, cmd_vel_msg):
      lin_vel_x = cmd_vel_msg.linear.x
      ang_vel_z = cmd_vel_msg.angular.z

      if lin_vel_x > self.config.max_lin_vel_x:
         lin_vel_x = self.config.max_lin_vel_x
      elif lin_vel_x < -self.config.max_lin_vel_x:
         lin_vel_x = -self.config.max_lin_vel_x

      if ang_vel_z > self.config.max_ang_vel_z:
         ang_vel_z = self.config.max_ang_vel_z
      elif ang_vel_z < -self.config.max_ang_vel_z:
         ang_vel_z = -self.config.max_ang_vel_z

      angular_speed_left_wheel, angular_speed_right_wheel = self.fnTransVelToWheelSpeed(lin_vel_x, ang_vel_z)

      self.fnSendWheelVelocity(angular_speed_left_wheel * self.config.wheel_radius * 1000, angular_speed_right_wheel * self.config.wheel_radius * 1000)

   def reset_odometry(self):
      self.last_speed_left_wheel = 0.0
      self.last_speed_right_wheel = 0.0

   def updatePose(self, enc_left_tot, enc_right_tot):
      enc_left_diff = enc_left_tot - self.enc_left_prev
      enc_right_diff = enc_right_tot - self.enc_right_prev
      self.enc_left_prev = enc_left_tot
      self.enc_right_prev = enc_right_tot

      timestamp_now = rospy.Time.now()
      timestamp_now_nsec = timestamp_now.to_nsec()
      d_time = (timestamp_now_nsec - self.odom_pose.timestamp) / 1000000000.0
      self.odom_pose.timestamp = timestamp_now_nsec

      d_s = (enc_left_diff + enc_right_diff) * self.config.encoder.Step / 2.0

      euler = euler_from_quaternion((self.orientation[0], self.orientation[1], self.orientation[2], self.orientation[3]))
      theta = euler[2]

      # theta = math.atan2(self.orientation[1] * self.orientation[2] + self.orientation[0] * self.orientation[3],
      #                0.5 - self.orientation[2] * self.orientation[2] - self.orientation[3] * self.orientation[3])
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

      # print self.odom_pose.x, self.odom_pose.y, self.odom_pose.theta, self.odom_vel.x, self.odom_vel.w

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
      
      # #print "x:{:.2f} y:{:.2f} theta:{:.2f}".format(pose.x, pose.y, pose.theta*180/math.pi)      
      self.odom_pub.publish(odom)
      # return pose

   def fnTransVelToWheelSpeed(self, lin_vel_x, ang_vel_z):
      angular_speed_left_wheel = (lin_vel_x - (self.config.wheel_separation / 2.0) * ang_vel_z) / self.config.wheel_radius
      angular_speed_right_wheel = (lin_vel_x + (self.config.wheel_separation / 2.0) * ang_vel_z) / self.config.wheel_radius

      return angular_speed_left_wheel, angular_speed_right_wheel

   def fnSendBaseVelocity(self, config, V_mm_s, W_mrad_s):
      if V_mm_s > config.max_lin_vel_x :
         V_mm_s = config.max_lin_vel_x
      elif V_mm_s < -config.max_lin_vel_x :
         V_mm_s = -config.max_lin_vel_x

      if W_mrad_s > config.max_ang_vel_z :
         W_mrad_s = config.max_ang_vel_z
      elif W_mrad_s < -config.max_ang_vel_z:
         W_mrad_s = -config.max_ang_vel_z

      self.packet_write_handler.write_base_velocity(V_mm_s, W_mrad_s)

   def fnSendWheelVelocity(self, linear_speed_left_wheel, linear_speed_right_wheel):
      if linear_speed_left_wheel > self.config.max_linear_speed_wheel :
         linear_speed_left_wheel = self.config.max_linear_speed_wheel
      elif linear_speed_left_wheel < -self.config.max_linear_speed_wheel :
         linear_speed_left_wheel = -self.config.max_linear_speed_wheel

      if linear_speed_right_wheel > self.config.max_linear_speed_wheel :
         linear_speed_right_wheel = self.config.max_linear_speed_wheel
      elif linear_speed_right_wheel < -self.config.max_linear_speed_wheel :
         linear_speed_right_wheel = -self.config.max_linear_speed_wheel
         
      self.packet_write_handler.write_wheel_velocity(linear_speed_left_wheel, linear_speed_right_wheel)
                    
   def main(self):
      rospy.spin()

      # self.timerControlMotion.join()
      # self.timerCliActionStepSMsg.join()

   def __del__(self):
      print("terminating omo_r1_motor_node")
      rospy.loginfo("Shutting down. velocity will be 0")

      del self.packet_write_handler
      del self.packet_read_handler
      del self.port_handler

if __name__ == '__main__':
    rospy.init_node('omo_r1_motor_node')
    node = OMOR1MotorNode()
    node.main()