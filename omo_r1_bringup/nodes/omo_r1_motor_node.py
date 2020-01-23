#!/usr/bin/env python

"""omo_r1_motor_node.py: ROS driver for Omorobot R1"""
# For more information, please visit our website www.omorobot.com
# Want to discuss with developers using our robots? Please visit our forum website at http://omorobot1.synology.me
# Also note that this software is for experimental and subject to change
# without any notifications.
__license__ = "MIT"
__version__ = "0.1.3"
__status__ = "Experimental"
'''
## License
The MIT License (MIT)
R1 and R1 mini driver for ROS: an open source platform for driving a robot with ROS.
Copyright (C) 2019  OMOROBOT Inc
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
'''

# modified by Bishop Pearson

import sys
import rospy
import serial
import io
import numpy as np
import math
import os

from time import sleep
from std_msgs.msg import UInt8, Int8, Int16, Float64, Float32
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from omo_r1_bringup.msg import R1MotorStatusLR, R1MotorStatus

from copy import copy, deepcopy
from sensor_msgs.msg import Imu
# from sensor_msgs.msg import Joy

from geometry_msgs.msg import Twist, TwistWithCovariance, Pose, Point, Vector3, Quaternion
from tf.broadcaster import TransformBroadcaster
from nav_msgs.msg import Odometry
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

class ArrowCon:
   setFwd = 0           # 1:Fwd, -1: Rev
   setRot = 0           # 1:CCW(Turn Left), -1: CW(Turn Right)
   targetOdo_L = 0      # Odometry target
   targetOdo_R = 0
   isFinished = True    # True: If arrow motion is completed
   cnt = 0
   
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

class Robot:
   rospy.init_node('omo_r1_motor_node', anonymous=True)
   # fetch /global parameters
   param_port = rospy.get_param('~port')
   param_baud = rospy.get_param('~baud')
   # param_joy_en = rospy.get_param('~joy_enable')
   # print('PARAM JOY_ENABLE:')
   # print(param_joy_en)
   # Open Serial port with parameter settings
   ser = serial.Serial(param_port, param_baud)
   #ser = serial.Serial('/dev/ttyS0', 115200) #For raspberryPi
   ser_io = io.TextIOWrapper(io.BufferedRWPair(ser, ser, 1),
                           newline = '\r',
                           line_buffering = True)
   config = VehicleConfig()
   odom_pose = OdomPose()
   odom_vel = OdomVel()
   joyAxes = []
   joyButtons = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]    # Buttons 15
   joyDeadband = 0.15
   exp = 0.3            # Joystick expo setting
   # if param_joy_en == 1:
   #    isAutoMode = False
   #    print "In Manual mode"
   # else :
   isAutoMode = True
   print "In Auto mode"
   isArrowMode = False  # Whether to control robo with arrow key or not
   arrowCon = ArrowCon
   
   #initialize data
   cmd = Command
   enc_L = 0.0          # Left wheel encoder count from QENCOD message
   enc_R = 0.0          # Right wheel encoder count from QENCOD message
   enc_L_prev = 0.0
   enc_R_prev = 0.0
   enc_offset_left = 0.0
   enc_offset_right = 0.0
   is_enc_offset_set = False
   odo_L = 0.0          # Left Wheel odometry returned from QODO message
   odo_R = 0.0          # Right Wheel odometry returned from QODO message
   RPM_L = 0.0          # Left Wheel RPM returned from QRPM message
   RPM_R = 0.0          # Right Wheel RPM returned from QRPM message
   speed_left_wheel = 0.0         # Left Wheel speed returned from QDIFF message
   speed_right_wheel = 0.0         # Reft Wheel speed returned from QDIFF message
   vel = 0.0            # Velocity returned from CVW command
   rot = 0.0            # Rotational speed returned from CVR command
   def __init__(self):
      ## Set vehicle specific configurations
      self.config.wheel_separation = 0.591        # Apply vehicle width for R1 version
      self.config.wheel_radius = 0.11       # Apply wheel radius for R1 version
      self.config.max_linear_speed_wheel = 1200.0  # Maximum speed can be applied to each wheel (mm/s)
      self.config.max_lin_vel_x = 0.6        # Limited speed (m/s)
      self.config.max_ang_vel_z = 0.1
      self.config.encoder.direction = 1.0
      self.config.encoder.PPR = 1000
      self.config.encoder.GearRatio = 15

      self.is_enc_offset_set = False
      self.is_imu_offset_set = False
      self.orientation = [0.0, 0.0, 0.0, 0.0]
      self.last_theta = 0.0
      self.enc_left_prev, self.enc_right_prev = 0.0, 0.0
         
      print('Wheel Track:{:.2f}m, Radius:{:.3f}m'.format(self.config.wheel_separation, self.config.wheel_radius))
      self.config.BodyCircumference = self.config.wheel_separation * math.pi
      print('Platform Rotation arc length: {:04f}m'.format(self.config.BodyCircumference))
      self.config.WheelCircumference = self.config.wheel_radius * 2 * math.pi
      print('Wheel circumference: {:04f}m'.format(self.config.WheelCircumference))
      self.config.encoder.Step = self.config.WheelCircumference / (self.config.encoder.PPR * self.config.encoder.GearRatio * 4)
      print('Encoder step: {:04f}m/pulse'.format(self.config.encoder.Step))
      self.config.encoder.PPWheelRev = self.config.WheelCircumference / self.config.encoder.Step
      print('Encoder pulses per wheel rev: {:.2f} pulses/rev'.format(self.config.encoder.PPWheelRev))
      print('Serial port:'+self.ser.name)         # Print which port was really used
      self.joyAxes = [0,0,0,0,0,0,0,0]
      self.joyButtons = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
      # Configure data output
      if self.ser.isOpen():
         print("Serial Open")
         self.resetODO()
         sleep(0.05)
         self.reset_odometry()
         self.setREGI(0,'QENCOD')
         sleep(0.05)
         self.setREGI(1,'QODO')
         sleep(0.05)
         self.setREGI(2,'QDIFFV')
         sleep(0.05)
	 self.setREGI(3,'0')
         sleep(0.05)
	 self.setREGI(4,'0')
         #self.setREGI(3,'QVW')
         #sleep(0.05)
         #self.setREGI(4,'QRPM')
         sleep(0.05)
         self.setSPERI(20)
         sleep(0.05)
         self.setPEEN(1)
         sleep(0.05)
         
      self.reset_odometry()   
      # Subscriber
      rospy.Subscriber("cmd_vel", Twist, self.callbackCmdVel)
      rospy.Subscriber("imu", Imu, self.callbackIMU)

      # publisher
      self.pub_enc_left = rospy.Publisher('motor/encoder/left', Float64, queue_size=10)
      self.pub_enc_right = rospy.Publisher('motor/encoder/right', Float64, queue_size=10)
      self.pub_motor_status = rospy.Publisher('motor/status', R1MotorStatusLR, queue_size=10)
      self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=10)
      self.odom_broadcaster = TransformBroadcaster()
      
      rate = rospy.Rate(rospy.get_param('~hz', 30)) # 30hz
      # rospy.Timer(rospy.Duration(0.05), self.joytimer)
      rospy.Timer(rospy.Duration(0.01), self.serReader)
      self.odom_pose.timestamp = rospy.Time.now().to_nsec()

      while not rospy.is_shutdown():
         if self.cmd.isAlive == True:
             self.cmd.cnt += 1
             if self.cmd.cnt > 1000:         #Wait for about 3 seconds 
                 self.cmd.isAlive = False
                 self.isAutoMode = False
         rate.sleep()
         
      self.ser.close()

   def serReader(self, event):
      reader = self.ser_io.readline()
      if reader:
         packet = reader.split(",")
         try:
            header = packet[0].split("#")[1]
            if header.startswith('QVW'):
               self.vel = int(packet[1])
               self.rot = int(packet[2])
            elif header.startswith('QENCOD'):
               enc_left_now = int(packet[1])
               enc_right_now = int(packet[2])
               if self.is_enc_offset_set == False:
                  self.enc_offset_left = enc_left_now
                  self.enc_offset_right = enc_right_now
                  self.is_enc_offset_set = True
               enc_left_tot = enc_left_now * self.config.encoder.direction - self.enc_offset_left
               enc_right_tot = enc_right_now * self.config.encoder.direction - self.enc_offset_right
               # self.pub_enc_left.publish(Float64(data=enc_left_tot))
               # self.pub_enc_right.publish(Float64(data=enc_right_tot))
               self.updatePose(enc_left_tot, enc_right_tot)
               #print('Encoder:L{:.2f}, R:{:.2f}'.format(enc_left_tot, enc_right_tot))
            elif header.startswith('QODO'):
               self.odo_L = float(packet[1]) * self.config.encoder.direction
               self.odo_R = float(packet[2]) * self.config.encoder.direction
               # print('Odo:{:.2f}mm,{:.2f}mm'.format(self.odo_L, self.odo_R))
            elif header.startswith('QRPM'):
               self.RPM_L = int(packet[1])
               self.RPM_R = int(packet[2])
               #print('RPM:{:.2f}mm,{:.2f}mm'.format(self.RPM_L, self.RPM_R))
            elif header.startswith('QDIFFV'):
               self.speed_left_wheel = int(packet[1])
               self.speed_right_wheel = int(packet[2])
               # print('SPD:{:.2f}mm,{:.2f}mm'.format(self.speed_left_wheel, self.speed_right_wheel))
         except:
            pass
         # status_left = R1MotorStatus(low_voltage = 0, overloaded = 0, power = 0,
         #                   encoder = self.enc_left, RPM = self.RPM_L, ODO = self.odo_L, speed = self.speed_left_wheel)
         # status_right = R1MotorStatus(low_voltage = 0, overloaded = 0, power = 0,
         #                   encoder = self.enc_right, RPM = self.RPM_R, ODO = self.odo_R, speed = self.speed_right_wheel)
         # self.pub_motor_status.publish(R1MotorStatusLR(header=Header(stamp=rospy.Time.now()),
         #                   Vspeed = self.vel, Vomega = self.rot,
         #                   left=status_left, right=status_right))        

   def callbackIMU(self, imu_msg):
      self.orientation[0] = imu_msg.orientation.x
      self.orientation[1] = imu_msg.orientation.y
      self.orientation[2] = imu_msg.orientation.z
      self.orientation[3] = imu_msg.orientation.w

   def callbackCmdVel(self, cmd_vel_msg):
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

      self.sendCDIFFVcontrol(angular_speed_left_wheel * self.config.wheel_radius * 1000, angular_speed_right_wheel * self.config.wheel_radius * 1000)

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

      odom_orientation_quat = quaternion_from_euler(0, 0, self.odom_pose.theta)
      self.odom_broadcaster.sendTransform((self.odom_pose.x, self.odom_pose.y, 0.), odom_orientation_quat, timestamp_now, 'base_link', 'odom')
      
      odom = Odometry()
      odom.header.stamp = timestamp_now
      odom.header.frame_id = 'odom'
      odom.child_frame_id = 'base_link'
      odom.pose.pose = Pose(Point(self.odom_pose.x, self.odom_pose.y, 0.), Quaternion(*odom_orientation_quat))
      odom.twist.twist = Twist(Vector3(self.odom_vel.x, self.odom_vel.y, 0), Vector3(0, 0, self.odom_vel.w))
      
      # #print "x:{:.2f} y:{:.2f} theta:{:.2f}".format(pose.x, pose.y, pose.theta*180/math.pi)      
      self.odom_pub.publish(odom)
      # return pose

   def fnTransVelToWheelSpeed(self, lin_vel_x, ang_vel_z):
      angular_speed_left_wheel = (lin_vel_x - (self.config.wheel_separation / 2.0) * ang_vel_z) / self.config.wheel_radius
      angular_speed_right_wheel = (lin_vel_x + (self.config.wheel_separation / 2.0) * ang_vel_z) / self.config.wheel_radius

      return angular_speed_left_wheel, angular_speed_right_wheel

   def sendCVWcontrol(self, config, V_mm_s, W_mrad_s):
      """ Set Vehicle velocity and rotational speed """
      if V_mm_s > config.max_lin_vel_x :
         V_mm_s = config.max_lin_vel_x
      elif V_mm_s < -config.max_lin_vel_x :
         V_mm_s = -config.max_lin_vel_x
      if W_mrad_s > config.max_ang_vel_z :
         W_mrad_s = config.max_ang_vel_z
      elif W_mrad_s < -config.max_ang_vel_z:
         W_mrad_s = -config.max_ang_vel_z
      # Make a serial message to be sent to motor driver unit
      cmd = '$CVW,{:.0f},{:.0f}'.format(V_mm_s, W_mrad_s)
      print cmd
      if self.ser.isOpen():
         print cmd
         self.ser.write(cmd+"\r"+"\n")

   def sendCDIFFVcontrol(self, linear_speed_left_wheel, linear_speed_right_wheel):
      
      if linear_speed_left_wheel > self.config.max_linear_speed_wheel :
         linear_speed_left_wheel = self.config.max_linear_speed_wheel
      elif linear_speed_left_wheel < -self.config.max_linear_speed_wheel :
         linear_speed_left_wheel = -self.config.max_linear_speed_wheel

      if linear_speed_right_wheel > self.config.max_linear_speed_wheel :
         linear_speed_right_wheel = self.config.max_linear_speed_wheel
      elif linear_speed_right_wheel < -self.config.max_linear_speed_wheel :
         linear_speed_right_wheel = -self.config.max_linear_speed_wheel
         
      # Make a serial message to be sent to motor driver unit
      cmd = '$CDIFFV,{:.0f},{:.0f}'.format(linear_speed_left_wheel, linear_speed_right_wheel)
      if self.ser.isOpen():
         self.ser.write(cmd+"\r"+"\n")
                    
   def setREGI(self, param1, param2):
      msg = "$SREGI,"+str(param1)+','+param2
      self.ser.write(msg+"\r"+"\n")
        
   def setSPERI(self, param):
      msg = "$SPERI,"+str(param)
      self.ser.write(msg+"\r"+"\n")

   def setPEEN(self, param):
      msg = "$SPEEN,"+str(param)
      self.ser.write(msg+"\r"+"\n")
     
   def resetODO(self):
      self.ser.write("$SODO\r\n")
        
if __name__ == '__main__':
   try:
      Robot()
   except rospy.ROSInterruptException:
      pass
    

