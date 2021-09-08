import serial
import rospy
import io
from time import sleep

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

class PacketHandler:
    def __init__(self):
        self._vel = [0.0, 0.0]
        self._enc = [0.0, 0.0]
        self._wodom = [0.0, 0.0]
        self._rpm = [0.0, 0.0]
        self._wvel = [0.0, 0.0]
        port_name = rospy.get_param('~port', '/dev/ttyMotor')
        baud_rate = rospy.get_param('~baud', 115200)
        self._ser = serial.Serial(port_name, baud_rate)
        self._ser_io = io.TextIOWrapper(io.BufferedRWPair(self._ser, self._ser, 1), newline = '\r', line_buffering = True)
        self._rl = ReadLine(self._ser)

        self.incomming_info = ['QENOCD', 'QODO', 'QDIFFV']

        rospy.loginfo('Serial port: %s', port_name)
        rospy.loginfo('Serial baud rate: %s', baud_rate)

    def set_periodic_info(self, param1):
        for idx, each in enumerate(self.incomming_info):
            #print("$cREGI," + str(idx) + "," + each)
            self.write_port("$SREGI," + str(idx) + "," + each)

        self.write_port("$SPERI," + str(param1))
        sleep(0.01)
        self.write_port("$SPEEN,1")

    def get_port_state(self):
        return self._ser.isOpen()
        
    def read_port(self):
        return self._rl.readline()

    def read_packet(self):
        if self.get_port_state() == True:
            whole_packet = self.read_port()
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

    def write_odometry_reset(self):
        self.write_port("$SODO")
        sleep(0.05)

    def write_periodic_query_value(self, param):
        self.write_port("$SPERI," + str(param))
        sleep(0.05)

    def write_periodic_query_enable(self, param):
        self.write_port("$SPEEN," + str(param))
        sleep(0.05)

    def write_init_odometry(self):
        self.write_port("$SODO")
        sleep(0.05)

    def write_base_velocity(self, lin_vel, ang_vel):
        # lin_vel : mm/s, ang_vel : mrad/s
        self.write_port('$CVW,{:.0f},{:.0f}'.format(lin_vel, ang_vel))

    def write_wheel_velocity(self, wheel_l_lin_vel, wheel_r_lin_vel):
        self.write_port('$CDIFFV,{:.0f},{:.0f}'.format(wheel_l_lin_vel, wheel_r_lin_vel))

    def write_port(self, buffer):
        if self.get_port_state() == True:
            self._ser.write(buffer+"\r\n")
