echo "initialize robot"

echo "alias connect_1='ssh r1@192.168.<ip1.ip1>'" >> ~/.bashrc
echo "alias connect_2='ssh r1@192.168.<ip2.ip2>'" >> ~/.bashrc

echo "alias ready_robot='roslaunch omo_r1_bringup ready_robot.launch set_id:=1 set_port:=/dev/ttyUSB0 set_lift_port:=/dev/ttyUSB1'" >> ~/.bashrc
echo "alias go_lift_high='rostopic pub -r 1 /lift_pose std_msgs/Int32 \"data: 600\"'" >> ~/.bashrc
echo "alias go_lift_middel='rostopic pub -r 1 /lift_pose std_msgs/Int32 \"data: 300\"'" >> ~/.bashrc
echo "alias go_lift_bottom='rostopic pub -r 1 /lift_pose std_msgs/Int32 \"data: 100\"'" >> ~/.bashrc

echo "alias ready_key='roslaunch omo_r1_teleop omo_r1_teleop_key.launch'" >> ~/.bashrc

source ~/.bashrc