#! /usr/bin/env python
from __future__ import division
import rospy
#from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64
from std_msgs.msg import Int64
from math import sqrt
from math import pi
from math import acos
from math import cos
from math import sin

'''
def update_entrance_direction_info(data):
  data2 = data.data
  global entrance_direction_info
  #[x_centroid, orientation]
  entrance_direction_info = [data2[0], data2[1]-pi]
'''

def update_region(data):
  global region
  region = data.data

def update_x_centroid(data):
  global x_centroid
  x_centroid = data.data

def update_orientation(data):
  global line_orientation
  line_orientation = data.data

def update_lidar_orientation(data):
  global lidar_orientation
  lidar_orientation = data.data*pi/180

def update_region(data):
  region = data.data

#error positive when need to turn left
def entrance_acutation(e):
  global e_i
  global e_old
  #actuate motor with PID control based on the orientation error
  e_i = e_i + e
  e_d = e - e_old
  result = (e*p_gain + e_i*i_gain + e_d*d_gain)
  motor_cmd.data = [(7 - result), (7 + result)]
  pub.publish(motor_cmd)

  pub1.publish(suspension_cmd)
  pub2.publish(suspension_cmd)
  pub3.publish(suspension_cmd)
  pub4.publish(suspension_cmd)

  e_old = e
  #print('error: ', e)
  print('motor_cmd: ', motor_cmd.data)
  print('..')


#dummy initiation
motor_cmd = Float64MultiArray()
suspension_cmd = Float64()
car = [0, 0, 0]
region = 1
#entrance_direction_info = [0, 0, 0]
line_orientation = 0
lidar_orientation = 0
x_centroid = 960

#topic names
motor_cmd_topic_name = '/toy_car2/joint_vel_controller/command'
suspension_topic_front_left = '/toy_car2/front_left_suspension_controller/command'
suspension_topic_front_right = '/toy_car2/front_right_suspension_controller/command'
suspension_topic_rear_left = '/toy_car2/rear_left_suspension_controller/command'
suspension_topic_rear_right = '/toy_car2/rear_right_suspension_controller/command'


#publisher & subscriber
pub = rospy.Publisher(motor_cmd_topic_name, Float64MultiArray, queue_size=10)
pub1 = rospy.Publisher(suspension_topic_front_left, Float64, queue_size=10)
pub2 = rospy.Publisher(suspension_topic_front_right, Float64, queue_size=10)
pub3 = rospy.Publisher(suspension_topic_rear_left, Float64, queue_size=10)
pub4 = rospy.Publisher(suspension_topic_rear_right, Float64, queue_size=10)

rospy.Subscriber("/line/centroid", Int64, update_x_centroid)
rospy.Subscriber("/line/orientation", Float64, update_orientation)
rospy.Subscriber("/mapdata/mode", Float64, update_region)
rospy.Subscriber("/mapdata/enter_direction", Float64, update_lidar_orientation)

#node initiation
rospy.init_node("path_planning_entrance")


#suspension initiation
suspension_cmd.data = 0.03
pub1.publish(suspension_cmd)
pub2.publish(suspension_cmd)
pub3.publish(suspension_cmd)
pub4.publish(suspension_cmd)

#rate control
rate = rospy.Rate(20)

#parameters to adjust
[p_gain, i_gain, d_gain] = [5, 0, 0]
threshold = 0.3 #between 0 and 1
center = 960 #center x coordinate from image

e_old = 0
e_i = 0
stuck_list = []
while not rospy.is_shutdown():
  if region == 0:
    #[x_centroid, orientation] = entrance_direction_info
    if x_centroid < 0:
      print("No line detected. Using lidar")
      error = lidar_orientation
      print("lidar_orientation based error: ", error)
      entrance_acutation(error)
    else:
      if abs(x_centroid - center)/center > threshold:
        error = -(x_centroid - center)/center*(pi/2*1.5)
        print("Centroid based error: ", error)
        entrance_acutation(error)
      else:
        error = line_orientation
        print("Line_orientation based error: ", error)
        entrance_acutation(error)
    rate.sleep()
  else:
    rate.sleep()

