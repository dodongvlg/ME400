#! /usr/bin/env python
from __future__ import division
import rospy
#from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from numpy import linspace
from math import sqrt
from math import pi
from math import acos
from math import cos
from math import sin

def find_length(start, end):
  return sqrt((start[0]-end[0])**2+(start[1]-end[1])**2)

def update_robot_pos(data):
  data2 = data.pose[17]
  orientation_list = [data2.orientation.x, data2.orientation.y, data2.orientation.z, data2.orientation.w]
  (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

  global car
  car = [data2.position.x, data2.position.y, yaw + pi]


#dummy
motor_cmd = Float64MultiArray()
suspension_cmd = Float64()
car = [0, 0, 0]

#parameters to adjust
spd = 7
motor_cmd_topic_name = '/toy_car2/joint_vel_controller/command'
[p_gain, i_gain, d_gain] = [6, 0, 0]
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

rospy.Subscriber("/gazebo/model_states", ModelStates, update_robot_pos)


#node initiation
rospy.init_node("path_planning")

#suspension initiation
suspension_cmd.data = 0.03
pub1.publish(suspension_cmd)
pub2.publish(suspension_cmd)
pub3.publish(suspension_cmd)
pub4.publish(suspension_cmd)

#rate control
rate = rospy.Rate(10)

path_list = [[0.500000, 2.000000],[0.500000, 1.500000],[0.500000, 1.000000],[0.500000, 1.000000],[0.510000, 0.900501],[0.520000, 0.860000],[0.530000, 0.829413],[0.540000, 0.804041],[0.550000, 0.782055],[0.560000, 0.762513],[0.570000, 0.744853],[0.580000, 0.728707],[0.590000, 0.713818],[0.600000, 0.700000],[0.610000, 0.687110],[0.620000, 0.675038],[0.630000, 0.663697],[0.640000, 0.653013],[0.650000, 0.642929],[0.660000, 0.633394],[0.670000, 0.624367],[0.680000, 0.615813],[0.690000, 0.607699],[0.700000, 0.600000],[0.710000, 0.592692],[0.720000, 0.585754],[0.730000, 0.579167],[0.740000, 0.572917],[0.750000, 0.566987],[0.750000, 0.566987],[0.800000, 0.541742],[0.850000, 0.523030],[0.900000, 0.510102],[0.950000, 0.502506],[1.000000, 0.500000],[1.050000, 0.502506],[1.100000, 0.510102],[1.150000, 0.523030],[1.200000, 0.541742],[1.250000, 0.566987],[1.250000, 0.566987],[1.260000, 0.572917],[1.270000, 0.579167],[1.280000, 0.585754],[1.290000, 0.592692],[1.300000, 0.600000],[1.310000, 0.607699],[1.320000, 0.615813],[1.330000, 0.624367],[1.340000, 0.633394],[1.350000, 0.642929],[1.360000, 0.653013],[1.370000, 0.663697],[1.380000, 0.675038],[1.390000, 0.687110],[1.400000, 0.700000],[1.410000, 0.713818],[1.420000, 0.728707],[1.430000, 0.744853],[1.440000, 0.762513],[1.450000, 0.782055],[1.460000, 0.804041],[1.470000, 0.829413],[1.480000, 0.860000],[1.490000, 0.900501],[1.500000, 1.000000],[1.500000, 2.000000],[1.510000, 2.099499],[1.520000, 2.140000],[1.530000, 2.170587],[1.540000, 2.195959],[1.550000, 2.217945],[1.560000, 2.237487],[1.570000, 2.255147],[1.580000, 2.271293],[1.590000, 2.286182],[1.600000, 2.300000],[1.610000, 2.312890],[1.620000, 2.324962],[1.630000, 2.336303],[1.640000, 2.346987],[1.650000, 2.357071],[1.660000, 2.366606],[1.670000, 2.375633],[1.680000, 2.384187],[1.690000, 2.392301],[1.700000, 2.400000],[1.710000, 2.407308],[1.720000, 2.414246],[1.730000, 2.420833],[1.740000, 2.427083],[1.750000, 2.433013],[1.750000, 2.433013],[1.800000, 2.458258],[1.850000, 2.476970],[1.900000, 2.489898],[1.950000, 2.497494],[2.000000, 2.500000],[2.050000, 2.497494],[2.100000, 2.489898],[2.150000, 2.476970],[2.200000, 2.458258],[2.250000, 2.433013],[2.250000, 2.433013],[2.260000, 2.427083],[2.270000, 2.420833],[2.280000, 2.414246],[2.290000, 2.407308],[2.300000, 2.400000],[2.310000, 2.392301],[2.320000, 2.384187],[2.330000, 2.375633],[2.340000, 2.366606],[2.350000, 2.357071],[2.360000, 2.346987],[2.370000, 2.336303],[2.380000, 2.324962],[2.390000, 2.312890],[2.400000, 2.300000],[2.410000, 2.286182],[2.420000, 2.271293],[2.430000, 2.255147],[2.440000, 2.237487],[2.450000, 2.217945],[2.460000, 2.195959],[2.470000, 2.170587],[2.480000, 2.140000],[2.490000, 2.099499],[2.500000, 2.000000],[2.500000, 1.000000],[2.510000, 0.900501],[2.520000, 0.860000],[2.530000, 0.829413],[2.540000, 0.804041],[2.550000, 0.782055],[2.560000, 0.762513],[2.570000, 0.744853],[2.580000, 0.728707],[2.590000, 0.713818],[2.600000, 0.700000],[2.610000, 0.687110],[2.620000, 0.675038],[2.630000, 0.663697],[2.640000, 0.653013],[2.650000, 0.642929],[2.660000, 0.633394],[2.670000, 0.624367],[2.680000, 0.615813],[2.690000, 0.607699],[2.700000, 0.600000],[2.710000, 0.592692],[2.720000, 0.585754],[2.730000, 0.579167],[2.740000, 0.572917],[2.750000, 0.566987],[2.750000, 0.566987],[2.800000, 0.541742],[2.850000, 0.523030],[2.900000, 0.510102],[2.950000, 0.502506],[3.000000, 0.500000]]

e_old = 0
e_i = 0
k = 0

end = path_list[0]
start = [car[0], car[1]]
while not rospy.is_shutdown():
  while find_length(start, end) > 0.2:
    start = [car[0], car[1]]
    end = path_list[k]
    print('end: ', end)
    path_v = [(end[0] - start[0]), (end[1] - start[1])]
    path_angle = acos(path_v[0]/sqrt(path_v[0]**2 + path_v[1]**2))
    if path_v[1] < 0:
      path_angle = 2*pi - path_angle
    e = path_angle - car[2] - pi
    #print('error: ', e)
    if e > pi:
      e = e - 2*pi
    if e < -pi:
      e = e + 2*pi
    print('error: ', e)
    e_i = e_i + e
    e_d = e - e_old
    result = (e*p_gain + e_i*i_gain + e_d*d_gain)
    #print('result: ', result)
    motor_cmd.data = [(spd - result), (spd + result)]
    print('motor_cmd: ', motor_cmd.data)
    print('car: ', start)
    print('..')
    pub.publish(motor_cmd)
    e = e_old
    pub1.publish(suspension_cmd)
    pub2.publish(suspension_cmd)
    pub3.publish(suspension_cmd)
    pub4.publish(suspension_cmd)
    rate.sleep()
  k = k + 1
  end = path_list[k]
  rate.sleep()

