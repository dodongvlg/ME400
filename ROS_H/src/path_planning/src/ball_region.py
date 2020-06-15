#! /usr/bin/env python
from __future__ import division
import rospy
#from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64
from std_msgs.msg import Int64
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import sqrt
from math import pi
from math import acos
from math import cos
from math import sin
def find_length(start, end):
  return sqrt((start[0]-end[0])**2+(start[1]-end[1])**2)

def find_intersection(start,end,obstacle,r):
  #line y = ax + b
  a = (end[1]-start[1])/(end[0]-start[0])
  b = start[1] - a*start[0]
  #circle (x-A)^2 + (y-B)^2 = r^2
  X = obstacle[0]
  Y = obstacle[1]

  A = (1+a**2)
  B = 2*(-X+a*(b-Y))
  C = (X**2 + (b-Y)**2 - r**2)

  result = [0, 0]
  det = B**2 - 4*A*C
  if det > 0:
    x = [(-B + sqrt(det))/(2*A), (-B - sqrt(det))/(2*A)]
    return [[x[0], a*x[0]+b], [x[1], a*x[1]+b]]
  else:
    return False


#set cricital l & r
#calculate whether the intersection length is within l_cr
#if yes, return [True, 0] 
#else, return [False, mid_path]
def div_path(start, end, obstacle, real_end, mode):
  r = 0.4
  l_cr = r/3

  #circle (x-A)^2 + (y-B)^2 = r^2
  X = obstacle[0]
  Y = obstacle[1]

  state = [0, 0]
  if find_length(start, [X,Y]) < r:  #start is within the circle
    state[0] = 1
  if find_length(end, [X,Y]) < r: #end is within the circle
    state[1] = 1

  if state == [1, 1]:
    intersection = [start, end]
    l = find_length(start, end)
  elif state == [1, 0] or state == [0, 1]:
    x = find_intersection(start,end,obstacle,r)
    if state == [1, 0]:
      if find_length(x[0],end) > find_length(x[1],end):
        intersection = x[1]
      else:
        intersection = x[0]
      intersection = [start, intersection]
      l = find_length(intersection[0], intersection[1])
      if find_length(start,real_end) < find_length(intersection[1], real_end):
        l = 0
    else:
      if find_length(x[0],start) > find_length(x[1],start):
        intersection = x[1]
      else:
        intersection = x[0]
      intersection = [end, intersection]
      l = find_length(intersection[0], intersection[1])
  else:
    x = find_intersection(start,end,obstacle,r)
    if x == False: #does not intersect with obstacle:
      l = 0
    else: #intersect with obstacle
      l = find_length(start, end)
      l_start = min(find_length(start,x[0]), find_length(start,x[1]))
      l_end = min(find_length(end,x[0]), find_length(end,x[1]))
      if l_start + l_end > l: #intersection not within the line segment connecting start to end 
        l = 0
      else:
        intersection  = x
        l = find_length(intersection[0], intersection[1])

  if l < l_cr or len(path_list) > 50:
    return [True, 0] 
  
  #calculate mid_path
  int0 = intersection[0]
  int1 = intersection[1]
  midpoint = [(int0[0]+int1[0])/2, (int0[1]+int1[1])/2]
  a2 = -1/((int0[1]-int1[1])/(int0[0]-int1[0]))
  extra = [midpoint[0] + 1, midpoint[1]+a2]
  x = find_intersection(midpoint,extra,obstacle,r)
  mid_path = x[0]
  if find_length(x[0], midpoint) > find_length(x[1], midpoint):
    mid_path = x[1]
  if mode == 1:
    pillar_list = []
    for pillar in obstacle_list:
      pillar_list.append([find_length(pillar, obstacle), pillar])
    pillar_list.sort()
    pillar = pillar_list[1][1]
    print("nearest pillar: ", pillar)
    if find_length(mid_path, pillar) > 0.4:
      return [False, mid_path]
    else:
      if mid_path == x[0]:
        return [False, x[1]]
      return [False, x[0]]
  return [False, mid_path]

#update path_list by adding midpoint to index i+1
def update_list(path_list, midpoint, i):
  k = len(path_list)
  path_list.append(midpoint)
  while k > i:
    path_list[k] = path_list[k-1]
    k = k - 1
  path_list[i+1] = midpoint
  #print(path_list)
  return path_list

#find detoured path for one ostacle
def detour(path_list, obstacle):
  i = len(path_list)-2
  final_destination = path_list[-1]
  if len(obstacle_list) > 4 and obstacle == obstacle_list[-1]:
    result = div_path(path_list[i], final_destination, obstacle, final_destination, 1)
  else:
    result = div_path(path_list[i], final_destination, obstacle, final_destination, 0)
  if result[0] == True:
    return path_list
  else:
    path_list = update_list(path_list, result[1], i)
    n = len(path_list)
    while i+1 < n:
      result = div_path(path_list[i], path_list[i+1], obstacle, final_destination,0)
      if result[0] == True:
        i = i + 1
      else:
        path_list = update_list(path_list, result[1], i)
      n = len(path_list)
    return path_list


def get_path_list(obstacle_list, start):
  global path_list
  dist_list = []
  i = 0
  for obstacle in obstacle_list:
    dist_list.append([find_length(start, obstacle), i])
    i = i + 1
  dist_list.sort()
  
  sorted_obstacle_list = []
  for element in dist_list:
    sorted_obstacle_list.append(obstacle_list[element[1]])

  for obstacle in sorted_obstacle_list:
    path_list = detour(path_list, obstacle)
  
  return path_list

def update_robot_pos(data):
  data2 = data.data
  theta = -data2[0]*pi/180 + pi
  if theta < 0:
    theta = 2*pi + theta
  global car
  car = [data2[1], data2[2], theta]
  global docking
  docking = [data2[3], data2[4]]
  global target
  target = [data2[5], data2[6]]
  '''
  data2 = data.pose[17]
  orientation_list = [data2.orientation.x, data2.orientation.y, data2.orientation.z, data2.orientation.w]
  (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

  global car
  car = [data2.position.x, data2.position.y, yaw + pi]
  '''

def update_robot_pos2(data):

  data2 = data.pose[17]
  orientation_list = [data2.orientation.x, data2.orientation.y, data2.orientation.z, data2.orientation.w]
  (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

  global car2
  car2 = [data2.position.x, data2.position.y, yaw + pi]



def motor_actuation(path_list, car):
  
  global e_i
  global e_old

  #get path vector & angle
  path_v = [(path_list[1][0] - path_list[0][0]), (path_list[1][1] - path_list[0][1])]
  path_angle = acos(path_v[0]/sqrt(path_v[0]**2 + path_v[1]**2))
  if path_v[1] < 0:
    path_angle = 2*pi - path_angle

  #calculate orientation error
  e = path_angle - car[2] - pi
  #print('error: ', e)
  if e > pi:
    e = e - 2*pi
  if e < -pi:
    e = e + 2*pi
  print('error: ', e)

  #calculate spd
  e_spd = find_length(path_v, [0,0])
  spd = e_spd*4.5
  if spd > 4.5:
    spd = 4.5

  #actuate motor with PID control based on the orientation error
  e_i = e_i + e
  e_d = e - e_old
  result = (e*p_gain + e_i*i_gain + e_d*d_gain)
  motor_cmd.data = [(2.5 + spd - result), (2.5 + spd + result)]
  pub.publish(motor_cmd)

  e_old = e

  print('motor_cmd: ', motor_cmd.data)
  print('car: ', car)
  print('real_car: ', car2)
  print('..')

def motor_actuation_to_goal(path_list, car, speed):
  
  global e_i
  global e_old

  #get path vector & angle
  path_v = [(path_list[1][0] - path_list[0][0]), (path_list[1][1] - path_list[0][1])]
  path_angle = acos(path_v[0]/sqrt(path_v[0]**2 + path_v[1]**2))
  if path_v[1] < 0:
    path_angle = 2*pi - path_angle

  #calculate orientation error
  e = path_angle - car[2] - pi
  #print('error: ', e)
  if e > pi:
    e = e - 2*pi
  if e < -pi:
    e = e + 2*pi
  print('error: ', e)

  #actuate motor with PID control based on the orientation error
  e_i = e_i + e
  e_d = e - e_old
  result = (e*p_gain + e_i*i_gain + e_d*d_gain)
  motor_cmd.data = [(speed - result), (speed + result)]
  pub.publish(motor_cmd)

  e_old = e

  print('motor_cmd: ', motor_cmd.data)
  print('car: ', car)
  print('real_car: ', car2)
  print('..')

def align(target_ball_pos):
  #rotate correct direction
  e = 10
  while abs(e) > 0.04:
    path_v = [(target_ball_pos[0] - car[0]), (target_ball_pos[1] - car[1])]
    path_angle = acos(path_v[0]/sqrt(path_v[0]**2 + path_v[1]**2))

    if path_v[1] < 0:
      path_angle = 2*pi - path_angle

    e = path_angle - car[2] - pi
    if e > pi:
      e = e - 2*pi
    if e < -pi:
      e = e + 2*pi
    while holder_state == 2:
      if e > 0:
        motor_cmd.data = [5, -5]
        pub.publish(motor_cmd)
      else:
        motor_cmd.data = [-5, +5]
        pub.publish(motor_cmd)  
    if e > 0:
      if abs(e) > 0.4:
        motor_cmd.data = [-4, +4]
        pub.publish(motor_cmd)
      else:
        motor_cmd.data = [-1, +1]
        pub.publish(motor_cmd)
    else:
      if abs(e) > 0.4:
        motor_cmd.data = [+4, -4]
        pub.publish(motor_cmd)
      else:
        motor_cmd.data = [+1, -1]
        pub.publish(motor_cmd)
    print("aligning")
    print("error: ", e)
    print('motor_cmd: ', motor_cmd.data)
    print('car: ', car)
    print('real_car: ', car2)
    print('..')
    rate.sleep()


#dummy
motor_cmd = Float64MultiArray()
suspension_cmd = Float64()
car = [0, 0, 0]
car2 = [0,0,0]
target = [5,1.5]
docking = [6,1.6]
region = -1
holder_state = -1

#fixed parameters
obstacle_list = [[4,1.5], [5.5,0.7], [5.5,2.3], [7,1.5]]

#topic names
motor_cmd_topic_name = '/toy_car2/joint_vel_controller/command'
suspension_topic_front_left = '/toy_car2/front_left_suspension_controller/command'
suspension_topic_front_right = '/toy_car2/front_right_suspension_controller/command'
suspension_topic_rear_left = '/toy_car2/rear_left_suspension_controller/command'
suspension_topic_rear_right = '/toy_car2/rear_right_suspension_controller/command'

#temporary update functions
def update_region(data):
  global region
  region = data.data

def update_holder_state(data):
  global holder_state
  holder_state = data.data

#publisher & subscriber
pub = rospy.Publisher(motor_cmd_topic_name, Float64MultiArray, queue_size=10)
pub1 = rospy.Publisher(suspension_topic_front_left, Float64, queue_size=10)
pub2 = rospy.Publisher(suspension_topic_front_right, Float64, queue_size=10)
pub3 = rospy.Publisher(suspension_topic_rear_left, Float64, queue_size=10)
pub4 = rospy.Publisher(suspension_topic_rear_right, Float64, queue_size=10)
rospy.Subscriber("/gazebo/model_states", ModelStates, update_robot_pos2)
rospy.Subscriber("/mapdata/stage_position", Float64MultiArray, update_robot_pos)

rospy.Subscriber("/regionFlag", Int64, update_region)
rospy.Subscriber("/holderFlag", Int64, update_holder_state)


#node initiation
rospy.init_node("path_planning")


#suspension initiation
suspension_cmd.data = 0.03
pub1.publish(suspension_cmd)
pub2.publish(suspension_cmd)
pub3.publish(suspension_cmd)
pub4.publish(suspension_cmd)

#rate control
rate = rospy.Rate(20)

#parameters to adjust
spd = 7
[p_gain, i_gain, d_gain] = [7, 0, 0]
close_enough = 0.045
goal = [8,1.5]
goal_region = 0.3

e_old = 0
e_i = 0
stuck_list = []
while not rospy.is_shutdown():
  print("region: ", region)
  print("holder_state: ", holder_state)
  kk = -1
  start = [car[0], car[1]]
  #if stuck, move backwards
  stuck_list.append(start)
  if len(stuck_list) > 51:
    print("stuck? ", find_length(stuck_list[0], stuck_list[50]))
    k = 0
    if find_length(stuck_list[0], stuck_list[50]) < 0.1:
      while k < 30:
        print("stucked.. moving backwards")
        motor_cmd.data = [-2,-2]
        pub.publish(motor_cmd)
        rate.sleep()
        k = k + 1
        print("k: ", k)
      stuck_list = []
    else:
      stuck_list.pop(0)

  if holder_state != 1:
    obstacle_list.append(target)
    end = docking

    # after goal in
    while find_length(start, goal) < goal_region: 
      start = [car[0], car[1]]
      motor_cmd.data = [-2,-2] #to avoid wheel to fall in the hole, move backwards
      pub.publish(motor_cmd)
      rate.sleep()

    #approaching the docking point
    if find_length(start, end) > close_enough: 
      print("Approaching docking point: ", docking)
      path_list = [start, end]
      path_list = get_path_list(obstacle_list, start)
      motor_actuation(path_list, car)

    else: #approached the proper docking point
      print("At docking point")
      align(target) #align with the blue ball
      motor_cmd.data = [0,0]
      pub.publish(motor_cmd)
      k = 0
      while holder_state!= 1 and k < 40:
        print("going for the blue ball")
        print("holder_state: ", holder_state)
        print("stuck if > 50: ", k)
        motor_cmd.data = [5,5]
        pub.publish(motor_cmd)
        k = k + 1
        rate.sleep()
      if k >= 40:
        kk = 10

    obstacle_list.pop()

  while holder_state == 1 or kk > 0:
    print("Going to Goal region")
    end = goal  #goal position
    start = [car[0], car[1]]
    if find_length(start, end) > 0.07: #approaching the goal area
      path_list = [start, end]
      path_list = get_path_list(obstacle_list, start)
      motor_actuation_to_goal(path_list, car, 7) # no speed reduction
    if holder_state != 1:
      kk = kk - 1
      print("kk: ",kk)
    rate.sleep()

  
  rate.sleep()

