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

#find length from start to end coordinates
def find_length(start, end):
  return sqrt((start[0]-end[0])**2+(start[1]-end[1])**2)

#find intersection between the line connecting start and end and the circle centered at obstacle with radius r
#return intersection points
#return False if no intersection
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
#r defines the radius of the danger zone due to pillars
#calculate whether the intersection segment inside the circle is smaller than l_cr
#if yes, return [True, 0] 
#else, return [False, mid_path]
def div_path(start, end, obstacle, real_end, mode):
  r = 0.4
  l_cr = r/3

  #circle (x-A)^2 + (y-B)^2 = r^2
  X = obstacle[0]
  Y = obstacle[1]

  #state means whether the start and end points are inside the danger zone circle. 1 means insdie. 0 means outisde.
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
  #mode 1 means the target ball is too close to the pillar
  if mode == 1:
    pillar_list = []
    for pillar in obstacle_list:
      pillar_list.append([find_length(pillar, obstacle), pillar])
    pillar_list.sort()
    pillar = pillar_list[1][1]
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

#find detoured path for the ostacle
def detour(path_list, obstacle):
  i = len(path_list)-2
  final_destination = path_list[-1]
  #if the obstacle is the target blue ball, there may be overlapping danger zone. so perform div_path with mode 1
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

#calculate the detoured path for all obstacles in the obstacle_list
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

#callback function to get position and orientation of the robot from daga_integrate node, which calculates it using lidar data
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

#actuate the motors given the orientation and position of the car and desired path
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
  speed = e_spd*(spd - min_spd)
  if speed > (spd - min_spd):
    speed = (spd - min_spd)

  #actuate motor with PID control based on the orientation error
  e_i = e_i + e
  e_d = e - e_old
  result = (e*p_gain + e_i*i_gain + e_d*d_gain)
  motor_cmd.data = [(min_spd + speed - result), (min_spd + speed + result)]
  if motor_cmd.data[0] < -spd:
    motor_cmd.data[0] = -spd
  if motor_cmd.data[0] > spd:
    motor_cmd.data[0] = spd
  if motor_cmd.data[1] < -spd:
    motor_cmd.data[1] = -spd
  if motor_cmd.data[1] > spd:
    motor_cmd.data[1] = spd
  pub.publish(motor_cmd)
  pub1.publish(suspension_cmd)
  pub2.publish(suspension_cmd)
  pub3.publish(suspension_cmd)
  pub4.publish(suspension_cmd)

  e_old = e

  print('motor_cmd: ', motor_cmd.data)
  print('car: ', car)
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
  reduced_speed = spd
  motor_cmd.data = [(speed - result), (speed + result)]
  if motor_cmd.data[0] < -spd:
    motor_cmd.data[0] = -spd
  if motor_cmd.data[0] > spd:
    motor_cmd.data[0] = spd
  if motor_cmd.data[1] < -spd:
    motor_cmd.data[1] = -spd
  if motor_cmd.data[1] > spd:
    motor_cmd.data[1] = spd
  pub.publish(motor_cmd)
  pub1.publish(suspension_cmd)
  pub2.publish(suspension_cmd)
  pub3.publish(suspension_cmd)
  pub4.publish(suspension_cmd)

  e_old = e

  print('motor_cmd: ', motor_cmd.data)
  print('car: ', car)
  print('..')

#align to the target ball position
#if center camera finds the ball, run align_vision()
def align(target_ball_pos):
  #rotate correct direction
  e = 10
  kk = 0
  while kk <60:
  #while abs(e) > 0.04:
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
        motor_cmd.data = [spd*5/6, -spd*5/6]
        pub.publish(motor_cmd)
      else:
        motor_cmd.data = [-spd*5/6, +spd*5/6]
        pub.publish(motor_cmd)  
    if e > 0:
      if abs(e) > 0.5:
        motor_cmd.data = [-spd*3/6, +spd*3/6]
        pub.publish(motor_cmd)
      else:
        vel = abs(e)*spd*2/6+0.1 
        motor_cmd.data = [-vel, +vel]
        pub.publish(motor_cmd)
    else:
      if abs(e) > 0.5:
        motor_cmd.data = [+spd*3/6, -spd*3/6]
        pub.publish(motor_cmd)
      else:
        vel = abs(e)*spd*2/6+0.1
        motor_cmd.data = [+vel, -vel]
        pub.publish(motor_cmd)
    
    if vision_alignment_error != -2:
      align_vision()
      return 0
    pub1.publish(suspension_cmd)
    pub2.publish(suspension_cmd)
    pub3.publish(suspension_cmd)
    pub4.publish(suspension_cmd)
    print("aligning")
    print("error: ", e)
    print('motor_cmd: ', motor_cmd.data)
    print('car: ', car)
    print("vision alignment error: ", vision_alignment_error)
    print('..')
    kk = kk + 1
    print("kk: ", kk)
    rate.sleep()
  return 1

#more precise alignment done with center camera vision
def align_vision():
  k = 0
  kk = 0
  while abs(vision_alignment_error) > 0.01 and kk < 100:#960px*0.05 ~= 50px
    if vision_alignment_error == -2:
      vel = spd/6
    else:
      vel = abs(vision_alignment_error)*spd*2/6
    if vision_alignment_error > 0:
      motor_cmd.data = [-vel, +vel]
      pub.publish(motor_cmd)
    else:
      motor_cmd.data = [+vel, -vel]
      pub.publish(motor_cmd)
    if abs(vision_alignment_error) < 0.01:
      k = k + 1
    if k > 10:
      return 1
    pub1.publish(suspension_cmd)
    pub2.publish(suspension_cmd)
    pub3.publish(suspension_cmd)
    pub4.publish(suspension_cmd)
    print("vision aligning")
    print("error: ", vision_alignment_error)
    print('motor_cmd: ', motor_cmd.data)
    print('car: ', car)
    print('..')
    kk = kk + 1
    rate.sleep()
  return 1

#if safe to move backwards, return 0
def check_flipping():
  #weak points are the backside of front wheels and caster wheels [x,y,proximity limit in m]
  weak_points = []
  weak_points.append([car[0] + l1*cos(car[2]+pi-1.797429905695436), car[1] + l1*sin(car[2]+pi-1.797429905695436), 0.02])
  weak_points.append([car[0] + l1*cos(car[2]+pi+1.797429905695436), car[1] + l1*sin(car[2]+pi+1.797429905695436), 0.02])
  weak_points.append([car[0] + l2*cos(car[2]+pi-2.800216520092484), car[1] + l2*sin(car[2]+pi-2.800216520092484), 0.1])
  weak_points.append([car[0] + l2*cos(car[2]+pi+2.800216520092484), car[1] + l2*sin(car[2]+pi+2.800216520092484), 0.1])
  weak_points.append([car[0] + l3*cos(car[2]+pi+pi), car[1] + l3*sin(car[2]+pi+pi), 0.1])
  print(".....")
  print(weak_points)

  for obstacle in obstacle_list:
    for weak_point in weak_points:
      if find_length(weak_point, obstacle) < 0.07 + weak_point[2]:
        print("too close to a pillar")
        return 1
      if weak_point[0] < 3+weak_point[2] or weak_point[0] > 8-weak_point[2] or weak_point[1] < weak_point[2] or weak_point[1] > 3-weak_point[2]:
        print("too close to a wall")
        return 1
  return 0

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
  
#dummy
motor_cmd = Float64MultiArray()
suspension_cmd = Float64()
#no_ball = Int64()
car = [0, 0, 0]
target = [5,1.5]
docking = [6,1.6]
region = 0
holder_state = -1
vision_alignment_error = -2
align_result = 1
line_orientation = 0
lidar_orientation = 0
x_centroid = 960
scan_path = [[7.5,0.7], [3.5,2.3], [7.5,2.3], [3.5, 0.7], [7.5,0.7], [7.5,2.3], [5.5,1.5], [3.5,2.3], [3.5, 0.7], [5.5,1.5]]
#fixed parameters
obstacle_list = [[4,1.5], [5.5,0.7], [5.5,2.3], [7,1.5]]

#topic names
motor_cmd_topic_name = '/toy_car2/joint_vel_controller/command'
suspension_topic_front_left = '/toy_car2/front_left_suspension_controller/command'
suspension_topic_front_right = '/toy_car2/front_right_suspension_controller/command'
suspension_topic_rear_left = '/toy_car2/rear_left_suspension_controller/command'
suspension_topic_rear_right = '/toy_car2/rear_right_suspension_controller/command'
#no_ball_topic_name = '/no_ball'

#temporary update functions
def update_region(data):
  global region
  region = data.data

def update_holder_state(data):
  global holder_state
  holder_state = data.data

def update_vision_alignment_error(data):
  global vision_alignment_error
  vision_alignment_error = data.data

#publisher & subscriber
pub = rospy.Publisher(motor_cmd_topic_name, Float64MultiArray, queue_size=10)
pub1 = rospy.Publisher(suspension_topic_front_left, Float64, queue_size=10)
pub2 = rospy.Publisher(suspension_topic_front_right, Float64, queue_size=10)
pub3 = rospy.Publisher(suspension_topic_rear_left, Float64, queue_size=10)
pub4 = rospy.Publisher(suspension_topic_rear_right, Float64, queue_size=10)
rospy.Subscriber("/mapdata/stage_position", Float64MultiArray, update_robot_pos)
#rospy.Subscriber("/regionFlag", Int64, update_region)
rospy.Subscriber("/holderFlag", Int64, update_holder_state)
rospy.Subscriber("/docking/centroid", Float64, update_vision_alignment_error)
rospy.Subscriber("/mapdata/mode", Float64, update_region)
rospy.Subscriber("/line/centroid", Int64, update_x_centroid)
rospy.Subscriber("/line/orientation", Float64, update_orientation)
rospy.Subscriber("/mapdata/mode", Float64, update_region)
rospy.Subscriber("/mapdata/enter_direction", Float64, update_lidar_orientation)



#node initiation
rospy.init_node("path_planning")


#suspension initiation
suspension_cmd.data = 0.03
pub1.publish(suspension_cmd)
pub2.publish(suspension_cmd)
pub3.publish(suspension_cmd)
pub4.publish(suspension_cmd)

#no_ball.data = 0
#pub_no_ball.publish(no_ball)

#rate control
rate = rospy.Rate(20)

#parameters to adjust
spd = 6 #max forward speed
back_spd = 2 #speed when moving backwards
min_spd = 2.5 #min speed forward
close_enough = 0.2
goal = [8,1.5] #goal hole coordinate
goal_region = 0.3 #size of goal region
l1 = 0.24036657 #2D distance from origin to back side of front wheel
l2 = 0.30616743 #2D distance from origin to caster wheel
l3 = 0.2885
stuck_back_time = 70/back_spd #number of loops for moving backwards if stuck
docking_time = 180/spd*2 #number of loops for going forward for docking

#parameters to adjust
threshold = 0.3 #between 0 and 1
center = 960 #center x coordinate from image


e_old = 0
e_i = 0
stuck_list = []
initiate = 0
while not rospy.is_shutdown():
  #no_ball.data = 0
  #pub_no_ball.publish(no_ball)
  if region == 1:
    [p_gain, i_gain, d_gain] = [6, 0, 0.01] #pid gains for motor actuation
    if initiate == 0:
      while find_length([car[0], car[1]], [3.4,0.5]) > 0.05:
        path_list = [[car[0], car[1]], [3.4,0.5]]
        motor_actuation(path_list, car)
        rate.sleep()
      initiate = 1
    print("holder_state: ", holder_state)
    print("target: ", target)
    kk = -1
    start = [car[0], car[1]]

    #if stuck for 52 loops, move backwards
    stuck_list.append(start)
    if len(stuck_list) > 51:
      print("stucked? (if < 0.1) ", find_length(stuck_list[0], stuck_list[50]))
      k = 0
      if find_length(stuck_list[0], stuck_list[50]) < 0.1:
        i = 0
        while i < 10:
          motor_cmd.data = [0,0] #first stop so that doesn't flip forward
          pub.publish(motor_cmd)
          i = i + 1
          rate.sleep()
        while k < stuck_back_time:
          print("stucked..")
          flip = check_flipping()
          if flip == 0:
            print("moving backwards")
            motor_cmd.data = [-back_spd,-back_spd]
          else: # possibility of flipping
            print("FLIP WARNNING: shouldn't move backwards!")
            motor_cmd.data = [0,0]
          pub.publish(motor_cmd)
          pub1.publish(suspension_cmd)
          pub2.publish(suspension_cmd)
          pub3.publish(suspension_cmd)
          pub4.publish(suspension_cmd)
          rate.sleep()
          k = k + 1
          print("k: ", k)
          print("...")
        stuck_list = []
      else:
        stuck_list.pop(0)

    k = 0
    start = [car[0], car[1]]

    if holder_state != 1:
      # after goal in
      while find_length(start, goal) < goal_region: 
        while k < 5:
          motor_cmd.data = [0,0] #first stop so that doesn't flip forward
          pub.publish(motor_cmd)
          k = k + 1
          rate.sleep()
        start = [car[0], car[1]]
        motor_cmd.data = [-back_spd,-back_spd] #to avoid wheel to fall in the hole, move backwards
        pub1.publish(suspension_cmd)
        pub2.publish(suspension_cmd)
        pub3.publish(suspension_cmd)
        pub4.publish(suspension_cmd)
        pub.publish(motor_cmd)
        rate.sleep()
      if target[0] < 0:
        print("scanning")
        print("scan_path: ", scan_path)
        scan_path.insert(0, [car[0], car[1]])
        path_list = [scan_path[0], scan_path[1]]
        path_list = get_path_list(obstacle_list, scan_path[0])
        motor_actuation(path_list, car)
        scan_path.pop(0)
        if find_length(scan_path[0], car) < close_enough:
          scan_path.append(scan_path.pop(0))
      else:
        obstacle_list.append(target)
        end = docking

        #approaching the docking point
        if find_length(start, end) > close_enough: 
          print("Approaching docking point: ", docking)
          path_list = [start, end]
          path_list = get_path_list(obstacle_list, start)
          motor_actuation(path_list, car)

        else: #approached the proper docking point
          print("At docking point")
          align_result = align(target) #align with the blue ball
          motor_cmd.data = [0,0]
          pub.publish(motor_cmd)
          k = 0
          while holder_state!= 1 and k < docking_time and align_result == 0:
            print("going for the blue ball")
            print("holder_state: ", holder_state)
            print("stuck if > 60: ", k)
            motor_cmd.data = [spd/2,spd/2]
            pub1.publish(suspension_cmd)
            pub2.publish(suspension_cmd)
            pub3.publish(suspension_cmd)
            pub4.publish(suspension_cmd)
            pub.publish(motor_cmd)
            k = k + 1
            print("...")
            rate.sleep()
          
          if align_result == 1:
            k = docking_time*2
            i = 0
            while i  < docking_time*4/6:
              motor_cmd.data = [spd/2,spd/2]
              pub1.publish(suspension_cmd)
              pub2.publish(suspension_cmd)
              pub3.publish(suspension_cmd)
              pub4.publish(suspension_cmd)
              pub.publish(motor_cmd)
              i = i + 1
              print("car: ", car)
              print("...")
              rate.sleep()
          
          if k < docking_time-1:
            kk = 10

        obstacle_list.pop()

    #when blue ball inside the holder after docking
    while holder_state == 1 or kk > 0:
      print("Going to Goal region")
      print("holder_state: ", holder_state)
      end = goal  #goal position
      start = [car[0], car[1]]
      if find_length(start, end) > 0.07: #approaching the goal area
        path_list = [start, end]
        path_list = get_path_list(obstacle_list, start)
        motor_actuation_to_goal(path_list, car, spd) # no speed reduction
      if holder_state != 1:
        kk = kk - 1
        print("kk: ",kk)
      rate.sleep()
    rate.sleep()

  else:
    [p_gain, i_gain, d_gain] = [5, 0, 0]
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
