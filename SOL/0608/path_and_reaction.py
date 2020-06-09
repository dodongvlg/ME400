#! /usr/bin/env python
from __future__ import division
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from numpy import linspace
from math import sqrt
from math import pi
from math import acos
from math import cos
from math import sin
import matplotlib.pyplot as plt 


def find_length(start, end):

	return sqrt((start[0]-end[0])**2+(start[1]-end[1])**2)

def find_intersection(start,end,obstacle,radius):

	#line y = ax + b
	#circle (x-X)^2 + (y-Y)^2 = r^2
	X = obstacle[0]
	Y = obstacle[1]

	if (end[0]-start[0]) ==0 : 

		end[0] =+ 0.01
   

	a = (end[1]-start[1])/(end[0]-start[0])
	b = start[1] - a*start[0]
	A = (1+a**2)
	B = 2*(-X+a*(b-Y))
	C = (X**2 + (b-Y)**2 - radius**2)
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

def div_path(start, end, obstacle, real_end, radius):
  
	#  r = 0.4
	l_cr = radius/3

	#circle (x-A)^2 + (y-B)^2 = r^2
	X = obstacle[0]
	Y = obstacle[1]

	state = [0, 0]
	if find_length(start, [X,Y]) < radius:  #start is within the circle

		state[0] = 1

	if find_length(end, [X,Y]) < radius: #end is within the circle

		state[1] = 1

	if state == [1, 1]:

		intersection = [start, end]
		l = find_length(start, end)

	elif state == [1, 0] or state == [0, 1]:

		x = find_intersection(start,end,obstacle,radius)
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

	else: #state == [0,0]

		x = find_intersection(start,end,obstacle,radius)
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

	if l < l_cr:

		return [True, 0] 
  
	#calculate mid_path
	int0 = intersection[0]
	int1 = intersection[1]
	midpoint = [(int0[0]+int1[0])/2, (int0[1]+int1[1])/2]
  
	if int0[1]==int1[1]:

		int0[1] =+ 0.01

	a2 = -(int0[0]-int1[0])/(int0[1]-int1[1])
	extra = [midpoint[0] + 1, midpoint[1]+a2]
	x = find_intersection(midpoint,extra,obstacle,radius)
	mid_path = x[0]

	if find_length(x[0], midpoint) > find_length(x[1], midpoint):

		mid_path = x[1]
  
	return[False, mid_path]

#update path_list by adding midpoint to index i+1
def update_list(path_list, midpoint, i):

	k = len(path_list)
	path_list.append(midpoint)
	while k > i:
		path_list[k] = path_list[k-1]
		k = k - 1
	path_list[i+1] = midpoint

	return path_list

#find detoured path for one ostacle
def detour(path_list, obstacle, radius):

	i = len(path_list)-2
	final_destination = path_list[-1]
	result = div_path(path_list[i], final_destination, obstacle, final_destination, radius)

	if result[0] == True:

		return path_list
	else:

		path_list = update_list(path_list, result[1], i)
		n = len(path_list)
		while i+1 < n:

			result = div_path(path_list[i], path_list[i+1], obstacle, final_destination, radius)
			if result[0] == True:

				i = i + 1
			else:

				path_list = update_list(path_list, result[1], i)
			
			n = len(path_list)

		return path_list


def update_robot_pos(data):

	# Car position data 

	data2 = data.pose[17]

	orientation_list = [data2.orientation.x, data2.orientation.y, data2.orientation.z, data2.orientation.w]
	(roll, pitch, yaw) = euler_from_quaternion (orientation_list)

	global car
	car = [data2.position.x, data2.position.y, yaw + pi]

	# Blue balls position data

	blue_ball_1= data.pose[9]
	blue_ball_2 = data.pose[10]
	blue_ball_3 = data.pose[11]
	
	global target_1, target_2, target_3 
	target_1 = [blue_ball_1.position.x, blue_ball_1.position.y]
	target_2 = [blue_ball_2.position.x, blue_ball_2.position.y]
	target_3 = [blue_ball_3.position.x, blue_ball_3.position.y]

def update_target(target_1,target_2,target_3):
  
	
	blue_ball = [target_1,target_2,target_3]
	final_target = [] 
	
	for i in range(3) : 

		if blue_ball[i][0] >= 8 or ( blue_ball[i][0] >= 7.9 and abs(blue_ball[i][1]-1.5) < 0.05 ):
			
			continue
		else : 

			length = find_length(car, blue_ball[i])
			candidate = length, blue_ball[i]
			final_target.append(candidate)
	
	final_target.sort()
	final_length, target = final_target[0] # Absolute coordinate of the closest blue ball

	return target

"""

toy_car = data.pose[17]

red_ball_0 = data.pose[6]
red_ball_1 = data.pose[7]
red_ball_2 = data.pose[8]

blue_ball_0 = data.pose[9]
blue_ball_1 = data.pose[10]
blue_ball_2 = data.pose[11]

gole_0 = data.pose[12]

"""

def progressive_proof(end,car,obstacle_list,radius):

	#calculate path_list
	#adjust start point 
	start = car
	"""
	start = [car[0] - 0.1*sin(car[2]), car[1] + 0.1*cos(car[2])]
	"""
	path_list = [start, end]

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
		path_list = detour(path_list, obstacle, radius)

	return path_list 

def drive_force(path_list,car,gain_list):
	
	global e_i, e_old

	path_v = [(path_list[1][0] - path_list[0][0]), (path_list[1][1] - path_list[0][1])]
	path_angle = acos(path_v[0]/sqrt(path_v[0]**2 + path_v[1]**2))


	if path_v[1] < 0:
		path_angle = 2*pi - path_angle

	e = car[2] - path_angle
	#print('error: ', e)

	if e > pi:
		e = e - 2*pi
	if e < -pi:
		e = e + 2*pi
	e_i =+ e
	e_d = e - e_old
	adjustment = (e*p_gain + e_i*i_gain + e_d*d_gain)

	return adjustment

#dummy
motor_cmd = Float64MultiArray()
car = [0, 0, 0]
target_1 = [0,0]
target_2 = [0,0]
target_3 = [0,0]
data1=[]
data2= False

#fixed parameters
obstacle_list = [[4,1.5], [5.5,0.7], [5.5,2.3], [7,1.5]]
target=[]

#parameters to adjust 
spd = 7
motor_cmd_topic_name = '/toy_car/joint_vel_controller/command'
[p_gain, i_gain, d_gain] = [6, 0.01, 0.01]
gain_list = [p_gain, i_gain, d_gain]


#publisher & subscriber
pub = rospy.Publisher(motor_cmd_topic_name, Float64MultiArray, queue_size=10)
rospy.Subscriber("/gazebo/model_states", ModelStates, update_robot_pos)
rospy.init_node("path_planning") 
	# Register client node with the master of rospy system.
rate = rospy.Rate(5)
long_rate = rospy.Rate(20)

def call_back_1(True):

	global data1, data2
	data1 = car[0:2]

	if find_length(data1,car[0:2]) < 0.2 :

		motor_cmd.data = [0,0]
		pub.publish(motor_cmd)
		rate.sleep()
		

		motor_cmd.data = [-6,-6]
		pub.publish(motor_cmd)
		rate.sleep()

		motor_cmd.data = [0,0]
		pub.publish(motor_cmd)
		rate.sleep()
	
		data2 = True


def check():
	
	rospy.Timer(rospy.Duration(10), call_back_1)
		


def main(end):
	
	check()
	while find_length(car, end) > 0.1 :
		
		
		path_list = progressive_proof(end,car,obstacle_list,radius)
		adjustment = drive_force(path_list, car, gain_list)
		motor_cmd.data = [(spd + adjustment), (spd - adjustment)]

		pub.publish(motor_cmd)
		e = e_old	

		rate.sleep()

		if data2 :

			break

	motor_cmd.data = [0,0]
	pub.publish(motor_cmd)
	print("Desired destination: ", end, "    car_pos: ", car)
	print("..")
	long_rate.sleep()




#parameters to input  (Relativistic)

def ask():

	x_end = float(input("Enter x_coordinate of destination coordinate within (3~8): "))
	y_end = float(input("Enter y_coordinate of destination coordinate within (0~3): "))
	radius = float(input("Decide_the_size_of_the_circle:   "))
	end = [car[0] + x_end, car[1] + y_end]

	return (end, radius) 


#end = [7.4,1.77]
#also change r, l_cr inside div_path
#r = 0.1

e_old = 0
e_i = 0
plt.ion()
start = [car[0], car[1]]


#while not rospy.is_shutdown():
while True : 
	
	target = update_target(target_1,target_2,target_3) #absolute coordinate from car to end
	radius = 0.55 # The radius fixed  
	
	end = target 
 	main(end)
	long_rate.sleep()

	end = [8,1.5]  
	main(end)
	long_rate.sleep()

"""
  #draw current path_planning
  plt.clf()
  x = []
  y = []
  for path in path_list:
    x.append(path[0])
    y.append(path[1])
  plt.plot(x,y, 'o-')

  #draw obstacle
  for obstacle in obstacle_list:
    yy1 = []
    yy2 = []
    X = obstacle[0]
    Y = obstacle[1]
    xx = linspace(X-(r-0.0001), X+(r-0.0001), 100)
    for x in xx:
      yy1.append(sqrt(r**2 - (x-X)**2)+Y)
      yy2.append(-sqrt(r**2 - (x-X)**2)+Y)
    plt.plot(xx,yy1)
    plt.plot(xx,yy2)
  plt.axis('equal')
  plt.draw()
"""

		


