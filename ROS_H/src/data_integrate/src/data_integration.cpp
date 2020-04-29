/*

Data Integration Node
KAIST ME400 Team H
Last modified by Gunwoo Park, 2020. 04. 29.
Version 1 : Implemented Map Reconstruction

This code is composed with init, loop, and two callback parts.

Init : Executes once if the node starts to run.
Loop : Executes periodically during the node runtime.
Callback : Executes every time the subscriber recieves the message.

*/

// Standard Header Files for C++
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <string>
#include <vector>

// Standard Header Files for C
#include <stdio.h>
#include <signal.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <boost/thread.hpp>

// Header Files for ROS & OpenCV
#include <ros/ros.h>
#include <ros/package.h>
#include <core_msgs/ball_position.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <opencv2/opencv.hpp>

// Additional Header Files for Matrix Computation
#include <Eigen/Core>

#define DEG(x) ((x)*180./M_PI)
#define RAD(x) ((x)*M_PI/180)

boost::mutex map_mutex;

// Global Variables for Lidar
const int n_lidar = 360;
float lidar_angle[n_lidar];
float lidar_distance[n_lidar];

// Global Variables for Camera
int ball_number;
float ball_X[20];
float ball_Y[20];
float ball_distance[20];

Eigen::Matrix<float, 2, 2> calc_rotmat(int theta) {
	Eigen::Matrix<float, 2, 2> rotmat;
	rotmat(0, 0) = cos(RAD(theta));
	rotmat(0, 1) = sin(RAD(theta));
	rotmat(1, 0) = -sin(RAD(theta));
	rotmat(1, 1) = cos(RAD(theta));
	return rotmat;
}

float *calc_dim(float *map_dim, Eigen::Matrix<float, 2, n_lidar> ldrxy) {
	int i_lidar;
	float min_x = 0;
	float min_y = 0;
	float max_x = 0;
	float max_y = 0;

	for (i_lidar = 0; i_lidar < n_lidar; i_lidar++) {
		if (ldrxy(0, i_lidar) < min_x) min_x = ldrxy(0, i_lidar);
		if (ldrxy(0, i_lidar) > max_x) max_x = ldrxy(0, i_lidar);
		if (ldrxy(1, i_lidar) < min_y) min_y = ldrxy(1, i_lidar);
		if (ldrxy(1, i_lidar) > max_y) max_y = ldrxy(1, i_lidar);
	}

	map_dim[0] = (max_x - min_x)*(max_y - min_y);
	map_dim[1] = max_x - min_x;
	map_dim[2] = max_y - min_y;
	map_dim[3] = - min_x;
	map_dim[4] = - min_y;
	return map_dim;
}

void lidar_Callback(const sensor_msgs::LaserScan::ConstPtr& scan) {
	// Callback 1 : Get array of distance and angle from lidar
	map_mutex.lock();
	int i_lidar;
	for(i_lidar = 0; i_lidar < n_lidar; i_lidar++)
	{
		lidar_angle[i_lidar] = scan->angle_min + scan->angle_increment * i_lidar;
		lidar_distance[i_lidar] = scan->ranges[i_lidar];
		if (lidar_distance[i_lidar] > 3.5) lidar_distance[i_lidar] = 3.5;
	}
	map_mutex.unlock();
}

void camera_Callback(const core_msgs::ball_position::ConstPtr& position) {
	// Callback 2 : Get ball position from OpenCV
	map_mutex.lock();
	int count = position->size;
	ball_number=count;
	for(int i = 0; i < count; i++)
	{
		ball_X[i] = position->img_x[i];
		ball_Y[i] = position->img_y[i];
		// std::cout << "degree : "<< ball_degree[i];
		// std::cout << "   distance : "<< ball_distance[i]<<std::endl;
		ball_distance[i] = ball_X[i]*ball_X[i]+ball_Y[i]*ball_X[i];
	}
	map_mutex.unlock();
}


int main(int argc, char **argv)
{
	// Local Variables for main()
	Eigen::Matrix<float, 2, n_lidar> ldrxy_raw, ldrxy_rot;
	Eigen::Matrix<float, 2, 2> rotmat;

	// Local Variables for loop 1
	int i_lidar;
	int theta, theta_step, theta_dir;
	float area, area_min;
	float map_dim[5];
	float *map_ptr;
	std::vector<double> map_data(5); 

	// Init : ROS initialization and configuration
	ros::init(argc, argv, "data_integration");
	ros::NodeHandle n;
	ros::Rate loop_rate(5);
	ros::Subscriber sub_lidar = n.subscribe<sensor_msgs::LaserScan>("/scan", 256, lidar_Callback);
	ros::Subscriber sub_camera = n.subscribe<core_msgs::ball_position>("/position", 256, camera_Callback);

	ros::Publisher pub_left_wheel= n.advertise<std_msgs::Float64>("/turtlebot3_waffle_sim/left_wheel_velocity_controller/command", 10);
	ros::Publisher pub_right_wheel= n.advertise<std_msgs::Float64>("/turtlebot3_waffle_sim/right_wheel_velocity_controller/command", 10);

	ros::Publisher pub_map = n.advertise<std_msgs::Float64MultiArray>("/mapdata", 16);
	std_msgs::Float64MultiArray map_msg; // map_data : [theta, width, height, x, y]

	// Loop : Process data and Publish message every 200ms
	while(ros::ok){

		// Loop 0 : Check if the sensor data input is sane

		/**
		for(int i = 0; i < lidar_size; i++
		{
			std::cout << "degree : "<< lidar_angle[i];
			std::cout << "   distance : "<< lidar_distance[i]<<std::endl;
		}
		for(int i = 0; i < ball_number; i++)
		{
			std::cout << "ball_X : "<< ball_X[i];
			std::cout << "ball_Y : "<< ball_Y[i]<<std::endl;
		}
		**/

		// Loop 1 : Map Reconstruction
		for (i_lidar = 0; i_lidar < n_lidar; i_lidar++) {
			ldrxy_raw(0, i_lidar) = lidar_distance[i_lidar]*cos(lidar_angle[i_lidar]);
			ldrxy_raw(1, i_lidar) = lidar_distance[i_lidar]*sin(lidar_angle[i_lidar]);
		}
		
		theta = 0;
		theta_step = 16; // DEG
		while (theta_step >= 1) {
			// Check area of theta
			rotmat = calc_rotmat(theta);
			ldrxy_rot = rotmat*ldrxy_raw;
			map_ptr = calc_dim(map_dim, ldrxy_rot);
			area_min = map_ptr[0];
			theta_dir = 0;
			
			// Check area of theta - step
			rotmat = calc_rotmat(theta - theta_step);
			ldrxy_rot = rotmat*ldrxy_raw;
			map_ptr = calc_dim(map_dim, ldrxy_rot);
			if (map_ptr[0] < area_min) {
				area = area_min;
				theta_dir = -1;
			}
			
			// Check area of theta + step
			rotmat = calc_rotmat(theta + theta_step);
			ldrxy_rot = rotmat*ldrxy_raw;
			map_ptr = calc_dim(map_dim, ldrxy_rot);
			if (map_ptr[0] < area_min) {
				area = area_min;
				theta_dir = 1;
			}
			theta += theta_dir*theta_step;
			if (theta_dir == 0) theta_step = theta_step / 2;
		}

		rotmat = calc_rotmat(theta);
		ldrxy_rot = rotmat*ldrxy_raw;
		map_ptr = calc_dim(map_dim, ldrxy_rot);
		map_data.at(0) = theta;
		map_data.at(1) = map_ptr[1];
		map_data.at(2) = map_ptr[2];
		map_data.at(3) = map_ptr[3];
		map_data.at(4) = map_ptr[4];

		map_msg.data = map_data;
		pub_map.publish(map_msg);
		std::cout << map_data.at(0) << " " << map_data.at(1) << " " << map_data.at(2) << " " << map_data.at(3) << " " << map_data.at(4) << std::endl;
	
		// Loop 2 : Publish message to actuate the model

		/** 
		std_msgs::Float64 left_wheel_msg;
		std_msgs::Float64 right_wheel_msg;

		left_wheel_msg.data=1;   // set left_wheel velocity
		right_wheel_msg.data=1;  // set right_wheel velocity

		pub_left_wheel.publish(left_wheel_msg);   // publish left_wheel velocity
		pub_right_wheel.publish(right_wheel_msg);  // publish right_wheel velocity

		**/

		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}
