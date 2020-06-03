/*
Data Integration Node
KAIST ME400 Team H
Last modified by Gunwoo Park, 2020. 06. 02.
Version 3 : Implemented Mode and Entrance Data Processing
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
#define ENTER 0
#define STAGE 1

boost::mutex map_mutex;

// Global Variables for Lidar
const int n_ldr = 360;
float lidar_angle[n_ldr];
float lidar_distance[n_ldr];

// Global Variables for Camera
int ball_number;
float ball_X[20];
float ball_Y[20];
float ball_distance[20];

// Matrix rotation
// Input : 2x2 matrix to rotate
// Output : 2x2 matrix rotated

Eigen::Matrix<float, 2, 2> calc_rotmat(int theta) {
	Eigen::Matrix<float, 2, 2> rotmat;
	rotmat(0, 0) = cos(RAD(theta));
	rotmat(0, 1) = sin(RAD(theta));
	rotmat(1, 0) = -sin(RAD(theta));
	rotmat(1, 1) = cos(RAD(theta));
	return rotmat;
}

// Map dimension calculation
// Input : address to return, laserscan coordinates
// Output : array with map dimensions
float *calc_map(float *map_dim, Eigen::Matrix<float, 2, n_ldr> ldrxy) {
	int i_ldr;
	float ldr_x, ldr_y;
	float min_x = 0;
	float min_y = 0;
	float max_x = 0;
	float max_y = 0;

	for (i_ldr = 0; i_ldr < n_ldr; i_ldr++) {
		ldr_x = ldrxy(0, i_ldr);
		ldr_y = ldrxy(1, i_ldr);
		if (ldr_x < min_x) min_x = ldr_x;
		if (ldr_x > max_x) max_x = ldr_x;
		if (ldr_y < min_y) min_y = ldr_y;
		if (ldr_y > max_y) max_y = ldr_y;
	}

	map_dim[0] = (max_x - min_x)*(max_y - min_y);
	map_dim[1] = max_x - min_x; // width
	map_dim[2] = max_y - min_y; // height
	map_dim[3] = - min_x; // system x coordinate
	map_dim[4] = - min_y; // system y coordinate
	return map_dim;
}

// Near point count (on x-axis)
// Input : laserscan coordinates, reference, threshold
// Output : number of laserscan points that is close to the reference
int count_x(Eigen::Matrix<float, 2, n_ldr> ldrxy, float x_ref, float thr) {
	int i_ldr, count;
	float x_ldr;
	count = 0;
	for (i_ldr = 0; i_ldr < n_ldr; i_ldr++) {
		x_ldr = ldrxy(0, i_ldr);
		if (abs(x_ldr - x_ref) < thr) count++;
	}
	return count;
}

// Near point average (on x-axis)
// Input : laserscan coordinates, reference, threshold
// Output : number of laserscan points that is close to the reference
float avg_x(Eigen::Matrix<float, 2, n_ldr> ldrxy, float x_ref, float thr) {
	int i_ldr, count;
	float x_ldr, sum;
	count = 0;
	sum = 0;
	for (i_ldr = 0; i_ldr < n_ldr; i_ldr++) {
		x_ldr = ldrxy(0, i_ldr);
		if (abs(x_ldr - x_ref) < thr) {
			sum += x_ldr;
			count++;
		}
	}
	if (!count) return -1;
	return sum / count;
}

// Angular distance calculation
// Input : two angles
// Output : angular distance
int calc_agl(int agl_1, int agl_2) {
	if (abs(agl_1 - agl_2) > 180) return 360 - abs(agl_1 - agl_2);
	return abs(agl_1 - agl_2);
}

// Callback 1 : Get array of distance and angle from lidar
void lidar_Callback(const sensor_msgs::LaserScan::ConstPtr& scan) {
	map_mutex.lock();
	int i_ldr;
	for(i_ldr = 0; i_ldr < n_ldr; i_ldr++)
	{
		lidar_angle[i_ldr] = scan->angle_min + scan->angle_increment * i_ldr;
		lidar_distance[i_ldr] = scan->ranges[i_ldr];
		if (lidar_distance[i_ldr] > 3.5) lidar_distance[i_ldr] = 3.5;
	}
	map_mutex.unlock();
}

// Callback 2 : Get ball position from OpenCV
void camera_Callback(const core_msgs::ball_position::ConstPtr& position) {
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


// MAIN FUNCTION
int main(int argc, char **argv)
{
	// Local Variables for main()
	Eigen::Matrix<float, 2, n_ldr> ldrxy_raw, ldrxy_rot;
	Eigen::Matrix<float, 2, 2> rotmat;

	// Local Variables for loop 1
	int i_ldr, cur_cnt, max_cnt;
	int ent_cnt, ent_avg, ent_cnts, ent_avgs, ent_thresh; // For entrance navigation
	int theta, theta_step, theta_dir, theta_prev, theta_offset; // For theta optimization
	int mode = ENTER; // ENTER == 0, STAGE == 1
	float area, area_min, x_abs, y_abs, x_ref, x_max, x_prev, y_prev, x_ldr, x_sum; // Map parameters
	float map_dim[5];
	float *map_ptr;
	std::vector<double> map_data(3);
	std::vector<double> obj_data(20);

	// Init : ROS initialization and configuration
	ros::init(argc, argv, "data_integration");
	ros::NodeHandle n;
	ros::Rate loop_rate(5);
	ros::Subscriber sub_lidar = n.subscribe<sensor_msgs::LaserScan>("/scan", 256, lidar_Callback);
	ros::Subscriber sub_camera = n.subscribe<core_msgs::ball_position>("/position", 256, camera_Callback);

	ros::Publisher pub_left_wheel= n.advertise<std_msgs::Float64>("/turtlebot3_waffle_sim/left_wheel_velocity_controller/command", 10);
	ros::Publisher pub_right_wheel= n.advertise<std_msgs::Float64>("/turtlebot3_waffle_sim/right_wheel_velocity_controller/command", 10);

	ros::Publisher pub_map = n.advertise<std_msgs::Float64MultiArray>("/mapdata", 16);
	std_msgs::Float64MultiArray map_msg; // map_data : [theta, x, y]

	ent_thresh = 50;
        theta_prev = 0;
	x_prev = -0.25;
	y_prev = 0.5;
	theta_offset = 180;

	// Loop : Process data and Publish message every 200ms
	while(ros::ok){

		ros::spinOnce(); // Run callback functions

		// Check whether the vehicle entered the stage
		ent_cnt = 0;
		for (i_ldr = 90; i_ldr < n_ldr - 90; i_ldr++) {
			// Consider 180 deg front
			if (lidar_distance[i_ldr] > 3) {
				ent_cnt++;
			}
		}
		if (ent_cnt > ent_thresh) mode = STAGE; // Mode transition
		std::cout << "Current sight : " << ent_cnt << " mode : " << mode << std::endl;

		// Loop 1 : Branched navigation
		if (mode) {

			// Loop A : STAGE mode
			// Loop A - 1 : Find exact direction and position again, trimming out the enterence area

			for (i_ldr = 0; i_ldr < n_ldr; i_ldr++) {
				ldrxy_raw(0, i_ldr) = lidar_distance[i_ldr]*cos(lidar_angle[i_ldr] + RAD(theta_offset));
				ldrxy_raw(1, i_ldr) = lidar_distance[i_ldr]*sin(lidar_angle[i_ldr] + RAD(theta_offset));
			}

			// Optimization loop for theta
			theta = 45;
			theta_step = 16; // DEG
			while (theta_step >= 1) {
				// Check area of theta
				rotmat = calc_rotmat(theta);
				ldrxy_rot = rotmat*ldrxy_raw;
				map_ptr = calc_map(map_dim, ldrxy_rot);
				area_min = map_ptr[0];
				theta_dir = 0;
				
				// Check area of theta - step
				rotmat = calc_rotmat(theta - theta_step);
				ldrxy_rot = rotmat*ldrxy_raw;
				map_ptr = calc_map(map_dim, ldrxy_rot);
				if (map_ptr[0] < area_min) {
					area = area_min;
					theta_dir = -1;
				}
				
				// Check area of theta + step
				rotmat = calc_rotmat(theta + theta_step);
				ldrxy_rot = rotmat*ldrxy_raw;
				map_ptr = calc_map(map_dim, ldrxy_rot);
				if (map_ptr[0] < area_min) {
					area = area_min;
					theta_dir = 1;
				}
				theta += theta_dir*theta_step;
				if (theta_dir == 0) theta_step = theta_step / 2;
			}
			theta = (theta + 360) % 360;
			
			rotmat = calc_rotmat(theta);
			ldrxy_rot = rotmat*ldrxy_raw;
			map_ptr = calc_map(map_dim, ldrxy_rot);

			if (map_ptr[1] < map_ptr[2]) {
				theta += 90;
				theta = theta % 360;
				rotmat = calc_rotmat(theta);
				ldrxy_rot = rotmat*ldrxy_raw;
				map_ptr = calc_map(map_dim, ldrxy_rot);
			}
			if (calc_agl(theta, theta_prev) > calc_agl(theta + 180, theta_prev)) {
				theta += 180;
				theta = theta % 360;
				rotmat = calc_rotmat(theta);
				ldrxy_rot = rotmat*ldrxy_raw;
				map_ptr = calc_map(map_dim, ldrxy_rot);
			}

			// Coarsely find the wall position
			x_abs = 0;
			max_cnt = 0;
			for (x_ref = -2.6; x_ref < 2.6; x_ref += 0.1) {
				cur_cnt = count_x(ldrxy_rot, x_ref, 0.05);
				if (cur_cnt > max_cnt) {
					x_max = x_ref;
					max_cnt = cur_cnt;
				}
			}
			x_abs = - avg_x(ldrxy_rot, x_max, 0.05);
			if (x_max > 0) {
				x_abs += 5;
			}
			if (abs(x_abs - x_prev) > abs(x_abs - 1 - x_prev)) x_abs -= 1;

			y_abs = map_ptr[4];
			
			if (abs(x_abs - x_prev) > 0.5 || abs(y_abs - y_prev) > 0.5 || (x_abs == 0 && y_abs == 0)) {
				std::cout << "Error : x at " << x_abs << " y at " << y_abs << std::endl;
				theta = theta_prev;
				x_abs = x_prev;
				y_abs = y_prev;
			}

			map_data.at(0) = theta;
			map_data.at(1) = x_abs;
			map_data.at(2) = y_abs;

			map_msg.data = map_data;
			pub_map.publish(map_msg);
			std::cout << "t : " << map_data.at(0) << "\tx : " << map_data.at(1) << "\ty : " << map_data.at(2) << std::endl;
			
			theta_prev = theta;
			x_prev = x_abs;
			y_prev = y_abs;

			// Loop A - 2 : Position objects on the map, to be implemented
			
			


			// Loop A - 3 : Find optimal global & local path, to be implemented
		}
		else {
			// Loop B : ENTER MODE (entrance)

			ent_cnt = 0;
			ent_avg = 0;
			ent_cnts = 0;
			ent_avgs = 0;
			for (i_ldr = 90; i_ldr < n_ldr - 90; i_ldr++) {
				// Consider 180 deg front
				if (lidar_distance[i_ldr] > 1) {
					ent_cnt++;
					ent_avg += DEG(lidar_angle[i_ldr] - RAD(theta_offset));
				}
				else {
					if (ent_cnt > ent_cnts) {
						ent_cnts = ent_cnt;
						ent_avgs = ent_avg;
						ent_cnt = 0;
						ent_avg = 0;
					}
				}
			}
			if (ent_cnt > ent_cnts) {
				ent_cnts = ent_cnt;
				ent_avgs = ent_avg;
				ent_cnt = 0;
				ent_avg = 0;
			}
			if (ent_cnts > 0) ent_avgs /= ent_cnts;
			std::cout << "Entrance navigation should preceed on " << ent_avgs << std::endl;

		}

		loop_rate.sleep();
	}

	return 0;
}
