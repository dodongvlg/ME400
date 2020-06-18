/*
Data Integration Node
KAIST ME400 Team H
Last modified by Gunwoo Park, 2020. 06. 17.
Version 7 : Setting docking position considering walls / Get flags from OpenCV
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
#include <gazebo_msgs/ModelStates.h>
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

// Global Variables directly from Gazebo
float x_ball[6];
float y_ball[6];

// Global Variables for ball coordinates management.
///// TO DO /////
int ballpos_map[500][300];
float ballpos_temp[10];

int region_flag = 0;
// int noball_flag = 0;

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

// L2 distance calculation
// Input : two points
// Output : angular distance
float calc_dist(float x1, float x2, float y1, float y2) {
	return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
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

// Callback 3 : Get ball position from Gazebo (To be replaced to OpenCV)
// void model_Callback(const gazebo_msgs::ModelStates::ConstPtr& model) {
// 	map_mutex.lock();
// 	int i_ball;
// 	int n_ball = 6;
// 	// Modified to ignore balls in the hole.
// 	for (i_ball = 0; i_ball < n_ball; i_ball++) {
// 		if (model->pose[i_ball + 6].position.z > 0.3) {
// 			x_ball[i_ball] = model->pose[i_ball + 6].position.x;
// 			y_ball[i_ball] = model->pose[i_ball + 6].position.y;
// 		}
// 		else {
// 			x_ball[i_ball] = -8;
// 			y_ball[i_ball] = -3;
// 		}
// 	}
// 	map_mutex.unlock();
// }

// Callback 4 : Callback for ball coordinate management
///// TO DO ////
void ballpos_Callback(const std_msgs::Float64MultiArray::ConstPtr& ballpos) {
	map_mutex.lock();
	int i;
	for (i = 0; i < 10; i++) {
		ballpos_temp[i] = -1;
	}
	for (i = 0; i < sizeof(ballpos -> data); i++) {
		if (i > 10) {
			continue;
		}
		ballpos_temp[i] = ballpos -> data[i];
	}
	map_mutex.unlock();
}

void flag_Callback(const std_msgs::Float64::ConstPtr& flag) {
	map_mutex.lock();
	region_flag = flag->data;
	map_mutex.unlock();
}

// void noball_Callback(const std_msgs::Float64::ConstPtr& flag) {
// 	map_mutex.lock();
// 	noball_flag = flag->data;
// 	map_mutex.unlock();
// }


// MAIN FUNCTION
int main(int argc, char **argv)
{
	// Local Variables for main()
	Eigen::Matrix<float, 2, n_ldr> ldrxy_raw, ldrxy_rot;
	Eigen::Matrix<float, 2, 2> rotmat;

	// Local Variables for loop 1
	int i_ldr, pi_ldr, cur_cnt, max_cnt;
	int ent_cnt, ent_avg, ent_cnts, ent_avgs, ent_thresh; // For entrance navigation
	int theta, theta_step, theta_dir, theta_prev, theta_offset; // For theta optimization
	int mode = ENTER; // ENTER == 0, STAGE == 1
	float area, area_min, x_abs, y_abs, x_ref, x_max, x_prev, y_prev, x_ldr, x_sum; // Map parameters
	float map_dim[5];
	float *map_ptr;
	std::vector<double> map_data(7);
	std::vector<double> obj_data(20);
	float ldr_offset = 0.2;

	// Local Variables for loop 3
	int i_ball, i_obs, near_ball, danger;
	float agl_dock, agl_hole, agl_obs, dist, near_dist;
	float x_obj0, y_obj0, x_obj1, y_obj1;

	// Local Constants for loop 3
	int m_ball = 0; // TO DO
	int n_ball = 1; // TO DO
	int n_obs = 4; // TO DO
	float x_hole = 8;
	float y_hole = 1.5;
	float dst_dock = 0.4;
	float x_obs[4] = {4, 5.5, 5.5, 7};
	float y_obs[4] = {1.5, 0.7, 2.3, 1.5};
	float dst_obs = 0.4;

	// Init : ROS initialization and configuration
	ros::init(argc, argv, "data_integration");
	ros::NodeHandle n;
	ros::Rate loop_rate(20); // Modify this value to change loop rate for ~Hz.
	ros::Subscriber sub_lidar = n.subscribe<sensor_msgs::LaserScan>("/scan", 256, lidar_Callback);
	ros::Subscriber sub_camera = n.subscribe<core_msgs::ball_position>("/position", 256, camera_Callback);
	// ros::Subscriber sub_model = n.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 256, model_Callback);
	ros::Subscriber sub_ballpos = n.subscribe<std_msgs::Float64MultiArray>("/ball_position", 256, ballpos_Callback); ///// TO DO /////
	ros::Subscriber sub_flag = n.subscribe<std_msgs::Float64>("/regionFlag", 256, flag_Callback); ///// TO DO /////
	// ros::Subscriber sub_noball = n.subscribe<std_msgs::Float64>("/no_ball", 256, noball_Callback); ///// TO DO /////


	ros::Publisher pub_mode = n.advertise<std_msgs::Float64>("/mapdata/mode", 16);
	ros::Publisher pub_map = n.advertise<std_msgs::Float64MultiArray>("/mapdata/stage_position", 16);
	ros::Publisher pub_ent = n.advertise<std_msgs::Float64>("/mapdata/enter_direction", 16);

	std_msgs::Float64MultiArray map_msg; // vehicle state : [theta, x, y]
	std_msgs::Float64 ent_msg; // entrance direction (message)
	std_msgs::Float64 mode_msg; // mode (message)

	ent_thresh = 50;
        theta_prev = 0;
	x_prev = 2.75;
	y_prev = 0.5;
	theta_offset = 0;

			map_msg.data = map_data;
			pub_map.publish(map_msg);

	///// TO DO /////
	// Initialize map
	for (int tmp1=0; tmp1 < 500; tmp1++) {
		for (int tmp2=0; tmp2 < 300; tmp2++) {
			ballpos_map[tmp1][tmp2] = 0;
		}
	}

	// Loop : Process data and Publish message every 200ms
	while(ros::ok){

		ros::spinOnce(); // Run callback functions

		// Check whether the vehicle entered the stage
		ent_cnt = 0;
		for (i_ldr = -90; i_ldr <= 90; i_ldr++) {
			pi_ldr = i_ldr;
			if (i_ldr < 0) pi_ldr = i_ldr + 360;
			// Consider 180 deg front
			if (lidar_distance[pi_ldr] > 3) {
				ent_cnt++;
			}
		}

		if (ent_cnt > ent_thresh && region_flag) mode = STAGE; // Mode transition
		std::cout << std::endl;
		std::cout << "Current mode : " << mode << " open " << ent_cnt << std::endl;

		mode_msg.data = mode;
		pub_mode.publish(mode_msg);		

		// mode = 1;  TO DO --- ADDED FOR DEBUGGING //
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

			y_abs = map_ptr[4] * 3 / map_ptr[2];

			x_abs = x_abs + ldr_offset * cos(RAD(theta));
			y_abs = y_abs - ldr_offset * sin(RAD(theta));

			x_abs += 3; // Fit to global coordinate

			if (abs(x_abs - x_prev) > abs(x_abs - 1 - x_prev)) x_abs -= 1;
			
			if (abs(x_abs - x_prev) > 0.5 || abs(y_abs - y_prev) > 0.5 || (x_abs == 0 && y_abs == 0)) {
				std::cout << "Error : x at " << x_abs << " y at " << y_abs << std::endl;
				theta = theta_prev;
				x_abs = x_prev;
				y_abs = y_prev;
			}

			map_data.at(0) = theta;
			map_data.at(1) = x_abs;
			map_data.at(2) = y_abs;
			
			theta_prev = theta;
			x_prev = x_abs;
			y_prev = y_abs;
			std::cout << "Position : " << map_data.at(1) << ", " << map_data.at(2) << "  Orientation : " << map_data.at(0) << std::endl;

			// Loop A - 2 : Position objects on the map, to be implemented
			// TODO //
			// std::cout << "Entered loop2" << std::endl;;
			
			float theta_temp[5];
			float dist_temp;
			float ballx_temp, bally_temp;
			int ballx, bally;
			int a2_cnt;
			float theta_rad = RAD(-theta);

			if (ballpos_temp[0] == -1) {
				std::cout << "No new info about ball" << std::endl;
			}
			else if (std::isnan(x_abs) || std::isnan(y_abs)) {
				std::cout << "Car pos is nan so cannot calculate ball position" << std::endl;
			}
			else {
				for (a2_cnt=0; a2_cnt < 10; a2_cnt += 2) { // Calculate angle to the ball
					if (ballpos_temp[a2_cnt] != 0) {
						theta_temp[a2_cnt / 2] = atan(ballpos_temp[a2_cnt + 1] / ballpos_temp[a2_cnt]);
					}
					else {
						theta_temp[a2_cnt / 2] = -1;
					}
				}

				for (a2_cnt=0; a2_cnt < 5; a2_cnt++) { // Put ball position to ballpos_map
					if (theta_temp[a2_cnt] == -1) { // Case no ball
						continue;
					}
					else { // Calculation by distance and angle to the ball
						ballx_temp = ballpos_temp[2 * a2_cnt];
						bally_temp = ballpos_temp[2 * a2_cnt + 1];
						dist_temp = sqrt(ballx_temp * ballx_temp + bally_temp * bally_temp);
						ballx_temp = x_abs + dist_temp * cos(theta_temp[a2_cnt] + theta_rad);
						bally_temp = y_abs + dist_temp * sin(theta_temp[a2_cnt] + theta_rad);
						ballx_temp = round(100 * ballx_temp);
						bally_temp = round(100 * bally_temp);
						ballx = (int) ballx_temp;
						bally = (int) bally_temp;
						if ((ballx >= 307) && (ballx <= 793) && (bally >= 7) && (bally <= 293)) { 
							ballpos_map[ballx - 300][bally] += 1;
						}
					}
				}
			}

			float car_p1_x, car_p2_x, car_p3_x, car_p4_x;
			float car_p1_y, car_p2_y, car_p3_y, car_p4_y;
			float car_r1 = sqrt(0.144 * 0.144 + 0.23422 * 0.23422);
			float car_r2 = sqrt(0.3235 * 0.3235 + 0.23422 * 0.23422);
			float car_th1 = atan(234.22 / 144);
			float car_th2 = atan(234.22 / 323.5);
			int cx1, cx2, cy1, cy2;

			car_p1_x = x_abs + car_r1 * cos(theta_rad + car_th1);
			car_p1_y = y_abs + car_r1 * sin(theta_rad + car_th1);
			car_p2_x = x_abs + car_r1 * cos(theta_rad - car_th1);
			car_p2_y = y_abs + car_r1 * sin(theta_rad - car_th1);
			car_p3_x = x_abs - car_r2 * cos(theta_rad + car_th2);
			car_p3_y = y_abs - car_r2 * sin(theta_rad + car_th2);
			car_p4_x = x_abs - car_r2 * cos(theta_rad - car_th2);
			car_p4_y = y_abs - car_r2 * sin(theta_rad - car_th2);

			int i_car_p1_x, i_car_p2_x, i_car_p3_x, i_car_p4_x;
			int i_car_p1_y, i_car_p2_y, i_car_p3_y, i_car_p4_y;

			i_car_p1_x = (int) round(100 * car_p1_x) - 300;
			i_car_p1_y = (int) round(100 * car_p1_y);
			i_car_p2_x = (int) round(100 * car_p2_x) - 300;
			i_car_p2_y = (int) round(100 * car_p2_y);
			i_car_p3_x = (int) round(100 * car_p3_x) - 300;
			i_car_p3_y = (int) round(100 * car_p3_y);
			i_car_p4_x = (int) round(100 * car_p4_x) - 300;
			i_car_p4_y = (int) round(100 * car_p4_y);

			// float ml1 = (i_car_p2_y - i_car_p1_y) / (i_car_p2_x - i_car_p1_x);
			// float ml2 = (i_car_p3_y - i_car_p2_y) / (i_car_p3_x - i_car_p2_x);
			// float ml3 = (i_car_p4_y - i_car_p3_y) / (i_car_p4_x - i_car_p3_x);
			// float ml4 = (i_car_p1_y - i_car_p4_y) / (i_car_p1_x - i_car_p4_x);
			float ml1 = (car_p2_y - car_p1_y) / (car_p2_x - car_p1_x);
			float ml2 = (car_p3_y - car_p2_y) / (car_p3_x - car_p2_x);
			float ml3 = (car_p4_y - car_p3_y) / (car_p4_x - car_p3_x);
			float ml4 = (car_p1_y - car_p4_y) / (car_p1_x - car_p4_x);

			// std::cout << "mls : " << ml1 << ml2 << ml3 << ml4 << std::endl;
			

			if (theta_rad < 0) {
				theta_rad = theta_rad + 2 * M_PI;
			}

			int me1, me2;

			// std::cout << "theta_rad : " << theta_rad << std::endl;
			if ((theta_rad >= 0) && (theta_rad < M_PI/2)) {
				// std::cout << "p1x : " << car_p1_x << std::endl;
				// std::cout << "p1y : " << car_p1_y << std::endl;
				// std::cout << "p2x : " << car_p2_x << std::endl;
				// std::cout << "p2y : " << car_p2_y << std::endl;
				// std::cout << "p3x : " << car_p3_x << std::endl;
				// std::cout << "p3y : " << car_p3_y << std::endl;
				// std::cout << "p4x : " << car_p4_x << std::endl;
				// std::cout << "p4y : " << car_p4_y << std::endl;

				// std::cout << "i_p1x : " << i_car_p1_x << std::endl;
				// std::cout << "i_p1y : " << i_car_p1_y << std::endl;
				// std::cout << "i_p2x : " << i_car_p2_x << std::endl;
				// std::cout << "i_p2y : " << i_car_p2_y << std::endl;
				// std::cout << "i_p3x : " << i_car_p3_x << std::endl;
				// std::cout << "i_p3y : " << i_car_p3_y << std::endl;
				// std::cout << "i_p4x : " << i_car_p4_x << std::endl;
				// std::cout << "i_p4y : " << i_car_p4_y << std::endl;

				// cx1 = (int) round(100 * car_p4_x);
				// cx2 = (int) round(100 * car_p2_x);
				// cy1 = (int) round(100 * car_p3_y);
				// cy2 = (int) round(100 * car_p1_y);
				// std::cout << "1";
				// std::cout << cx1 << " " << cx2 << " " << cy1 << " " << cy2 << std::endl;
				for (me1 = i_car_p4_x; me1 < i_car_p2_x; me1++) {
					for (me2 = i_car_p3_y; me2 < i_car_p1_y; me2++) {
						// std::cout << "fuck";
						if ((me2 < (ml1 * (me1 - i_car_p1_x) + i_car_p1_y)) && (me2 > (ml2 * (me1 - i_car_p2_x) + i_car_p2_y))
								&& (me2 > (ml3 * (me1 - i_car_p3_x) + i_car_p3_y)) && (me2 < (ml4 * (me1 - i_car_p4_x) + i_car_p4_y))) {
									if ((me1 >= 0) && (me1 < 500) && (me2 >= 0) && (me2 < 300)) {
										// std::cout << "me1 : " << me1 << "me2 : " << me2 << std::endl;
										ballpos_map[me1][me2] = 0;
									}
						}
					}
				}
				// for (cx1; cx1 < cx2; cx1++) {
				// 	for (cy1; cy1 < cy2; cy1++) {
				// 		if ((cy1 < ml1 * (cx1 - car_p1_x) + car_p1_y) && (cy1 > ml2 * (cx1 - car_p2_x) + car_p2_y)
				// 				&& (cy1 > ml3 * (cx1 - car_p3_x) + car_p3_y) && (cy1 < ml4 * (cx1 - car_p4_x) + car_p4_y)) {
				// 					std::cout << "1";
				// 					ballpos_map[cx1 - 300][cy1] = 0;
				// 		}
				// 	}
				// }
			}
			else if ((theta_rad >= M_PI/2) && (theta_rad < M_PI)) {
				for (me1 = i_car_p1_x; me1 < i_car_p3_x; me1++) {
					for (me2 = i_car_p4_y; me2 < i_car_p2_y; me2++) {
						if ((me2 < (ml1 * (me1 - i_car_p1_x) + i_car_p1_y)) && (me2 < (ml2 * (me1 - i_car_p2_x) + i_car_p2_y))
								&& (me2 > (ml3 * (me1 - i_car_p3_x) + i_car_p3_y)) && (me2 > (ml4 * (me1 - i_car_p4_x) + i_car_p4_y))) {
									if ((me1 >= 0) && (me1 < 500) && (me2 >= 0) && (me2 < 300)) {
										ballpos_map[me1][me2] = 0;
									}
						}
					}
				}
			}
			else if ((theta_rad >= M_PI) && (theta_rad < 3 * M_PI/2)) {
				for (me1 = i_car_p2_x; me1 < i_car_p4_x; me1++) {
					for (me2 = i_car_p1_y; me2 < i_car_p3_y; me2++) {
						if ((me2 > (ml1 * (me1 - i_car_p1_x) + i_car_p1_y)) && (me2 < (ml2 * (me1 - i_car_p2_x) + i_car_p2_y))
								&& (me2 < (ml3 * (me1 - i_car_p3_x) + i_car_p3_y)) && (me2 > (ml4 * (me1 - i_car_p4_x) + i_car_p4_y))) {
									if ((me1 >= 0) && (me1 < 500) && (me2 >= 0) && (me2 < 300)) {
										ballpos_map[me1][me2] = 0;
									}
						}
					}
				}
			}
			else {
				for (me1 = i_car_p3_x; me1 < i_car_p1_x; me1++) {
					for (me2 = i_car_p2_y; me2 < i_car_p4_y; me2++) {
						if ((me2 > (ml1 * (me1 - i_car_p1_x) + i_car_p1_y)) && (me2 > (ml2 * (me1 - i_car_p2_x) + i_car_p2_y))
								&& (me2 < (ml3 * (me1 - i_car_p3_x) + i_car_p3_y)) && (me2 < (ml4 * (me1 - i_car_p4_x) + i_car_p4_y))) {
									if ((me1 >= 0) && (me1 < 500) && (me2 >= 0) && (me2 < 300)) {
										ballpos_map[me1][me2] = 0;
									}
						}
					}
				}
			}

			int old_ballx, old_bally;

			// if (noball_flag == 1) {
			// 	std::cout << "noball flag" << std::endl;
			// 	if ((map_data.at(5) > 3) && (map_data.at(6) > 0)) {
			// 		std::cout << "erasing" << std::endl;
			// 		old_ballx = (int) round(100 * map_data.at(5));
			// 		old_bally = (int) round(100 * map_data.at(6));
			// 		for (cx1 = -7; cx1 < 8; cx1++) {
			// 			for (cx2 = -7; cx2 < 8; cx2++) {
			// 				ballpos_map[old_ballx - 300 + cx1][old_bally + cx2] = 0;
			// 			}
			// 		}
			// 	} 
			// }

			// Find top area
			int top[3];
			int findx, findy;

			for (findx = 0; findx < 3; findx++) {
				top[findx] = -1;
			}

			for (findx = 0; findx < 500; findx++) {
				for (findy = 0; findy < 300; findy++) {
					if ((ballpos_map[findx][findy] != 0) && (ballpos_map[findx][findy] > top[0])) {
						top[0] = ballpos_map[findx][findy];
						top[1] = findx;
						top[2] = findy;
					}
				}
			}

			if (top[0] == -1) {
				x_ball[0] = -1;
				y_ball[0] = -1;
			}
			else {
				x_ball[0] = ((float) top[1]) / 100 + 3;
				y_ball[0] = ((float) top[2]) / 100;
			}

			std::cout << "x_ball : " << x_ball[0] << std::endl;
			std::cout << "y_ball : " << y_ball[0] << std::endl;
			
			// TO DO ENDS //


			// Loop A - 3 : Find optimal global & local path
			// near_ball = 0;
			// near_dist = 5;
			// for (i_ball = m_ball; i_ball < n_ball; i_ball++) {
			// 	dist = calc_dist(x_abs, x_ball[i_ball], y_abs, y_ball[i_ball]);
			// 	if (dist < near_dist) {
			// 		near_dist = dist;
			// 		near_ball = i_ball;
			// 	}
			// }
			// x_obj1 = x_ball[near_ball];
			// y_obj1 = y_ball[near_ball];
			near_ball = 0;
			x_obj1 = x_ball[near_ball];
			y_obj1 = y_ball[near_ball];

			// std::cout << "xobj0 : " << x_obj0 << std::endl;
			// std::cout << "yobj0 : " << y_obj0 << std::endl;
			// std::cout << "xobj1 : " << x_obj1 << std::endl;
			// std::cout << "yobj1 : " << y_obj1 << std::endl;

			if ((x_obj1 == (float) -1) || (y_obj1 == (float) -1)) {
				// std::cout << "case -1" << std::endl;
				x_obj0 = -1;
				y_obj0 = -1;
				x_obj1 = -1;
				y_obj1 = -1;
			}
			else {
				// std::cout << "not case -1" << std::endl;
				agl_dock = atan((y_hole - y_obj1) / (x_hole - x_obj1));
				
				danger = 1;
				while (danger) {
					x_obj0 = x_ball[near_ball] - dst_dock * cos(agl_dock);
					y_obj0 = y_ball[near_ball] - dst_dock * sin(agl_dock);
					danger = 0;
					for (i_obs = 0; i_obs < n_obs; i_obs++) {
						if (calc_dist(x_obj0, x_obs[i_obs], y_obj0, y_obs[i_obs]) < dst_obs) {
							danger = 1;
							agl_obs = atan((x_hole - x_obs[i_obs]) / (y_hole - y_obs[i_obs]));
							agl_dock = agl_dock + RAD(2 * (agl_dock - agl_obs) / abs(agl_dock - agl_obs));
						}
					}
					
				}
				// std::cout << "xobj0 : " << x_obj0 << std::endl;
				// std::cout << "yobj0 : " << y_obj0 << std::endl;
				// std::cout << "xobj1 : " << x_obj1 << std::endl;
				// std::cout << "yobj1 : " << y_obj1 << std::endl;


				if (x_obj0 < 3.25) x_obj0 = 3.25;
				if (x_obj0 > 7.75) x_obj0 = 7.75;
				if (y_obj0 < 0.25) y_obj0 = 0.25;
				if (y_obj0 > 2.75) y_obj0 = 2.75;
			}

			std::cout << "Path 1 : " << x_abs << ", " << y_abs << " -> " << x_obj0 << ", " << y_obj0 << std::endl;
			std::cout << "Path 2 : " << x_obj0 << ", " << y_obj0 << " -> " << x_obj1 << ", " << y_obj1 << std::endl;
			std::cout << "Path 3 : " << x_obj0 << ", " << y_obj1 << " -> " << x_hole - dst_dock << ", " << y_hole << std::endl;
			

			map_data.at(3) = x_obj0; // x coordinate of docking position
			map_data.at(4) = y_obj0; // y coordinate of docking position
			map_data.at(5) = x_obj1; // x coordinate of ball position
			map_data.at(6) = y_obj1; // y coordinate of ball position
			std::cout << std::endl;

			map_msg.data = map_data;
			pub_map.publish(map_msg);

		}
		else {
			// Loop B : ENTER MODE (entrance)

			ent_cnt = 0;
			ent_avg = 0;
			ent_cnts = 0;
			ent_avgs = 0;

			for (i_ldr = -90; i_ldr <= 90; i_ldr++) {
				pi_ldr = i_ldr;
				if (i_ldr < 0) pi_ldr = i_ldr + 360;
				// Consider 180 deg front
				if (lidar_distance[pi_ldr] > 1) {
					ent_cnt++;
					ent_avg += DEG(lidar_angle[pi_ldr]);
					if (i_ldr < 0) ent_avg -= 360;
					
					// ent_avg += DEG(lidar_angle[pi_ldr] - RAD(theta_offset));
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

			if (ent_cnts > 0) ent_avgs /= ent_cnts;
			std::cout << "Entrance navigation should preceed on " << ent_avgs << std::endl;
			ent_msg.data = ent_avgs;
			pub_ent.publish(ent_msg);

		}

		loop_rate.sleep();
	}

	return 0;
}
