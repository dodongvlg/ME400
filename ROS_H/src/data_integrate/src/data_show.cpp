/*

Data Integration Node
KAIST ME400 Team H
Last modified by Gunwoo Park, 2020. 04. 29.
Version 1 : Implemented Map Visualization

This code is composed with init, loop, and two callback parts.

Init : Executes once if the node starts to run.
Loop : Executes periodically during the node runtime.
Callback : Executes every time the subscriber recieves the message.

*/

// Standard Header Files for C++
#include <iostream>
#include <stdio.h>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <string>

// Standard Header Files for C
#include <signal.h>
#include <math.h>
#include <boost/thread.hpp>

// Header Files for ROS & OpenCV
#include <ros/ros.h>
#include <ros/package.h>
#include <core_msgs/ball_position.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float64MultiArray.h>
#include <opencv2/opencv.hpp>

#define DEG(x) ((x)*180./M_PI)
#define RAD(x) ((x)*M_PI/180)

boost::mutex map_mutex;

// Global Variables : DATA
float MAP_CX = 200.5;
float MAP_CY = 200.5;
float MAP_RESOL = 0.015;             // Map resoultion [cm]
int MAP_WIDTH = 400;
int MAP_HEIGHT = 400;
int MAP_CENTER = 50;
int OBSTACLE_PADDING = 2;           // Obstacle Size
int OBSTACLE_CONNECT_MAX = 15;      // Range to connect obstacles

int lidar_size;
float lidar_degree[400];
float lidar_distance[400];

int ball_number;
float ball_X[20];
float ball_Y[20];

// Global Variables for Map Reconstruction
int theta;
float width, height, sys_x, sys_y;

/*

bool check_point_range(int cx, int cy)
{
	return (cx<MAP_WIDTH-1)&&(cx>0)&&(cy<MAP_HEIGHT-1)&&(cy>0);
}

void camera_Callback(const core_msgs::ball_position::ConstPtr& position)
{
	map_mutex.lock();
	int count = position->size;
	ball_number=count;
	for(int i = 0; i < count; i++)
	{
		ball_X[i] = position->img_x[i];
		ball_Y[i]=position->img_y[i];
		std::cout << "ball_X : "<< ball_X[i];
		std::cout << "ball_Y : "<< ball_Y[i]<<std::endl;
	}
	map_mutex.unlock();
}

void lidar_Callback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	map_mutex.lock();
	int count = scan->angle_max / scan->angle_increment;
	lidar_size=count;
	for(int i = 0; i < count; i++)
	{
		lidar_degree[i] = scan->angle_min + scan->angle_increment * i;
		lidar_distance[i]=scan->ranges[i];
	}
	map_mutex.unlock();
}
*/

void map_Callback(const std_msgs::Float64MultiArray map_msg)
{
	map_mutex.lock();

	std::vector<double> map_data = map_msg.data;
	
	theta = map_data.at(0);
	width = map_data.at(1)*100;
	height = map_data.at(2)*100;
	sys_x = map_data.at(3)*100;
	sys_y = map_data.at(4)*100;

	map_mutex.unlock();
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "data_show_node");
	ros::NodeHandle n;
	ros::Rate loop_rate(5);
	ros::Subscriber sub_map = n.subscribe<std_msgs::Float64MultiArray>("/mapdata", 256, map_Callback);

	// ros::Subscriber sub_lidar = n.subscribe<sensor_msgs::LaserScan>("/scan", 256, lidar_Callback);
	// ros::Subscriber sub_camera = n.subscribe<core_msgs::ball_position>("/position", 256, camera_Callback);

	while(ros::ok){

		// Drawing Map Data
		if (width > 0 && height >0)
		{
			cv::Mat map = cv::Mat::zeros(height, width, CV_8UC3);

			cv::circle(map,cv::Point(sys_x, height - sys_y),3,cv::Scalar(255,255,0),-1);
			
			cv::imshow("Frame",map);
			cv::waitKey(50);
		}

		if(cv::waitKey(50)==113) return 0;



	/**
		cv::Mat map = cv::Mat::zeros(MAP_WIDTH, MAP_HEIGHT, CV_8UC3);

		// Drawing Lidar data

		float obstacle_x, obstacle_y;
		int cx, cy;
		int cx1, cx2, cy1, cy2;
		for(int i = 0; i < lidar_size; i++)
		{
			obstacle_x = lidar_distance[i]*cos(lidar_degree[i]);
			obstacle_y = lidar_distance[i]*sin(lidar_degree[i]);
			cx = MAP_WIDTH/2 + (int)(obstacle_y/MAP_RESOL);
			cy = MAP_HEIGHT/2 + (int)(obstacle_x/MAP_RESOL);
			cx1 = cx-OBSTACLE_PADDING;
			cy1 = cy-OBSTACLE_PADDING;
			cx2 = cx+OBSTACLE_PADDING;
			cy2 = cy+OBSTACLE_PADDING;

			if(check_point_range(cx,cy) && check_point_range(cx1,cy1) && check_point_range(cx2,cy2))
			{
				cv::line(map, cv::Point(MAP_WIDTH/2, MAP_HEIGHT/2),cv::Point(cx, cy),cv::Scalar(63,63,0), 1);
				cv::rectangle(map,cv::Point(cx1, cy1),cv::Point(cx2, cy2), cv::Scalar(255,255,0), -1);
			}
		}
		// Drawing ball
		for(int i = 0; i < ball_number; i++)
		{
			cx =(int)(ball_X[i]/4);
			cy =(int)(ball_Y[i]/4);
			cx1 = cx-OBSTACLE_PADDING*2;
			cy1 = cy-OBSTACLE_PADDING*2;
			cx2 = cx+OBSTACLE_PADDING*2;
			cy2 = cy+OBSTACLE_PADDING*2;
	
			if(check_point_range(cx,cy) && check_point_range(cx1,cy1) && check_point_range(cx2,cy2))
			{
				cv::rectangle(map,cv::Point(cx1, cy1),cv::Point(cx2, cy2), cv::Scalar(0,0,255), -1);
			}
		}
		// Drawing ROBOT
		cv::circle(map,cv::Point(MAP_WIDTH/2, MAP_HEIGHT/2),3,cv::Scalar(255,0,0),-1);
		cv::imshow("Frame",map);
		cv::waitKey(50);

		if(cv::waitKey(50)==113){  //wait for a key command. if 'q' is pressed, then program will be terminated.
			return 0;
		}

	**/
		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}
