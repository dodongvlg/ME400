/*
CUI manipulator for simple_car model
KAIST ME400 Team H
Gunwoo Park, 2020. 04. 24.
*/

#include "ros/ros.h"
#include <std_msgs/Float64MultiArray.h>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
/*
float *setvel(float *vel, float *xvel, float spd)
{
  int i_vel;
  for (i_vel = 0; i_vel < 4; i_vel++)
  {
    vel[i_vel] = xvel[i_vel] * spd;
  }
  return vel;
}
*/

int main(int argc, char **argv)
{

  ros::init(argc, argv, "rc_simcar");
  ros::NodeHandle n;
  ros::Publisher remote_pub = n.advertise<std_msgs::Float64MultiArray>("/simple_car/joint_vel_controller/command", 256);
  ros::Rate loop_rate(10);

  char command[4];
  float spd;
  std::vector<double> vel(4); 


  std::cout << "RC_SIMCAR Running" << std::endl;
  std::cout << "w[vel] : Go straight" << std::endl;
  std::cout << "a[vel] : Turn left" << std::endl;
  std::cout << "d[vel] : Turn right" << std::endl;
  std::cout << "s[vel] : Stop" << std::endl;

  while (ros::ok())
  {
    std::cout << "[rc_simcar] Command : ";
    std::cin >> command;
    std::istringstream(command + 1) >> spd;
    
    switch(command[0])
    {
      case 'w' :
      std::cout << "[rc_simcar] Go straight ";
      vel = {-spd, -spd, spd, spd};
      break;
      case 'a' :
      std::cout << "[rc_simcar] Turn left ";
      vel = {spd, -spd, spd, spd};
      break;
      case 'd' :
      std::cout << "[rc_simcar] Turn right ";
      vel = {-spd, -spd, -spd, spd};
      break;
      case 's' :
      std::cout << "[rc_simcar] Stop ";
      vel = {0, 0, 0, 0};
      break;
    }

    std::cout << "with speed " << spd << std::endl;

    std_msgs::Float64MultiArray vel_msg;
    vel_msg.data = vel;
    remote_pub.publish(vel_msg);
  }

  return 0;
}
