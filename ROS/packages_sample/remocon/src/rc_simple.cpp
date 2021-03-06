#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <iostream>
#include <string>
#include <sstream>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "rc_simple");
  ros::NodeHandle n;
  ros::Publisher remote_pub = n.advertise<std_msgs::Float64>("/simple_machine/joint1_position_controller/command", 1000);
  ros::Rate loop_rate(100);


  float angle = 0;
  int i_rot, n_rot;
  float d_angle;
  char command[16];

  std::cout << "[rc_simple] Command format is (+ or -)(number to rotate)\n";
  while (ros::ok())
  {
    std::cout << "[rc_simple] Command : ";
    std::cin >> command;
    if (command[0] == '+'){
      std::cout << "[rc_simple] Direciton (+) ";
      d_angle = 0.1;
    }
    else if (command[0] == '-'){
      std::cout << "[rc_simple] Direciton (-) ";
      d_angle = -0.1;
    }
    else{
      std::cout << "[rc_simple] Invalid format\n";
      continue;
    }
    std::istringstream(command + 1) >> n_rot;
    std::cout << "[rc_simple] Rotation " << n_rot << "rad\n";

    for (i_rot = 0; i_rot < n_rot * 10; i_rot++){
      std_msgs::Float64 angle_msg;
      angle_msg.data = angle;
      remote_pub.publish(angle_msg);
      loop_rate.sleep();
      angle = angle + d_angle;
    }
  }

  return 0;
}
