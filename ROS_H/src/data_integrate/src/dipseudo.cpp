// MAIN Function
int main(int argc, char **argv)
{

  // Initialization : declare local variables and configure ROS structrues
    
  // Loop : run conditional statements with 20Hz rate
	while(ros::ok){

    // Run Callbacks : recieve data from Lidar and OpenCV, then store it to global variables
    // Mode Transition : check mode transition condition
    
    if(mode){
      // Stage Navigation (mode == 1) : localization and global path planning
    }
    else{
      // Entrance Navigation (mode == 0) : line tracing
    }
    
  }
  
}


