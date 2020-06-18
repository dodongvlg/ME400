### 1. installation and build the file

install dependent packages
```
sudo apt-get install ros-melodic-joy ros-melodic-teleop-twist-joy ros-melodic-teleop-twist-keyboard ros-melodic-laser-proc ros-melodic-rgbd-launch ros-melodic-depthimage-to-laserscan ros-melodic-rosserial-arduino ros-melodic-rosserial-python ros-melodic-rosserial-server ros-melodic-rosserial-client ros-melodic-rosserial-msgs ros-melodic-amcl ros-melodic-map-server ros-melodic-move-base ros-melodic-urdf ros-melodic-xacro ros-melodic-compressed-image-transport ros-melodic-rqt-image-view ros-melodic-gmapping ros-melodic-navigation ros-melodic-interactive-markers

```

install gazebo_ros_control
```
sudo apt-get install ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-ros-control
sudo apt install python-pip
pip install scikit-image
sudo apt get update
sudo apt get upgrade
```
down load source file
```
git clone https://github.com/dodongvlg/ME400.git
```
build the code
```
cd ~/ROS_H
catkin_make
```


### 2. running the code

map open
```
roslaunch map_generate import_world.launch
```
spawn the robot
```
roslaunch toy_car two_spawn_entrance.launch
```
Data Integration node on
```
rosrun data_integrate data_integration_node
```
OpenCV Node on
```
rosrun opencv linetracing.py
rosrun opencv opencv.py
```
Start autonomous driving
```
rosrun path_planning path_planning.py
```
