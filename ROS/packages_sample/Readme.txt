remocon :
- CUI manipulation for sample packages.

simple_car:
to spawn simple car model in already open environment in Gazebo (ex. )
--> roslaunch simple_car spawn_simple_car.launch

to open empty world and spawn simple car model
--> roslaunch simple_car simple_car_empty.launch

to load velocity controller, state publisher to simple_car
--> roslaunch simple_car simple_control.launch

to load CUI manipulator, run this node.
--> rosrun remocon rc_simcar

simple_car_1_urdf: contains modelling info about simple_car. made by SolidWorks.

path_planning:
make sure to make python script executable
chmod +x path/to/path_planning.py
--> rosrun path_planning path_planning.py

toy_car:
--> roslaunch toy_car two_spawn_control.launch
This spawns the robot with controller attached as well.


Package remocon and simple_car first should be built in your catkin workspace, added by source ... / catkin_ws/devel/setup.bash


