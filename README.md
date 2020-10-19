# RoboticsM

An assignment for the Robot Programming module for MSc Robotics and Autonomous Systems course in University of Lincoln

The chosen focus of area was Navigation, the robot has to navigate through the environment with various different objects, from static to dynamic.

A package converts pointclouds from velodyne rings into laserscan data, this was used to detect objects that were low down.

To install:  
`rosdep --from-paths . -i -y`  
`catkin_make`  
`source ./devel/setup.bash`  
`roslaunch ass1_1 mapping.launch`  
`rosrun ass1_1 test.py`  

Unique features:  
 - can detect and map low objects
 - can stop before hitting objects
 - converts pointclouds to laserscan
