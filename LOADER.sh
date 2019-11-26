#!/bin/sh

sudo apt-get update && sudo apt-get upgrade
sudo apt-get purge "*gazebo*"
sudo apt-get install ros-kinetic-uol-cmp9767m-base
sudo apt-get install ros-kinetic-image-view
sudo apt-get install ros-kinetic-rqt-graph
sudo apt-get install ros-kinetic-rviz

sudo apt-get install \
    ros-kinetic-robot-localization \
    ros-kinetic-topological-navigation \
    ros-kinetic-amcl \
    ros-kinetic-fake-localization \
    ros-kinetic-carrot-planner \
    ros-kinetic-gmapping\
    ros-kinetic-roslint\
    ros-kinetic-velodyne-laserscan\
    libpcap-dev\


source ./RoboticsM/CMP9767M-master/catkin_ws/devel/setup.bash

#source /opt/ros/kinetic/setup.bash
#roslaunch uol_cmp9767m_base thorvald-sim.launch

#rosrun rviz rviz
#rviz -d `rospack find uol_cmp9767m_base`/rviz/thorvald.rviz

#https://github.com/LCAS/CMP9767M/wiki/Workshop-1---Introduction-and-ROS-Basics

#https://github.com/LCAS/teaching/wiki/CMP3103M
#http://wiki.ros.org/ROS/Tutorials/CreatingPackage
