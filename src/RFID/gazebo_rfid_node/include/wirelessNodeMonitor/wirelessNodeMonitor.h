// based on a basic gazebo topic listener at:
// https://bitbucket.org/osrf/gazebo/src/256f98f2318bf2078e708f069367f1b71549ffb6/examples/stand_alone/listener/listener.cc
// mfc

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include "gazebo/sensors/sensors.hh"
#include <gazebo/gazebo.hh>
#include <gazebo/gazebo_client.hh>
#include "gazebo/common/Time.hh"

#include "ros/ros.h"
#include <ros/console.h>
#include "ros/this_node.h"

#include <string>
#include <iostream>
#include <sstream>

#include <rfid_node/TagReading.h>
