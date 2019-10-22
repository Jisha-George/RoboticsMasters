cmake_minimum_required(VERSION 2.8.3)
project(rol_server)

find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  rospy
)

add_service_files(
   FILES
   findObject.srv
 )

## .. and services with any dependencies
generate_messages(
  DEPENDENCIES
   std_msgs  
)

catkin_package(
   CATKIN_DEPENDS message_runtime std_msgs rospy
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  scripts/rol_server_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
