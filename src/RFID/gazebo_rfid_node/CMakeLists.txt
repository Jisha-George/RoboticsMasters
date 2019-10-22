cmake_minimum_required(VERSION 2.8.3)
project(gazebo_rfid_node)

find_package(catkin REQUIRED COMPONENTS roscpp  rfid_node)
include_directories(include ${catkin_INCLUDE_DIRS})

## Find gazebo
find_package(gazebo REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

## Find Boost
find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})

##################################
## Update the GAZEBO_MODEL_PATH ##
##   thanks to oru guys ...     ##
##################################

if(CMAKE_HOST_UNIX)
  catkin_add_env_hooks(90.gazebo_models_rfid SHELLS sh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)
else()
  message(FATAL_ERROR "Only UNIX builds supported.")
#  catkin_add_env_hooks(90.gazebo_models_oru SHELLS bat DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)
endif()

catkin_package(
#  DEPENDS gazebo
  CATKIN_DEPENDS roscpp  rfid_node
  INCLUDE_DIRS include
  LIBRARIES
)

###########
## Build ##
###########
###########

add_executable(wirelessNodeMonitor src/wirelessNodeMonitor.cpp)
target_link_libraries(wirelessNodeMonitor ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(TARGETS
  wirelessNodeMonitor
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY
  models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
  scripts/download_all_gazebo_org_models.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
