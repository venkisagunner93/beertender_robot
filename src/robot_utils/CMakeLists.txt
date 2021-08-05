cmake_minimum_required(VERSION 3.0.2)
project(robot_utils)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2_ros
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
)

###########
## Build ##
###########

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/ 
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

