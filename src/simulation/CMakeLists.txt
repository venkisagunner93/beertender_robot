cmake_minimum_required(VERSION 3.0.2)
project(simulation)

find_package(gazebo 11 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  robot_utils
  roscpp
  tf
  tf2_ros
  std_msgs
  gazebo_plugins
  create_description
)

###########
## Build ##
###########

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${robot_utils_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS gazebo_plugins robot_utils roscpp std_msgs tf tf2_ros
)

#############
## Install ##
#############

# Install assets
install(DIRECTORY launch config worlds models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)