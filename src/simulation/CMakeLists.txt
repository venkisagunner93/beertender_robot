cmake_minimum_required(VERSION 3.0.2)
project(simulation)

find_package(gazebo 9 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  robot_utils
  roscpp
  tf
  tf2_ros
  std_msgs
  gazebo_plugins
  create_description
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS gazebo_plugins robot_utils roscpp std_msgs tf tf2_ros
)

#############
## Install ##
#############

# Install assets
install(DIRECTORY launch worlds models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)