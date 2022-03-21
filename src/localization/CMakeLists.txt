cmake_minimum_required(VERSION 3.0.2)
project(localization)

find_package(catkin REQUIRED COMPONENTS
  cartographer_ros
  robot_utils
  roscpp
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS cartographer_ros robot_utils roscpp std_msgs
)

#############
## Install ##
#############

# Install assets
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)