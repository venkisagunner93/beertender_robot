cmake_minimum_required(VERSION 3.0.2)
project(robot)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2_ros
  urdf
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES robot_lib
)

###########
## Build ##
###########

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_library(robot_lib
  src/car_robot.cpp)
target_link_libraries(robot_lib ${catkin_LIBRARIES})

#############
## Install ##
#############

# Install libraries
install(TARGETS robot_lib
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/ 
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

