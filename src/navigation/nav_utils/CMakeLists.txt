cmake_minimum_required(VERSION 3.0.2)
project(nav_utils)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2_ros
  robot
  message_generation
  nav_msgs
)

add_service_files(
  FILES
)

add_action_files(
  FILES 
  FindGlobalPath.action
  ReachGlobalPose.action
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  nav_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  DEPENDS robot
  CATKIN_DEPENDS message_runtime
  LIBRARIES ${PROJECT_NAME}_lib
)

###########
## Build ##
###########

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${robot_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_lib
  src/map.cpp
  src/tf_helper.cpp
)

target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_lib nav_utils_generate_messages_cpp)

#############
## Install ##
#############

# Install libraries
install(TARGETS ${PROJECT_NAME}_lib
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/ 
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

