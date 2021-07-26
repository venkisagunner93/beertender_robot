cmake_minimum_required(VERSION 3.0.2)
project(robot_sim)

find_package(catkin REQUIRED COMPONENTS
  ackermann_msgs
  nav_utils
  robot
  roscpp
  tf2_ros
  nav_msgs
  laser_geometry
  tf2_bullet
  message_generation
)

## Add find-modules and define external dependencies
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(Bullet REQUIRED)
find_package(OpenCV REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  OverlayClouds.msg
  LocalizedCloud.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
  nav_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  DEPENDS robot nav_utils nav_msgs geometry_msgs sensor_msgs laser_geometry roscpp tf2_bullet message_runtime
)

###########
## Build ##
###########

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${robot_INCLUDE_DIRS}
  ${nav_utils_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${BULLET_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(grid_utils
  src/occupancy_grid_utils/ray_tracer.cpp
  src/occupancy_grid_utils/coordinate_conversions.cpp
)

target_link_libraries(grid_utils ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(grid_utils robot_sim_generate_messages_cpp)
# set_target_properties(grid_utils PROPERTIES COMPILE_OPTIONS "-std=c++11")

add_library(${PROJECT_NAME}_lib
  src/robot_sim.cpp)

target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_lib grid_utils nav_utils_generate_messages_cpp)

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_main.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_lib grid_utils ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_node nav_utils_generate_messages_cpp)

#############
## Install ##
#############

# Install libraries
install(TARGETS ${PROJECT_NAME}_lib grid_utils
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}_node
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/ 
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)