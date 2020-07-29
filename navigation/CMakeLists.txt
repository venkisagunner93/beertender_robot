cmake_minimum_required(VERSION 3.0.2)
project(navigation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2_ros
  robot
  message_generation
)

add_service_files(
  FILES
  SetGoal.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  DEPENDS robot
  CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${robot_INCLUDE_DIRS}
)

add_library(global_planner_lib
  src/global_planner/breadth_first_search.cpp)

add_library(local_planner_lib
  src/local_planner/dynamic_window_approach.cpp)

add_library(planner_lib
  src/planner.cpp)

target_link_libraries(planner_lib global_planner_lib local_planner_lib ${catkin_LIBRARIES})
add_dependencies(planner_lib navigation_generate_messages_cpp)

add_executable(navigation src/planner_main.cpp)
target_link_libraries(navigation planner_lib ${catkin_LIBRARIES})
add_dependencies(navigation navigation_generate_messages_cpp)

#############
## Install ##
#############

# Install libraries
install(TARGETS global_planner_lib planner_lib local_planner_lib
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS navigation
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/ 
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

