cmake_minimum_required(VERSION 3.0.2)
project(planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2_ros
  robot_utils
  nav_utils
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  DEPENDS robot_utils nav_utils
)

###########
## Build ##
###########

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${robot_utils_INCLUDE_DIRS}
  ${nav_utils_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_lib
  src/planner.cpp)

  target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES})
  add_dependencies(${PROJECT_NAME}_lib nav_utils_generate_messages_cpp)
  
  add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_main.cpp)
  target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_lib ${catkin_LIBRARIES})
  add_dependencies(${PROJECT_NAME}_node nav_utils_generate_messages_cpp)

#############
## Install ##
#############

# Install libraries
install(TARGETS ${PROJECT_NAME}_lib
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

# Install assets
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
