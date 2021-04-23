cmake_minimum_required(VERSION 3.0.2)
project(environments)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

#############
## Install ##
#############

# Install assets
install(DIRECTORY launch maps rviz
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)