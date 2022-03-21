#!/bin/sh

ROS_DISTRO=$1

mkdir catkin_isolated_ws
cd catkin_isolated_ws
wstool init src
wstool merge -t src https://raw.githubusercontent.com/cartographer-project/cartographer_ros/master/cartographer_ros.rosinstall
wstool update -t src

rosdep update
rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y
src/cartographer/scripts/install_abseil.sh
sudo apt-get remove ros-${ROS_DISTRO}-abseil-cpp
. /opt/ros/${ROS_DISTRO}/setup.sh && catkin_make_isolated --install --use-ninja