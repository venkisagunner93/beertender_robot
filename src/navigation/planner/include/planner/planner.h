/**
 * @file planner.h
 * @author Venkatavaradhan Vembanoor Lakshmi Narayanan (venkatavaradhan93@gmail.com)
 * @brief A class for orchestrating planning and navigation
 * @version 0.1
 * @date 2020-07-26
 * @copyright Copyright (c) 2020
 */

#ifndef PLANNER_H
#define PLANNER_H

#include <queue>
#include <thread>
#include <ros/ros.h>
#include <stack>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_ros/transform_listener.h>
#include "nav_utils/map.h"
#include "robot/car_robot.h"

/**
 * @brief A class for orchestrating planning and navigation
 */
class Planner
{
public:
  /**
   * @brief Construct a new Planner object
   * @param nh - ROS NodeHandle for communication
   */
  Planner(ros::NodeHandle& nh);
  /**
   * @brief Destroy the Planner object
   */
  ~Planner();
  /**
   * @brief A method to run planner
   */
  void run();

private:
  /**
   * @brief Transform buffer
   */
  tf2_ros::Buffer buffer_;
  /**
   * @brief Transform listener
   */
  tf2_ros::TransformListener listener_;
};

#endif  // PLANNER_H
