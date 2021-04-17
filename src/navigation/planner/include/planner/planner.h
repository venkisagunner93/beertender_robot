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
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PointStamped.h>
#include "nav_utils/SetGoal.h"
#include "nav_utils/map.h"
#include "robot/car_robot.h"
#include <stack>

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
   * @brief A callback to get global path
   * @param request
   * @param response
   * @return true
   * @return false
   */
  bool setGoal(nav_utils::SetGoalRequest& request, nav_utils::SetGoalResponse& response);
  /**
   * @brief Transform buffer
   */
  tf2_ros::Buffer buffer_;
  /**
   * @brief Transform listener
   */
  tf2_ros::TransformListener listener_;
  /**
   * @brief Set goal service
   */
  ros::ServiceServer set_goal_service_;
};

#endif  // PLANNER_H
