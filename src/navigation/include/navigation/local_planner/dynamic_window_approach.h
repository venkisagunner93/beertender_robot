/**
 * @file dynamic_window_approach.h
 * @author Venkatavaradhan Vembanoor Lakshmi Narayanan (venkatavaradhan93@gmail.com)
 * @brief A class for dynamic window approach local planner
 * @version 0.1
 * @date 2020-07-27
 * @copyright Copyright (c) 2020
 */

#ifndef DYNAMIC_WINDOW_APPROACH_H
#define DYNAMIC_WINDOW_APPROACH_H

#include <algorithm>
#include <nav_msgs/Path.h>
#include "navigation/local_planner/local_planner.h"
#include "robot/car_robot.h"

/**
 * @brief A class for dynamic window approach local planner
 */
class DWA : public LocalPlanner
{
public:
  /**
   * @brief Construct a new DWA object
   * @param nh - ROS NodeHandle for communication
   */
  DWA(ros::NodeHandle& nh);
  /**
   * @brief Destroy the Dwa Local Planner object
   */
  ~DWA();

private:
  /**
   * @brief DWA configuration parameters
   */
  DWAConfig config_;
  nav_msgs::Path trajectory_;
  ros::Publisher trajectory_publisher_;
};

#endif  // DYNAMIC_WINDOW_APPROACH_H