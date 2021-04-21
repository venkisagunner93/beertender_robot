/**
 * @file global_planner.h
 * @author Venkatavaradhan Vembanoor Lakshmi Narayanan
 * (venkatavaradhan93@gmail.com)
 * @brief A base class for global plannner
 * @version 0.1
 * @date 2020-07-25
 * @copyright Copyright (c) 2020
 */

#ifndef GLOBAL_PLANNER_H
#define GLOBAL_PLANNER_H

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>
#include "nav_utils/map.h"

/**
 * @brief A base class for global planner
 */
class GlobalPlanner
{
public:
  /**
   * @brief Destroy the Global Planner object
   */
  virtual ~GlobalPlanner() {}
  /**
   * @brief Get the Global Path object
   * @param start - Start co-ordinate
   * @param goal - Goal co-ordinate
   * @return nav_msgs::Path
   */
  virtual nav_msgs::Path getGlobalPath(const geometry_msgs::PointStamped& start,
                                       const geometry_msgs::PointStamped& goal) = 0;
  /**
   * @brief A pure virtual function for running global planning
   */
  virtual void runGlobalPlanning() = 0;
};

#endif  // GLOBAL_PLANNER_H