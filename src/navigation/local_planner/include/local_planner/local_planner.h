/**
 * @file local_planner.h
 * @author Venkatavaradhan Vembanoor Lakshmi Narayanan (venkatavaradhan93@gmail.com)
 * @brief A base class for local planner
 * @version 0.1
 * @date 2020-07-27
 * @copyright Copyright (c) 2020
 */

#ifndef LOCAL_PLANNER_H
#define LOCAL_PLANNER_H

#include "robot/robot.h"

/**
 * @brief A base class for local planner
 */
class LocalPlanner
{
public:
  /**
   * @brief Destroy the Local Planner object
   */
  virtual ~LocalPlanner() {}
};

#endif  // LOCAL_PLANNER_H