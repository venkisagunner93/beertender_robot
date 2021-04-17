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

#include <ros/ros.h>
#include "robot/robot.h"
#include "robot/robot_types.h"

/**
 * @brief A structure for DWA configuration
 */
struct DWAConfig
{
  float sim_time;              /*< Simulation time */
  float dt;                    /*< Sample time */
  int sim_samples;             /*< Total sim samples */
  int obstacle_gain;           /*< Obstacle gain */
  int max_velocity_gain;       /*<  Max velocity gain */
  float distance_to_goal_gain; /*< Distance to goal gain */
  int window_size;             /*< Window size */
  /**
   * @brief Construct a new DWAConfig object
   */
  DWAConfig()
  {
    sim_time = 0.0;
    dt = 0.0;
    sim_samples = 0;
    obstacle_gain = 0;
    max_velocity_gain = 0;
    distance_to_goal_gain = 0;
    window_size = 0;
  }
};

/**
 * @brief A structure for dynamic window
 */
struct DynamicWindow
{
  std::vector<float> linear_velocity;  /*< Linear velocity (m/s) */
  std::vector<float> angular_velocity; /*< Angular velocity (rad/s) */
  /**
   * @brief Construct a new Dynamic Window object
   */
  DynamicWindow()
  {
    linear_velocity.clear();
    angular_velocity.clear();
  }
};

/**
 * @brief A base class for local planner
 */
class LocalPlanner
{
public:
  /**
   * @brief Destroy the Local Planner object
   */
  virtual ~LocalPlanner()
  {
  }
};

#endif  // LOCAL_PLANNER_H