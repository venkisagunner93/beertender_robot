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
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
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
  DWA(ros::NodeHandle& nh, Robot* robot);
  /**
   * @brief Destroy the Dwa Local Planner object
   */
  ~DWA()
  {
  }
  /**
   * @brief A method to perform local planning for the robot
   * @param pose Final pose for the trajectory
   * @param input Best input for the robot
   * @return true
   * @return false
   */
  bool performLocalPlanning(const geometry_msgs::PoseStamped& pose, ControlInput& input);

private:
  /**
   * @brief A method to generate velocity samples
   * @return std::vector<ControlInput>
   */
  std::vector<ControlInput> generateVelocitySamples();
  /**
   * @brief A method to simulate a trajectory from control input
   * @param control_input Control input
   * @return nav_msgs::Path
   */
  nav_msgs::Path simulateTrajectory(const ControlInput& control_input);
  /**
   * @brief A method to calculate distance to goal cost for a trajectory
   * @param trajectory Simulated trajectory
   * @param goal Final goal
   * @return float 
   */
  float calculateGoalDistanceCost(const nav_msgs::Path& trajectory,
                                  const geometry_msgs::PoseStamped& goal);
  /**
   * @brief A method to calculate max velocity cost for a trajectory
   * @param control_input Control input simulated
   * @return float 
   */
  float calculateMaxVelocityCost(const ControlInput& control_input);
  /**
   * @brief DWA configuration parameters
   */
  DWAConfig config_;
  /**
   * @brief Robot instance
   */
  Robot* robot_;
  /**
   * @brief Publish all trajectories
   */
  ros::Publisher trajectory_publisher_;
};

#endif  // DYNAMIC_WINDOW_APPROACH_H