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
#include <cmath>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <actionlib/server/simple_action_server.h>
#include <thread>
#include "local_planner/local_planner.h"
#include "robot/car_robot.h"
#include "nav_utils/ReachGlobalPoseAction.h"
#include "nav_utils/tf_helper.h"

#define PARENT_FRAME "map"
#define CHILD_FRAME "base_link"

/**
 * @brief A structure for DWA gains
 */
struct Gains
{
  float distance_to_goal_gain; /*< Distance to goal gain */
  int obstacle_gain;           /*< Obstacle gain */
  int max_velocity_gain;       /*<  Max velocity gain */
  /**
   * @brief Construct a new Gains object
   */
  Gains()
  {
    distance_to_goal_gain = 0.0;
    obstacle_gain = 0;
    max_velocity_gain = 0;
  }
};

/**
 * @brief A structure for DWA limits
 */
struct Limits
{
  float min_v;     /*< Min linear velocity */
  float max_v;     /*< Max linear velocity */
  float min_w;     /*< Min angular velocity */
  float max_w;     /*< Max angular velocity */
  float max_v_dot; /*< Max linear acceleration */
  float max_w_dot; /*< Max angular acceleration */
  float max_theta; /*< Max theta */
  /**
   * @brief Construct a new Limits object
   */
  Limits()
  {
    min_v = 0.0;
    max_v = 0.0;
    min_w = 0.0;
    max_w = 0.0;
    max_v_dot = 0.0;
    max_w_dot = 0.0;
    max_theta = 0.0;
  }
};

/**
 * @brief A structure for DWA configuration
 */
struct DWAConfig
{
  Gains gains;       /*< Gains instance */
  Limits limits;     /*< DWA limits instance */
  float sim_time;    /*< Simulation time */
  float dt;          /*< Sample time */
  float goal_region; /*< Goal region */
  int sim_samples;   /*< Total sim samples */
  int window_size;   /*< Window size */
  /**
   * @brief Construct a new DWAConfig object
   */
  DWAConfig()
  {
    sim_time = 0.0;
    dt = 0.0;
    goal_region = 0.0;
    sim_samples = 0;
    window_size = 0;
  }
};

/**
 * @brief A class for dynamic window approach local planner
 */
class DWA : public LocalPlanner
{
public:
  /**
   * @brief Construct a new DWA object
   * @param nh ROS Nodehandle for communication
   * @param action_name Action server name
   */
  DWA(ros::NodeHandle* nh, std::string action_name);
  /**
   * @brief Destroy the Dwa Local Planner object
   */
  ~DWA() {}
  /**
   * @brief A method to update robot's current pose
   */
  void broadcastCurrentPose();

private:
  /**
   * @brief A method to generate velocity samples
   * @return std::vector<ControlInput>
   */
  std::vector<ackermann_msgs::AckermannDrive> generateVelocitySamples();
  /**
   * @brief A method to simulate a trajectory from control input
   * @param u Control input
   * @return nav_msgs::Path
   */
  nav_msgs::Path simulateTrajectory(const ackermann_msgs::AckermannDrive& u);
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
   * @param u Control input simulated
   * @return float
   */
  float calculateMaxVelocityCost(const ackermann_msgs::AckermannDrive& u);
  /**
   * @brief A method to load init state from parameter server
   * @param nh ROS Nodehandle for communication
   */
  void loadInitState(ros::NodeHandle* nh);
  /**
   * @brief A method to load DWA config from parameter server
   * @param nh ROS Nodehandle for communication
   */
  void loadDWAConfig(ros::NodeHandle* nh);
  /**
   * @brief A method to check whether the robot is inside goal region
   * @param goal Goal to be reached
   * @return true
   * @return false
   */
  bool isInsideGoalRegion(const geometry_msgs::PoseStamped& goal);
  /**
   * @brief A method to perform local planning for the robot
   * @param goal Goal to reach
   */
  void performLocalPlanning(const nav_utils::ReachGlobalPoseGoalConstPtr& goal);
  /**
   * @brief DWA configuration parameters
   */
  DWAConfig config_;
  /**
   * @brief Robot instance
   */
  std::unique_ptr<Robot> robot_;
  /**
   * @brief Publish all trajectories
   */
  ros::Publisher trajectory_publisher_;
  /**
   * @brief Velocity command publisher
   */
  ros::Publisher cmd_vel_publisher_;
  /**
   * @brief Local planner action server
   */
  actionlib::SimpleActionServer<nav_utils::ReachGlobalPoseAction> as_;
  /**
   * @brief Action name
   */
  std::string action_name_;
  /**
   * @brief Action server feedback
   */
  nav_utils::ReachGlobalPoseFeedback feedback_;
  /**
   * @brief Action server result
   */
  nav_utils::ReachGlobalPoseResult result_;
};

#endif  // DYNAMIC_WINDOW_APPROACH_H