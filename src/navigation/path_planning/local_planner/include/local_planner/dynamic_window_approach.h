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
#include <mutex>

#include <actionlib/server/simple_action_server.h>
#include <dynamic_reconfigure/server.h>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <sensor_msgs/Joy.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "robot_utils/kinematics/state_update.h"

#include "nav_utils/ReachGlobalPoseAction.h"
#include "nav_utils/tf_helper.h"

#include "local_planner/local_planner.h"
#include "local_planner/DWAConfig.h"

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
  int loop_rate;     /*< Loop rate */
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
    loop_rate = 0;
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
   * @brief DWA configuration parameters
   */
  DWAConfig config_;

private:
  /**
   * @brief A method to perform local planning for the robot
   * @param goal Goal to reach
   */
  void performLocalPlanning(const nav_utils::ReachGlobalPoseGoalConstPtr& goal);
  /**
   * @brief A method to update robot state and publish velocity
   * @param msg Best setpoint
   */
  void updateStateAndPublish(const ackermann_msgs::AckermannDrive& msg);
  /**
   * @brief A dynamic reconfigure callback
   * @param config
   * @param level
   */
  void reconfigCallback(local_planner::DWAConfig& config, uint32_t level);
  /**
   * @brief A subscriber callback for joystick msgs
   * @param msg Joy msgs
   */
  void joyCallback(const sensor_msgs::JoyConstPtr& msg);
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
  /**
   * @brief Zero velocity setpoint
   */
  ackermann_msgs::AckermannDrive zero_u_;
  /**
   * @brief ROS subscriber for joystick msgs
   */
  ros::Subscriber joy_subscriber_;
  /**
   * @brief Mutex to handle joy state
   */
  std::mutex joy_mutex_;
  /**
   * @brief Is joystick active flag
   */
  bool is_joystick_active_;
  /**
   * @brief Joystick message
   */
  sensor_msgs::Joy joy_;
  /**
   * @brief Current state of the robot
   */
  robot_utils::State state_;
  /**
   * @brief Previous time for dt calculation
   */
  ros::Time prev_time_;
  /**
   * @brief Dynamic reconfigure server
   */
  boost::shared_ptr<dynamic_reconfigure::Server<local_planner::DWAConfig>> reconfig_server_;
  /**
   * @brief Mutex for dynamic reconfigure
   */
  boost::recursive_mutex dr_mutex_;
  /**
   * @brief TF helper instance
   */
  tf_helper::TFHelper tf_helper_;
};

#endif  // DYNAMIC_WINDOW_APPROACH_H