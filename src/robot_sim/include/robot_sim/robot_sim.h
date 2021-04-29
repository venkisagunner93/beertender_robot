/**
 * @file robot_sim.h
 * @author Venkatavaradhan Vembanoor Lakshmi Narayanan (venkatavaradhan93@gmail.com)
 * @brief A class for simulating robot behaviors
 * @version 0.1
 * @date 2021-04-21
 * @copyright Copyright (c) 2021
 */

#ifndef ROBOT_SIM_H
#define ROBOT_SIM_H

#include <ros/ros.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <random>
#include "nav_utils/tf_helper.h"
#include "nav_utils/map.h"

static const float AXLE_LENGTH = 0.235;

/**
 * @brief A structure for noise parameters
 */
struct Noise
{
  float odom_variance;  /*< Odometry measurement error variance */
  float laser_variance; /*< Laser measurement error variance */
  /**
   * @brief Construct a new Noise object
   */
  Noise()
  {
    odom_variance = 0.0;
    laser_variance = 0.0;
  }
};

/**
 * @brief A structure for sim config parameters
 */
struct SimConfig
{
  Noise noise; /*< Noise instance */
  /**
   * @brief Construct a new Sim Config object
   */
  SimConfig() {}
};

/**
 * @brief A class for simuating robot behaviors
 */
class RobotSim
{
public:
  /**
   * @brief Construct a new Robot Sim object
   */
  RobotSim(ros::NodeHandle* nh);
  /**
   * @brief Destroy the Robot Sim object
   */
  ~RobotSim() {}
  /**
   * @brief A method to publish velocities in wheel frame from robot frame
   */
  void publishIndividualWheelVelocity();
  /**
   * @brief A method to publish laser scan
   */
  void publishLaserScan();

private:
  /**
   * @brief A subscriber callback for velocity commands
   * @param msg Velocity command
   */
  void cmdVelCallback(const ackermann_msgs::AckermannDrive& msg);
  /**
   * @brief A method to load parameter from param server
   * @param nh ROS Nodehandle for communication
   */
  void loadConfig(ros::NodeHandle* nh);
  /**
   * @brief ROS subscriber for velocity commands
   */
  ros::Subscriber cmd_vel_subscriber_;
  /**
   * @brief ROS publisher for individual wheel velocities
   */
  ros::Publisher wheel_vel_publisher_;
  /**
   * @brief ROS publisher for laser scans
   */
  ros::Publisher laser_scan_publisher_;
  /**
   * @brief Current control input from planner
   */
  ackermann_msgs::AckermannDrive current_u_;
  /**
   * @brief Sim config parameters
   */
  SimConfig config_;
  /**
   * @brief Noise generator
   */
  std::default_random_engine generator_;
  /**
   * @brief Transform tree helper
   */
  tf_helper::TFHelper tf_helper_;
  /**
   * @brief Map instance
   */
  Map map_;
};

#endif  // ROBOT_SIM_H