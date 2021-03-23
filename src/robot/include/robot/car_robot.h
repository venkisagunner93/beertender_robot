/**
 * @file car_robot.h
 * @author Venkatavaradhan Vembanoor Lakshmi Narayanan
 * (venkatavaradhan93@gmail.com)
 * @brief A class for car like robot
 * @version 0.1
 * @date 2020-07-21
 * @copyright Copyright (c) 2020
 */

#ifndef CAR_ROBOT_H
#define CAR_ROBOT_H

#include "robot/robot.h"
#include <geometry_msgs/TransformStamped.h>
#include <math.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <urdf/model.h>

/**
 * @brief A class for car like robot
 */
class CarRobot : public Robot
{
public:
  /**
   * @brief Construct a new Car Robot object
   * @param drive_limits - Drive limits
   */
  CarRobot(const DriveLimits& drive_limits);
  /**
   * @brief Destroy the Car Robot object
   */
  ~CarRobot()
  {
  }
  /**
   * @brief A method to update state
   * @param control_input - Control input
   * @return State
   */
  State executeCommand(const ControlInput& control_input);
  /**
   * @brief A method to display robot details
   */
  void displayRobotDetails() const;
  /**
   * @brief Get robot's drive limits
   * @return DriveLimits
   */
  DriveLimits getDriveLimits() const;
  /**
   * @brief Get dimensions of the robot
   * @return Dimension
   */
  Dimension getDimension() const;
  /**
   * @brief Get current state of the robot
   * @return State
   */
  State getCurrentState() const;

private:
  /**
   * @brief A method to broadcast pose
   */
  void broadcastPose();
  /**
   * @brief Robot car dimension
   */
  Dimension dimension_;
  /**
   * @brief Robot car drive limits
   */
  DriveLimits drive_limits_;
  /**
   * @brief Current state of the robot
   */
  State state_;
  /**
   * @brief Previous time stamp for dt
   */
  ros::Time prev_time_;
};

#endif  // CAR_ROBOT_H