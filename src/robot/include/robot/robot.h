/**
 * @file robot.h
 * @author Venkatavaradhan Vembanoor Lakshmi Narayanan (venkatavaradhan93@gmail.com)
 * @brief A class for simple robot model
 * @version 0.1
 * @date 2020-07-21
 * @copyright Copyright (c) 2020
 */

#ifndef ROBOT_H
#define ROBOT_H

#include <geometry_msgs/TransformStamped.h>
#include "robot/robot_types.h"

/**
 * @brief A base class for Robot
 */
class Robot
{
public:
  /**
   * @brief Destroy the Robot object
   */
  virtual ~Robot()
  {
  }
  /**
   * @brief A method to update state
   * @param control_input - Control input
   * @return State
   */
  virtual State executeCommand(const ControlInput& control_input) = 0;
  /**
   * @brief A method to display robot details
   */
  virtual void displayRobotDetails() const
  {
  }
  /**
   * @brief Get the Drive Limits object
   * @return DriveLimits
   */
  virtual DriveLimits getDriveLimits() const = 0;
  /**
   * @brief Get the Dimension object
   * @return Dimension
   */
  virtual Dimension getDimension() const = 0;
  /**
   * @brief Get the State object
   * @return State
   */
  virtual State getCurrentState() const = 0;
  /**
   * @brief A method to broadcast pose to tf tree
   */
  virtual void broadcastPose()
  {
  }
};

#endif  // ROBOT_H