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

#include <math.h>
#include <urdf/model.h>
#include <ros/ros.h>
#include "robot/robot.h"
#include "nav_utils/tf_helper.h"

/**
 * @brief A class for car like robot
 */
class CarRobot : public Robot
{
public:
  /**
   * @brief Construct a new Car Robot object
   * @param init_state Initial state
   */
  CarRobot(const State& init_state);
  /**
   * @brief Destroy the Car Robot object
   */
  ~CarRobot() {}
  /**
   * @brief A method to update robot state
   * @param v Forward velocity
   * @param w Angular velocity
   * @return State 
   */
  State updateRobotState(const float& v, const float& w);
  /**
   * @brief A method to update robot state
   * @param v Forward velocity
   * @param w Angular velocity
   * @param dt Time step
   * @return State 
   */
  State updateRobotState(const float& v, const float& w, const float& dt);
  /**
   * @brief A method to display robot details
   */
  void displayRobotDetails() const;
  /**
   * @brief Get dimensions of the robot
   * @return Dimension
   */
  Dimension getDimension() const;
  /**
   * @brief Get current state of the robot
   * @return State
   */
  State getState() const;
  /**
   * @brief Set the Current State object
   * @param state Current state
   */
  void setState(const State& state);

private:
  /**
   * @brief Robot car dimension
   */
  Dimension dimension_;
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