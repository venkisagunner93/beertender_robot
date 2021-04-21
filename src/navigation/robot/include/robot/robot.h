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

#include <string>

/**
 * @brief A structure for robot dimension
 */
struct Dimension
{
  float length;     /*< Length (m) */
  float width;      /*< Width (m) */
  float height;     /*< Height (m) */
  std::string name; /*< Robot name */
  /**
   * @brief Construct a new Dimension object
   */
  Dimension()
  {
    length = 0.0;
    width = 0.0;
    height = 0.0;
    name = "No name";
  }
};

/**
 * @brief A structure for robot state
 */
struct State
{
  float x;     /*< X position (m) */
  float y;     /*< Y position (m) */
  float theta; /*< Orientation in yaw (rad) */
  float v;     /*< Linear velocity (m/s) */
  float w;     /*< Angular velocity (rad/s) */
  /**
   * @brief Construct a new State object
   */
  State()
  {
    x = 0.0;
    y = 0.0;
    theta = 0.0;
    v = 0.0;
    w = 0.0;
  }
};

/**
 * @brief A base class for Robot
 */
class Robot
{
public:
  /**
   * @brief Destroy the Robot object
   */
  virtual ~Robot() {}
  /**
   * @brief A method to update robot state
   * @param v Forward velocity
   * @param w Angular velocity
   * @return State
   */
  virtual State updateRobotState(const float& v, const float& w) = 0;
  /**
   * @brief A method to update robot state
   * @param v Forward velocity
   * @param w Angular velocity
   * @param dt Time step
   * @return State 
   */
  virtual State updateRobotState(const float& v, const float& w, const float& dt) = 0;
  /**
   * @brief A method to display robot details
   */
  virtual void displayRobotDetails() const {}
  /**
   * @brief Get the Dimension object
   * @return Dimension
   */
  virtual Dimension getDimension() const = 0;
  /**
   * @brief Get the State object
   * @return State
   */
  virtual State getState() const = 0;
  /**
   * @brief Set the Current State object
   * @param state New state
   */
  virtual void setState(const State& state) = 0;
};

#endif  // ROBOT_H