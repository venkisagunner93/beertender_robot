/**
 * @file robot_types.h
 * @author Venkatavaradhan Vembanoor Lakshmi Narayanan (venkatavaradhan93@gmail.com)
 * @brief A file for robot types
 * @version 0.1
 * @date 2020-07-24
 * @copyright Copyright (c) 2020
 */

#ifndef ROBOT_TYPES_H
#define ROBOT_TYPES_H

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
 * @brief A structure for robot drive limits
 */
struct DriveLimits
{
  float max_linear_velocity;      /*< Max linear velocity (m/s) */
  float min_linear_velocity;      /*< Min linear velocity (m/s) */
  float max_angular_velocity;     /*< Max angular velocity (rad/s) */
  float min_angular_velocity;     /*< Min angular velocity (rad/s) */
  float max_linear_acceleration;  /*< Max linear acceleration (m/s^2) */
  float max_angular_acceleration; /*< Max angular acceleration (rad/s^2) */
  float max_steering_angle;       /*< Max steering angle (rad) */
  /**
   * @brief Construct a new Drive Limits object
   */
  DriveLimits()
  {
    max_linear_velocity = 0.0;
    min_linear_velocity = 0.0;
    max_angular_velocity = 0.0;
    min_angular_velocity = 0.0;
    max_linear_acceleration = 0.0;
    max_angular_acceleration = 0.0;
    max_steering_angle = 0.0;
  }
};

/**
 * @brief A structure for robot state
 */
struct State
{
  float x;                /*< X position (m) */
  float y;                /*< Y position (m) */
  float theta;            /*< Orientation in yaw (rad) */
  float linear_velocity;  /*< Linear velocity (m/s) */
  float angular_velocity; /*< Angular velocity (rad/s) */
  /**
   * @brief Construct a new State object
   */
  State()
  {
    x = 0.0;
    y = 0.0;
    theta = 0.0;
    linear_velocity = 0.0;
    angular_velocity = 0.0;
  }
};

/**
 * @brief A structure for control input
 */
struct ControlInput
{
  float forward_velocity;  /*< Forward velocity (m/s) */
  float steering_velocity; /*< Steering velocity (rad/s) */
  /**
   * @brief Construct a new Control Input object
   */
  ControlInput()
  {
    forward_velocity = 0.0;
    steering_velocity = 0.0;
  }
};

#endif  // ROBOT_TYPES_H