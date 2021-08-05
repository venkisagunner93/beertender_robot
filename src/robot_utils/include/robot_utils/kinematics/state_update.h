#ifndef STATE_UPDATE_H
#define STATE_UPDATE_H

#include "math.h"
#include "ros/ros.h"

namespace robot_utils
{
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
 * @brief A funciton to update robot's state
 * @param state Current state to be updated
 * @param v Forward velocity
 * @param w Steering velocity
 * @param dt Timestep
 */
inline void updateState(State& state, const float& v, const float& w, const float& dt)
{
  state.theta = w * dt + state.theta;

  state.x = v * dt * cos(state.theta) + state.x;
  state.y = v * dt * sin(state.theta) + state.y;

  state.v = v;
  state.w = w;
}
}  // namespace robot_utils

#endif  // STATE_UPDATE_H