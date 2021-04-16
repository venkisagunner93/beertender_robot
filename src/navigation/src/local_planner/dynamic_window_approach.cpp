#include "navigation/local_planner/dynamic_window_approach.h"
#include <cmath>

DWA::DWA(ros::NodeHandle& nh, Robot* robot) : robot_(robot)
{
  config_.sim_time = 2.0;                               // in seconds
  config_.dt = 0.05;                                    // time step
  config_.sim_samples = config_.sim_time / config_.dt;  // Total sim samples
  config_.window_size = 5;                              // Velocity space max window size
  config_.obstacle_gain = 0;
  config_.distance_to_goal_gain = 1;
  config_.max_velocity_gain = 1;

  trajectory_publisher_ = nh.advertise<nav_msgs::Path>("/nav/local_trajectory", 100);
}

std::vector<ControlInput> DWA::generateVelocitySamples()
{
  std::vector<ControlInput> control_inputs;

  State current_state = robot_->getCurrentState();
  DriveLimits limits = robot_->getDriveLimits();

  float min_lin_velocity =
      std::max((current_state.linear_velocity - limits.max_linear_acceleration * config_.dt),
               limits.min_linear_velocity);
  float max_lin_velocity =
      std::min((current_state.linear_velocity + limits.max_linear_acceleration * config_.dt),
               limits.max_linear_velocity);
  float min_ang_velocity =
      std::max((current_state.angular_velocity - limits.max_angular_acceleration * config_.dt),
               limits.min_angular_velocity);
  float max_ang_velocity =
      std::min((current_state.angular_velocity + limits.max_angular_acceleration * config_.dt),
               limits.max_angular_velocity);

  float lin_velocity_step = (max_lin_velocity + abs(min_lin_velocity)) / (config_.window_size - 1);
  float ang_velocity_step = (max_ang_velocity + abs(min_ang_velocity)) / (config_.window_size - 1);

  float lin_velocity_samples[config_.window_size];
  float ang_velocity_samples[config_.window_size];

  for (int i = 0; i < config_.window_size; i++)
  {
    lin_velocity_samples[i] = min_lin_velocity + i * lin_velocity_step;
  }

  for (int i = 0; i < config_.window_size; i++)
  {
    ang_velocity_samples[i] = min_ang_velocity + i * ang_velocity_step;
  }

  for (int i = 0; i < config_.window_size; i++)
  {
    for (int j = 0; j < config_.window_size; j++)
    {
      ControlInput control_input;
      control_input.forward_velocity = lin_velocity_samples[i];
      control_input.steering_velocity = ang_velocity_samples[j];

      control_inputs.push_back(control_input);
    }
  }

  return control_inputs;
}

nav_msgs::Path DWA::simulateTrajectory(const ControlInput& control_input)
{
  nav_msgs::Path trajectory;
  trajectory.header.frame_id = "map";
  CarRobot robot;
  robot.setNewState(robot_->getCurrentState());

  for (int i = 0; i < config_.sim_samples; i++)
  {
    geometry_msgs::PoseStamped pose;
    State state = robot.executeCommand(control_input);
    pose.pose.position.x = state.x;
    pose.pose.position.y = state.y;
    tf2::Quaternion q;
    q.setRPY(0, 0, state.theta);
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();

    trajectory.poses.push_back(pose);
  }

  return trajectory;
}

float DWA::calculateGoalDistanceCost(const nav_msgs::Path& trajectory,
                                     const geometry_msgs::PoseStamped& goal)
{
  float cost = 1e5;
  if (!trajectory.poses.empty())
  {
    geometry_msgs::PoseStamped simulated_trajectory_end = trajectory.poses.back();
    cost = config_.distance_to_goal_gain *
           sqrt(pow((simulated_trajectory_end.pose.position.x - goal.pose.position.x), 2) +
                pow((simulated_trajectory_end.pose.position.y - goal.pose.position.y), 2));
  }
  return cost;
}

float DWA::calculateMaxVelocityCost(const ControlInput& control_input)
{
  return config_.max_velocity_gain *
         (robot_->getDriveLimits().max_linear_velocity - control_input.forward_velocity);
}

bool DWA::performLocalPlanning(const geometry_msgs::PoseStamped& pose, ControlInput& best_input)
{
  bool goal_status = false;

  std::vector<ControlInput> control_inputs = generateVelocitySamples();

  float trajectory_cost = 0.0;

  float lowest_cost = 1e6;

  for (auto& control_input : control_inputs)
  {
    nav_msgs::Path trajectory = simulateTrajectory(control_input);
    float cost = calculateGoalDistanceCost(trajectory, pose);
    // cost += calculateMaxVelocityCost(control_input);

    if (cost < lowest_cost)
    {
      lowest_cost = cost;
      best_input = control_input;
    }
    trajectory_publisher_.publish(trajectory);
  }

  if (best_input.forward_velocity >= robot_->getDriveLimits().max_linear_velocity)
  {
    best_input.forward_velocity = robot_->getDriveLimits().max_linear_velocity;
  }

  if (best_input.steering_velocity >= robot_->getDriveLimits().max_angular_velocity)
  {
    best_input.steering_velocity = robot_->getDriveLimits().max_angular_velocity;
  }

  State current_state = robot_->getCurrentState();

  float dist = sqrt(pow((current_state.x - pose.pose.position.x), 2) +
                    pow((current_state.y - pose.pose.position.y), 2));

  if (dist <= 0.05)
  {
    goal_status = true;
  }

  return goal_status;
}
