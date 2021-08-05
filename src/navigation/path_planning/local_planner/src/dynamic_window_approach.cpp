#include "local_planner/dynamic_window_approach.h"

DWA::DWA(ros::NodeHandle* nh, std::string action_name)
  : as_(*nh, action_name, boost::bind(&DWA::performLocalPlanning, this, _1), false)
  , action_name_(action_name)
  , prev_time_(ros::Time::now())
{
  is_joystick_active_ = false;
  joy_subscriber_ = nh->subscribe("joy", 1, &DWA::joyCallback, this);
  trajectory_publisher_ = nh->advertise<nav_msgs::Path>("nav/local_trajectory", 1);
  cmd_vel_publisher_ = nh->advertise<ackermann_msgs::AckermannDrive>("cmd_vel", 1);

  // Load configurations
  loadDWAConfig(nh);

  reconfig_server_.reset(new dynamic_reconfigure::Server<local_planner::DWAConfig>());
  dynamic_reconfigure::Server<local_planner::DWAConfig>::CallbackType f =
      boost::bind(&DWA::reconfigCallback, this, _1, _2);
  reconfig_server_->setCallback(f);

  zero_u_.speed = 0.0;
  zero_u_.steering_angle_velocity = 0.0;

  // Loading initial state of the robot
  nh->param<float>("robot_sim/init_state/x", state_.x, 0.0);
  nh->param<float>("robot_sim/init_state/y", state_.y, 0.0);
  nh->param<float>("robot_sim/init_state/theta", state_.theta, M_PI / 2);
  nh->param<float>("robot_sim/init_state/v", state_.v, 0.0);
  nh->param<float>("robot_sim/init_state/w", state_.w, 0.0);

  as_.start();
}

void DWA::broadcastCurrentPose()
{
  if (is_joystick_active_)
  {
    float y = joy_.axes[0];
    float x = joy_.axes[1];

    int turbo = joy_.buttons[7];

    ackermann_msgs::AckermannDrive u;
    u.speed = x * (0.2 + turbo * 0.5);
    u.steering_angle_velocity = y * (0.2 + turbo * 0.4);
    updateStateAndPublish(u);
  }
}

void DWA::joyCallback(const sensor_msgs::JoyConstPtr& msg)
{
  {
    std::lock_guard<std::mutex> lock(joy_mutex_);
    is_joystick_active_ = true;
  }

  joy_ = *msg;
}

void DWA::loadDWAConfig(ros::NodeHandle* nh)
{
  // Loading all DWA configuration parameters
  nh->param<float>("local_planner/sim_time", config_.sim_time, 2.0);
  nh->param<float>("local_planner/dt", config_.dt, 0.05);
  nh->param<float>("local_planner/goal_region", config_.goal_region, 0.05);
  nh->param<int>("local_planner/window_size", config_.window_size, 5);
  nh->param<int>("local_planner/loop_rate", config_.loop_rate, 100);
  config_.sim_samples = static_cast<int>(config_.sim_time / config_.dt);

  // Loading all DWA gains
  nh->param<float>("local_planner/gains/distance_to_goal_gain", config_.gains.distance_to_goal_gain,
                   1.0);
  nh->param<int>("local_planner/gains/obstacle_gain", config_.gains.obstacle_gain, 1);
  nh->param<int>("local_planner/gains/max_velocity_gain", config_.gains.max_velocity_gain, 1);

  // Loading all DWA limits
  nh->param<float>("local_planner/limits/min_v", config_.limits.min_v, 0.025);
  nh->param<float>("local_planner/limits/max_v", config_.limits.max_v, 0.2);
  nh->param<float>("local_planner/limits/min_w", config_.limits.min_w, -0.7);
  nh->param<float>("local_planner/limits/max_w", config_.limits.max_w, 0.7);
  nh->param<float>("local_planner/limits/max_v_dot", config_.limits.max_v_dot, 0.01);
  nh->param<float>("local_planner/limits/max_w_dot", config_.limits.max_w_dot, 1.0);
  nh->param<float>("local_planner/limits/max_theta", config_.limits.max_theta, 0.5235);
}

void DWA::reconfigCallback(local_planner::DWAConfig& config, uint32_t level)
{
  config_.limits.min_v = static_cast<float>(config.min_v);
  config_.limits.max_v = static_cast<float>(config.max_v);
  config_.limits.min_w = static_cast<float>(config.min_w);
  config_.limits.max_w = static_cast<float>(config.max_w);
  config_.limits.max_v_dot = static_cast<float>(config.max_v_dot);
  config_.limits.max_w_dot = static_cast<float>(config.max_w_dot);
  config_.limits.max_theta = static_cast<float>(config.max_theta);
}

std::vector<ackermann_msgs::AckermannDrive> DWA::generateVelocitySamples()
{
  std::vector<ackermann_msgs::AckermannDrive> u_vec;

  float min_v = std::max((state_.v - config_.limits.max_v_dot * config_.dt), config_.limits.min_v);
  float max_v = std::min((state_.v + config_.limits.max_v_dot * config_.dt), config_.limits.max_v);
  float min_w = std::max((state_.w - config_.limits.max_w_dot * config_.dt), config_.limits.min_w);
  float max_w = std::min((state_.w + config_.limits.max_w_dot * config_.dt), config_.limits.max_w);

  float v_step = (max_v + abs(min_v)) / (config_.window_size - 1);
  float w_step = (max_w + abs(min_w)) / (config_.window_size - 1);

  float v_samples[config_.window_size];
  float w_samples[config_.window_size];

  for (int i = 0; i < config_.window_size; i++)
  {
    v_samples[i] = min_v + i * v_step;
  }

  for (int i = 0; i < config_.window_size; i++)
  {
    w_samples[i] = min_w + i * w_step;
  }

  for (int i = 0; i < config_.window_size; i++)
  {
    for (int j = 0; j < config_.window_size; j++)
    {
      ackermann_msgs::AckermannDrive u;
      u.speed = v_samples[i];
      u.steering_angle_velocity = w_samples[j];

      u_vec.push_back(u);
    }
  }

  return u_vec;
}

nav_msgs::Path DWA::simulateTrajectory(const ackermann_msgs::AckermannDrive& u)
{
  nav_msgs::Path trajectory;
  trajectory.header.stamp = ros::Time::now();
  trajectory.header.frame_id = PARENT_FRAME;

  robot_utils::State sim_state = state_;

  for (int i = 0; i < config_.sim_samples; i++)
  {
    geometry_msgs::PoseStamped pose;
    updateState(sim_state, u.speed, u.steering_angle_velocity, config_.dt);
    pose.pose.position.x = sim_state.x;
    pose.pose.position.y = sim_state.y;
    tf2::Quaternion q;
    q.setRPY(0, 0, sim_state.theta);
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
    cost = config_.gains.distance_to_goal_gain *
           sqrt(pow((simulated_trajectory_end.pose.position.x - goal.pose.position.x), 2) +
                pow((simulated_trajectory_end.pose.position.y - goal.pose.position.y), 2));
  }
  return cost;
}

float DWA::calculateMaxVelocityCost(const ackermann_msgs::AckermannDrive& u)
{
  return config_.gains.max_velocity_gain * (config_.limits.max_v - u.speed);
}

bool DWA::isInsideGoalRegion(const geometry_msgs::PoseStamped& goal)
{
  float distance_to_goal = 1e5;
  geometry_msgs::PoseStamped current_pose;
  if (tf_helper_.getCurrentPoseFromTF(PARENT_FRAME, CHILD_FRAME, &current_pose))
  {
    feedback_.current_pose = current_pose;
    as_.publishFeedback(feedback_);
    distance_to_goal = sqrt(pow((current_pose.pose.position.x - goal.pose.position.x), 2) +
                            pow((current_pose.pose.position.y - goal.pose.position.y), 2));
  }

  return distance_to_goal <= config_.goal_region;
}

void DWA::updateStateAndPublish(const ackermann_msgs::AckermannDrive& msg)
{
  ros::Duration dt = ros::Time::now() - prev_time_;
  prev_time_ = ros::Time::now();

  updateState(state_, msg.speed, msg.steering_angle_velocity, dt.toSec());
  cmd_vel_publisher_.publish(msg);
}

void DWA::performLocalPlanning(const nav_utils::ReachGlobalPoseGoalConstPtr& goal)
{
  result_.has_reached = false;

  ros::Rate rate(config_.loop_rate);

  while (ros::ok())
  {
    {
      std::lock_guard<std::mutex> lock(joy_mutex_);
      if (is_joystick_active_)
      {
        is_joystick_active_ = false;
        as_.setAborted(result_);
        return;
      }
    }

    if (as_.isPreemptRequested())
    {
      break;
    }

    if (!isInsideGoalRegion(goal->goal))
    {
      float trajectory_cost = 0.0;
      float lowest_cost = 1e6;
      ackermann_msgs::AckermannDrive best_u;

      std::vector<ackermann_msgs::AckermannDrive> u_vec = generateVelocitySamples();

      for (int i = 0; i < u_vec.size(); i++)
      {
        nav_msgs::Path trajectory = simulateTrajectory(u_vec[i]);
        float cost =
            calculateGoalDistanceCost(trajectory, goal->goal) + calculateMaxVelocityCost(u_vec[i]);

        if (cost < lowest_cost)
        {
          lowest_cost = cost;
          best_u = u_vec[i];
        }
        trajectory_publisher_.publish(trajectory);
      }

      if (best_u.speed >= config_.limits.max_v)
      {
        best_u.speed = config_.limits.max_v;
      }

      if (best_u.steering_angle_velocity >= config_.limits.max_w)
      {
        best_u.steering_angle_velocity = config_.limits.max_w;
      }

      updateStateAndPublish(best_u);
    }
    else
    {
      result_.has_reached = true;
      updateStateAndPublish(zero_u_);
      as_.setSucceeded(result_);
      break;
    }

    rate.sleep();
  }

  if (!result_.has_reached)
  {
    updateStateAndPublish(zero_u_);
    as_.setAborted(result_);
  }
}
