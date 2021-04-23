#include "local_planner/dynamic_window_approach.h"

DWA::DWA(ros::NodeHandle* nh, std::string action_name)
  : as_(*nh, action_name, boost::bind(&DWA::performLocalPlanning, this, _1), false)
  , action_name_(action_name)
{
  trajectory_publisher_ = nh->advertise<nav_msgs::Path>("nav/local_trajectory", 100);
  cmd_vel_publisher_ = nh->advertise<ackermann_msgs::AckermannDrive>("cmd_vel", 100);

  // Load initial state and configurations
  loadInitState(nh);
  loadDWAConfig(nh);

  as_.start();
}

void DWA::broadcastCurrentPose()
{
  State state = robot_->getState();
  
  // Broadcasting initial state of the robot
  tf_helper::broadcastCurrentPoseToTF(state.x, state.y, state.theta, PARENT_FRAME,
                                      CHILD_FRAME);
}

void DWA::loadInitState(ros::NodeHandle* nh)
{
  // Loading initial state for the robot
  State init_state;
  nh->param<float>("planner/init/x", init_state.x, 0.0);
  nh->param<float>("planner/init/y", init_state.y, 0.0);
  nh->param<float>("planner/init/theta", init_state.theta, M_PI / 2);
  nh->param<float>("planner/init/v", init_state.v, 0.0);
  nh->param<float>("planner/init/w", init_state.w, 0.0);

  robot_ = std::make_unique<CarRobot>(init_state);
}

void DWA::loadDWAConfig(ros::NodeHandle* nh)
{
  // Loading all DWA configuration parameters
  nh->param<float>("local_planner/sim_time", config_.sim_time, 2.0);
  nh->param<float>("local_planner/dt", config_.dt, 0.05);
  nh->param<float>("local_planner/goal_region", config_.goal_region, 0.05);
  nh->param<int>("local_planner/window_size", config_.window_size, 5);
  config_.sim_samples = static_cast<int>(config_.sim_time / config_.dt);

  // Loading all DWA gains
  nh->param<float>("local_planner/gains/distance_to_goal_gain", config_.gains.distance_to_goal_gain,
                   1.0);
  nh->param<int>("local_planner/gains/obstacle_gain", config_.gains.obstacle_gain, 1);
  nh->param<int>("local_planner/gains/max_velocity_gain", config_.gains.max_velocity_gain, 1);

  // Loading all DWA limits
  nh->param<float>("local_planner/limits/min_v", config_.limits.min_v, 0.2);
  nh->param<float>("local_planner/limits/max_v", config_.limits.max_v, 0.025);
  nh->param<float>("local_planner/limits/min_w", config_.limits.min_w, 0.7);
  nh->param<float>("local_planner/limits/max_w", config_.limits.max_w, -0.7);
  nh->param<float>("local_planner/limits/max_v_dot", config_.limits.max_v_dot, 0.01);
  nh->param<float>("local_planner/limits/max_w_dot", config_.limits.max_w_dot, 1.0);
  nh->param<float>("local_planner/limits/max_theta", config_.limits.max_theta, 0.5235);
}

std::vector<ackermann_msgs::AckermannDrive> DWA::generateVelocitySamples()
{
  std::vector<ackermann_msgs::AckermannDrive> u_vec;

  State current_state = robot_->getState();

  float min_v =
      std::max((current_state.v - config_.limits.max_v_dot * config_.dt), config_.limits.min_v);
  float max_v =
      std::min((current_state.v + config_.limits.max_v_dot * config_.dt), config_.limits.max_v);
  float min_w =
      std::max((current_state.w - config_.limits.max_w_dot * config_.dt), config_.limits.min_w);
  float max_w =
      std::min((current_state.w + config_.limits.max_w_dot * config_.dt), config_.limits.max_w);

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

      // ROS_INFO_STREAM(u.speed << "," << u.steering_angle_velocity);

      u_vec.push_back(u);
    }
  }

  return u_vec;
}

nav_msgs::Path DWA::simulateTrajectory(const ackermann_msgs::AckermannDrive& u)
{
  nav_msgs::Path trajectory;
  trajectory.header.frame_id = PARENT_FRAME;
  CarRobot robot(robot_->getState());

  for (int i = 0; i < config_.sim_samples; i++)
  {
    geometry_msgs::PoseStamped pose;
    State state = robot.updateRobotState(u.speed, u.steering_angle_velocity);
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
  geometry_msgs::PoseStamped current_pose;
  float distance_to_goal = 1e5;
  if (tf_helper::getCurrentPoseFromTF(PARENT_FRAME, CHILD_FRAME, &current_pose))
  {
    feedback_.current_pose = current_pose;
    as_.publishFeedback(feedback_);

    distance_to_goal = sqrt(pow((current_pose.pose.position.x - goal.pose.position.x), 2) +
                            pow((current_pose.pose.position.y - goal.pose.position.y), 2));
    // ROS_INFO_STREAM(distance_to_goal);
  }
  return distance_to_goal <= config_.goal_region;
}

void DWA::performLocalPlanning(const nav_utils::ReachGlobalPoseGoalConstPtr& goal)
{
  ros::Rate rate(20);

  while (ros::ok())
  {
    if (as_.isPreemptRequested())
    {
      as_.setPreempted();
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
        float cost = calculateGoalDistanceCost(trajectory, goal->goal);

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

      cmd_vel_publisher_.publish(best_u);
      
      State state = robot_->updateRobotState(best_u.speed, best_u.steering_angle_velocity);
      tf_helper::broadcastCurrentPoseToTF(state.x, state.y, state.theta, PARENT_FRAME, CHILD_FRAME);
    }
    else
    {
      result_.has_reached = true;
      as_.setSucceeded(result_);
      break;
    }

    rate.sleep();
    ros::spinOnce();
  }
}
