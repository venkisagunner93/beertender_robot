#include "navigation/local_planner/dynamic_window_approach.h"

DWA::DWA(ros::NodeHandle& nh)
{
  config_.sim_time = 2.0;
  config_.dt = 0.05;
  config_.window_size = 15;
  config_.obstacle_gain = 0;
  config_.distance_to_goal_gain = 0;
  config_.max_velocity_gain = 0;

  trajectory_publisher_ = nh.advertise<nav_msgs::Path>("/nav/local_trajectory", 100);
}

DWA::~DWA()
{
}