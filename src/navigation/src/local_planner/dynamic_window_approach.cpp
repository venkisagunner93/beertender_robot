#include "navigation/local_planner/dynamic_window_approach.h"

DWA::DWA(ros::NodeHandle& nh, Robot* robot) : robot_(robot)
{
  config_.sim_time = 2.0;    // in seconds
  config_.dt = 0.05;         // time step
  config_.window_size = 15;  // Velocity space max window size
  config_.obstacle_gain = 0;
  config_.distance_to_goal_gain = 0;
  config_.max_velocity_gain = 0;

  trajectory_publisher_ = nh.advertise<nav_msgs::Path>("/nav/local_trajectory", 100);
}

void DWA::executeTrajectory(const geometry_msgs::PoseStamped& pose)
{

}

