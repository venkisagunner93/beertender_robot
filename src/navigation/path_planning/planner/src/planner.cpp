#include "planner/planner.h"

Planner::Planner(ros::NodeHandle* nh)
  : bfs_ac_("bfs_global_plan", true), dwa_ac_("dwa_local_plan", true)
{
  global_goal_subscriber_ = nh->subscribe("nav/global_goal", 1, &Planner::globalGoalCallback, this);
}

Planner::~Planner() {}

void Planner::globalGoalCallback(const geometry_msgs::PointStampedConstPtr& msg)
{
  nav_msgs::Path global_path = callBFSActionServer(msg);

  for (auto& pose : global_path.poses)
  {
    callDWAActionServer(pose);
    // ROS_INFO_STREAM(pose.pose.position.x << "," << pose.pose.position.y);
  }
}

nav_msgs::Path Planner::callBFSActionServer(const geometry_msgs::PointStampedConstPtr& msg)
{
  nav_msgs::Path global_path;

  if (bfs_ac_.waitForServer(ros::Duration(2.0)))
  {
    nav_utils::FindGlobalPathGoal goal;
    goal.goal = *msg;

    bfs_ac_.sendGoal(goal);

    if (bfs_ac_.waitForResult(ros::Duration(5.0)))
    {
      nav_utils::FindGlobalPathResultConstPtr result = bfs_ac_.getResult();

      if (result->success)
      {
        ROS_INFO_STREAM("[Planner]: Path to requested global goal calculated successfully");
        global_path = result->global_path;
      }
      else
      {
        ROS_WARN_STREAM(
            "[Planner]: Requested global goal cannot be achieved. Try a new global goal");
      }
    }
  }

  return global_path;
}

void Planner::callDWAActionServer(const geometry_msgs::PoseStamped& pose)
{
  if (dwa_ac_.waitForServer(ros::Duration(2.0)))
  {
    nav_utils::ReachGlobalPoseGoal goal;
    goal.goal = pose;

    dwa_ac_.sendGoal(goal);

    if (dwa_ac_.waitForResult(ros::Duration(10.0)))
    {
      nav_utils::ReachGlobalPoseResultConstPtr result = dwa_ac_.getResult();

      if (result->has_reached)
      {
        ROS_INFO_STREAM("[Planner]: Reached local goal");
      }
    }
    else
    {
      ROS_WARN_STREAM("[Planner]: Timed out");
      dwa_ac_.cancelGoal();
    }
  }
}

void Planner::run() {}