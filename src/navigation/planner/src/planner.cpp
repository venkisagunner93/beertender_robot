#include "planner/planner.h"

Planner::Planner(ros::NodeHandle& nh) : listener_(buffer_)
{
  set_goal_service_ = nh.advertiseService("nav/set_goal", &Planner::setGoal, this);
}

Planner::~Planner()
{
}

bool Planner::setGoal(nav_utils::SetGoalRequest& request, nav_utils::SetGoalResponse& response)
{
  geometry_msgs::PointStamped goal;
  goal.header.stamp = request.goal.header.stamp;
  goal.point.x = request.goal.point.x;
  goal.point.y = request.goal.point.y;

  geometry_msgs::TransformStamped tf_current_pose;
  try
  {
    tf_current_pose = buffer_.lookupTransform("map", "base_link", ros::Time(0));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN_STREAM(ex.what());
  }

  geometry_msgs::PointStamped start;
  start.header.stamp = ros::Time::now();
  start.point.x = tf_current_pose.transform.translation.x;
  start.point.y = tf_current_pose.transform.translation.y;

  ROS_INFO_STREAM(goal);

  return true;
}

void Planner::run()
{
}