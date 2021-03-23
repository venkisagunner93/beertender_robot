#include "navigation/planner.h"

Planner::Planner(ros::NodeHandle& nh)
{
  map_subscriber_ = nh.subscribe("basic_map", 100, &Planner::getMap, this);
  global_path_publisher_ = nh.advertise<nav_msgs::Path>("robot/global_path", 100);
  set_goal_service_ = nh.advertiseService("nav/set_goal", &Planner::setGoal, this);
}

Planner::~Planner()
{
}

void Planner::getMap(const nav_msgs::OccupancyGrid& msg)
{
  bfs_.updateMap(msg);
}

bool Planner::setGoal(navigation::SetGoalRequest& request, navigation::SetGoalResponse& response)
{
  geometry_msgs::PointStamped goal;
  goal.header.stamp = request.goal.header.stamp;
  goal.point.x = request.goal.point.x;
  goal.point.y = request.goal.point.y;

  geometry_msgs::PointStamped start;
  start.header.stamp = ros::Time::now();
  start.point.x = 1;
  start.point.y = 1;

  nav_msgs::Path global_path = bfs_.getGlobalPath(start, goal);

  if (!global_path.poses.empty())
  {
    global_path_publisher_.publish(global_path);
  }

  return true;
}

void Planner::planMotion()
{
  std::cout << "tst";
  std::cout << "tst";
}
