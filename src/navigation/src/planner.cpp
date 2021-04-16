#include "navigation/planner.h"

Planner::Planner(ros::NodeHandle& nh, Robot* robot)
  : listener_(buffer_), robot_(robot), dwa_(nh, robot)
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
  map_.setMap(msg);
}

bool Planner::setGoal(navigation::SetGoalRequest& request, navigation::SetGoalResponse& response)
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

  // Find shortest global path - aka. global planning

  nav_msgs::Path global_path = bfs_.getGlobalPath(start, goal, &map_);

  if (!global_path_waypoints_.empty())
  {
    stop_local_planner_ = true;
  }
  else
  {
    stop_local_planner_ = false;
  }

  if (!global_path.poses.empty())
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto& pose : global_path.poses)
    {
      global_path_waypoints_.push(pose);
    }

    global_path_publisher_.publish(global_path);
  }

  return true;
}

void Planner::run()
{
  ControlInput command;

  if (!global_path_waypoints_.empty())
  {
    std::lock_guard<std::mutex> lock(mutex_);
    waypoint_reached_ = dwa_.performLocalPlanning(global_path_waypoints_.top(), command);
    if (waypoint_reached_)
    {
      ROS_DEBUG_STREAM("Goal: [" << global_path_waypoints_.top().pose.position.x << ","
                                 << global_path_waypoints_.top().pose.position.y << "]");
      global_path_waypoints_.pop();
    }
  }

  robot_->executeCommand(command);
  robot_->broadcastPose();
}