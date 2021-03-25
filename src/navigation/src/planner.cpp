#include "navigation/planner.h"

Planner::Planner(ros::NodeHandle& nh, Robot* robot) : listener_(buffer_), robot_(robot), dwa_(nh, robot)
{
  map_subscriber_ = nh.subscribe("basic_map", 100, &Planner::getMap, this);
  global_path_publisher_ = nh.advertise<nav_msgs::Path>("robot/global_path", 100);
  set_goal_service_ = nh.advertiseService("nav/set_goal", &Planner::setGoal, this);

  stop_motion_planner_ = false;
  motion_planner_thread_ = std::make_shared<std::thread>(std::bind(&Planner::motionPlanner, this));
}

Planner::~Planner()
{
  if (motion_planner_thread_->joinable())
  {
    stop_motion_planner_ = true;
    motion_planner_thread_->join();
    motion_planner_thread_.reset();
  }
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

  {
    std::lock_guard<std::mutex> lock(mutex_);

    // Find shortest global path - aka. global planning
    global_path_ = bfs_.getGlobalPath(start, goal, &map_);

    if (!global_path_.poses.empty())
    {
      global_path_publisher_.publish(global_path_);
    }
  }

  return true;
}

void Planner::motionPlanner()
{
  while (!stop_motion_planner_)
  {
    nav_msgs::Path global_path;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      global_path = global_path_;
      global_path_.poses.clear();
    }

    for (auto& pose : global_path.poses)
    {
      dwa_.executeTrajectory(pose);
    }
  }
}
