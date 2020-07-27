#include "navigation/planner.h"

Planner::Planner(ros::NodeHandle& nh, Robot* robot)
{
    robot_ = robot;
    map_subscriber_ = nh.subscribe("basic_map", 100, &Planner::getMap, this);
    global_path_publisher_ = nh.advertise<nav_msgs::Path>("robot/global_path", 100);

    global_path_service_ = nh.advertiseService("nav/get_global_path", &Planner::getGlobalPath, this);    
}

Planner::~Planner()
{
    
}

void Planner::getMap(const nav_msgs::OccupancyGrid& msg)
{
    bfs_.updateMap(msg);
}

bool Planner::getGlobalPath(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response)
{
    Coordinate start;
    Coordinate goal;
    
    goal.x = 1;
    goal.y = 7;

    global_path_publisher_.publish(bfs_.getGlobalPath(start, goal));

    return true;
}

void Planner::updateRobotPose()
{
    robot_->broadcastPose();
}
