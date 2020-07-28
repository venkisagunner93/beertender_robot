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

bool Planner::getGlobalPath(navigation::SetGoalRequest& request, navigation::SetGoalResponse& response)
{
    Coordinate start;
    start.x = robot_->getCurrentPose().transform.translation.y;
    start.y = robot_->getCurrentPose().transform.translation.x;

    Coordinate goal;
    goal.x = request.y;
    goal.y = request.x;

    global_path_publisher_.publish(bfs_.getGlobalPath(start, goal));

    return true;
}

void Planner::updateRobotPose()
{
    State new_state;
    new_state.x = 5;
    new_state.y = 3;
    new_state.theta = 0;
    robot_->setNewPose(new_state);
}
