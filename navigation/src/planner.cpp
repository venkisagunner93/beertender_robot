#include "navigation/planner.h"

Planner::Planner(ros::NodeHandle& nh, Robot* robot) : dwa_(nh, robot)
{
    robot_ = robot;

    map_subscriber_ = nh.subscribe("basic_map", 100, &Planner::getMap, this);
    global_path_publisher_ = nh.advertise<nav_msgs::Path>("robot/global_path", 100);
    set_goal_service_ = nh.advertiseService("nav/set_goal", &Planner::setGoal, this);

    init_state_.x = 2.0;
    init_state_.y = 3.0;

    ControlInput input;

    robot_->updateState(init_state_, input);
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
    goal.point.x = request.goal.point.y;
    goal.point.y = request.goal.point.x;

    goal_queue_.push(goal);

    return true;
}

void Planner::planMotion()
{
    State state;
    geometry_msgs::PointStamped start;
    start.header.stamp = robot_->getCurrentPose().header.stamp;
    start.point.x = robot_->getCurrentPose().transform.translation.y;
    start.point.y = robot_->getCurrentPose().transform.translation.x;
    
    state = init_state_;

    if(!goal_queue_.empty())
    {
        nav_msgs::Path global_path = bfs_.getGlobalPath(start, goal_queue_.front());
        goal_queue_.pop();

        // for(auto pose: global_path.poses)
        // {
            dwa_.moveTo(global_path.poses[0]);
        // }

        global_path_publisher_.publish(global_path);
    }

    robot_->setNewPose(state);
}
