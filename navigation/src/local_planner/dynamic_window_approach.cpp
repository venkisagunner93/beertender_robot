#include "navigation/local_planner/dynamic_window_approach.h"

DWA::DWA(ros::NodeHandle& nh, Robot* robot, const DWAConfig& config) : robot_(robot), config_(config)
{

}

DWA::~DWA()
{

}

void DWA::setGlobalPath(const nav_msgs::Path& msg)
{
    global_path_ = msg;
}

void DWA::computeDynamicWindow()
{
    DriveLimits limits = robot_->getDriveLimits();
    State state = robot_->getState();

    float min_v = std::max(-1 * limits.max_linear_velocity, state.linear_velocity - limits.max_linear_acceleration * config_.dt);
    float max_v = std::min(limits.max_linear_velocity, state.linear_velocity + limits.max_linear_acceleration * config_.dt);
    
    float min_w = std::max(-1 * limits.max_angular_velocity, state.angular_velocity - limits.max_angular_acceleration * config_.dt);
    float max_w = std::max(limits.max_angular_velocity, state.angular_velocity + limits.max_angular_acceleration * config_.dt);

    for(int i = 0; i < config_.window_size; i++)
    {
        
    }
}