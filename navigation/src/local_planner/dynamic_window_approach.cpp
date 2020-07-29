#include "navigation/local_planner/dynamic_window_approach.h"

DWA::DWA(ros::NodeHandle& nh, Robot* robot) : robot_(robot)
{
    config_.sim_time = 2.0;
    config_.dt = 0.05;
    config_.window_size = 15;
    config_.obstacle_gain = 0;
    config_.distance_to_goal_gain = 0;
    config_.max_velocity_gain = 0;

    trajectory_publisher_ = nh.advertise<nav_msgs::Path>("/nav/local_trajectory", 100);
}

DWA::~DWA()
{

}

void DWA::computeDynamicWindow(DynamicWindow& window)
{
    DriveLimits limits = robot_->getDriveLimits();
    State state = robot_->getState();

    float min_v = std::max(-1 * limits.max_linear_velocity, state.linear_velocity - limits.max_linear_acceleration * config_.dt);
    float max_v = std::min(limits.max_linear_velocity, state.linear_velocity + limits.max_linear_acceleration * config_.dt);
    
    float min_w = std::max(-1 * limits.max_angular_velocity, state.angular_velocity - limits.max_angular_acceleration * config_.dt);
    float max_w = std::min(limits.max_angular_velocity, state.angular_velocity + limits.max_angular_acceleration * config_.dt);

    // ROS_INFO_STREAM("Dynamic window: [" << min_v << "," << max_v << "], [" << min_w << "," << max_w << "]");

    float v_sample = 2 * max_v / config_.window_size;
    float w_sample = 2 * max_w / config_.window_size;

    window.linear_velocity.resize(config_.window_size + 1);
    window.angular_velocity.resize(config_.window_size + 1);

    for(int i = 0; i < config_.window_size + 1; i++)
    {
        window.linear_velocity[i] = min_v + v_sample * i;
        window.angular_velocity[i] = min_w + w_sample * i;
        // ROS_INFO_STREAM(window.linear_velocity[i] << "," << window.angular_velocity[i]);
    }
}

void DWA::computeTrajectory(const ControlInput& input)
{
    State state = robot_->getState();
    // ROS_INFO_STREAM(state.x << "," << state.y << "," << state.theta);
    

    CarRobot car_robot(robot_->getDriveLimits());

    float t = 0.0;

    trajectory_.header.frame_id = "map";

    while(t <= config_.sim_time)
    {
        state = car_robot.updateState(state, input);
        
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = car_robot.getState().x;
        pose.pose.position.y = car_robot.getState().y;
        tf2::Quaternion q;
        q.setRPY(0, 0, state.theta);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();
        trajectory_.poses.push_back(pose);

        ROS_INFO_STREAM_THROTTLE(1, state.x << "," << state.y << "," << state.theta);

        t += config_.dt;
    }

    trajectory_publisher_.publish(trajectory_);
}

void DWA::moveTo(const geometry_msgs::PoseStamped& pose)
{
    DynamicWindow window;
    computeDynamicWindow(window);

    for(auto v: window.linear_velocity)
    {
        for(auto w: window.angular_velocity)
        {
            ControlInput input;
            input.forward_velocity = v;
            input.steering_velocity = w;

            computeTrajectory(input);
        }
    }
}