#include "robot/car_robot.h"

CarRobot::CarRobot(const DriveLimits& drive_limits) : listener_(buffer_)
{
    urdf::Model model;

    if(model.initParam("robot"))
    {
        dimension_.name = model.getName();

        if(model.getLink("base_link").get())
        {
            urdf::Box* box = dynamic_cast<urdf::Box*>(model.getLink("base_link")->visual->geometry.get());
            dimension_.width = box->dim.x;
            dimension_.length = box->dim.y;
            dimension_.height = box->dim.z;
        }
    }
    prev_time_ = ros::Time::now();

    drive_limits_ = drive_limits;
}

void CarRobot::displayRobotDetails() const
{
    ROS_INFO_STREAM("Robot name: " << dimension_.name.c_str());
    ROS_INFO_STREAM("Robot length: " << dimension_.length);
    ROS_INFO_STREAM("Robot width: " << dimension_.width);
    ROS_INFO_STREAM("Robot height: " << dimension_.height);
}

State CarRobot::updateState(const State& current_state, const ControlInput& control_input)
{
    ros::Duration dt = ros::Duration(0.1);
    
    state_.theta = control_input.steering_velocity * dt.toSec() + current_state.theta;

    state_.x = control_input.forward_velocity * dt.toSec() * cos(state_.theta) + current_state.x;
    state_.y = control_input.forward_velocity * dt.toSec() * sin(state_.theta) + current_state.y;

    state_.linear_velocity = control_input.forward_velocity;
    state_.angular_velocity = control_input.steering_velocity;

    prev_time_ = ros::Time::now();

    return state_;
}

geometry_msgs::TransformStamped CarRobot::getCurrentPose() const
{
    geometry_msgs::TransformStamped pose;
    try
    {
        pose = buffer_.lookupTransform("map", "base_link", ros::Time(0));
    }
    catch(tf2::TransformException &ex)
    {
        ROS_WARN_STREAM(ex.what());
    }
    return pose;
}

void CarRobot::setNewPose(const State& state)
{
    geometry_msgs::TransformStamped pose;
  
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "map";
    pose.child_frame_id = "base_link";
    pose.transform.translation.x = state.x;
    pose.transform.translation.y = state.y;
    pose.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, state.theta);
    pose.transform.rotation.x = q.x();
    pose.transform.rotation.y = q.y();
    pose.transform.rotation.z = q.z();
    pose.transform.rotation.w = q.w();

    broadcaster_.sendTransform(pose);
}

DriveLimits CarRobot::getDriveLimits() const
{
    return drive_limits_;
}

Dimension CarRobot::getDimension() const
{
    return dimension_;
}

State CarRobot::getState() const
{
    return state_;
}