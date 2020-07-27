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
    State next_state;

    ros::Duration dt = ros::Time::now() - prev_time_;
    
    next_state.theta = control_input.steering_velocity * dt.toSec() + current_state.theta;

    next_state.x = control_input.forward_velocity * dt.toSec() * cos(next_state.theta) + current_state.x;
    next_state.y = control_input.forward_velocity * dt.toSec() * sin(next_state.theta) + current_state.y;

    next_state.x_dot = (next_state.x - current_state.x) / dt.toSec();
    next_state.y_dot = (next_state.y - current_state.y) / dt.toSec();
    next_state.theta_dot = (next_state.theta - current_state.theta) / dt.toSec();

    return next_state;
}

geometry_msgs::TransformStamped CarRobot::getCurrentPose() const
{
    geometry_msgs::TransformStamped pose;
    try
    {
        pose = buffer_.lookupTransform("/map", "/base_link", ros::Time(0));
    }
    catch(tf2::TransformException &ex)
    {
        ROS_WARN_STREAM(ex.what());
    }
    return pose;
}

void CarRobot::broadcastPose()
{
    geometry_msgs::TransformStamped transformStamped;
  
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = "base_link";
    transformStamped.transform.translation.x = 0.0;
    transformStamped.transform.translation.y = 0.0;
    transformStamped.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    broadcaster_.sendTransform(transformStamped);
}