#include "robot/car_robot.h"

CarRobot::CarRobot(const DriveLimits& drive_limits)
{
  urdf::Model model;

  if (model.initParam("robot"))
  {
    dimension_.name = model.getName();

    if (model.getLink("base_link").get())
    {
      urdf::Box* box = dynamic_cast<urdf::Box*>(model.getLink("base_link")->visual->geometry.get());
      dimension_.width = box->dim.x;
      dimension_.length = box->dim.y;
      dimension_.height = box->dim.z;
    }
  }
  prev_time_ = ros::Time::now();

  drive_limits_ = drive_limits;

  state_.x = 1.0;
  state_.y = 1.0;
  state_.theta = 0.0;
}

void CarRobot::displayRobotDetails() const
{
  ROS_INFO_STREAM("Robot name: " << dimension_.name.c_str());
  ROS_INFO_STREAM("Robot length: " << dimension_.length);
  ROS_INFO_STREAM("Robot width: " << dimension_.width);
  ROS_INFO_STREAM("Robot height: " << dimension_.height);
}

State CarRobot::executeCommand(const ControlInput& control_input)
{
  ros::Duration dt = ros::Duration(0.1);

  state_.theta = control_input.steering_velocity * dt.toSec() + state_.theta;

  state_.x = control_input.forward_velocity * dt.toSec() * cos(state_.theta) + state_.x;
  state_.y = control_input.forward_velocity * dt.toSec() * sin(state_.theta) + state_.y;

  state_.linear_velocity = control_input.forward_velocity;
  state_.angular_velocity = control_input.steering_velocity;

  prev_time_ = ros::Time::now();

  broadcastPose();

  return state_;
}

void CarRobot::broadcastPose()
{
  static tf2_ros::TransformBroadcaster broadcaster;
  geometry_msgs::TransformStamped current_pose;

  current_pose.header.stamp = ros::Time::now();
  current_pose.header.frame_id = "map";
  current_pose.child_frame_id = "base_link";
  current_pose.transform.translation.x = state_.x;
  current_pose.transform.translation.y = state_.y;
  current_pose.transform.translation.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, state_.theta);
  current_pose.transform.rotation.x = q.x();
  current_pose.transform.rotation.y = q.y();
  current_pose.transform.rotation.z = q.z();
  current_pose.transform.rotation.w = q.w();

  broadcaster.sendTransform(current_pose);
}

DriveLimits CarRobot::getDriveLimits() const
{
  return drive_limits_;
}

Dimension CarRobot::getDimension() const
{
  return dimension_;
}

State CarRobot::getCurrentState() const
{
  return state_;
}