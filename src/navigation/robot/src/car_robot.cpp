#include "robot/car_robot.h"

CarRobot::CarRobot(const State& init_state) : state_(init_state)
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

  state_ = init_state;
}

void CarRobot::displayRobotDetails() const
{
  ROS_INFO_STREAM("Robot name: " << dimension_.name.c_str());
  ROS_INFO_STREAM("Robot length: " << dimension_.length);
  ROS_INFO_STREAM("Robot width: " << dimension_.width);
  ROS_INFO_STREAM("Robot height: " << dimension_.height);
}

State CarRobot::updateRobotState(const float& v, const float& w)
{
  ros::Duration dt = ros::Time::now() - prev_time_;
  prev_time_ = ros::Time::now();

  state_.theta = w * dt.toSec() + state_.theta;

  state_.x = v * dt.toSec() * cos(state_.theta) + state_.x;
  state_.y = v * dt.toSec() * sin(state_.theta) + state_.y;

  state_.v = v;
  state_.w = w;

  return state_;
}

State CarRobot::updateRobotState(const float& v, const float& w, const float& dt)
{
  state_.theta = w * dt + state_.theta;

  state_.x = v * dt * cos(state_.theta) + state_.x;
  state_.y = v * dt * sin(state_.theta) + state_.y;

  state_.v = v;
  state_.w = w;

  return state_;
}

Dimension CarRobot::getDimension() const
{
  return dimension_;
}

State CarRobot::getState() const
{
  return state_;
}

void CarRobot::setState(const State& state)
{
  state_ = state;
}