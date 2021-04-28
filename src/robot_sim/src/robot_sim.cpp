#include "robot_sim/robot_sim.h"

RobotSim::RobotSim(ros::NodeHandle* nh) : generator_(std::chrono::system_clock::now().time_since_epoch().count())
{
  cmd_vel_subscriber_ = nh->subscribe("cmd_vel", 1, &RobotSim::cmdVelCallback, this);
  wheel_vel_publisher_ = nh->advertise<sensor_msgs::JointState>("joint_state", 1);

  current_u_.speed = 0.0;
  current_u_.steering_angle_velocity = 0.0;

  loadConfig(nh);
}

void RobotSim::loadConfig(ros::NodeHandle* nh)
{
  nh->param<float>("robot_sim/noise/variance", config_.noise.variance, 0.0);
}

void RobotSim::cmdVelCallback(const ackermann_msgs::AckermannDrive& msg)
{
  current_u_ = msg;
}

void RobotSim::publishIndividualWheelVelocity()
{
  sensor_msgs::JointState u_wheel_frame;
  std::normal_distribution<float> noise(0.0, sqrt(config_.noise.variance));

  u_wheel_frame.name.resize(2);
  u_wheel_frame.position.resize(2);
  u_wheel_frame.velocity.resize(2);

  u_wheel_frame.name[0] = "left_wheel_joint";
  u_wheel_frame.name[1] = "right_wheel_joint";

  u_wheel_frame.velocity[0] =
      current_u_.speed - (AXLE_LENGTH / 2.0 * current_u_.steering_angle_velocity) + noise(generator_);

  u_wheel_frame.velocity[1] =
      current_u_.speed + (AXLE_LENGTH / 2.0 * current_u_.steering_angle_velocity) + noise(generator_);

  wheel_vel_publisher_.publish(u_wheel_frame);
}