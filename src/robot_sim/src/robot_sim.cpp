#include "robot_sim/robot_sim.h"
#include "occupancy_grid_utils/ray_tracer.cpp"

RobotSim::RobotSim(ros::NodeHandle* nh)
  : map_(nh), generator_(std::chrono::system_clock::now().time_since_epoch().count())
{
  cmd_vel_subscriber_ = nh->subscribe("cmd_vel", 1, &RobotSim::cmdVelCallback, this);
  wheel_vel_publisher_ = nh->advertise<sensor_msgs::JointState>("joint_states", 100);
  laser_scan_publisher_ = nh->advertise<sensor_msgs::LaserScan>("scan", 1000);

  current_u_.speed = 0.0;
  current_u_.steering_angle_velocity = 0.0;

  loadConfig(nh);
  loadInitState(nh);

  prev_time_ = ros::Time::now();
}

void RobotSim::loadInitState(ros::NodeHandle* nh)
{
  // Loading initial state of the robot
  nh->param<float>("robot_sim/init_state/x", odom_state_.x, 0.0);
  nh->param<float>("robot_sim/init_state/y", odom_state_.y, 0.0);
  nh->param<float>("robot_sim/init_state/theta", odom_state_.theta, M_PI / 2);
  nh->param<float>("robot_sim/init_state/v", odom_state_.v, 0.0);
  nh->param<float>("robot_sim/init_state/w", odom_state_.w, 0.0);
}

void RobotSim::loadConfig(ros::NodeHandle* nh)
{
  // Laser configuration parameters
  nh->param<float>("robot_sim/laser_config/angle_min", laser_info_.angle_min, -1.57);
  nh->param<float>("robot_sim/laser_config/angle_max", laser_info_.angle_max, 1.57);
  nh->param<float>("robot_sim/laser_config/angle_increment", laser_info_.angle_increment, 0.01);
  nh->param<float>("robot_sim/laser_config/range_min", laser_info_.range_min, 0.0);
  nh->param<float>("robot_sim/laser_config/range_max", laser_info_.range_max, 3.0);
  nh->param<float>("robot_sim/laser_config/time_increment", laser_info_.time_increment, 0.001);

  // Odometery noise config
  nh->param<float>("robot_sim/noise/odom_variance", config_.noise.odom_variance, 0.0);
  nh->param<float>("robot_sim/noise/laser_variance", config_.noise.laser_variance, 0.0);
}

void RobotSim::cmdVelCallback(const ackermann_msgs::AckermannDrive& msg)
{
  current_u_ = msg;
}

void RobotSim::publishIndividualWheelVelocity()
{
  sensor_msgs::JointState u_wheel_frame;
  std::normal_distribution<float> noise(0.0, sqrt(config_.noise.odom_variance));

  ros::Duration dt = ros::Time::now() - prev_time_;
  prev_time_ = ros::Time::now();

  // u_wheel_frame.name.resize(2);
  u_wheel_frame.position.resize(2);
  u_wheel_frame.velocity.resize(2);

  u_wheel_frame.header.stamp = ros::Time::now();

  u_wheel_frame.name.push_back("left_wheel_joint");
  u_wheel_frame.name.push_back("right_wheel_joint");

  u_wheel_frame.velocity[0] = current_u_.speed -
                              (AXLE_LENGTH / 2.0 * current_u_.steering_angle_velocity) +
                              noise(generator_);

  u_wheel_frame.velocity[1] = current_u_.speed +
                              (AXLE_LENGTH / 2.0 * current_u_.steering_angle_velocity) +
                              noise(generator_);

  float v = 0.5 * (u_wheel_frame.velocity[0] + u_wheel_frame.velocity[1]);
  float w = (u_wheel_frame.velocity[1] - u_wheel_frame.velocity[0]) / AXLE_LENGTH;

  tf_helper_.broadcastCurrentPoseToTF(odom_state_.x, odom_state_.y, odom_state_.theta, "map",
                                      "odom");

  robot_utils::updateState(state_, v, w, dt.toSec());
  tf_helper_.broadcastCurrentPoseToTF(state_.x, state_.y, state_.theta, "odom", "base_footprint");

  wheel_vel_publisher_.publish(u_wheel_frame);
}

void RobotSim::publishLaserScan()
{
  geometry_msgs::Pose pose;
  tf_helper_.getCurrentPoseFromTF("map", "base_link", &pose);

  sensor_msgs::LaserScan::Ptr laser_scan =
      occupancy_grid_utils::simulateRangeScan(map_.getOccupancyGrid(), pose, laser_info_, false);

  laser_scan->header.frame_id = "base_laser";
  laser_scan->header.stamp = ros::Time::now();
  laser_scan_publisher_.publish(laser_scan);
}