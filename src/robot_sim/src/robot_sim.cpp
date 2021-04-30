#include "robot_sim/robot_sim.h"

RobotSim::RobotSim(ros::NodeHandle* nh)
  : map_(nh)
  , generator_(std::chrono::system_clock::now().time_since_epoch().count())
  , odom_robot_(State())
{
  cmd_vel_subscriber_ = nh->subscribe("cmd_vel", 1, &RobotSim::cmdVelCallback, this);
  wheel_vel_publisher_ = nh->advertise<sensor_msgs::JointState>("joint_state", 1);
  laser_scan_publisher_ = nh->advertise<sensor_msgs::LaserScan>("scan", 1);

  current_u_.speed = 0.0;
  current_u_.steering_angle_velocity = 0.0;

  geometry_msgs::PoseStamped pose;

  if (tf_helper_.getCurrentPoseFromTF("map", "base_link", &pose))
  {
    State state;
    tf::Quaternion q;
    q.setX(pose.pose.orientation.x);
    q.setY(pose.pose.orientation.y);
    q.setZ(pose.pose.orientation.z);
    q.setW(pose.pose.orientation.w);

    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    state.x = pose.pose.position.x;
    state.y = pose.pose.position.y;
    state.theta = yaw;

    odom_robot_.setState(state);
  }

  loadConfig(nh);
}

void RobotSim::loadConfig(ros::NodeHandle* nh)
{
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

  u_wheel_frame.name.resize(2);
  u_wheel_frame.position.resize(2);
  u_wheel_frame.velocity.resize(2);

  u_wheel_frame.name[0] = "left_wheel_joint";
  u_wheel_frame.name[1] = "right_wheel_joint";

  u_wheel_frame.velocity[0] = current_u_.speed -
                              (AXLE_LENGTH / 2.0 * current_u_.steering_angle_velocity) +
                              noise(generator_);

  u_wheel_frame.velocity[1] = current_u_.speed +
                              (AXLE_LENGTH / 2.0 * current_u_.steering_angle_velocity) +
                              noise(generator_);

  float v = 0.5 * (u_wheel_frame.velocity[0] + u_wheel_frame.velocity[1]);
  float w = (u_wheel_frame.velocity[1] - u_wheel_frame.velocity[0]) / AXLE_LENGTH;

  State state = odom_robot_.updateRobotState(v, w);
  tf_helper_.broadcastCurrentPoseToTF(state.x, state.y, state.theta, "map", "odom");

  wheel_vel_publisher_.publish(u_wheel_frame);
}

void RobotSim::publishLaserScan()
{
  sensor_msgs::LaserScan laser_scan;
  laser_scan.angle_min = -1.5;
  laser_scan.angle_max = 1.5;
  laser_scan.angle_increment = 0.05;
  laser_scan.range_min = 0.0;
  laser_scan.range_max = 3.0;
  laser_scan.time_increment = 0.001;
  laser_scan.header.frame_id = "base_link";

  float theta = laser_scan.angle_min;
  std::vector<float> ranges;

  while (theta <= laser_scan.angle_max)
  {
    float distance = laser_scan.range_min;
    while (distance <= laser_scan.range_max)
    {
      std::normal_distribution<float> noise(0.0, sqrt(config_.noise.laser_variance));
      geometry_msgs::PoseStamped pose;
      if (!tf_helper_.getCurrentPoseFromTF("map", "base_link", &pose))
      {
        ROS_WARN_STREAM("[RobotSim]: tf link between map and base_link not available");
        return;
      }

      tf::Quaternion q;
      q.setX(pose.pose.orientation.x);
      q.setY(pose.pose.orientation.y);
      q.setZ(pose.pose.orientation.z);
      q.setW(pose.pose.orientation.w);

      tf::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      pose.pose.position.x = pose.pose.position.x + distance * cos(theta + yaw);
      pose.pose.position.y = pose.pose.position.y + distance * sin(theta + yaw);

      // Check world for obstacles
      if (!map_.getNodeFromMap(pose.pose.position.x, pose.pose.position.y) ||
          map_.getNodeFromMap(pose.pose.position.x, pose.pose.position.y)->is_obstacle)
      {
        break;
      }

      distance = distance + map_.getResolution() + noise(generator_);
    }
    ranges.push_back(distance);
    theta = theta + laser_scan.angle_increment;
  }
  laser_scan.ranges = ranges;
  laser_scan.header.stamp = ros::Time::now();
  laser_scan_publisher_.publish(laser_scan);
}