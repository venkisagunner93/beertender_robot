#include "local_planner/local_planner.h"

int main(int argc, char** argv)
{
  //   DriveLimits limits;
  //   limits.max_linear_velocity = 0.2;       // (m/s)
  //   limits.min_linear_velocity = 0.025;     // (m/s)
  //   limits.max_angular_velocity = 0.7;      // (rad/s)
  //   limits.min_angular_velocity = -0.7;     // (rad/s)
  //   limits.max_linear_acceleration = 0.01;  // (m/s^2)
  //   limits.max_angular_acceleration = 1.0;  // (rad/s^2)
  //   limits.max_steering_angle = 0.5235;     // (rad)

  ros::init(argc, argv, "local_planner_node");
  ros::NodeHandle nh;
  while (ros::ok())
  {
    ros::spinOnce();
  }
  return 0;
}