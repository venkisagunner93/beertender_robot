#include "navigation/planner.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "planner");
  ros::NodeHandle nh;

  DriveLimits limits;
  limits.max_linear_velocity = 0.2;       // (m/s)
  limits.min_linear_velocity = 0.025;     // (m/s)
  limits.max_angular_velocity = 0.7;      // (rad/s)
  limits.min_angular_velocity = -0.7;     // (rad/s)
  limits.max_linear_acceleration = 0.01;  // (m/s^2)
  limits.max_angular_acceleration = 1.0;  // (rad/s^2)
  limits.max_steering_angle = 0.5235;     // (rad)

  CarRobot car_robot(limits);
  car_robot.displayRobotDetails();

  Planner planner(nh, &car_robot);

  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  {
    ros::console::notifyLoggerLevelsChanged();
  }

  ros::Rate rate(10);

  while (ros::ok())
  {
    planner.run();
    rate.sleep();
    ros::spinOnce();
  }
  return 0;
}