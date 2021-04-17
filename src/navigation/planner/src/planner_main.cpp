#include "planner/planner.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "planner");
  ros::NodeHandle nh;

  Planner planner(nh);

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