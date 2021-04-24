#include "planner/planner.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "planner");
  ros::NodeHandle nh;

  Planner planner(&nh);

  ros::Rate rate(100);

  while (ros::ok())
  {
    planner.run();
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}