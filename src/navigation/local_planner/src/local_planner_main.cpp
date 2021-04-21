#include "local_planner/dynamic_window_approach.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "local_planner_node");
  ros::NodeHandle nh;
  DWA dwa(&nh, "dwa_local_plan");
  while (ros::ok())
  {
    ros::spinOnce();
  }
  return 0;
}