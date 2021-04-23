#include "local_planner/dynamic_window_approach.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "local_planner_node");
  ros::NodeHandle nh;
  DWA dwa(&nh, "dwa_local_plan");
  ros::Rate rate(100);

  while (ros::ok())
  {
    // For visualization purposes
    dwa.broadcastCurrentPose();
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}