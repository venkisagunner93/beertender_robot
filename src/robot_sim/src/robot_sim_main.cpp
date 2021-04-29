#include "robot_sim/robot_sim.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_sim");
  ros::NodeHandle nh;
  RobotSim robot_sim(&nh);

  ros::Rate rate(10);
  while (ros::ok())
  {
		robot_sim.publishIndividualWheelVelocity();
		robot_sim.publishLaserScan();
		rate.sleep();
		ros::spinOnce();
  }
}