#include "robot_sim/robot_sim.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_sim");
  ros::NodeHandle nh;
  RobotSim robot_sim(&nh);

  ros::Rate rate(100);
  while (ros::ok())
  {
		robot_sim.publishIndividualWheelVelocity();
		rate.sleep();
		ros::spinOnce();
  }
}