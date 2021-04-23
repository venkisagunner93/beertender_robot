/**
 * @file robot_sim.h
 * @author Venkatavaradhan Vembanoor Lakshmi Narayanan (venkatavaradhan93@gmail.com)
 * @brief A class for simulating robot behaviors
 * @version 0.1
 * @date 2021-04-21
 * @copyright Copyright (c) 2021
 */

#ifndef ROBOT_SIM_H
#define ROBOT_SIM_H

#include <ros/ros.h>
#include <ackermann_msgs/AckermannDrive.h>

/**
 * @brief A class for simuating robot behaviors
 */
class RobotSim
{
public:
  /**
   * @brief Construct a new Robot Sim object
   */
  RobotSim(ros::NodeHandle* nh);
  /**
   * @brief Destroy the Robot Sim object
   */
  ~RobotSim() {}
	void updateRobotPose()

private:
	/**
	 * @brief A subscriber callback for velocity commands
	 * @param msg Velocity command
	 */
	void cmdVelCallback(const ackermann_msgs::AckermannDriveConstPtr& msg);
};

#endif  // ROBOT_SIM_H