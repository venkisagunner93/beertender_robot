/**
 * @file planner.h
 * @author Venkatavaradhan Vembanoor Lakshmi Narayanan (venkatavaradhan93@gmail.com)
 * @brief A class for orchestrating planning and navigation
 * @version 0.1
 * @date 2020-07-26
 * @copyright Copyright (c) 2020
 */

#ifndef PLANNER_H
#define PLANNER_H

#include <mutex>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <geometry_msgs/PointStamped.h>
#include "nav_utils/FindGlobalPathAction.h"
#include "nav_utils/ReachGlobalPoseAction.h"

/**
 * @brief A class for orchestrating planning and navigation
 */
class Planner
{
public:
  /**
   * @brief Construct a new Planner object
   * @param nh ROS NodeHandle for communication
   */
  Planner(ros::NodeHandle* nh);
  /**
   * @brief Destroy the Planner object
   */
  ~Planner();
  /**
   * @brief A method to run planner
   */
  void run();

private:
  /**
   * @brief A subscriber callback for global goal
   * @param msg Global goal
   */
  void globalGoalCallback(const geometry_msgs::PointStampedConstPtr& msg);
  /**
   * @brief A method to call BFS action server
   * @param msg Global goal
   * @return nav_msgs::Path 
   */
  nav_msgs::Path callBFSActionServer(const geometry_msgs::PointStampedConstPtr& msg);
  /**
   * @brief A method to call DWA action server
   * @param pose Local goal
   */
  void callDWAActionServer(const geometry_msgs::PoseStamped& pose);
  /**
   * @brief Global goal subscriber
   */
  ros::Subscriber global_goal_subscriber_;
  /**
   * @brief Action client for global planner
   */
  actionlib::SimpleActionClient<nav_utils::FindGlobalPathAction> bfs_ac_;
  /**
   * @brief Action client for local planner
   */
  actionlib::SimpleActionClient<nav_utils::ReachGlobalPoseAction> dwa_ac_;
};

#endif  // PLANNER_H
