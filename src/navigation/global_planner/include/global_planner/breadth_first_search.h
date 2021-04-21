/**
 * @file breadth_first_search.h
 * @author Venkatavaradhan Vembanoor Lakshmi Narayanan (venkatavaradhan93@gmail.com)
 * @brief A class for breadth first search global plan
 * @version 0.1
 * @date 2020-07-20
 * @copyright Copyright (c) 2020
 */

#ifndef BREADTH_FIRST_SEARCH_H
#define BREADTH_FIRST_SEARCH_H

#include <queue>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2/LinearMath/Quaternion.h>
#include "global_planner/global_planner.h"
#include "nav_utils/map.h"

#define PARENT_FRAME "map"
#define CHILD_FRAME "base_link"

/**
 * @brief A class for breadth first search global plan
 */
class BFS : public GlobalPlanner
{
public:
  /**
   * @brief Construct a new BFS object
   */
  BFS(ros::NodeHandle* nh);
  /**
   * @brief Destroy the BFS object
   */
  ~BFS() {}
  /**
   * @brief A method to run global planning
   */
  void runGlobalPlanning();

private:
  /**
   * @brief A method to initialize ROS subscribers
   * @param nh ROS Nodehandle for communication
   */
  void initializeSubscribers(ros::NodeHandle* nh);
  /**
   * @brief A method to initialize ROS publishers
   * @param nh ROS Nodehandle for communication
   */
  void initializePublishers(ros::NodeHandle* nh);
  /**
   * @brief A subscriber callback for global goal
   * @param msg Global goal message
   */
  void globalGoalCallback(const geometry_msgs::PointStamped& msg);
  /**
   * @brief Get the Path object
   * @param start - Start co-ordinate
   * @param goal - Goal co-ordinate
   * @return nav_msgs::Path
   */
  nav_msgs::Path getGlobalPath(const geometry_msgs::PointStamped& start,
                               const geometry_msgs::PointStamped& goal);
  /**
   * @brief A method to create a Path object
   * @param goal_node - Goal node
   * @return nav_msgs::Path
   */
  nav_msgs::Path createPath(Node* goal_node);
  /**
   * @brief Map instance for global planning
   */
  Map map_;
  /**
   * @brief ROS subscriber for getting global goal
   */
  ros::Subscriber global_goal_subscriber_;
  /**
   * @brief ROS publisher for global path
   */
  ros::Publisher global_path_publisher_;
  /**
   * @brief Global goal message
   */
  geometry_msgs::PointStamped global_goal_;
};

#endif  // BREADTH_FIRST_SEARCH_H