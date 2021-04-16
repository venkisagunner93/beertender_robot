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

#include <queue>
#include <thread>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PointStamped.h>
#include "navigation/global_planner/breadth_first_search.h"
#include "navigation/local_planner/dynamic_window_approach.h"
#include "navigation/SetGoal.h"
#include "navigation/map.h"
#include "robot/car_robot.h"
#include <stack>

/**
 * @brief A class for orchestrating planning and navigation
 */
class Planner
{
public:
  /**
   * @brief Construct a new Planner object
   * @param nh - ROS NodeHandle for communication
   */
  Planner(ros::NodeHandle& nh, Robot* robot);
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
   * @brief A callback to load map from map server
   * @param msg - Occupancy grid
   */
  void getMap(const nav_msgs::OccupancyGrid& msg);
  /**
   * @brief A callback to get global path
   * @param request
   * @param response
   * @return true
   * @return false
   */
  bool setGoal(navigation::SetGoalRequest& request, navigation::SetGoalResponse& response);
  /**
   * @brief Transform buffer
   */
  tf2_ros::Buffer buffer_;
  /**
   * @brief Transform listener
   */
  tf2_ros::TransformListener listener_;
  /**
   * @brief Breadth first search instance
   */
  BFS bfs_;
  /**
   * @brief Dynamic window approach instance
   */
  DWA dwa_;
  /**
   * @brief Map instance
   */
  Map map_;
  /**
   * @brief Robot instance
   */
  Robot* robot_;
  /**
   * @brief Mutex for global path
   */
  std::mutex mutex_;
  /**
   * @brief Flag to stop local planner
   */
  bool stop_local_planner_;
  /**
   * @brief Flag to check whether waypoint is reached
   */
  bool waypoint_reached_ = true;
  /**
   * @brief Global path
   */
  std::stack<geometry_msgs::PoseStamped> global_path_waypoints_;
  /**
   * @brief Map subscriber
   */
  ros::Subscriber map_subscriber_;
  /**
   * @brief Global path publisher
   */
  ros::Publisher global_path_publisher_;
  /**
   * @brief Set goal service
   */
  ros::ServiceServer set_goal_service_;
};

#endif  // PLANNER_H
