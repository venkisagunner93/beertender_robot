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
#include "navigation/global_planner/global_planner.h"
#include "navigation/map.h"

/**
 * @brief A class for breadth first search global plan
 */
class BFS : public GlobalPlanner
{
public:
  /**
   * @brief Get the Path object
   * @param start - Start co-ordinate
   * @param goal - Goal co-ordinate
   * @param map - Map instance
   * @return nav_msgs::Path
   */
  nav_msgs::Path getGlobalPath(const geometry_msgs::PointStamped& start,
                               const geometry_msgs::PointStamped& goal,
                               Map* map);

private:
  /**
   * @brief A method to create a Path object
   * @param goal_node - Goal node
   * @return nav_msgs::Path
   */
  nav_msgs::Path createPath(Node* goal_node, Map* map);
};

#endif  // BREADTH_FIRST_SEARCH_H