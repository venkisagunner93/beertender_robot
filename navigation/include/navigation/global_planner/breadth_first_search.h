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
#include "navigation/global_planner/global_planner.h"

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
         * @return nav_msgs::Path 
         */
        nav_msgs::Path getGlobalPath(const geometry_msgs::PointStamped& start, const geometry_msgs::PointStamped& goal);
        /**
         * @brief A method to update map
         * @param map - Updated map
         */
        void updateMap(const nav_msgs::OccupancyGrid& map);
        /**
         * @brief A method to display graph
         */
        void displayGraph() const;
    
    private:
        /**
         * @brief Map occupancy grid
         */
        nav_msgs::OccupancyGrid map_;
        /**
         * @brief Graph from map
         */
        std::vector<std::vector<Node*>> graph_;
        /**
         * @brief A method to reset graph of the map
         */
        void resetGraph();
        /**
         * @brief A method to create a Graph From Map object
         */
        void createGraphFromMap();
        /**
         * @brief A method to find neighbors of a given node
         * @param node - Node containing neighbors
         * @return std::vector<Node*> 
         */
        std::vector<Node*> findNeighbors(Node* node);
        /**
         * @brief A method to create a Path object
         * @param goal_node - Goal node
         * @return nav_msgs::Path 
         */
        nav_msgs::Path createPath(Node* goal_node);
};

#endif  // BREADTH_FIRST_SEARCH_H