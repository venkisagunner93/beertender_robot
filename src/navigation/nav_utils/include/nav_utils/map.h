/**
 * @file map.h
 * @author Venkatavaradhan Vembanoor Lakshmi Narayanan
 * (venkatavaradhan93@gmail.com)
 * @brief A class for map and utilties
 * @version 0.1
 * @date 2021-03-21
 * @copyright Copyright (c) 2021
 */

#ifndef MAP_H
#define MAP_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>

/**
 * @brief A structure for graph node
 */
struct Node
{
  unsigned int x;   /*< X co-ordinate */
  unsigned int y;   /*< Y co-ordinate */
  bool is_obstacle; /*< Obstacle flag */
  bool is_visited;  /*< Visited flag */
  Node* parent;     /*< Parent */
  /**
   * @brief Construct a new Node object
   */
  Node()
  {
    x = 0;
    y = 0;
    is_obstacle = false;
    is_visited = false;
    parent = nullptr;
  }
};

/**
 * @brief A class for map and utilities
 */
class Map
{
public:
  /**
   * @brief Construct a new Map object
   * @param nh ROS Nodehandle for communication
   */
  Map(ros::NodeHandle* nh);
  /**
   * @brief Get the Width In Pixels object
   * @return int
   */
  int getWidthInPixels() const;
  /**
   * @brief Get the Height In Pixels object
   * @return int
   */
  int getHeightInPixels() const;
  /**
   * @brief Get the Resolution object
   * @return float
   */
  float getResolution() const;
  /**
   * @brief Get the Width In Meters object
   * @return float
   */
  float getWidthInMeters();
  /**
   * @brief Get the Height In Meters object
   * @return float
   */
  float getHeightInMeters();
  /**
   * @brief A method to get graph
   * @return std::vector<std::vector<Node*>>
   */
  std::vector<std::vector<Node*>> getGraph() const;
  /**
   * @brief A helper method to display graph
   */
  void displayGraph() const;
  /**
   * @brief A method to reset the entire graph to default state
   */
  void resetGraph();
  /**
   * @brief Get the Node from graph (inputs are in map frame)
   * @param x 
   * @param y 
   * @return Node* 
   */
  Node* getNodeFromMap(const float& x, const float& y);
  /**
   * @brief A method to get the occupancy grid
   * @return nav_msgs::OccupancyGrid 
   */
  nav_msgs::OccupancyGrid getOccupancyGrid() const;

private:
  /**
   * @brief A subscriber callback for global map
   * @param msg Occupancy grid message
   */
  void mapCallback(const nav_msgs::OccupancyGrid& msg);
  /**
   * @brief Create a Graph From Map object
   */
  void createGraphFromMap();
  /**
   * @brief Map width
   */
  int width_;
  /**
   * @brief Map height
   */
  int height_;
  /**
   * @brief Map resolution
   */
  float resolution_;
  /**
   * @brief Map from map server
   */
  nav_msgs::OccupancyGrid map_;
  /**
   * @brief 2D graph datastructure
   */
  std::vector<std::vector<Node*>> graph_;
  /**
   * @brief Global map client
   */
  ros::ServiceClient get_map_client_;
};

#endif  // MAP_H