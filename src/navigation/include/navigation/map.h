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

/**
 * @brief A class for map and utilities
 */
class Map
{
public:
  /**
   * @brief Construct a new Map object
   * @param map
   */
  Map(const nav_msgs::OccupancyGrid& map);
  /**
   * @brief Destroy the Map object
   */
  ~Map();

private:
  /**
   * @brief Map width
   */
  int width_;
  /**
   * @brief Map height
   */
  int height_;
};

#endif  // MAP_H