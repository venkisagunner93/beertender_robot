/**
 * @file global_planner_types.h
 * @author Venkatavaradhan Vembanoor Lakshmi Narayanan (venkatavaradhan93@gmail.com)
 * @brief A file for global planner types
 * @version 0.1
 * @date 2020-07-26
 * @copyright Copyright (c) 2020
 */

#ifndef GLOBAL_PLANNER_TYPES_H
#define GLOBAL_PLANNER_TYPES_H

/**
 * @brief A structure for Coordinate
 */
struct Coordinate
{
    unsigned int x;  /*< X co-ordinate */
    unsigned int y;  /*< Y co-ordinate */
    /**
     * @brief Construct a new Coordinate object
     */
    Coordinate()
    {
        x = 0; y = 0;
    }
};

/**
 * @brief A structure for graph node
 */
struct Node
{
    unsigned int x;  /*< X co-ordinate */
    unsigned int y;  /*< Y co-ordinate */
    bool is_obstacle;  /*< Obstacle flag */
    bool is_visited;  /*< Visited flag */
    Node* parent;  /*< Parent */
    /**
     * @brief Construct a new Node object
     */
    Node()
    {
        x = 0; y = 0;
        is_obstacle = false; is_visited = false;
        parent = nullptr;
    }
};

#endif  // GLOBAL_PLANNER_TYPES