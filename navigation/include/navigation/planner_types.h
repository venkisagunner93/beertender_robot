/**
 * @file planner_types.h
 * @author Venkatavaradhan Vembanoor Lakshmi Narayanan (venkatavaradhan93@gmail.com)
 * @brief A file for planner types
 * @version 0.1
 * @date 2020-07-26
 * @copyright Copyright (c) 2020
 */

#ifndef PLANNER_TYPES_H
#define PLANNER_TYPES_H

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

/**
 * @brief A structure for DWA configuration
 */
struct DWAConfig
{
    float sim_time;  /*< Simulation time */
    float dt;  /*< Sample time */
    int obstacle_gain;  /*< Obstacle gain */
    int max_velocity_gain;  /*<  Max velocity gain */
    int distance_to_goal_gain;  /*< Distance to goal gain */
    int window_size;  /*< Window size */
    /**
     * @brief Construct a new DWAConfig object
     */
    DWAConfig()
    {
        sim_time = 0.0; dt = 0.0;
        obstacle_gain = 0; max_velocity_gain = 0; distance_to_goal_gain = 0;
        window_size = 0;
    }
};

/**
 * @brief A structure for dynamic window
 */
struct DynamicWindow
{
    std::vector<float> linear_velocity;  /*< Linear velocity (m/s) */
    std::vector<float> angular_velocity;  /*< Angular velocity (rad/s) */
    /**
     * @brief Construct a new Dynamic Window object
     */
    DynamicWindow()
    {
        linear_velocity.clear(); angular_velocity.clear();
    }
};

#endif  // PLANNER_TYPES_H