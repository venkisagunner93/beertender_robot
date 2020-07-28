/**
 * @file dynamic_window_approach.h
 * @author Venkatavaradhan Vembanoor Lakshmi Narayanan (venkatavaradhan93@gmail.com)
 * @brief A class for dynamic window approach local planner
 * @version 0.1
 * @date 2020-07-27
 * @copyright Copyright (c) 2020
 */

#ifndef DYNAMIC_WINDOW_APPROACH_H
#define DYNAMIC_WINDOW_APPROACH_H

#include <algorithm>
#include <nav_msgs/Path.h>
#include "navigation/local_planner/local_planner.h"
#include "robot/car_robot.h"

/**
 * @brief A class for dynamic window approach local planner
 */
class DWA : public LocalPlanner
{
    public:
        /**
         * @brief Construct a new DWA object
         * @param nh - ROS NodeHandle for communication
         * @param robot - Robot instance
         * @param dwa_config - DWA configuration parameters
         */
        DWA(ros::NodeHandle& nh, Robot* robot, const DWAConfig& dwa_config);
        /**
         * @brief Destroy the Dwa Local Planner object
         */
        ~DWA();
        /**
         * @brief Set the Global Path object
         * @param msg - Global path
         */
        void setGlobalPath(const nav_msgs::Path& msg);

    private:
        /**
         * @brief Robot instance
         */
        Robot* robot_;
        /**
         * @brief Global path
         */
        nav_msgs::Path global_path_;
        /**
         * @brief DWA configuration parameters
         */
        DWAConfig config_;
        /**
         * @brief Dynamic window
         */
        DynamicWindow dynamic_window_;
        /**
         * @brief A method to compute dynamic window
         */
        void computeDynamicWindow();
        
};

#endif  // DYNAMIC_WINDOW_APPROACH_H