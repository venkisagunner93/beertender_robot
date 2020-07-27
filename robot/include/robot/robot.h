/**
 * @file robot.h
 * @author Venkatavaradhan Vembanoor Lakshmi Narayanan (venkatavaradhan93@gmail.com)
 * @brief A class for simple robot model
 * @version 0.1
 * @date 2020-07-21
 * @copyright Copyright (c) 2020
 */

#ifndef ROBOT_H
#define ROBOT_H

#include "robot/robot_types.h"

/**
 * @brief A base class for Robot
 */
class Robot
{
    public:
        /**
         * @brief Destroy the Robot object
         */
        virtual ~Robot() {}
        /**
         * @brief A method to update state
         * @param current_state - Current state
         * @param control_input - Control input
         * @return State 
         */
        virtual State updateState(const State& current_state, const ControlInput& control_input) = 0;
        /**
         * @brief A method to display robot details
         */
        virtual void displayRobotDetails() const {}
        /**
         * @brief A method to get current pose from TF tree
         * @return geometry_msgs::TransformStamped 
         */
        virtual geometry_msgs::TransformStamped getCurrentPose() const = 0;
        /**
         * @brief A method to broadcast pose
         */
        virtual void broadcastPose() = 0;

};

#endif  // ROBOT_H