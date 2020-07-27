/**
 * @file car_robot.h
 * @author Venkatavaradhan Vembanoor Lakshmi Narayanan (venkatavaradhan93@gmail.com)
 * @brief A class for car like robot 
 * @version 0.1
 * @date 2020-07-21
 * @copyright Copyright (c) 2020
 */

#ifndef CAR_ROBOT_H
#define CAR_ROBOT_H

#include <ros/ros.h>
#include <math.h>
#include <urdf/model.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include "robot/robot.h"

/**
 * @brief A class for car like robot
 */
class CarRobot : public Robot
{
    public:
        /**
         * @brief Construct a new Car Robot object
         * @param drive_limits - Drive limits
         */
        CarRobot(const DriveLimits& drive_limits);
        /**
         * @brief Destroy the Car Robot object
         */
        ~CarRobot() {}
        /**
         * @brief A method to update state
         * @param current_state - Current state
         * @param control_input - Control input
         * @return State 
         */
        State updateState(const State& current_state, const ControlInput& control_input);
        /**
         * @brief A method to display robot details
         */
        void displayRobotDetails() const;
        /**
         * @brief A method to get current pose from TF tree
         * @return geometry_msgs::TransformStamped 
         */
        geometry_msgs::TransformStamped getCurrentPose() const;
        /**
         * @brief A method to broadcast pose
         */
        void broadcastPose();

    private:
        /**
         * @brief Robot car dimension
         */
        Dimension dimension_;
        /**
         * @brief Robot car drive limits
         */
        DriveLimits drive_limits_;
        /**
         * @brief Previous time stamp for dt
         */
        ros::Time prev_time_;
        /**
         * @brief Transform broadcaster
         */
        tf2_ros::TransformBroadcaster broadcaster_;
        /**
         * @brief Transform buffer
         */
        tf2_ros::Buffer buffer_;
        /**
         * @brief Transform listener
         */
        tf2_ros::TransformListener listener_;
};

#endif  // CAR_ROBOT_H