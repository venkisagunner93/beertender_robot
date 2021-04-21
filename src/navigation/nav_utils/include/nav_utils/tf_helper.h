/**
 * @file tf_helper.h
 * @author Venkatavaradhan Vembanoor Lakshmi Narayanan (venkatavaradhan93@gmail.com)
 * @brief A helper translational unit for transform tree
 * @version 0.1
 * @date 2021-04-19
 * @copyright Copyright (c) 2021
 */

#ifndef TF_HELPER_H
#define TF_HELPER_H

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>

namespace tf_helper
{
/**
 * @brief Get the Current Location From TF object
 * @param parent_frame Parent frame in TF tree
 * @param child_frame Child frame in TF tree
 * @param pose Current pose
 * @return true 
 * @return false 
 */
bool getCurrentLocationFromTF(std::string parent_frame, std::string child_frame, geometry_msgs::PoseStamped* pose);
}  // namespace tf_helper

#endif  // TF_HELPER_H