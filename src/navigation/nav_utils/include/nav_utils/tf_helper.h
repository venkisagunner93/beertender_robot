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
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>

namespace tf_helper
{
/**
 * @brief A helper class for accessing transform tree
 */
class TFHelper
{
public:
	/**
	 * @brief Construct a new TFHelper object
	 */
	TFHelper();
	/**
	 * @brief Destroy the TFHelper object
	 */
	~TFHelper(){}
  /**
   * @brief Get the Current Location From TF tree
   * @param parent_frame Parent frame in TF tree
   * @param child_frame Child frame in TF tree
   * @param pose Current pose
   * @return true
   * @return false
   */
  bool getCurrentPoseFromTF(std::string parent_frame, std::string child_frame,
                            geometry_msgs::PoseStamped* pose);
  /**
   * @brief Broadcast current pose to TF tree
   * @param x State x
   * @param y State y
   * @param theta State theta
   * @param parent_frame Parent frame in TF tree
   * @param child_frame Child frame in TF tree
   */
  void broadcastCurrentPoseToTF(float x, float y, float theta, std::string parent_frame,
                                std::string child_frame);

private:
	/**
	 * @brief TF buffer
	 */
  tf2_ros::Buffer tf_buffer_;
	/**
	 * @brief TF listener instance
	 */
  tf2_ros::TransformListener tf_listener_;
};
}  // namespace tf_helper

#endif  // TF_HELPER_H