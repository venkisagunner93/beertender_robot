#include "nav_utils/tf_helper.h"

namespace tf_helper
{
bool getCurrentLocationFromTF(std::string parent_frame, std::string child_frame, geometry_msgs::PoseStamped* pose)
{
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);

  geometry_msgs::TransformStamped tf_pose;

  try
  {
    tf_pose = tf_buffer.lookupTransform(child_frame, parent_frame, ros::Time(0));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN_STREAM(ex.what());
    return false;
  }

  pose->header = tf_pose.header;
  pose->pose.position.x = tf_pose.transform.translation.x;
  pose->pose.position.y = tf_pose.transform.translation.y;
  pose->pose.position.z = tf_pose.transform.translation.z;

  pose->pose.orientation.x = tf_pose.transform.rotation.x;
  pose->pose.orientation.y = tf_pose.transform.rotation.y;
  pose->pose.orientation.z = tf_pose.transform.rotation.z;
  pose->pose.orientation.w = tf_pose.transform.rotation.w;

  return true;
}
}  // namespace tf_helper