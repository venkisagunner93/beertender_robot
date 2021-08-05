#include "nav_utils/tf_helper.h"

namespace tf_helper
{
TFHelper::TFHelper() : tf_listener_(tf_buffer_) {}

geometry_msgs::TransformStamped TFHelper::getTFPose(std::string parent_frame, std::string child_frame)
{
  geometry_msgs::TransformStamped tf_pose;

  try
  {
    // Max wait time for transform will be 5s
    tf_pose =
        tf_buffer_.lookupTransform(parent_frame, child_frame, ros::Time(0), ros::Duration(5));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN_STREAM(ex.what());
  }

  return tf_pose;
}

bool TFHelper::getCurrentPoseFromTF(std::string parent_frame, std::string child_frame,
                                    geometry_msgs::PoseStamped* pose)
{
  geometry_msgs::TransformStamped tf_pose = getTFPose(parent_frame, child_frame);
  
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

bool TFHelper::getCurrentPoseFromTF(std::string parent_frame, std::string child_frame,
                                    geometry_msgs::Pose* pose)
{
  geometry_msgs::TransformStamped tf_pose = getTFPose(parent_frame, child_frame);

  pose->position.x = tf_pose.transform.translation.x;
  pose->position.y = tf_pose.transform.translation.y;
  pose->position.z = tf_pose.transform.translation.z;

  pose->orientation.x = tf_pose.transform.rotation.x;
  pose->orientation.y = tf_pose.transform.rotation.y;
  pose->orientation.z = tf_pose.transform.rotation.z;
  pose->orientation.w = tf_pose.transform.rotation.w;

  return true;
}

void TFHelper::broadcastCurrentPoseToTF(float x, float y, float theta, std::string parent_frame,
                                        std::string child_frame)
{
  static tf2_ros::TransformBroadcaster broadcaster;
  geometry_msgs::TransformStamped pose;

  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = parent_frame;
  pose.child_frame_id = child_frame;
  pose.transform.translation.x = x;
  pose.transform.translation.y = y;
  pose.transform.translation.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, theta);
  pose.transform.rotation.x = q.x();
  pose.transform.rotation.y = q.y();
  pose.transform.rotation.z = q.z();
  pose.transform.rotation.w = q.w();

  broadcaster.sendTransform(pose);
}
}  // namespace tf_helper