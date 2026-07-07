#pragma once

#include <iostream>
#include <memory>

// ROS
#if defined(ROS_DISTRO_foxy)
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#else
#include <message_filters/subscriber.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>
#include <tf2_ros/message_filter.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/transform_listener.hpp>
#endif

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

// ROS API
#include "inesctec_mrdt_slam_distmap_2d_ros/slam_ros2_api.hpp"

namespace inesctec_mrdt_slam_distmap_2d_ros
{

class SLAMROS2Online : public SLAMROS2API
{
 public:

  using ThisType = SLAMROS2Online;
  using BaseType = SLAMROS2API;

  using MsgFilterScanOdomSyncPolicyType =
      typename message_filters::sync_policies::ApproximateTime<
          nav_msgs::msg::Odometry, sensor_msgs::msg::LaserScan>;
  using MsgFilterScanOdomSyncType =
      message_filters::Synchronizer<MsgFilterScanOdomSyncPolicyType>;

 protected:

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_pub_odom_ = nullptr;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_pub_ext_odom_ =
      nullptr;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      m_pub_pose_ = nullptr;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      m_sub_scan_only_ = nullptr;
  std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>>
      m_sub_odom_ = nullptr;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::LaserScan>>
      m_sub_scan_ = nullptr;
  std::shared_ptr<MsgFilterScanOdomSyncType> m_sub_scan_odom_sync_ = nullptr;
  std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>
      m_sub_scan_tf_ = nullptr;

  std::unique_ptr<tf2_ros::TransformBroadcaster> m_tf2_pub_ = nullptr;
  std::unique_ptr<tf2_ros::TransformListener> m_tf2_sub_ = nullptr;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> m_tf2_pub_static_ =
      nullptr;

 public:

  RCLCPP_COMPONENTS_PUBLIC

  explicit SLAMROS2Online(
      const std::string& name = "slam_online",
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 protected:

  virtual void pubOdom(const std_msgs::msg::Header& header) final;
  virtual void pubPose(const std_msgs::msg::Header& header) final;
  virtual void pubExtOdom(
      const geometry_msgs::msg::TransformStamped& ext_odom_tf) final;
  virtual void pubExtOdomTF(const nav_msgs::msg::Odometry& ext_odom) final;

};  // class SLAMROS2Online : public SLAMROS2API

}  // namespace inesctec_mrdt_slam_distmap_2d_ros
