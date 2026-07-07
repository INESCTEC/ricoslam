#pragma once

#include <iostream>
#include <memory>

// ROS
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

// ROS API
#include "inesctec_mrdt_slam_distmap_2d_ros/tracker_ros1_api.hpp"

namespace inesctec_mrdt_slam_distmap_2d_ros
{

class TrackerROS1 : public TrackerROS1API
{
 public:

  using ThisType = TrackerROS1;
  using BaseType = TrackerROS1API;

  using MsgFilterScanOdomSyncPolicyType =
      typename message_filters::sync_policies::ApproximateTime<
          nav_msgs::Odometry, sensor_msgs::LaserScan>;
  using MsgFilterScanOdomSyncType =
      message_filters::Synchronizer<MsgFilterScanOdomSyncPolicyType>;

 protected:

  std::unique_ptr<ros::Publisher> m_pub_odom_ = nullptr;
  std::unique_ptr<ros::Publisher> m_pub_ext_odom_ = nullptr;

  std::unique_ptr<ros::Subscriber> m_sub_scan_only_ = nullptr;
  std::unique_ptr<message_filters::Subscriber<nav_msgs::Odometry>> m_sub_odom_ =
      nullptr;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::LaserScan>>
      m_sub_scan_ = nullptr;
  std::unique_ptr<MsgFilterScanOdomSyncType> m_sub_scan_odom_sync_ = nullptr;
  std::unique_ptr<tf2_ros::MessageFilter<sensor_msgs::LaserScan>>
      m_sub_scan_tf_ = nullptr;

  tf2_ros::TransformBroadcaster m_tf2_pub_;
  tf2_ros::TransformListener m_tf2_sub_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> m_tf2_pub_static_ =
      nullptr;

 public:

  TrackerROS1();

 protected:

  virtual void pubOdom(const std_msgs::Header& header) final;
  virtual void pubExtOdom(
      const geometry_msgs::TransformStamped& ext_odom_tf) final;
  virtual void pubExtOdomTF(const nav_msgs::Odometry::ConstPtr& ext_odom) final;

};  // class TrackerROS1

}  // namespace inesctec_mrdt_slam_distmap_2d_ros
