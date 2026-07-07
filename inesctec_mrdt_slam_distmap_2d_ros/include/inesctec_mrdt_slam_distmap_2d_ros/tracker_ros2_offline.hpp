#pragma once

#include <fstream>
#include <iostream>
#include <memory>
#include <utility>
#include <vector>

// ROS
#if defined(ROS_DISTRO_foxy)
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_ros/message_filter.h>
#else
#include <message_filters/sync_policies/approximate_time.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <tf2_ros/message_filter.hpp>
#endif

#include <rosbag2_cpp/reader.hpp>

// ROS API
#include "inesctec_mrdt_slam_distmap_2d_ros/tracker_ros2_api.hpp"

namespace inesctec_mrdt_slam_distmap_2d_ros
{

class TrackerROS2Offline : public TrackerROS2API
{
 public:

  using ThisType = TrackerROS2Offline;
  using BaseType = TrackerROS2API;

  using MsgFilterScanOdomSyncPolicyType =
      typename message_filters::sync_policies::ApproximateTime<
          nav_msgs::msg::Odometry, sensor_msgs::msg::LaserScan>;
  using MsgFilterScanOdomSyncType =
      message_filters::Synchronizer<MsgFilterScanOdomSyncPolicyType>;

 public:

  struct ParamOffline
  {
    std::vector<std::string> m_bags;  //!< set of ROS bag files to process
                                      //!< (define them by chronological order!)
    std::string m_odom_topic;         //!< odom topic name for odometry data
    std::string m_scan_topic;         //!< scan topic name for 2D laser data
    bool m_has_duration;     //!< duration from the start time set in options
    double m_time_start;     //!< start time (s) into the bag files
    double m_time_duration;  //!< duration (s) to only process from the bags
    bool m_enable_log;       //!< enable log of robot data (odom, pose) into TUM
    std::string m_log_filename;  //!< log filename
  };  // struct TrackerROS2Offline::ParamOffline

 protected:

  ParamOffline m_param_offline_;

  std::shared_ptr<MsgFilterScanOdomSyncType> m_sub_scan_odom_sync_ = nullptr;
  std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>
      m_sub_scan_tf_ = nullptr;

#if defined(ROS_DISTRO_foxy)
  std::vector<std::pair<std::unique_ptr<rosbag2_cpp::Reader>,
                        rosbag2_cpp::StorageOptions>>
      m_bags_;
#else
  std::vector<std::pair<std::unique_ptr<rosbag2_cpp::Reader>,
                        rosbag2_storage::StorageOptions>>
      m_bags_;
#endif

  std::ofstream m_log_file_;

 public:

  TrackerROS2Offline(
      const ParamOffline& param, const std::string& name = "slam_offline",
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  virtual ~TrackerROS2Offline();

  void run();

 protected:

  virtual void pubOdom(const std_msgs::msg::Header& header) final;
  virtual void pubExtOdom(
      const geometry_msgs::msg::TransformStamped& ext_odom_tf) final
  {
  }
  virtual void pubExtOdomTF(const nav_msgs::msg::Odometry& ext_odom) final {}

 private:

  TrackerROS2Offline() = delete;

};  // class TrackerROS2Offline : public TrackerROS2API

}  // namespace inesctec_mrdt_slam_distmap_2d_ros
