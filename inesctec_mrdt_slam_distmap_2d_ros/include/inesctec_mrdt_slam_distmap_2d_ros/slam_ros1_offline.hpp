#pragma once

#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>

#include <fstream>
#include <iostream>
#include <memory>
#include <vector>

// ROS
#include <message_filters/sync_policies/approximate_time.h>
#include <rosbag/bag.h>
#include <tf2_ros/message_filter.h>

// ROS API
#include "inesctec_mrdt_slam_distmap_2d_ros/slam_ros1_api.hpp"

namespace inesctec_mrdt_slam_distmap_2d_ros
{

class SLAMROS1Offline : public SLAMROS1API
{
 public:

  using ThisType = SLAMROS1Offline;
  using BaseType = SLAMROS1API;

  using MsgFilterScanOdomSyncPolicyType =
      typename message_filters::sync_policies::ApproximateTime<
          nav_msgs::Odometry, sensor_msgs::LaserScan>;
  using MsgFilterScanOdomSyncType =
      message_filters::Synchronizer<MsgFilterScanOdomSyncPolicyType>;

 public:

  struct ParamOffline
  {
    std::vector<std::string> m_bags;  //!< set of ROS bag files to process
    std::string m_odom_topic;         //!< odom topic name for odometry data
    std::string m_scan_topic;         //!< scan topic name for 2D laser data
    bool m_has_duration;     //!< duration from the start time set in options
    double m_time_start;     //!< start time (s) into the bag files
    double m_time_duration;  //!< duration (s) to only process from the bags
    bool m_enable_log;       //!< enable log of robot data (odom, pose) into TUM
    std::string m_log_filename;  //!< log filename
  };  // struct TrackerROS1Offline::ParamOffline

 protected:

  ParamOffline m_param_offline_;

  bool m_paused_ = false;

  bool m_terminal_modified_ = false;

  termios m_orig_flags_;

  std::unique_ptr<MsgFilterScanOdomSyncType> m_sub_scan_odom_sync_ = nullptr;
  std::unique_ptr<tf2_ros::MessageFilter<sensor_msgs::LaserScan>>
      m_sub_scan_tf_ = nullptr;

  std::vector<std::shared_ptr<rosbag::Bag>> m_bags_;

  std::ofstream m_log_file_odom_;
  std::ofstream m_log_file_pose_;

  int64_t m_sleep_ms = 0;

 public:

  SLAMROS1Offline(const ParamOffline& param);
  virtual ~SLAMROS1Offline();

  void run();

 protected:

  virtual void pubOdom(const std_msgs::Header& header) final;
  virtual void pubPose(const std_msgs::Header& header) final;
  virtual void pubExtOdom(const geometry_msgs::TransformStamped&) final {}
  virtual void pubExtOdomTF(const nav_msgs::Odometry::ConstPtr&) final {}

  virtual void setupTerminal();
  virtual void restoreTerminal();
  virtual void printTime(const ros::Time& t, const ros::Duration& duration,
                         const ros::Duration& bag_length) const;

  virtual char readTerminalKey() const;

 private:

  SLAMROS1Offline() = delete;

};  // class SLAMROS1Offline : public SLAMROS1API

}  // namespace inesctec_mrdt_slam_distmap_2d_ros
