#include "inesctec_mrdt_slam_distmap_2d_ros/tracker_ros2_offline.hpp"

#include <Eigen/Geometry>
#include <chrono>
#include <exception>
#include <filesystem>
#include <fstream>
#include <future>
#include <limits>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#if defined(ROS_DISTRO_foxy)
#include <tf2/exceptions.h>
#include <tf2/utils.h>

#include <rosbag2_cpp/readers/sequential_reader.hpp>
#else
#include <rosbag2_transport/reader_writer_factory.hpp>
#include <tf2/exceptions.hpp>
#include <tf2/utils.hpp>
#endif

#include <inesctec_mrdt_slam_distmap_2d/system_utils/filesystem.hpp>
#include <inesctec_mrdt_slam_distmap_2d/system_utils/time.hpp>

namespace inesctec_mrdt_slam_distmap_2d_ros
{

using namespace inesctec_mrdt_slam_distmap_2d;

TrackerROS2Offline::TrackerROS2Offline(const ParamOffline& param,
                                       const std::string& name,
                                       const rclcpp::NodeOptions& options)
    : BaseType::TrackerROS2API(name, options), m_param_offline_(param)
{
  rcl_ret_t ret =
      rcl_enable_ros_time_override(this->get_clock()->get_clock_handle());

  if (ret != RCL_RET_OK)
  {
    RCLCPP_ERROR(
        this->get_logger(),
        "[%s] Failed to enable ROS time override (error returned code: %d)",
        this->get_name(), ret);
  }

  // Quality of Service (to make it the same for all publishers and subscribers)
  rclcpp::SensorDataQoS qos;

  m_tf2_buffer_->setUsingDedicatedThread(true);

  // Print offline parametrization
  std::stringstream str;

  for (const std::string& bag_filename : m_param_offline_.m_bags)
  {
    str << "- " << bag_filename << std::endl;
  }

  RCLCPP_INFO(this->get_logger(), "[%s] bag files :\n%s", this->get_name(),
              str.str().c_str());
  RCLCPP_INFO(this->get_logger(), "[%s] odom topic: %s", this->get_name(),
              m_param_offline_.m_odom_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "[%s] scan topic: %s", this->get_name(),
              m_param_offline_.m_scan_topic.c_str());

  // Log file processing
  if (m_param_offline_.m_enable_log)
  {
    if (m_param_offline_.m_log_filename.empty())
    {
      throw std::runtime_error(
          "TrackerROS2Offline::TrackerROS2Offline | empty filename when log "
          "enabled");
    }

    std::string log_file_pose;

    try
    {
      std::filesystem::path log_file_path(m_param_offline_.m_log_filename);

      std::filesystem::path dir = log_file_path.parent_path();
      std::string stem = log_file_path.stem().string();
      std::string ext = log_file_path.extension().string();

      log_file_pose = (dir / (stem + "_ricoslam_tracker_odom" + ext)).string();

      RCLCPP_INFO(this->get_logger(), "[%s] log file  : %s", this->get_name(),
                  log_file_pose.c_str());
    }
    catch (const std::filesystem::filesystem_error& e)
    {
      throw std::runtime_error(
          "TrackerROS2Offline::TrackerROS2Offline | Error resolving paths for "
          "log files");
    }
    catch (const std::exception& e)
    {
      throw std::runtime_error(
          "TrackerROS2Offline::TrackerROS2Offline | Error when processing "
          "paths for log files");
    }
    catch (...)
    {
      throw std::runtime_error(
          "TrackerROS2Offline::TrackerROS2Offline | Unknown error when "
          "processing paths for log files");
    }

    system_utils::validateAndCreatePath(log_file_pose);

    try
    {
      m_log_file_ = std::ofstream(log_file_pose);

      if (!m_log_file_.is_open())
      {
        throw std::runtime_error(
            "TrackerROS2Offline::TrackerROS2Offline | file (" + log_file_pose +
            ") not opened");
      }
    }
    catch (const std::exception& e)
    {
      throw std::runtime_error(
          "TrackerROS2Offline::TrackerROS2Offline | error when opening the log "
          "file (" +
          log_file_pose + "): " + e.what());
    }
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "[%s] log file  : not enabled",
                this->get_name());
  }

  std::cout << std::endl;

  // ROS API
  if (m_param_.m_ext_odom_use)
  {
    if (m_param_.m_ext_odom_impl.compare("tf") == 0)
    {
      m_sub_scan_tf_ =
          std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(
              *m_tf2_buffer_, m_param_.m_ext_odom_frame,
              static_cast<uint32_t>(qos.get_rmw_qos_profile().depth),
              this->get_node_logging_interface(),
              this->get_node_clock_interface());

      m_sub_scan_tf_->registerCallback(std::bind(&BaseType::subScan,
                                                 static_cast<BaseType*>(this),
                                                 std::placeholders::_1));
    }
    else
    {
      m_sub_scan_odom_sync_ = std::make_unique<MsgFilterScanOdomSyncType>(
          MsgFilterScanOdomSyncPolicyType(10));

      m_sub_scan_odom_sync_->registerCallback(&BaseType::subOdomScan,
                                              static_cast<BaseType*>(this));
    }
  }
}

TrackerROS2Offline::~TrackerROS2Offline()
{
  if (m_param_offline_.m_enable_log && m_log_file_.is_open())
  {
    m_log_file_.close();
  }

  for (auto& [reader, opts] : m_bags_)
  {
    RCLCPP_INFO(this->get_logger(), "[%s] Closing %s", this->get_name(),
                opts.uri.c_str());

#if defined(ROS_DISTRO_foxy)
    reader.reset();
#else
    if (reader)
    {
      reader->close();
    }
#endif
  }
}

void TrackerROS2Offline::run()
{
  std::cout << std::endl;

  auto start = std::chrono::high_resolution_clock::now();

  for (const std::string& filename : m_param_offline_.m_bags)
  {
    RCLCPP_INFO(this->get_logger(), "[%s] Opening %s", this->get_name(),
                filename.c_str());

#if defined(ROS_DISTRO_foxy)
    rosbag2_cpp::StorageOptions opts;
#else
    rosbag2_storage::StorageOptions opts;
#endif

    opts.uri = filename;
    opts.storage_id = "mcap";

#if defined(ROS_DISTRO_foxy)
    std::unique_ptr<rosbag2_cpp::Reader> bag =
        std::make_unique<rosbag2_cpp::Reader>(
            std::make_unique<rosbag2_cpp::readers::SequentialReader>());
#else
    std::unique_ptr<rosbag2_cpp::Reader> bag =
        rosbag2_transport::ReaderWriterFactory::make_reader(opts);
#endif

    m_bags_.push_back(std::make_pair(std::move(bag), opts));
  }

  rcutils_time_point_value_t full_initial_time_ns =
      std::numeric_limits<rcutils_time_point_value_t>::max();
  rcutils_time_point_value_t full_finish_time_ns =
      std::numeric_limits<rcutils_time_point_value_t>::min();

  for (auto& [reader, opts] : m_bags_)
  {
    reader->open(
        opts, {rmw_get_serialization_format(), rmw_get_serialization_format()});

    const auto metadata = reader->get_metadata();
    const rcutils_time_point_value_t metadata_starting_time =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            metadata.starting_time.time_since_epoch())
            .count();
    const rcutils_time_point_value_t metadata_bag_duration =
        std::chrono::duration_cast<std::chrono::nanoseconds>(metadata.duration)
            .count();

    if (metadata_starting_time < full_initial_time_ns)
    {
      full_initial_time_ns = metadata_starting_time;
    }
    if (metadata_starting_time + metadata_bag_duration > full_finish_time_ns)
    {
      full_finish_time_ns = metadata_starting_time + metadata_bag_duration;
    }

#if defined(ROS_DISTRO_foxy)
    reader = std::make_unique<rosbag2_cpp::Reader>(
        std::make_unique<rosbag2_cpp::readers::SequentialReader>());
#else
    reader->close();
#endif
  }

  double full_initial_time = full_initial_time_ns * 1e-9;
  double full_finish_time = full_finish_time_ns * 1e-9;

  double initial_time = full_initial_time + m_param_offline_.m_time_start;
  double finish_time = full_finish_time;

  rcutils_time_point_value_t initial_time_ns =
      full_initial_time_ns + m_param_offline_.m_time_start * 1e9;
  rcutils_time_point_value_t finish_time_ns = full_finish_time_ns;

  double bag_length;

  RCLCPP_INFO(this->get_logger(), "[%s] Start  time (s): %.9lf",
              this->get_name(), initial_time);

  if (m_param_offline_.m_has_duration)
  {
    finish_time_ns = initial_time_ns + m_param_offline_.m_time_duration * 1e9;
    finish_time = initial_time + m_param_offline_.m_time_duration;
    bag_length = finish_time - initial_time;

    RCLCPP_INFO(this->get_logger(), "[%s] Finish time (s): %.9lf",
                this->get_name(), finish_time);
    RCLCPP_INFO(this->get_logger(), "[%s] Total  time (s): %.9lf\n",
                this->get_name(), bag_length);
  }
  else
  {
    bag_length = finish_time - initial_time;

    RCLCPP_INFO(this->get_logger(),
                "[%s] Finish time (s): %.9lf (end of the bags)",
                this->get_name(), finish_time);
    RCLCPP_INFO(this->get_logger(), "[%s] Total  time (s): %.9lf\n",
                this->get_name(), bag_length);
  }

  RCLCPP_INFO(this->get_logger(), "[%s] Loading TF static transforms...",
              this->get_name());

  rosbag2_storage::StorageFilter tf_static_filter;

  rclcpp::Serialization<tf2_msgs::msg::TFMessage> tf_serializer;

  tf_static_filter.topics.clear();
  tf_static_filter.topics.emplace_back("/tf_static");

  for (auto& [reader, opts] : m_bags_)
  {
    reader->open(
        opts, {rmw_get_serialization_format(), rmw_get_serialization_format()});
    reader->set_filter(tf_static_filter);

    while (reader->has_next())
    {
#if defined(ROS_DISTRO_foxy)
      std::shared_ptr<rosbag2_storage::SerializedBagMessage> msg =
          reader->read_next();
#else
      rosbag2_storage::SerializedBagMessageSharedPtr msg = reader->read_next();
#endif

      rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);

      tf2_msgs::msg::TFMessage tf_msg;

      tf_serializer.deserialize_message(&serialized_msg, &tf_msg);

      for (const geometry_msgs::msg::TransformStamped& transf :
           tf_msg.transforms)
      {
        m_tf2_buffer_->setTransform(transf, this->get_name(), true);

        RCLCPP_INFO(this->get_logger(), "[%s] TF static transform: %s --> %s",
                    this->get_name(), transf.header.frame_id.c_str(),
                    transf.child_frame_id.c_str());
      }
    }

#if defined(ROS_DISTRO_foxy)
    reader = std::make_unique<rosbag2_cpp::Reader>(
        std::make_unique<rosbag2_cpp::readers::SequentialReader>());
#else
    reader->close();
#endif
  }

  rclcpp::Serialization<sensor_msgs::msg::LaserScan> scan_serializer;
  rclcpp::Serialization<nav_msgs::msg::Odometry> odom_serializer;

  bool is_tf_mode = (m_param_.m_ext_odom_impl.compare("tf") == 0);

  for (auto& [reader, opts] : m_bags_)
  {
    reader->open(
        opts, {rmw_get_serialization_format(), rmw_get_serialization_format()});
    reader->reset_filter();

    const auto metadata = reader->get_metadata();
    const rcutils_time_point_value_t metadata_starting_time =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            metadata.starting_time.time_since_epoch())
            .count();
    const rcutils_time_point_value_t metadata_bag_duration =
        std::chrono::duration_cast<std::chrono::nanoseconds>(metadata.duration)
            .count();

    RCLCPP_INFO(this->get_logger(),
                "[%s] Opening %s to process its contents... (time: %lf s - %lf "
                "s ; duration: %lf)",
                this->get_name(), opts.uri.c_str(),
                metadata_starting_time * 1e-9,
                (metadata_starting_time + metadata_bag_duration) * 1e-9,
                metadata_bag_duration * 1e-9);

    while (reader->has_next())
    {
#if defined(ROS_DISTRO_foxy)
      std::shared_ptr<rosbag2_storage::SerializedBagMessage> msg =
          reader->read_next();
#else
      rosbag2_storage::SerializedBagMessageSharedPtr msg = reader->read_next();
#endif

#if defined(ROS_DISTRO_foxy) || defined(ROS_DISTRO_humble)
      if (msg->time_stamp < initial_time_ns || msg->time_stamp > finish_time_ns)
      {
        continue;
      }

      rcl_ret_t ret = rcl_set_ros_time_override(
          this->get_clock()->get_clock_handle(), msg->time_stamp);
#else
      if (msg->send_timestamp < initial_time_ns ||
          msg->send_timestamp > finish_time_ns)
      {
        continue;
      }

      rcl_ret_t ret = rcl_set_ros_time_override(
          this->get_clock()->get_clock_handle(), msg->send_timestamp);
#endif

      if (ret != RCL_RET_OK)
      {
        RCLCPP_ERROR(
            this->get_logger(),
            "[%s] Failed to override ROS time (error returned code: %d)",
            this->get_name(), ret);
      }

      /* RCLCPP_INFO(
          this->get_logger(),
          "[%s] rclcpp::now: %.9lf vs msg timestamp: %.9lf (topic name: %s)",
          this->get_name(), this->now().seconds(), msg->send_timestamp * 1e-9,
          msg->topic_name.c_str()); */

      if (msg->topic_name.compare(m_param_offline_.m_scan_topic) == 0)
      {
        rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);

        sensor_msgs::msg::LaserScan::SharedPtr laser_msg =
            std::make_shared<sensor_msgs::msg::LaserScan>();

        scan_serializer.deserialize_message(&serialized_msg, laser_msg.get());

        if (!m_param_.m_ext_odom_use)
        {
          subScan(laser_msg);
        }
        else
        {
          if (is_tf_mode)
          {
            m_sub_scan_tf_->add(laser_msg);
          }
          else
          {
            m_sub_scan_odom_sync_->add<1>(laser_msg);
          }
        }
      }
      else if (msg->topic_name.compare(m_param_offline_.m_odom_topic) == 0 &&
               m_param_.m_ext_odom_use && !is_tf_mode)
      {
        rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);

        nav_msgs::msg::Odometry::SharedPtr odom_msg =
            std::make_shared<nav_msgs::msg::Odometry>();

        odom_serializer.deserialize_message(&serialized_msg, odom_msg.get());

        m_sub_scan_odom_sync_->add<0>(odom_msg);
      }
      else if (msg->topic_name.compare("/tf") == 0 ||
               msg->topic_name.compare("/tf_static") == 0)
      {
        rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);

        tf2_msgs::msg::TFMessage::SharedPtr tf_msg =
            std::make_shared<tf2_msgs::msg::TFMessage>();

        tf_serializer.deserialize_message(&serialized_msg, tf_msg.get());

        bool is_static = (msg->topic_name.compare("/tf_static") == 0);

        for (const geometry_msgs::msg::TransformStamped& transf :
             tf_msg->transforms)
        {
          m_tf2_buffer_->setTransform(transf, this->get_name(), is_static);
        }
      }

      std::promise<void> promise;
      std::shared_future<void> future(promise.get_future());
      rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                         future, std::chrono::milliseconds(1));
    }

#if defined(ROS_DISTRO_foxy)
    reader.reset();
#else
    reader->close();
#endif
  }

  auto end = std::chrono::high_resolution_clock::now();

  std::cout << std::endl << std::flush;

  RCLCPP_INFO(this->get_logger(),
              "\n\n"
              "[%s] Finished processing the ROS bags.\n"
              "Elapsed time (s): %.3lf\n"
              "ros::spin to allow rosrun map_server map_saver OR rviz "
              "visualization.",
              this->get_name(),
              std::chrono::duration_cast<std::chrono::microseconds>(end - start)
                      .count() *
                  1e-6);

  if (m_param_offline_.m_enable_log && m_log_file_.is_open())
  {
    m_log_file_.close();
  }

  for (auto& [reader, opts] : m_bags_)
  {
    RCLCPP_INFO(this->get_logger(), "[%s] Closing %s", this->get_name(),
                opts.uri.c_str());

#if defined(ROS_DISTRO_foxy)
    reader.reset();
#else
    reader->close();
#endif
  }

  rclcpp::spin(this->get_node_base_interface());
}

void TrackerROS2Offline::pubOdom(const std_msgs::msg::Header& header)
{
  if (!m_param_offline_.m_enable_log)
  {
    return;
  }

  EstimateType odom_t = m_tracker_->getRobotInOdom();
  EstimateVectorType odom_v = geometry::t2v(odom_t);

  Eigen::Quaternionf odom_q(geometry::a2q(odom_v.z()));

  try
  {
    m_log_file_ << std::fixed << std::setprecision(9)
                << rclcpp::Time(header.stamp).seconds() << " " << odom_v.x()
                << " " << odom_v.y() << " " << 0 << " " << odom_q.x() << " "
                << odom_q.y() << " " << odom_q.z() << " " << odom_q.w()
                << std::endl;
  }
  catch (const std::exception& e)
  {
    throw std::runtime_error(
        "TrackerROS2Offline::pubOdom | error when logging the robot data (" +
        std::string(e.what()) + ")");
  }
}

}  // namespace inesctec_mrdt_slam_distmap_2d_ros
