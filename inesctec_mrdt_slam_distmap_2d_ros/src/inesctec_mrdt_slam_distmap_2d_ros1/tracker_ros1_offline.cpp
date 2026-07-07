#include "inesctec_mrdt_slam_distmap_2d_ros/tracker_ros1_offline.hpp"

#include <fcntl.h>
#include <sys/ioctl.h>

#include <Eigen/Geometry>
#include <chrono>
#include <exception>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

// ROS
#include <rosbag/query.h>
#include <rosbag/view.h>
#include <tf2/exceptions.h>
#include <tf2/utils.h>
#include <tf2_msgs/TFMessage.h>

#include <inesctec_mrdt_slam_distmap_2d/system_utils/filesystem.hpp>
#include <inesctec_mrdt_slam_distmap_2d/system_utils/time.hpp>

namespace inesctec_mrdt_slam_distmap_2d_ros
{

using namespace inesctec_mrdt_slam_distmap_2d;

TrackerROS1Offline::TrackerROS1Offline(const ParamOffline& param)
    : BaseType::TrackerROS1API(), m_param_offline_(param)
{
  ros::Time::init();

  m_tf2_buffer_.setUsingDedicatedThread(true);

  // Print offline parametrization
  std::stringstream str;

  for (const std::string& bag_filename : m_param_offline_.m_bags)
  {
    str << "- " << bag_filename << std::endl;
  }

  ROS_INFO("[%s] bag files :\n%s", m_name_.c_str(), str.str().c_str());
  ROS_INFO("[%s] odom topic: %s", m_name_.c_str(),
           m_param_offline_.m_odom_topic.c_str());
  ROS_INFO("[%s] scan topic: %s", m_name_.c_str(),
           m_param_offline_.m_scan_topic.c_str());

  // Log file processing
  if (m_param_offline_.m_enable_log)
  {
    if (m_param_offline_.m_log_filename.empty())
    {
      throw std::runtime_error(
          "TrackerROS1Offline::TrackerROS1Offline | empty filename when log "
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

      ROS_INFO("[%s] log file  : %s", m_name_.c_str(), log_file_pose.c_str());
    }
    catch (const std::filesystem::filesystem_error& e)
    {
      throw std::runtime_error(
          "TrackerROS1Offline::TrackerROS1Offline | Error resolving paths for "
          "log files");
    }
    catch (const std::exception& e)
    {
      throw std::runtime_error(
          "TrackerROS1Offline::TrackerROS1Offline | Error when processing "
          "paths for log files");
    }
    catch (...)
    {
      throw std::runtime_error(
          "TrackerROS1Offline::TrackerROS1Offline | Unknown error when "
          "processing paths for log files");
    }

    system_utils::validateAndCreatePath(log_file_pose);

    try
    {
      m_log_file_ = std::ofstream(log_file_pose);

      if (!m_log_file_.is_open())
      {
        throw std::runtime_error(
            "TrackerROS1Offline::TrackerROS1Offline | file (" + log_file_pose +
            ") not opened");
      }
    }
    catch (const std::exception& e)
    {
      throw std::runtime_error(
          "TrackerROS1Offline::TrackerROS1Offline | error when opening the log "
          "file (" +
          log_file_pose + "): " + e.what());
    }
  }
  else
  {
    ROS_INFO("[%s] log file  : not enabled", m_name_.c_str());
  }

  std::cout << std::endl;

  // ROS API
  if (m_param_.m_ext_odom_use)
  {
    if (m_param_.m_ext_odom_impl.compare("tf") == 0)
    {
      m_sub_scan_tf_ =
          std::make_unique<tf2_ros::MessageFilter<sensor_msgs::LaserScan>>(
              m_tf2_buffer_, m_param_.m_ext_odom_frame, 10, m_nh_priv_);

      m_sub_scan_tf_->registerCallback(&BaseType::subScan,
                                       static_cast<BaseType*>(this));
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

TrackerROS1Offline::~TrackerROS1Offline()
{
  if (m_param_offline_.m_enable_log && m_log_file_.is_open())
  {
    m_log_file_.close();
  }

  for (const std::shared_ptr<rosbag::Bag>& bag : m_bags_)
  {
    if (bag->isOpen())
    {
      ROS_INFO("[%s] Closing %s", m_name_.c_str(), bag->getFileName().c_str());

      bag->close();
    }
  }

  restoreTerminal();
}

void TrackerROS1Offline::run()
{
  std::cout << std::endl;

  setupTerminal();

  auto start = std::chrono::high_resolution_clock::now();

  for (const std::string& filename : m_param_offline_.m_bags)
  {
    ROS_INFO("[%s] Opening %s", m_name_.c_str(), filename.c_str());

    try
    {
      std::shared_ptr<rosbag::Bag> bag = std::make_shared<rosbag::Bag>();

      bag->open(filename, rosbag::bagmode::Read);

      m_bags_.push_back(bag);
    }
    catch (rosbag::BagException& e)
    {
      std::stringstream error;

      error << "Error when opening the ROS bag file (filename: " << filename
            << "; error: " << e.what() << ")";

      throw std::runtime_error(error.str());
    }
  }

  std::vector<std::string> tf_static_topics({"/tf_static"});

  rosbag::View full_view;
  rosbag::View tf_static_view;

  for (const std::shared_ptr<rosbag::Bag>& bag : m_bags_)
  {
    full_view.addQuery(*bag);
    tf_static_view.addQuery(*bag, rosbag::TopicQuery(tf_static_topics));
  }

  const ros::Time full_initial_time = full_view.getBeginTime();
  const ros::Time initial_time =
      full_initial_time + ros::Duration(m_param_offline_.m_time_start);
  ros::Time finish_time = ros::TIME_MAX;

  ros::Duration bag_length;

  ROS_INFO("[%s] Start  time (s): %.9lf", m_name_.c_str(),
           initial_time.toSec());

  if (m_param_offline_.m_has_duration)
  {
    finish_time =
        initial_time + ros::Duration(m_param_offline_.m_time_duration);
    bag_length = finish_time - initial_time;

    ROS_INFO("[%s] Finish time (s): %.9lf", m_name_.c_str(),
             finish_time.toSec());
    ROS_INFO("[%s] Total  time (s): %.9lf\n", m_name_.c_str(),
             bag_length.toSec());
  }
  else
  {
    bag_length = full_view.getEndTime() - initial_time;

    ROS_INFO("[%s] Finish time (s): %.9lf (end of the bags)", m_name_.c_str(),
             full_view.getEndTime().toSec());
    ROS_INFO("[%s] Total  time (s): %.9lf\n", m_name_.c_str(),
             bag_length.toSec());
  }

  ROS_INFO("[%s] Loading TF static transforms...", m_name_.c_str());

  for (rosbag::MessageInstance const& msg : tf_static_view)
  {
    if (msg.instantiate<tf2_msgs::TFMessage>() != nullptr)
    {
      tf2_msgs::TFMessagePtr tf_msg = msg.instantiate<tf2_msgs::TFMessage>();

      bool is_static = (msg.getTopic().compare("/tf_static") == 0);

      if (!is_static)
      {
        continue;
      }

      for (const geometry_msgs::TransformStamped& transf : tf_msg->transforms)
      {
        m_tf2_buffer_.setTransform(transf, m_name_, is_static);

        ROS_INFO("[%s] TF static transform: %s --> %s", m_name_.c_str(),
                 transf.header.frame_id.c_str(), transf.child_frame_id.c_str());
      }
    }
  }

  bool is_tf_mode = (m_param_.m_ext_odom_impl.compare("tf") == 0);

  rosbag::View view;

  for (const std::shared_ptr<rosbag::Bag>& bag : m_bags_)
  {
    view.addQuery(*bag, initial_time, finish_time);
  }

  ROS_INFO(
      "[%s] Press SPACE to pause/resume processing, 'q' to quit... "
      "(+/- to change sleep time between msg -- %ld ms)",
      m_name_.c_str(), m_sleep_ms);

  for (rosbag::MessageInstance const& msg : view)
  {
    while (true)
    {
      char key = readTerminalKey();

      if (key == ' ')
      {
        m_paused_ = !m_paused_;

        // could disable ECHO in termios terminal settings
        // but when Ctrl+C, the original settings may not be restored...
        std::cout << std::endl << std::flush;

        printTime(msg.getTime(), msg.getTime() - initial_time, bag_length);

        ROS_INFO(
            "[%s][%s] Press SPACE to pause/resume processing, 'q' to quit... "
            "(+/- to change sleep time between msg -- %ld ms)",
            m_name_.c_str(), m_paused_ ? "PAUSED " : "RESUMED", m_sleep_ms);
      }
      else if (key == 'q' || key == 'Q')
      {
        std::cout << "Processing stopped by user" << std::endl;
        goto exit_loop;
      }
      else if (key == '+')
      {
        // could disable ECHO in termios terminal settings
        // but when Ctrl+C, the original settings may not be restored...
        std::cout << std::endl << std::flush;

        m_sleep_ms += 10;

        ROS_INFO(
            "[%s][%s] Press SPACE to pause/resume processing, 'q' to quit... "
            "(+/- to change sleep time between msg -- %ld ms)",
            m_name_.c_str(), m_paused_ ? "PAUSED " : "RESUMED", m_sleep_ms);
      }
      else if (key == '-')
      {
        // could disable ECHO in termios terminal settings
        // but when Ctrl+C, the original settings may not be restored...
        std::cout << std::endl << std::flush;

        m_sleep_ms = std::max(m_sleep_ms - 10, static_cast<int64_t>(0));

        ROS_INFO(
            "[%s][%s] Press SPACE to pause/resume processing, 'q' to quit... "
            "(+/- to change sleep time between msg -- %ld ms)",
            m_name_.c_str(), m_paused_ ? "PAUSED " : "RESUMED", m_sleep_ms);
      }

      if (!m_paused_)
      {
        break;
      }

      ros::spinOnce();

      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    if (msg.instantiate<sensor_msgs::LaserScan>() != nullptr)
    {
      if (msg.getTopic() != m_param_offline_.m_scan_topic)
      {
        continue;
      }

      sensor_msgs::LaserScanPtr laser_msg =
          msg.instantiate<sensor_msgs::LaserScan>();

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
    else if (msg.instantiate<nav_msgs::Odometry>() != nullptr)
    {
      if ((msg.getTopic() != m_param_offline_.m_odom_topic) ||
          (!m_param_.m_ext_odom_use))
      {
        continue;
      }

      nav_msgs::OdometryPtr odom_msg = msg.instantiate<nav_msgs::Odometry>();

      if (!is_tf_mode)
      {
        m_sub_scan_odom_sync_->add<0>(odom_msg);
      }
    }
    else if (msg.instantiate<tf2_msgs::TFMessage>() != nullptr)
    {
      tf2_msgs::TFMessagePtr tf_msg = msg.instantiate<tf2_msgs::TFMessage>();

      bool is_static = (msg.getTopic().compare("/tf_static") == 0);

      for (const geometry_msgs::TransformStamped& transf : tf_msg->transforms)
      {
        m_tf2_buffer_.setTransform(transf, m_name_, is_static);
      }
    }

    if (is_tf_mode && m_param_.m_ext_odom_use)
    {
      ros::spinOnce();
    }

    if (m_sleep_ms > 0)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(m_sleep_ms));
    }
  }

// Exit loop if 'q' was pressed while processing the ROS bags
exit_loop:

  auto end = std::chrono::high_resolution_clock::now();

  restoreTerminal();

  std::cout << std::endl << std::flush;

  ROS_INFO(
      "\n\n"
      "[%s] Finished processing the ROS bags.\n"
      "Elapsed time (s): %.3lf\n"
      "ros::spin to the infinite...",
      m_name_.c_str(),
      std::chrono::duration_cast<std::chrono::microseconds>(end - start)
              .count() *
          1e-6);

  if (m_param_offline_.m_enable_log && m_log_file_.is_open())
  {
    m_log_file_.close();
  }

  for (const std::shared_ptr<rosbag::Bag>& bag : m_bags_)
  {
    if (bag->isOpen())
    {
      ROS_INFO("[%s] Closing %s", m_name_.c_str(), bag->getFileName().c_str());

      bag->close();
    }
  }

  ros::spin();
}

void TrackerROS1Offline::pubOdom(const std_msgs::Header& header)
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
    m_log_file_ << std::fixed << std::setprecision(9) << header.stamp.toSec()
                << " " << odom_v.x() << " " << odom_v.y() << " " << 0 << " "
                << odom_q.x() << " " << odom_q.y() << " " << odom_q.z() << " "
                << odom_q.w() << std::endl;
  }
  catch (const std::exception& e)
  {
    throw std::runtime_error(
        "TrackerROS1Offline::pubOdom | error when logging the robot data (" +
        std::string(e.what()) + ")");
  }
}

void TrackerROS1Offline::setupTerminal()
{
  if (m_terminal_modified_)
  {
    return;
  }

  // Save original terminal settings
  const int fd = fileno(stdin);
  tcgetattr(fd, &m_orig_flags_);

  // Set terminal to raw mode for immediate key detection
  struct termios raw = m_orig_flags_;
  raw.c_lflag &= ~(ICANON);  // noncanonical mode (input available immediately)
  raw.c_cc[VMIN] = 0;        // polling read mode
  raw.c_cc[VTIME] = 0;       // block if waiting for char

  tcsetattr(fd, TCSANOW, &raw);  // change occur immediately

  // Make stdin non-blocking
  // fcntl(fd, F_SETFL, O_NONBLOCK);

  // Hide cursor and clear screen
  std::cout << "\033[2J"  // clear entire screen
            << "\033[H"   // move cursor to home position
            << std::flush;

  m_terminal_modified_ = true;
}

void TrackerROS1Offline::restoreTerminal()
{
  if (!m_terminal_modified_)
  {
    return;
  }

  const int fd = fileno(stdin);
  m_orig_flags_.c_lflag |= (ICANON);  // always restore canonical mode
  tcsetattr(fd, TCSANOW, &m_orig_flags_);

  // int fl = fcntl(fd, F_GETFL);
  // fcntl(fd, F_SETFL, fl & ~O_NONBLOCK);

  m_terminal_modified_ = false;
}

void TrackerROS1Offline::printTime(const ros::Time& t,
                                   const ros::Duration& duration,
                                   const ros::Duration& bag_length) const
{
  ROS_INFO("[%s][%s] Bag Time: %13.6f   Duration: %.6f / %.6f", m_name_.c_str(),
           m_paused_ ? "PAUSED " : "RESUMED", t.toSec(), duration.toSec(),
           bag_length.toSec());
}

char TrackerROS1Offline::readTerminalKey() const
{
  char c;

  if (read(STDIN_FILENO, &c, 1) < 0)
  {
    return '\0';
  }

  // Filter out control characters and escape sequences
  if (c == '\033')
  {  // ESC character - start of escape sequence
    // Read and discard the rest of the escape sequence
    char temp;
    while (read(STDIN_FILENO, &temp, 1) > 0)
    {
      if (temp >= 'A' && temp <= 'Z') break;  // End of most escape sequences
      if (temp >= 'a' && temp <= 'z') break;
      if (temp == '~') break;  // End of some sequences
    }
    return '\0';
  }

  // Only return printable characters and specific control chars you want
  if ((c >= 32 && c <= 126) || c == '\n' || c == '\r' || c == '\t' || c == 27)
  {
    return c;
  }

  return '\0';  // Ignore other characters
}

}  // namespace inesctec_mrdt_slam_distmap_2d_ros
