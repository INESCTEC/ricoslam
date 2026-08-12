#include "inesctec_mrdt_slam_2d_datasets/carmen_to_rosbag_parser.hpp"

#include <nav_msgs/Odometry.h>
#include <rosbag/view.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_datatypes.h>
#include <tf2_msgs/TFMessage.h>
#include <yaml-cpp/yaml.h>

#include <exception>
#include <filesystem>
#include <fstream>
#include <sstream>

#include "inesctec_mrdt_slam_2d_datasets/utils.h"

#define OFFLNAME "inesctec_mrdt_slam_2d_datasets"

#define OFFLINFO(str, ...)  \
  printf(str, __VA_ARGS__); \
  std::cout << std::endl;

#define OFFLWARN(str, ...)           \
  std::cerr << "\033[1m\033[33m";    \
  fprintf(stderr, str, __VA_ARGS__); \
  std::cerr << "\033[0m" << std::endl;

#define OFFLERROR(str, ...)          \
  std::cerr << "\033[1m\033[31m";    \
  fprintf(stderr, str, __VA_ARGS__); \
  std::cerr << "\033[0m" << std::endl;

namespace inesctec_mrdt_slam_2d_datasets
{

template <typename T>
void readParam(const YAML::Node &config, std::string variable_name, T &variable)
{
  if (config[variable_name])
  {
    variable = config[variable_name].as<T>();
  }
  else
  {
    throw std::runtime_error("readParam | Parameter " + variable_name +
                             " not found...");
  }
}  // template <typename T> void readParam(const YAML::Node&, std::string, T&)

template <typename T>
void readParam(const YAML::Node &config, std::string variable_name, T &variable,
               const T &default_value)
{
  if (config[variable_name])
  {
    variable = config[variable_name].as<T>();
  }
  else
  {
    variable = default_value;
  }
}  // template <typename T> void readParam(const YAML::Node&, std::string, T&,
   // const T&)

void CARMENToROSbagParser::addOptions(
    boost::program_options::options_description &opts)
{
  opts.add_options()("config,c",
                     boost::program_options::value<std::string>(
                         &m_param_.m_opts_yaml_config_file_path)
                         ->required(),
                     "YAML configuration file")(
      "data,d",
      boost::program_options::value<std::string>(&m_param_.m_log_folder)
          ->required(),
      "data folder (input .clf)")(
      "output,o",
      boost::program_options::value<std::string>(&m_param_.m_bag_folder)
          ->default_value(""),
      "output folder for the ROSbags (defaults to the data folder)");

}  // void
   // CARMENToROSbagParser::addOptions(boost::program_options::options_description&)

void CARMENToROSbagParser::load()
{
  try
  {
    readConfig();
  }
  catch (std::exception &e)
  {
    std::stringstream error;

    error << "CARMENToROSbagParser::load | "
             "Error when reading the parameters from the YAML file "
             "(filename: "
          << m_param_.m_opts_yaml_config_file_path
          << "; "
             "error: "
          << e.what() << ")" << std::endl;

    std::cerr << "\033[1m\033[31m" << error.str() << "\033[0m" << std::endl
              << std::endl;

    throw std::runtime_error(error.str());
  }

  printConfig();

  std::ifstream logfile;
  logfile.open(m_param_.m_log_folder + m_param_.m_log_filename, std::ios::in);

  if (logfile.is_open())
  {
    std::vector<std::string> lines;

    parseLines(lines, logfile);

    m_stats_.reset();
    m_tf_static_written_ = false;

    std::string bag_folder = m_param_.m_bag_folder.empty()
                                 ? m_param_.m_log_folder
                                 : m_param_.m_bag_folder;
    if (!bag_folder.empty() && bag_folder.back() != '/')
    {
      bag_folder += '/';
    }
    std::filesystem::create_directories(bag_folder);

    // drop the .clf extension so bags are "<stem>.orig.bag" / "<stem>.sort.bag"
    std::string bag_stem = m_param_.m_log_filename;
    const std::size_t dot = bag_stem.find_last_of('.');
    if (dot != std::string::npos)
    {
      bag_stem = bag_stem.substr(0, dot);
    }
    const std::string bag_prefix = bag_folder + bag_stem;

    m_bag_orig_.setCompression(rosbag::CompressionType::BZ2);
    m_bag_sort_.setCompression(rosbag::CompressionType::BZ2);
    m_bag_orig_.open(bag_prefix + ".orig.bag", rosbag::bagmode::Write);
    m_bag_sort_.open(bag_prefix + ".sort.bag", rosbag::bagmode::Write);

    for (const auto &line : lines)
    {
      if (line.find("ROBOTLASER1 ") == 0)
      {
        parseRobotLaser(line);
      }
      else if (line.find("ODOM ") == 0)
      {
        parseOdom(line);
      }
      else if (line.find("FLASER ") == 0)
      {
        parseFLaser(line);
      }

      m_stats_.m_num_msgs_total =
          m_stats_.m_num_msgs_laser_total + m_stats_.m_num_msgs_odom;
    }

    OFFLINFO("[%s] parser stats (#msgs):", OFFLNAME);
    OFFLINFO("[%s] - odom       : %d", OFFLNAME, m_stats_.m_num_msgs_odom);
    OFFLINFO("[%s] - flaser     : %d", OFFLNAME, m_stats_.m_num_msgs_flaser);
    OFFLINFO("[%s] - robotlaser1: %d", OFFLNAME,
             m_stats_.m_num_msgs_robotlaser);
    OFFLINFO("[%s] - laser total: %d", OFFLNAME,
             m_stats_.m_num_msgs_laser_total);
    OFFLINFO("[%s] - total      : %d", OFFLNAME, m_stats_.m_num_msgs_total);
    OFFLINFO("[%s] laser range stats (#msgs):", OFFLNAME);
    OFFLINFO("[%s] - max_range    : %f (m)", OFFLNAME, m_stats_.m_max_range);
    OFFLINFO("[%s] - 2nd max_range: %f (m)", OFFLNAME,
             m_stats_.m_max_range_2nd);
    OFFLINFO("[%s] - min_range    : %f (m)", OFFLNAME, m_stats_.m_min_range);
    OFFLINFO("[%s] - 2nd min_range: %f (m)", OFFLNAME,
             m_stats_.m_min_range_2nd);

    m_bag_orig_.close();

    // reorder the header.seq indexation...
    // (rosbag::View provides an iterator interface across the bags, sorted
    // based on time)

    uint32_t idx_odom = 0;
    uint32_t idx_scan = 0;
    uint32_t idx_tf = 0;

    m_bag_orig_.open(bag_prefix + ".orig.bag", rosbag::bagmode::Read);

    for (rosbag::MessageInstance msg : rosbag::View(m_bag_orig_))
    {
      if (msg.instantiate<sensor_msgs::LaserScan>() != nullptr)
      {
        sensor_msgs::LaserScanPtr laser_msg =
            msg.instantiate<sensor_msgs::LaserScan>();

        laser_msg->header.seq = idx_scan++;

        m_bag_sort_.write(msg.getTopic(), msg.getTime(), *laser_msg);
        continue;
      }
      else if (msg.instantiate<nav_msgs::Odometry>() != nullptr)
      {
        nav_msgs::OdometryPtr odom_msg = msg.instantiate<nav_msgs::Odometry>();

        odom_msg->header.seq = idx_odom++;

        m_bag_sort_.write(msg.getTopic(), msg.getTime(), *odom_msg);
        continue;
      }
      else if (msg.instantiate<tf2_msgs::TFMessage>() != nullptr)
      {
        tf2_msgs::TFMessagePtr tf_msg = msg.instantiate<tf2_msgs::TFMessage>();

        for (geometry_msgs::TransformStamped &transf : tf_msg->transforms)
        {
          transf.header.seq = idx_tf++;
        }

        m_bag_sort_.write(msg.getTopic(), msg.getTime(), *tf_msg);
        continue;
      }

      m_bag_sort_.write(msg.getTopic(), msg.getTime(), msg);
    }

    m_bag_orig_.close();
    m_bag_sort_.close();
  }
  else
  {
    std::stringstream error;

    error << "CARMENToROSbagParser::load | "
             "Error when opening the log file "
             "(file path: "
          << m_param_.m_log_folder + m_param_.m_log_filename << ")"
          << std::endl;

    std::cerr << "\033[1m\033[31m" << error.str() << "\033[0m" << std::endl
              << std::endl;

    throw std::runtime_error(error.str());
  }

  logfile.close();

}  // void CARMENToROSbagParser::load()

void CARMENToROSbagParser::readConfig()
{
  YAML::Node config = YAML::LoadFile(m_param_.m_opts_yaml_config_file_path);

  readParam(config, "base_frame_id", m_param_.m_base_frame_id);
  readParam(config, "log_filename", m_param_.m_log_filename);

  readParam(config["laser"], "fov", m_param_.m_laser_config.m_fov);
  readParam(config["laser"], "range_min", m_param_.m_laser_config.m_range_min);
  readParam(config["laser"], "range_max", m_param_.m_laser_config.m_range_max);

  m_param_.m_laser_config.m_tf.setIdentity();

  if (config["laser"]["ROS"]["tf"])
  {
    if (config["laser"]["ROS"]["tf"].as<std::vector<double>>().size() != 3)
    {
      throw std::runtime_error(
          "readParam | Parameter tf of the laser invalid... "
          "(expected format: [ x (m) , y (m) , th (deg) ] )");
    }

    m_param_.m_laser_config.m_tf.getOrigin().setX(
        config["laser"]["ROS"]["tf"].as<std::vector<double>>().at(0));
    m_param_.m_laser_config.m_tf.getOrigin().setY(
        config["laser"]["ROS"]["tf"].as<std::vector<double>>().at(1));

    m_param_.m_laser_config.m_tf.setRotation(
        tf::createQuaternionFromYaw(degToRad(
            config["laser"]["ROS"]["tf"].as<std::vector<double>>().at(2))));
  }

  readParam(config["laser"]["ROS"], "topic", m_param_.m_laser_config.m_topic);
  readParam(config["laser"]["ROS"], "frame_id",
            m_param_.m_laser_config.m_frame_id);
  readParam(config["laser"]["ROS"], "save_odom",
            m_param_.m_laser_config.m_save_odom);

  readParam(config["odom"]["ROS"], "topic", m_param_.m_odom_config.m_topic);
  readParam(config["odom"]["ROS"], "frame_id",
            m_param_.m_odom_config.m_frame_id);
  readParam(config["odom"]["ROS"], "publish_tf",
            m_param_.m_odom_config.m_publish_tf);

  m_param_.m_laser_config.m_fov = degToRad(m_param_.m_laser_config.m_fov);

  m_param_.m_laser_config.m_tf.frame_id_ = m_param_.m_base_frame_id;
  m_param_.m_laser_config.m_tf.child_frame_id_ =
      m_param_.m_laser_config.m_frame_id;
}  // void CARMENToROSbagParser::config()

void CARMENToROSbagParser::printConfig()
{
  OFFLINFO("[%s] CARMEN to ROS Parser Parametrization", OFFLNAME);

  OFFLINFO("[%s] base_frame_id: %s", OFFLNAME,
           m_param_.m_base_frame_id.c_str());
  OFFLINFO("[%s] log:", OFFLNAME);
  OFFLINFO("[%s] - filename: %s", OFFLNAME, m_param_.m_log_filename.c_str());
  OFFLINFO("[%s] - folder  : %s", OFFLNAME, m_param_.m_log_folder.c_str());
  OFFLINFO("[%s] - path    : %s", OFFLNAME,
           (m_param_.m_log_folder + m_param_.m_log_filename).c_str());

  OFFLINFO("[%s] laser:", OFFLNAME);
  OFFLINFO("[%s] - fov       : %f (deg)", OFFLNAME,
           radToDeg(m_param_.m_laser_config.m_fov));
  OFFLINFO("[%s] - range_min : %f (m)", OFFLNAME,
           m_param_.m_laser_config.m_range_min);
  OFFLINFO("[%s] - range_max : %f (m)", OFFLNAME,
           m_param_.m_laser_config.m_range_max);
  OFFLINFO("[%s] - ROS", OFFLNAME);
  OFFLINFO("[%s]   - tf       : [ %f m ; %f m ; %f deg ]", OFFLNAME,
           m_param_.m_laser_config.m_tf.getOrigin().x(),
           m_param_.m_laser_config.m_tf.getOrigin().y(),
           radToDeg(tf::getYaw(m_param_.m_laser_config.m_tf.getRotation())));
  OFFLINFO("[%s]   - topic    : %s", OFFLNAME,
           m_param_.m_laser_config.m_topic.c_str());
  OFFLINFO("[%s]   - frame_id : %s", OFFLNAME,
           m_param_.m_laser_config.m_frame_id.c_str());
  OFFLINFO("[%s]   - save_odom: %s", OFFLNAME,
           m_param_.m_laser_config.m_save_odom ? "enabled" : "not enabled");

  OFFLINFO("[%s] odom:", OFFLNAME);
  OFFLINFO("[%s] - ROS", OFFLNAME);
  OFFLINFO("[%s]   - topic     : %s", OFFLNAME,
           m_param_.m_odom_config.m_topic.c_str());
  OFFLINFO("[%s]   - frame_id  : %s", OFFLNAME,
           m_param_.m_odom_config.m_frame_id.c_str());
  OFFLINFO("[%s]   - publish_tf: %s", OFFLNAME,
           m_param_.m_odom_config.m_publish_tf ? "enabled" : "not enabled");
}  // void CARMENToROSbagParser::printConfig()

void CARMENToROSbagParser::parseLines(std::vector<std::string> &lines,
                                      std::ifstream &file)
{
  lines.clear();

  for (std::string line; std::getline(file, line);)
  {
    if (((line.find("ROBOTLASER1 ") == 0) || (line.find("ODOM ") == 0) ||
         (line.find("FLASER ") == 0)) &&
        (!line.empty()))
    {
      lines.emplace_back(line);
    }
  }
}  // void CARMENToROSbagParser::parseLines(std::vector<std::string>&,
   // std::ifstream&)

void CARMENToROSbagParser::parseOdom(const std::string &line)
{
  std::string token;
  std::stringstream str(line);

  double ts = 0;
  double x = 0, y = 0, th = 0;
  double v = 0, vn = 0, w = 0;

  str >> token;         // ignore ODOM
  str >> x >> y >> th;  // x,y,th
  str >> v >> w;        // tv,rv
  str >> token;         // accel
  str >> ts;            // ipc_timestamp

  /* std::cout << line << std::endl; */

  addOdomMsg(ts, x, y, th, v, vn, w);
}  // void CARMENToROSbagParser::parseOdom(const std::string&)

void CARMENToROSbagParser::parseFLaser(const std::string &line)
{
  std::string token;
  std::stringstream str(line);

  int num_samples;

  double ts = 0;
  double odom_x = 0, odom_y = 0, odom_th = 0;

  std::vector<float> ranges;
  std::vector<float> intensities;

  str >> token;        // ignore FLASER
  str >> num_samples;  // number of laser range readings

  // FLASER carries neither FOV nor angular resolution (unlike ROBOTLASER1).
  // CARMEN convention: front SICK laser spanning 180 deg. Derive the angular
  // resolution from the sample count so the beams span exactly [-fov/2, +fov/2]
  // and angle_max == angle_min + (n-1)*angle_increment by construction.

  const float fov = m_param_.m_laser_config.m_fov;  // rad; default 180 deg
  const float angle_inc =
      (num_samples > 1) ? fov / static_cast<float>(num_samples - 1) : 0.0f;
  const float angle_min = -fov * 0.5f;
  const float angle_max =
      angle_min + angle_inc * static_cast<float>(num_samples - 1);

  ranges.resize(num_samples);
  for (int idx = 0; idx < num_samples; idx++)
  {
    str >> ranges[idx];

    if (ranges[idx] > m_stats_.m_max_range)
    {
      m_stats_.m_max_range_2nd = m_stats_.m_max_range;
      m_stats_.m_max_range = ranges[idx];
    }
    else if (ranges[idx] < m_stats_.m_max_range &&
             ranges[idx] > m_stats_.m_max_range_2nd)
    {
      m_stats_.m_max_range_2nd = ranges[idx];
    }

    if (ranges[idx] < m_stats_.m_min_range)
    {
      m_stats_.m_min_range_2nd = m_stats_.m_min_range;
      m_stats_.m_min_range = ranges[idx];
    }
    else if (ranges[idx] > m_stats_.m_min_range &&
             ranges[idx] < m_stats_.m_min_range_2nd)
    {
      m_stats_.m_min_range_2nd = ranges[idx];
    }
  }
  str >> token;  // x (in the laser frame? see messages.html from CARMEN doc...)
  str >> token;  // y
  str >> token;  // theta
  str >> odom_x;   // odom_x
  str >> odom_y;   // odom_y
  str >> odom_th;  // odom_theta
  str >> ts;       // ipc_timestamp

  /* std::cout << line << std::endl; */

  addLaserScanMsg(ts, angle_min, angle_max, angle_inc,
                  m_param_.m_laser_config.m_range_min,
                  m_param_.m_laser_config.m_range_max, ranges, intensities,
                  odom_x, odom_y, odom_th, 0.0, 0.0, 0.0);

  m_stats_.m_num_msgs_flaser++;
}  // void CARMENToROSbagParser::parseFLaser(const std::string&)

void CARMENToROSbagParser::parseRobotLaser(const std::string &line)
{
  std::string token;
  std::stringstream str(line);

  int num_samples;

  double ts = 0;
  double odom_x = 0, odom_y = 0, odom_th = 0;
  double odom_v = 0, odom_w = 0;

  float fov;

  float angle_min, angle_max, angle_inc;
  float range_max;

  std::vector<float> ranges;
  std::vector<float> intensities;

  str >> token;                          // ignore ROBOTLASER1
  str >> token;                          // laser_type
  str >> angle_min >> fov >> angle_inc;  // start_angle , FOV , res (rad)
  str >> range_max;                      // range max
  str >> token;                          // accuracy
  str >> token;                          // remission mode
  str >> num_samples;                    // number of laser range readings
  ranges.resize(num_samples);
  for (int idx = 0; idx < num_samples; idx++)
  {
    str >> ranges[idx];

    m_stats_.m_max_range = std::max(m_stats_.m_max_range, ranges[idx]);

    if ((ranges[idx] > m_stats_.m_max_range_2nd) &&
        (ranges[idx] < m_stats_.m_max_range))
    {
      m_stats_.m_max_range_2nd = ranges[idx];
    }

    m_stats_.m_min_range = std::min(m_stats_.m_min_range, ranges[idx]);

    if ((ranges[idx] > m_stats_.m_min_range) &&
        (ranges[idx] < m_stats_.m_min_range_2nd))
    {
      m_stats_.m_min_range_2nd = ranges[idx];
    }
  }
  str >> num_samples;  // number of laser remission readings
  if (num_samples > 0)
  {
    intensities.resize(num_samples);

    for (int idx = 0; idx < num_samples; idx++)
    {
      str >> intensities[idx];
    }
  }
  str >> token;    // x
  str >> token;    // y
  str >> token;    // theta
  str >> odom_x;   // odom_x
  str >> odom_y;   // odom_y
  str >> odom_th;  // odom_theta
  str >> odom_v;   // odom_tv  (not entirely sure...)
  str >> odom_w;   // odom_rv
  str >> token;    // forward_safety_dist
  str >> token;    // side_safety_dist
  str >> token;    // turn_axis
  str >> ts;       // ipc_timestamp

  angle_max = angle_min + fov;

  /* std::cout << line << std::endl; */

  addLaserScanMsg(ts, angle_min, angle_max, angle_inc,
                  m_param_.m_laser_config.m_range_min, range_max, ranges,
                  intensities, odom_x, odom_y, odom_th, odom_v, 0.0, odom_w);

  m_stats_.m_num_msgs_robotlaser++;
}  // void CARMENToROSbagParser::parseRobotLaser(const std::string&)

void CARMENToROSbagParser::addOdomMsg(double ts, double x, double y, double th,
                                      double v, double vn, double w)
{
  addTFStaticLaserToBase(ts);

  nav_msgs::Odometry msg;

  msg.header.stamp.fromSec(ts);
  msg.header.seq = static_cast<uint32_t>(m_stats_.m_num_msgs_odom++);
  msg.header.frame_id = m_param_.m_odom_config.m_frame_id;
  msg.child_frame_id = m_param_.m_base_frame_id;

  msg.pose.pose.position.x = x;
  msg.pose.pose.position.y = y;
  msg.pose.pose.position.z = 0;

  tf::Quaternion th_q = tf::createQuaternionFromYaw(th);

  msg.pose.pose.orientation.w = th_q.getW();
  msg.pose.pose.orientation.x = th_q.getX();
  msg.pose.pose.orientation.y = th_q.getY();
  msg.pose.pose.orientation.z = th_q.getZ();

  msg.twist.twist.linear.x = v;
  msg.twist.twist.linear.y = vn;
  msg.twist.twist.linear.z = 0;

  msg.twist.twist.angular.x = 0;
  msg.twist.twist.angular.y = 0;
  msg.twist.twist.angular.z = w;

  /* std::cout << "ODOM" << std::endl
            << "header" << std::endl
            << "- seq     : " << static_cast<int>(msg.header.seq) << std::endl
            << "- stamp   : " << std::fixed << msg.header.stamp.toSec() <<
     std::endl
            << "- frame_id: " << msg.header.frame_id << std::endl
            << "child_frame_id" << std::endl
            << "pose" << std::endl
            << "- pose" << std::endl
            << "  - position" << std::endl
            << "    - x: " << msg.pose.pose.position.x << std::endl
            << "    - y: " << msg.pose.pose.position.y << std::endl
            << "    - z: " << msg.pose.pose.position.z << std::endl
            << "  - orientation" << std::endl
            << "    - x : " << msg.pose.pose.orientation.x << std::endl
            << "    - y : " << msg.pose.pose.orientation.y << std::endl
            << "    - z : " << msg.pose.pose.orientation.z << std::endl
            << "    - w : " << msg.pose.pose.orientation.w << std::endl
            << "    - th: " << tf::getYaw(msg.pose.pose.orientation) <<
     std::endl
            << "twist" << std::endl
            << "- twist" << std::endl
            << "  - linear" << std::endl
            << "    - x: " << msg.twist.twist.linear.x << std::endl
            << "    - y: " << msg.twist.twist.linear.y << std::endl
            << "    - z: " << msg.twist.twist.linear.z << std::endl
            << "  - angular" << std::endl
            << "    - x: " << msg.twist.twist.angular.x << std::endl
            << "    - y: " << msg.twist.twist.angular.y << std::endl
            << "    - z: " << msg.twist.twist.angular.z << std::endl <<
     std::endl; */

  m_bag_orig_.write(m_param_.m_odom_config.m_topic, msg.header.stamp, msg);

  if (m_param_.m_odom_config.m_publish_tf)
  {
    tf2_msgs::TFMessage tf_msg;

    tf_msg.transforms.resize(1);

    tf_msg.transforms.at(0).header = msg.header;
    tf_msg.transforms.at(0).child_frame_id = msg.child_frame_id;

    tf_msg.transforms.at(0).transform.translation.x = msg.pose.pose.position.x;
    tf_msg.transforms.at(0).transform.translation.y = msg.pose.pose.position.y;
    tf_msg.transforms.at(0).transform.translation.z = msg.pose.pose.position.z;

    tf_msg.transforms.at(0).transform.rotation = msg.pose.pose.orientation;

    m_bag_orig_.write("/tf", msg.header.stamp, tf_msg);
  }
}  // void CARMENToROSbagParser::addOdomMsg(double, double, double, double,
   // double, double, double)

void CARMENToROSbagParser::addLaserScanMsg(
    double ts, float angle_min, float angle_max, float angle_inc,
    float range_min, float range_max, const std::vector<float> &ranges,
    const std::vector<float> &intensities, double odom_x, double odom_y,
    double odom_th, double odom_v, double odom_vn, double odom_w)
{
  addTFStaticLaserToBase(ts);

  sensor_msgs::LaserScan msg;

  msg.header.stamp.fromSec(ts);
  msg.header.seq = static_cast<uint32_t>(m_stats_.m_num_msgs_laser_total++);
  msg.header.frame_id = m_param_.m_laser_config.m_frame_id;

  msg.angle_min = angle_min;
  msg.angle_max = angle_max;
  msg.angle_increment = angle_inc;

  msg.time_increment = 0.f;
  msg.scan_time = 0.f;

  msg.range_min = range_min;
  msg.range_max = range_max;

  msg.ranges = ranges;
  msg.intensities = intensities;

  if (m_param_.m_laser_config.m_save_odom)
  {
    addOdomMsg(ts, odom_x, odom_y, odom_th, odom_v, odom_vn, odom_w);
  }

  /* std::cout << "FLASER" << std::endl
            << "header" << std::endl
            << "- seq     : " << static_cast<int>(msg.header.seq) << std::endl
            << "- stamp   : " << std::fixed << msg.header.stamp.toSec() <<
  std::endl
            << "- frame_id: " << msg.header.frame_id << std::endl
            << "angle_min      : " << msg.angle_min << std::endl
            << "angle_max      : " << msg.angle_max << std::endl
            << "angle_increment: " << msg.angle_increment << std::endl
            << "time_increment: " << msg.time_increment << std::endl
            << "scan_time     : " << msg.scan_time << std::endl
            << "range_min: " << msg.range_min << std::endl
            << "range_max: " << msg.range_max << std::endl
            << "ranges (# = " << msg.ranges.size() << "): [ ";
  for (const float& range : msg.ranges)
  {
    std::cout << range << " ";
  }
  std::cout << "]" << std::endl << std::endl; */

  m_bag_orig_.write(m_param_.m_laser_config.m_topic, msg.header.stamp, msg);
}  // void CARMENToROSbagParser::addLaserScanMsg(...)

void CARMENToROSbagParser::addTFStaticLaserToBase(double ts)
{
  if (m_tf_static_written_)
  {
    return;
  }

  m_tf_static_written_ = true;

  m_param_.m_laser_config.m_tf.stamp_.fromSec(ts);

  tf2_msgs::TFMessage tf_msg;

  tf_msg.transforms.resize(1);

  tf::transformStampedTFToMsg(m_param_.m_laser_config.m_tf,
                              tf_msg.transforms.at(0));

  m_bag_orig_.write("/tf_static", tf_msg.transforms.at(0).header.stamp, tf_msg);
}  // void CARMENToROSbagParser::addTFStaticLaserToBase(double)

}  // namespace inesctec_mrdt_slam_2d_datasets
