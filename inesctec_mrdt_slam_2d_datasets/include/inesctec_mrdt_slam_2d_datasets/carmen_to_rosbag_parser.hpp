#pragma once

#include <vector>
#include <limits>

#include <boost/program_options.hpp>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <tf/tf.h>





namespace inesctec_mrdt_slam_2d_datasets
{



/**
 * @brief CARMEN to ROS parser
 * @note supported messages: ODOM, FLASER, ROBOTLASER1
 * @note parse considers only 1 odometry + 1 laser data source
 */
class CARMENToROSbagParser
{

 public:

  /**
   * @brief laser parsing parametrization (FLASER or ROBOTLASER1)
   */
  struct LaserParam
  {
    float m_fov;                //!< field-of-view (rad)
    float m_resolution;         //!< resolution (rad)
    float m_range_min;          //!< minimum range (m)
    float m_range_max;          //!< maximum range (m)
    tf::StampedTransform m_tf;  //!< TF laser w.r.t. base frame
    std::string m_topic;        //!< laser message ROS topic name
    std::string m_frame_id;     //!< laser frame id
    bool m_save_odom;           //!< enable save odom data from laser message
  };// struct CARMENToROSbagParser::LaserParam

  /**
   * @brief odometry parsing parametrization (ODOM)
   */
  struct OdomParam
  {
    std::string m_topic;    //!< odometry message ROS topic name
    std::string m_frame_id; //!< odometry frame id
    bool m_publish_tf;      //!< enable publish TF base w.r.t. odom frame
  };// struct CARMENToROSbagParser::OdomParam

  /**
   * @brief CARMEN to ROS parser parametrization
   */
  struct Param
  {
    std::string m_opts_yaml_config_file_path; //!< YAML file path for the parser's parametrization
    std::string m_base_frame_id;  //!< base frame id
    std::string m_log_filename;   //!< data filename
    std::string m_log_folder;     //!< folder directory of the CARMEN log data files
    LaserParam m_laser_config;    //!< laser parsing parametrization
    OdomParam m_odom_config;      //!< odometry parsing parametrization
  };// struct CARMENToROSbagParser::Param

  /**
   * @brief CARMEN to ROS parser stats (count number of messages correctly parsed)
   */
  struct Stats
  {
    int m_num_msgs_odom = 0;        //!< #ODOM msgs
    int m_num_msgs_flaser = 0;      //!< #FLASER msgs
    int m_num_msgs_robotlaser = 0;  //!< #ROBOTLASER1 msgs
    int m_num_msgs_laser_total = 0; //!< total #laser msgs
    int m_num_msgs_total = 0;       //!< total #msgs

    float m_max_range     = -std::numeric_limits<float>::max(); //!< compute maximum laser range found in data
    float m_max_range_2nd = -std::numeric_limits<float>::max(); //!< compute second maximum range found in data
    float m_min_range     =  std::numeric_limits<float>::max(); //!< compute minimum laser range found in data
    float m_min_range_2nd =  std::numeric_limits<float>::max(); //!< compute second minimum range found in data

    inline void reset()
    {
      m_num_msgs_odom = 0;
      m_num_msgs_flaser = 0;
      m_num_msgs_robotlaser = 0;
      m_num_msgs_laser_total = 0;
      m_num_msgs_total = 0;

      m_max_range     = -std::numeric_limits<float>::max();
      m_max_range_2nd = -std::numeric_limits<float>::max();
      m_min_range     =  std::numeric_limits<float>::max();
      m_min_range_2nd =  std::numeric_limits<float>::max();
    }
  };// struct CARMENToROSbagParser::Stats



 protected:

  Param m_param_; //!< parametrization
  Stats m_stats_; //!< stats

  rosbag::Bag m_bag_orig_;  //!< ROS bag (original order from CARMEN log)
  rosbag::Bag m_bag_sort_;  //!< ROS bag (write order + header.seq sorted by timestamp)





 public:

  CARMENToROSbagParser() = default;

  /**
   * @brief add options to set the class's parametrization from command-line
   *        arguments (using boost::program_options)
   * @param[in,out] opts Boost program options for cmd arguments (argc, argv)
   */
  void addOptions(boost::program_options::options_description& opts);

  /**
   * @brief load the CARMEN-like log data
   */
  void load();



 public:

  const Param& getParam() const { return m_param_; }
  const Stats& getStats() const { return m_stats_; }

  void setParam(const Param& param) { m_param_ = param; }



 protected:

  void readConfig();
  void printConfig();

  void parseLines(std::vector<std::string>& lines, std::ifstream& file);

  void parseOdom(const std::string& line);
  void parseFLaser(const std::string& line);
  void parseRobotLaser(const std::string& line);

  void addTFStaticLaserToBase(double ts);

  void addOdomMsg(double ts,
                  double x, double y, double th,
                  double v, double vn, double w);

  void addLaserScanMsg(double ts,
                       float angle_min, float angle_max, float angle_inc,
                       float range_min, float range_max,
                       const std::vector<float>& ranges,
                       const std::vector<float>& intensities,
                       double odom_x, double odom_y, double odom_th,
                       double odom_v, double odom_vn, double odom_w);

};// class CARMENToROSbagParser



} // namespace inesctec_mrdt_slam_2d_datasets
