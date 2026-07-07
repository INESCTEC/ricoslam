#include "inesctec_mrdt_slam_distmap_2d_ros/ogba_app_ros1.hpp"

#include <ros/ros.h>
#include <rosbag/query.h>
#include <sensor_msgs/LaserScan.h>
#include <srrg2_core/srrg_pcl/instances.h>
#include <srrg2_solver/solver_core/internals/linear_solvers/sparse_block_linear_solver_cholesky_cholmod.h>

#include <exception>
#include <filesystem>
#include <fstream>
#include <inesctec_mrdt_slam_distmap_2d/data_structures/instances.hpp>
#include <inesctec_mrdt_slam_distmap_2d/srrg2_solver/instances.hpp>
#include <inesctec_mrdt_slam_distmap_2d/system_utils/ros_log_offline.hpp>
#include <inesctec_mrdt_slam_distmap_2d/system_utils/ros_yaml_offline.hpp>
#include <iomanip>
#include <thread>

using namespace inesctec_mrdt_slam_distmap_2d;
using namespace data_structures;

namespace inesctec_mrdt_slam_distmap_2d_ros
{

OgbaAppROS1::OgbaAppROS1(const Param& param)
    : m_param_(param),
      m_dbg_ogl_qapp_(nullptr),
      m_dbg_ogl_viewer_(nullptr),
      m_dbg_ogl_canvas_(nullptr)
{
  assertInit(m_param_);

  init();
  loadOGBAConfig();
  print();

  srrg2_core::point_cloud_registerTypes();
  srrg2_solver::inesctec_mrdt_slam_distmap_2d_data_structures_registerTypes();
  srrg2_solver::inesctec_mrdt_slam_distmap_2d_srrg2_solver_registerTypes();
}

OgbaAppROS1::~OgbaAppROS1() { clear(); }

void OgbaAppROS1::clear()
{
  for (const ROSBagTypePtr& bag : m_bags_)
  {
    bag->close();
  }

  m_bags_.clear();
  m_bags_view_full_.reset();
  m_bags_view_.reset();

  m_log_data_.clear();
}

void OgbaAppROS1::init()
{
  m_sensor_in_robot_v_ = EstimateVectorType(
      m_param_.m_sensor_in_robot_val[0], m_param_.m_sensor_in_robot_val[1],
      geometry::degToRad(m_param_.m_sensor_in_robot_val[2]));

  m_sensor_in_robot_t_ = geometry::v2t(m_sensor_in_robot_v_);

  // -------------------------
  // --> LOAD ROSBAG FILES <--
  // -------------------------

  for (const std::string& filename_bag : m_param_.m_filenames_bags)
  {
    OFFLINFO("Opening %s", filename_bag.c_str());

    try
    {
      ROSBagTypePtr bag = std::make_shared<ROSBagType>();

      bag->open(filename_bag, rosbag::bagmode::Read);

      m_bags_.push_back(bag);
    }
    catch (rosbag::BagException& e)
    {
      throw std::runtime_error(
          "Error when opening the ROS bag file (filename: " + filename_bag +
          "; error: " + e.what() + ")");
    }
  }

  m_bags_view_full_ = std::make_shared<ROSViewType>();

  for (const ROSBagTypePtr& bag : m_bags_)
  {
    m_bags_view_full_->addQuery(*bag);
  }

  const ros::Time full_initial_time = m_bags_view_full_->getBeginTime();
  const ros::Time initial_time =
      full_initial_time + ros::Duration(m_param_.m_time_start);
  ros::Time finish_time =
      m_param_.m_has_duration
          ? initial_time + ros::Duration(m_param_.m_time_duration)
          : ros::TIME_MAX;

  rosbag::TopicQuery query(m_param_.m_scan_topic);

  m_bags_view_ = std::make_shared<ROSViewType>();

  for (const ROSBagTypePtr& bag : m_bags_)
  {
    m_bags_view_->addQuery(*bag, query, initial_time, finish_time);
  }

  if (m_bags_view_->size() <= 0)
  {
    throw std::runtime_error("Scan topic '" + m_param_.m_scan_topic +
                             "' not found in the bag files");
  }

  m_bags_t_initial_ = m_bags_view_->getBeginTime().toSec();
  m_bags_t_finish_ = m_bags_view_->getEndTime().toSec();

  // ----------------------
  // --> PARSE TUM FILE <--
  // ----------------------

  OFFLINFO("Opening %s", m_param_.m_filename_log.c_str());

  std::ifstream file_log(m_param_.m_filename_log, std::ios::in);

  if (!file_log.is_open())
  {
    throw std::runtime_error(
        "log file could not be open (path: " + m_param_.m_filename_log + ")");
  }

  for (std::string line; std::getline(file_log, line);)
  {
    if (!line.empty())
    {
      TUMPoseType pose;

      if (TUMPoseType::getTUMPose(pose, line))
      {
        if ((pose.m_t >= m_bags_t_initial_) && (pose.m_t <= m_bags_t_finish_))
        {
          m_log_data_.emplace_back(pose);
        }
      }
      else
      {
        OFFLWARN(
            "Line [%ld] of the pose log file not compatible with TUM format "
            "('%s')",
            m_log_data_.size(), line.c_str());
      }
    }
  }

  file_log.close();

  if (m_log_data_.empty())
  {
    throw std::runtime_error("Log file empty / no valid TUM poses (path: " +
                             m_param_.m_filename_log + ")");
  }

  std::sort(m_log_data_.begin(), m_log_data_.end(),
            [](const TUMPoseType& a, const TUMPoseType& b)
            { return a.m_t < b.m_t; });
}

void OgbaAppROS1::print()
{
  std::cout << "Offline Global Bundle Adjustment App with Distance Maps:"
            << std::endl
            << "- bag file (format: ROS 1 bag):" << std::endl;
  for (const auto& filename_bag : m_param_.m_filenames_bags)
  {
    std::cout << "  [ " << filename_bag << " ]" << std::endl;
  }
  std::cout << "  - scan topic: " << m_param_.m_scan_topic << std::endl
            << "  - #scans    : " << m_bags_view_->size() << std::endl
            << std::fixed << std::setprecision(12)
            << "  - start  time (s): " << m_bags_t_initial_ << std::endl
            << "  - finish time (s): " << m_bags_t_finish_
            << (m_param_.m_has_duration ? "" : " (end of the bags)")
            << std::endl
            << "  - total  time (s): " << m_bags_t_finish_ - m_bags_t_initial_
            << std::endl;

  std::cout << "- log file (format: TUM poses): " << m_param_.m_filename_log
            << std::endl
            << "  - #poses         : " << m_log_data_.size() << std::endl
            << std::fixed << std::setprecision(12)
            << "  - start  time (s): " << m_log_data_.front().m_t << std::endl
            << "  - finish time (s): " << m_log_data_.back().m_t << std::endl;

  std::cout << std::defaultfloat;

  std::cout << "- sensor pose (x, y, th): [ " << m_sensor_in_robot_v_.x()
            << " m , " << m_sensor_in_robot_v_.y() << " m , "
            << geometry::radToDeg(m_sensor_in_robot_v_.z()) << " deg ]"
            << std::endl;

  std::cout << "- YAML configuration file: " << m_param_.m_ogba_yaml
            << std::endl;

  std::cout << "  - laser invert scan: "
            << (m_param_.m_laser_invert_scan ? "enabled" : "not enabled")
            << std::endl;
  std::cout << "  - ogba OpenGL visualization dbg           : "
            << (m_param_.m_ogba_debug ? "enabled" : "not enabled") << std::endl;
  if (m_param_.m_ogba_debug)
  {
    std::cout << "  - ogba OpenGL visualization dbg mode      : "
              << m_param_.m_ogba_solver_dbg_ogl_mode << std::endl;
    std::cout << "  - ogba OpenGL visualization dbg sleep (ms): "
              << m_param_.m_ogba_solver_dbg_ogl_interval << std::endl;
  }
  else
  {
    std::cout << "  - ogba OpenGL visualization dbg mode      : not enabled"
              << std::endl;
    std::cout << "  - ogba OpenGL visualization dbg sleep (ms): not enabled"
              << std::endl;
  }
  std::cout << "  - ogba solver verbose enabled   : "
            << (m_param_.m_ogba_solver_verbose ? "enabled" : "not enabled")
            << std::endl;
  std::cout << "  - ogba solver algorithm damping : "
            << m_param_.m_ogba_solver_damping << std::endl;
  std::cout << "  - ogba solver maximum iterations: "
            << m_param_.m_ogba_solver_max_iterations << std::endl;
  std::cout << "  - ogba solver termination criteria type     : "
            << (!m_param_.m_ogba_solver_criteria_type.empty() ? "enabled"
                                                              : "not enabled")
            << std::endl;
  if (!m_param_.m_ogba_solver_criteria_type.empty())
  {
    std::cout << "  - ogba solver termination criteria epsilon  : "
              << m_param_.m_ogba_solver_criteria_epsilon << std::endl;
  }
  std::cout << "  - ogba solver robustifier criteria type     : "
            << (!m_param_.m_ogba_solver_robustifier_type.empty()
                    ? m_param_.m_ogba_solver_robustifier_type.c_str()
                    : "not enabled")
            << std::endl;
  if (!m_param_.m_ogba_solver_robustifier_type.empty())
  {
    std::cout << "  - ogba solver robustifier criteria threshold: "
              << m_param_.m_ogba_solver_robustifier_threshold << std::endl;
  }

  std::cout << "  - distance map resolution          (m): "
            << m_param_.m_dist_map_res << std::endl;
  std::cout << "  - distance map size                (m): "
            << m_param_.m_dist_map_size << std::endl;
  std::cout << "  - distance map nn maximum distance (m): "
            << m_param_.m_dist_map_max_dist << std::endl;

  std::cout << "  - ogba preprocessing voxelization (m): "
            << (!m_param_.m_enable_voxelization
                    ? "not enabled"
                    : std::to_string(m_param_.m_voxel_scale).c_str())
            << std::endl;
  std::cout << "  - ogba preprocessing normals computation: "
            << (m_param_.m_enable_normals ? "enabled" : "not enabled")
            << std::endl;
  if (m_param_.m_enable_normals)
  {
    std::cout << "    - minimum #pts     : "
              << m_param_.m_normals_computator_min_pts << std::endl;
    std::cout << "    - search radius (m): "
              << m_param_.m_normals_computator_radius << std::endl;
  }
  std::cout << "  - ogba preprocessing range: "
            << ((m_param_.m_range_min >= 0) || (m_param_.m_range_max >= 0)
                    ? "enabled"
                    : "not enabled")
            << std::endl;
  if ((m_param_.m_range_min >= 0) || (m_param_.m_range_max >= 0))
  {
    std::cout << "    - range minimum (m): "
              << (m_param_.m_range_min >= 0
                      ? std::to_string(m_param_.m_range_min).c_str()
                      : "not enabled")
              << std::endl;
    std::cout << "    - range maximum (m): "
              << (m_param_.m_range_max >= 0
                      ? std::to_string(m_param_.m_range_max).c_str()
                      : "not enabled")
              << std::endl;
  }

  std::cout << "  - ogba prune poses criteria parametrization: {" << std::endl
            << "\033[3m" << *m_param_.m_graph_criteria << "\033[0m" << "}"
            << std::endl;

  std::cout << "  - ogba covisibility graph inliers ratio threshold  (0..1): "
            << m_param_.m_covisibility_ratio_inliers << std::endl;
  std::cout << "  - ogba covisibility graph inliers nn maximum distance (m): "
            << m_param_.m_covisibility_nn_distance << std::endl;
  std::cout << "  - ogba covisibility graph poses maximum distance      (m): "
            << m_param_.m_covisibility_max_distance << std::endl;
  std::cout << "  - ogba covisibility OpenGL visualization dbg sleep   (ms): "
            << m_param_.m_covisibility_dbg_ogl_interval << std::endl;

  std::cout << "  - ogba global poses merger parametrization: {" << std::endl
            << "\033[3m" << m_param_.m_global_merger->getParam() << "\033[0m"
            << "}" << std::endl
            << std::endl;
}

void OgbaAppROS1::loadOGBAConfig()
{
  try
  {
    YAML::Node config = YAML::LoadFile(m_param_.m_ogba_yaml);

    // ----- OFFLINE GLOBAL BUNDLE ADJUSTMENT (OGBA) (MATCHER-LIKE SOLVER PARAM)

    system_utils::getParam(config, "ogba_debug", m_param_.m_ogba_debug, true);
    system_utils::getParam(config, "ogba_solver_verbose",
                           m_param_.m_ogba_solver_verbose, true);
    system_utils::getParam(config, "ogba_solver_damping",
                           m_param_.m_ogba_solver_damping, 1.f);
    system_utils::getParam(config, "ogba_solver_max_iterations",
                           m_param_.m_ogba_solver_max_iterations, 1000);
    system_utils::getParam(config, "ogba_solver_criteria_type",
                           m_param_.m_ogba_solver_criteria_type,
                           std::string("stepnorm"));
    system_utils::getParam(config, "ogba_solver_criteria_epsilon",
                           m_param_.m_ogba_solver_criteria_epsilon, 1e-5f);

    system_utils::getParam(config, "ogba_solver_robustifier_type",
                           m_param_.m_ogba_solver_robustifier_type,
                           std::string("cauchy"));

    if (!m_param_.m_ogba_solver_robustifier_type.empty())
    {
      system_utils::getParam(config, "ogba_solver_robustifier_threshold",
                             m_param_.m_ogba_solver_robustifier_threshold);
    }
    else
    {
      m_param_.m_ogba_solver_robustifier_threshold = -1.f;
    }

    system_utils::getParam(config, "ogba_solver_dbg_ogl_interval",
                           m_param_.m_ogba_solver_dbg_ogl_interval, int64_t(0));
    system_utils::getParam(config, "ogba_solver_dbg_ogl_mode",
                           m_param_.m_ogba_solver_dbg_ogl_mode,
                           std::string(""));

    // ----- DISTANCE MAP ------------------------------------------------------

    system_utils::getParam(config, "dist_map_res", m_param_.m_dist_map_res,
                           0.01f);
    system_utils::getParam(config, "dist_map_size", m_param_.m_dist_map_size,
                           15.f);
    system_utils::getParam(config, "dist_map_max_dist",
                           m_param_.m_dist_map_max_dist, 0.25f);

    // ----- PREPROCESS (VOXELIZATION, NORMALS, ADDITIONAL RANGE THRESHOLDS) ---

    system_utils::getParam(config, "laser_invert_scan",
                           m_param_.m_laser_invert_scan, false);

    system_utils::getParam(config, "enable_voxelization",
                           m_param_.m_enable_voxelization);

    if (m_param_.m_enable_voxelization)
    {
      system_utils::getParam(config, "voxel_scale", m_param_.m_voxel_scale,
                             m_param_.m_dist_map_res);
    }
    else
    {
      m_param_.m_voxel_scale = -1.f;
    }

    if ((system_utils::hasParam(config, "normals_computator_min_pts")) ||
        (system_utils::hasParam(config, "normals_computator_radius")))
    {
      m_param_.m_enable_normals = true;
      system_utils::getParam(config, "normals_computator_min_pts",
                             m_param_.m_normals_computator_min_pts, 5);
      system_utils::getParam(config, "normals_computator_radius",
                             m_param_.m_normals_computator_radius, 0.1f);
    }
    else
    {
      m_param_.m_enable_normals = false;
      m_param_.m_normals_computator_min_pts = -1;
      m_param_.m_normals_computator_radius = -1.f;
    }

    system_utils::getParam(config, "range_min", m_param_.m_range_min, -1.f);
    system_utils::getParam(config, "range_max", m_param_.m_range_max, -1.f);

    // ----- POSE GRAPH TRIGGERING CRITERIA ------------------------------------

    std::string graph_criteria_type;

    system_utils::getParam(config, "graph_criteria_type", graph_criteria_type);

    if (graph_criteria_type.compare("rotation") == 0)
    {
      CriteriaRotationTypePtr criteria =
          std::make_shared<CriteriaRotationType>();

      system_utils::getParam(config, "graph_criteria_min_travel_heading",
                             criteria->m_min_travel_heading, 15.0f);

      criteria->m_min_travel_heading =
          geometry::degToRad(criteria->m_min_travel_heading);

      m_param_.m_graph_criteria =
          std::dynamic_pointer_cast<CriteriaType>(criteria);
    }
    else if (graph_criteria_type.compare("translation") == 0)
    {
      CriteriaTranslationTypePtr criteria =
          std::make_shared<CriteriaTranslationType>();

      system_utils::getParam(config, "graph_criteria_min_travel_dist",
                             criteria->m_min_travel_dist, 0.25f);

      m_param_.m_graph_criteria =
          std::dynamic_pointer_cast<CriteriaType>(criteria);
    }
    else if (graph_criteria_type.compare("viewpoint") == 0)
    {
      CriteriaViewpointTypePtr criteria =
          std::make_shared<CriteriaViewpointType>();

      system_utils::getParam(config, "graph_criteria_min_travel_dist",
                             criteria->m_min_travel_heading, 15.0f);
      system_utils::getParam(config, "graph_criteria_min_travel_heading",
                             criteria->m_min_travel_dist, 0.25f);

      criteria->m_min_travel_heading =
          geometry::degToRad(criteria->m_min_travel_heading);

      m_param_.m_graph_criteria =
          std::dynamic_pointer_cast<CriteriaType>(criteria);
    }
    else if (graph_criteria_type.compare("gridcell") == 0)
    {
      CriteriaGridCellTypePtr criteria =
          std::make_shared<CriteriaGridCellType>();

      system_utils::getParam(config, "graph_criteria_cell_size",
                             criteria->m_cell_size_2, 2.5f);

      criteria->m_cell_size_2 *= 0.5f;

      m_param_.m_graph_criteria =
          std::dynamic_pointer_cast<CriteriaType>(criteria);
    }
    else if (graph_criteria_type.compare("time") == 0)
    {
      CriteriaTimeTypePtr criteria = std::make_shared<CriteriaTimeType>();

      system_utils::getParam(config, "graph_criteria_min_time_interval",
                             criteria->m_min_time_interval, 5.0);
      system_utils::getParam(config, "graph_criteria_max_ratio_inliers",
                             criteria->m_max_ratio_inliers, 0.60f);

      m_param_.m_graph_criteria =
          std::dynamic_pointer_cast<CriteriaType>(criteria);
    }
    else if (graph_criteria_type.compare("inliers") == 0)
    {
      CriteriaInliersRatioTypePtr criteria =
          std::make_shared<CriteriaInliersRatioType>();

      system_utils::getParam(config, "graph_criteria_min_travel_dist",
                             criteria->m_min_travel_dist, 0.10f);
      system_utils::getParam(config, "graph_criteria_min_travel_heading",
                             criteria->m_min_travel_heading, 5.0f);
      system_utils::getParam(config, "graph_criteria_max_ratio_inliers",
                             criteria->m_max_ratio_inliers, 0.60f);
      system_utils::getParam(config, "graph_criteria_max_travel_dist",
                             criteria->m_max_travel_dist, -1.0f);
      system_utils::getParam(config, "graph_criteria_max_travel_heading",
                             criteria->m_max_travel_heading, -1.0f);

      criteria->m_min_travel_heading =
          geometry::degToRad(criteria->m_min_travel_heading);
      criteria->m_max_travel_heading =
          geometry::degToRad(criteria->m_max_travel_heading);

      m_param_.m_graph_criteria =
          std::dynamic_pointer_cast<CriteriaType>(criteria);
    }
    else
    {
      throw std::runtime_error(
          "OgbaAppROS1::loadConfig | unknown or invalid graph_criteria_type "
          "value: " +
          graph_criteria_type +
          " (rotation|translation|viewpoint|inliers|time|gridcell)");
    }

    // ----- CO-VISIBILITY GRAPH COMPUTATION -----------------------------------
    system_utils::getParam(config, "covisibility_ratio_inliers",
                           m_param_.m_covisibility_ratio_inliers, 0.6f);
    system_utils::getParam(config, "covisibility_nn_distance",
                           m_param_.m_covisibility_nn_distance, 0.1f);
    system_utils::getParam(config, "covisibility_max_distance",
                           m_param_.m_covisibility_max_distance, 5.0f);
    system_utils::getParam(config, "covisibility_dbg_ogl_interval",
                           m_param_.m_covisibility_dbg_ogl_interval,
                           int64_t(0));

    // ----- GLOBAL MERGER (DBG PURPOSES) --------------------------------------
    MergerType::Param merger_param;

    system_utils::getParam(config, "merger_ray_min_dist_through",
                           merger_param.m_ray_min_dist_through, 0.20f);
    system_utils::getParam(config, "merger_ray_min_dist_add_new",
                           merger_param.m_ray_min_dist_add_new, 0.20f);
    system_utils::getParam(config, "merger_enable_avg",
                           merger_param.m_enable_avg, false);

    if (merger_param.m_enable_avg)
    {
      if (!system_utils::hasParam(config, "merger_weight_old") ||
          !system_utils::hasParam(config, "merger_weight_new"))
      {
        throw std::runtime_error(
            "OgbaAppROS1::loadConfig | "
            "merger_weight_old and merger_weight_new must be defined if "
            "merger_enable_avg is enabled...");
      }

      system_utils::getParam(config, "merger_weight_old",
                             merger_param.m_weight_old);
      system_utils::getParam(config, "merger_weight_new",
                             merger_param.m_weight_new);
    }
    else
    {
      merger_param.m_weight_old = -1.f;
      merger_param.m_weight_new = -1.f;
    }

    m_param_.m_global_merger = std::make_shared<MergerType>(merger_param);
  }
  catch (const YAML::ParserException& e)
  {
    throw std::runtime_error(
        "OgbaAppROS1::loadOGBAConfig | YAML configuration file for OGBA "
        "malformed");
  }
  catch (const YAML::Exception& e)
  {
    throw std::runtime_error(
        "OgbaAppROS1::loadOGBAConfig | unexpected error when processing the "
        "YAML file (error: " +
        std::string(e.what()) + ")");
  }
  catch (const std::exception& e)
  {
    throw std::runtime_error(
        "OgbaAppROS1::loadOGBAConfig | ...unexpected error... (error: " +
        std::string(e.what()) + ")");
  }
  catch (...)
  {
    throw std::runtime_error(
        "OgbaAppROS1::loadOGBAConfig | ...unexpected error...");
  }
}

void OgbaAppROS1::assertInit(const Param& param)
{
  if (param.m_filenames_bags.empty())
  {
    throw std::runtime_error(
        "OgbaAppROS1::assertInit | empty filenames vector of ROS 1 bags");
  }
  for (const std::string& filename_bag : param.m_filenames_bags)
  {
    try
    {
      std::filesystem::path path(filename_bag);

      if (std::filesystem::exists(path))
      {
        if (!std::filesystem::is_regular_file(path))
        {
          throw std::runtime_error(
              "OgbaAppROS1::assertInit | bag filename path '" + filename_bag +
              "' is not even a file!");
        }
      }
      else
      {
        throw std::runtime_error(
            "OgbaAppROS1::assertInit | bag filename path '" + filename_bag +
            "' does not exist!");
      }

      if (path.extension().string().compare(".bag"))
      {
        throw std::runtime_error(
            "OgbaAppROS1::assertInit | extension of the bag filename path '" +
            filename_bag + "' must be a ROS 1 bagfile (aka '.bag')!");
      }
    }
    catch (std::filesystem::filesystem_error& e)
    {
      throw std::runtime_error(
          "OgbaAppROS1::assertInit | something happened when checking if the "
          "bags "
          "filename paths exist (filename: " +
          filename_bag + "; error: " + e.what() + ")");
    }
    catch (const std::exception& e)
    {
      throw std::runtime_error(
          "OgbaAppROS1::assertInit | something unexpected happened when "
          "checking "
          "if the bags filename paths exist (filename: " +
          filename_bag + "; error: " + e.what() + ")");
    }
  }
  try
  {
    std::filesystem::path path(param.m_filename_log);

    if (std::filesystem::exists(path))
    {
      if (!std::filesystem::is_regular_file(path))
      {
        throw std::runtime_error(
            "OgbaAppROS1::assertInit | log filename path '" +
            param.m_filename_log + "' is not even a file!");
      }
    }
    else
    {
      throw std::runtime_error("OgbaAppROS1::assertInit | log filename path '" +
                               param.m_filename_log + "' does not exist!");
    }
  }
  catch (std::filesystem::filesystem_error& e)
  {
    throw std::runtime_error(
        "OgbaAppROS1::assertInit | something happened when checking if the log "
        "filename path exist (filename: " +
        param.m_filename_log + "; error: " + e.what() + ")");
  }
  catch (const std::exception& e)
  {
    throw std::runtime_error(
        "OgbaAppROS1::assertInit | something unexpected happened when checking "
        "if the log filename path exist (filename: " +
        param.m_filename_log + "; error: " + e.what() + ")");
  }
  if (param.m_scan_topic.empty())
  {
    throw std::runtime_error(
        "OgbaAppROS1::assertInit | empty scan topic string");
  }
  if (param.m_sensor_in_robot_val.size() != 3)
  {
    throw std::runtime_error(
        "OgbaAppROS1::assertInit | sensor pose w.r.t. robot must have only "
        "three "
        "values (m m deg) or do not set that option to assume an identity "
        "isometry");
  }
  if (param.m_time_start < 0)
  {
    throw std::runtime_error(
        "OgbaAppROS1::assertInit | start time must be equal or greater than "
        "0s");
  }
  if (param.m_has_duration && param.m_time_duration <= 0.0)
  {
    throw std::runtime_error(
        "OgbaAppROS1::assertInit | when duration is enabled, it must be "
        "greater "
        "than 0s");
  }
  try
  {
    std::filesystem::path path(param.m_ogba_yaml);

    if (std::filesystem::exists(path))
    {
      if (!std::filesystem::is_regular_file(path))
      {
        throw std::runtime_error(
            "OgbaAppROS1::assertInit | YAML filename path '" +
            param.m_ogba_yaml + "' is not even a file!");
      }
    }
    else
    {
      throw std::runtime_error(
          "OgbaAppROS1::assertInit | YAML filename path '" + param.m_ogba_yaml +
          "' does not exist!");
    }

    if (path.extension().string().compare(".yaml"))
    {
      throw std::runtime_error(
          "OgbaAppROS1::assertInit | extension of the YAML filename path '" +
          param.m_ogba_yaml + "' must be YAML (dumb question!)!");
    }
  }
  catch (std::filesystem::filesystem_error& e)
  {
    throw std::runtime_error(
        "OgbaAppROS1::assertInit | something happened when checking if the "
        "YAML "
        "filename path exist (filename: " +
        param.m_ogba_yaml + "; error: " + e.what() + ")");
  }
  catch (const std::exception& e)
  {
    throw std::runtime_error(
        "OgbaAppROS1::assertInit | something unexpected happened when checking "
        "if the YAML filename path exist (filename: " +
        param.m_ogba_yaml + "; error: " + e.what() + ")");
  }
}

void OgbaAppROS1::getPoses()
{
  GlobalMapAlignerType::Param global_aligner_param;

  global_aligner_param.m_debug = m_param_.m_ogba_debug;
  global_aligner_param.m_solver_verbose = false;
  global_aligner_param.m_solver_damping = 1.f;
  global_aligner_param.m_solver_max_iterations = 1;
  global_aligner_param.m_solver_criteria_type = "";
  global_aligner_param.m_solver_criteria_epsilon = 0.f;
  global_aligner_param.m_solver_robustifier_type = "";
  global_aligner_param.m_solver_robustifier_num_iterations_coarse = 0;
  global_aligner_param.m_solver_robustifier_threshold_coarse = -1.f;
  global_aligner_param.m_solver_robustifier_threshold_fine = -1.f;
  global_aligner_param.m_failure_ratio_inliers = -1.f;
  global_aligner_param.m_failure_chi_inliers = -1.f;
  global_aligner_param.m_dbg_ogl_interval = 0;
  global_aligner_param.m_dbg_ogl_mode = m_param_.m_ogba_solver_dbg_ogl_mode;
  global_aligner_param.m_dbg_ogl_show_correspondences = false;
  global_aligner_param.m_dbg_ogl_show_normals = false;

  GlobalMapAlignerType::DistMapParam global_aligner_param_distmap;

  global_aligner_param_distmap.m_dist_map_res = m_param_.m_dist_map_res;
  global_aligner_param_distmap.m_dist_map_size = -1.f;
  global_aligner_param_distmap.m_dist_map_max_dist =
      m_param_.m_covisibility_nn_distance;
  global_aligner_param_distmap.m_solver_regularize_correspondences = false;

  m_global_map_ = std::make_shared<PointCloudType>();
  m_global_aligner_ = std::make_shared<GlobalMapAlignerType>(
      global_aligner_param, global_aligner_param_distmap);

  m_global_aligner_->setupDbgOGL(m_dbg_ogl_qapp_, m_dbg_ogl_viewer_,
                                 m_dbg_ogl_canvas_);

  GlobalMapAlignerType::Stats stats;

  stats.m_inliers_ratio = 1.f;

  m_graph_ = std::make_shared<FactorGraphType>();

  double msg_stamp_prev = -1;

  int64_t graph_id = 0;

  for (rosbag::MessageInstance const& msg : *m_bags_view_)
  {
    sensor_msgs::LaserScanPtr rosmsg =
        msg.instantiate<sensor_msgs::LaserScan>();

    if (rosmsg)
    {
      auto it = std::lower_bound(m_log_data_.begin(), m_log_data_.end(),
                                 rosmsg->header.stamp.toSec(),
                                 [](const TUMPoseType& pose, double timestamp)
                                 { return pose.m_t < timestamp; });

      if (it != m_log_data_.end())
      {
        if (it->m_t == rosmsg->header.stamp.toSec())
        {
          LaserScanMsgTypePtr scan = std::make_shared<LaserScanMsgType>(
              inesctec_mrdt_slam_distmap_2d::slam::msg::Header(
                  rosmsg->header.stamp.toSec(), rosmsg->header.frame_id),
              rosmsg->angle_min, rosmsg->angle_max, rosmsg->angle_increment,
              rosmsg->time_increment, rosmsg->scan_time, rosmsg->range_min,
              rosmsg->range_max, rosmsg->ranges, rosmsg->intensities);

          PointCloudTypePtr pc = std::make_shared<PointCloudType>();

          EstimateType initial_pose = geometry::v2t(
              Eigen::Vector3f(static_cast<float>(it->m_pos.x()),
                              static_cast<float>(it->m_pos.y()),
                              static_cast<float>(geometry::q2D2a(it->m_q))));

          preprocessScan(*scan, *pc);

          if (msg_stamp_prev < 0)
          {
            m_global_map_ = std::make_shared<PointCloudType>();

            VariableType* const v = addVariable<VariableType>(
                graph_id++, VariableType::Status::Fixed, initial_pose,
                m_sensor_in_robot_t_, scan, pc);

            *m_global_map_ =
                pc->transform(v->getSensorInRobot().inverse() * v->estimate() *
                              v->getSensorInRobot());

            if (dynamic_cast<CriteriaInliersRatioType*>(
                    m_param_.m_graph_criteria.get()))
            {
              m_global_aligner_->setTarget(m_global_map_, m_sensor_in_robot_t_);
            }

            msg_stamp_prev = rosmsg->header.stamp.toSec();

            m_param_.m_graph_criteria->reset(*scan, initial_pose);

            continue;
          }

          double msg_stamp_curr = rosmsg->header.stamp.toSec();

          if (msg_stamp_curr <= msg_stamp_prev)
          {
            OFFLERROR(
                "causality assumption broken! you are looking into the past... "
                "(t_curr: %lf vs t_prev: %lf)",
                msg_stamp_curr, msg_stamp_prev);

            continue;
          }

          msg_stamp_prev = msg_stamp_curr;

          if (dynamic_cast<CriteriaInliersRatioType*>(
                  m_param_.m_graph_criteria.get()))
          {
            if (m_param_.m_ogba_debug)
            {
              for (auto variable : m_graph_->variables())
              {
                const VariableBaseType* v =
                    static_cast<const VariableBaseType*>(variable.second);

                m_dbg_ogl_canvas_->pushColor();
                m_dbg_ogl_canvas_->setColor(
                    srrg2_core::ColorPalette::color4fDarkRed(0.5));
                m_dbg_ogl_canvas_->pushMatrix();
                m_dbg_ogl_canvas_->multMatrix(
                    srrg2_core::geometry3d::get3dFrom2dPose(v->estimate())
                        .matrix());
                m_dbg_ogl_canvas_->putSphere(0.05);
                m_dbg_ogl_canvas_->popMatrix();
                m_dbg_ogl_canvas_->popAttribute();
              }
            }

            m_global_aligner_->setSource(pc, m_sensor_in_robot_t_);
            m_global_aligner_->setSourceInTarget(initial_pose);
            m_global_aligner_->setPriorEnabled(false);
            m_global_aligner_->computeStatsOnly();

            stats = m_global_aligner_->getStats();
          }
          else
          {
            if (m_param_.m_ogba_debug)
            {
              updateDbgOPGLCanvas();
            }
          }

          m_param_.m_graph_criteria->compute(*scan, initial_pose,
                                             stats.m_inliers_ratio);

          if (m_param_.m_graph_criteria->isToTrigger())
          {
            m_param_.m_global_merger->merge(*pc, *scan, m_sensor_in_robot_t_,
                                            initial_pose, m_global_map_.get());

            if (dynamic_cast<CriteriaInliersRatioType*>(
                    m_param_.m_graph_criteria.get()))
            {
              m_global_aligner_->setTarget(m_global_map_, m_sensor_in_robot_t_);
            }

            msg_stamp_prev = rosmsg->header.stamp.toSec();

            m_param_.m_graph_criteria->reset(*scan, initial_pose);

            addVariable<VariableType>(graph_id++, VariableType::Status::Active,
                                      initial_pose, m_sensor_in_robot_t_, scan,
                                      pc);

            if (graph_id % 10 == 0)
            {
              OFFLINFO("...processing... (#poses: %ld)", graph_id);
            }
          }
        }
        else
        {
          std::cout << "timestamps mismatch failure... "
                       "continuing to next message..."
                    << std::endl;
        }
      }
    }
  }

  std::cout << std::endl
            << "#poses to bundle: " << m_graph_->variables().size() << std::endl
            << "#global map pts : " << m_global_map_->size() << std::endl;

  m_global_map_->voxelize(m_param_.m_dist_map_res);

  std::cout << "#global map pts : " << m_global_map_->size() << " (voxelized)"
            << std::endl
            << std::endl;

  m_global_aligner_.reset();

  if (m_param_.m_ogba_debug)
  {
    updateDbgOPGLCanvas();
  }
}

void OgbaAppROS1::computeCovisibilityGraph()
{
  CovisibilityAlignerType::Param covisibility_aligner_param;

  covisibility_aligner_param.m_debug = false;
  covisibility_aligner_param.m_solver_verbose = false;
  covisibility_aligner_param.m_solver_damping = 1.f;
  covisibility_aligner_param.m_solver_max_iterations = 1;
  covisibility_aligner_param.m_solver_criteria_type = "";
  covisibility_aligner_param.m_solver_criteria_epsilon = 0.f;
  covisibility_aligner_param.m_solver_robustifier_type = "";
  covisibility_aligner_param.m_solver_robustifier_num_iterations_coarse = 0;
  covisibility_aligner_param.m_solver_robustifier_threshold_coarse = -1.f;
  covisibility_aligner_param.m_solver_robustifier_threshold_fine = -1.f;
  covisibility_aligner_param.m_failure_ratio_inliers = -1.f;
  covisibility_aligner_param.m_failure_chi_inliers = -1.f;
  covisibility_aligner_param.m_dbg_ogl_interval = 0;
  covisibility_aligner_param.m_dbg_ogl_mode = "";
  covisibility_aligner_param.m_dbg_ogl_show_correspondences = false;
  covisibility_aligner_param.m_dbg_ogl_show_normals = false;

  CovisibilityAlignerType::DistMapParam covisibility_aligner_param_distmap;

  covisibility_aligner_param_distmap.m_dist_map_res = m_param_.m_dist_map_res;
  covisibility_aligner_param_distmap.m_dist_map_size = -1.f;
  covisibility_aligner_param_distmap.m_dist_map_max_dist =
      m_param_.m_covisibility_nn_distance;
  covisibility_aligner_param_distmap.m_solver_regularize_correspondences =
      false;

  CovisibilityAlignerTypePtr covisibility_aligner =
      std::make_shared<CovisibilityAlignerType>(
          covisibility_aligner_param, covisibility_aligner_param_distmap);

  for (srrg2_core::AbstractMapView_<srrg2_solver::VariableBase::Id,
                                    srrg2_solver::VariableBase*>::iterator
           v1_it = m_graph_->variables().begin();
       v1_it != m_graph_->variables().end(); ++v1_it)
  {
    VariableType* const v1 = dynamic_cast<VariableType*>(v1_it.value());

    if (v1->graphId() % 10 == 0)
    {
      OFFLINFO("...%ld / %ld... LET'S GO!!!... MOVE YOUR ASS BITCH!!!",
               v1->graphId(), m_graph_->variables().size());
    }

    covisibility_aligner->setTarget(v1);

    for (srrg2_core::AbstractMapView_<srrg2_solver::VariableBase::Id,
                                      srrg2_solver::VariableBase*>::iterator
             v2_it = v1_it;
         v2_it != m_graph_->variables().end(); ++v2_it)
    {
      VariableType* const v2 = dynamic_cast<VariableType*>(v2_it.value());

      if (v1->graphId() == v2->graphId())
      {
        continue;
      }

      const Eigen::Vector3f delta =
          geometry::t2v(v1->estimate().inverse() * v2->estimate());

      float delta_dist = delta.head(2).norm();

      if (delta_dist > m_param_.m_covisibility_max_distance)
      {
        continue;
      }

      covisibility_aligner->setSource(v2);
      covisibility_aligner->setSourceInTarget(v1->estimate().inverse() *
                                              v2->estimate());

      covisibility_aligner->setPriorEnabled(false);
      covisibility_aligner->computeStatsOnly();

      auto stats = covisibility_aligner->getStats();

      if (stats.m_inliers_ratio > m_param_.m_covisibility_ratio_inliers)
      {
        FactorTypePtr f = std::make_shared<FactorType>();

        f->setVariableId(0, v2->graphId());
        f->setVariableId(1, v1->graphId());
        f->setInformationMatrix(FactorType::InformationMatrixType::Identity());
        f->setSensorInRobot(m_sensor_in_robot_t_);
        f->setMoving(*v2->measurement()->getPointCloud());

        m_graph_->addFactor(f);
      }

      if (m_param_.m_ogba_debug)
      {
        updateDbgOPGLCanvas();

        if (m_param_.m_covisibility_dbg_ogl_interval > 0)
        {
          std::this_thread::sleep_for(std::chrono::milliseconds(
              m_param_.m_covisibility_dbg_ogl_interval));
        }
      }
    }
  }

  OFFLINFO("#graph variables: %ld", m_graph_->variables().size());
  OFFLINFO("#graph edges    : %ld\n", m_graph_->factors().size());

  if (m_param_.m_ogba_debug)
  {
    updateDbgOPGLCanvas();
  }
}

void OgbaAppROS1::compute()
{
  m_solver_.setGraph(m_graph_);
  m_solver_.param_verbose.setValue(m_param_.m_ogba_solver_verbose);
  m_solver_.param_termination_criteria.setValue(nullptr);
  m_solver_.param_max_iterations.setValue(
      {m_param_.m_ogba_solver_max_iterations});

  m_solver_.param_robustifier_policies.pushBack(
      std::make_shared<srrg2_solver::RobustifierPolicyByType>());

  std::shared_ptr<srrg2_solver::RobustifierCauchy> cauchy_robustifier =
      std::make_shared<srrg2_solver::RobustifierCauchy>();

  cauchy_robustifier->setName("cauchy");
  cauchy_robustifier->param_chi_threshold.setValue(
      m_param_.m_ogba_solver_robustifier_threshold);

  m_solver_.param_robustifier_policies.value(0)->param_robustifier.setValue(
      cauchy_robustifier);

  m_solver_.param_actions.pushBack(
      std::make_shared<srrg2_solver::SolverVerboseAction>());

  srrg2_solver::SolverGenericAtIterationEndActionPtr view_graph_dynamic =
      std::make_shared<srrg2_solver::SolverGenericAtIterationEndAction>();

  auto ogba = this;

  view_graph_dynamic->m_action = [&ogba]()
  {
    ogba->updateDbgOPGLCanvas();
    std::cout << "...next update..." << std::endl;
  };

  m_solver_.param_actions.pushBack(view_graph_dynamic);

  m_solver_.param_algorithm.setValue(
      std::make_shared<srrg2_solver::IterationAlgorithmGN>());
  dynamic_cast<srrg2_solver::IterationAlgorithmGN*>(
      m_solver_.param_algorithm.value().get())
      ->param_damping.setValue(1.0f);

  m_solver_.param_linear_solver.setValue(
      std::make_shared<srrg2_solver::SparseBlockLinearSolverCholeskyCholmod>());

  OFFLINFO("...original graph...");

  updateDbgOPGLCanvas();

  OFFLINFO(
      "solver compute global bundle adjustment graph optimization "
      "(gba)...");
  m_solver_.compute();
  OFFLINFO("FINISHED!");
}

void OgbaAppROS1::setupDbgOGL(const OGLAppTypePtr& qapp,
                              const OGLViewerTypePtr& viewer,
                              const OGLCanvasTypePtr& canvas)
{
  if (!m_param_.m_ogba_debug)
  {
    throw std::runtime_error(
        "OgbaAppROS1::setupDbgOGL | "
        "debug mode not enabled (only w/ dbg enabled the OGL viewer can be "
        "setup)");
  }
  if (!qapp || !viewer)
  {
    throw std::runtime_error(
        "OgbaAppROS1::setupDbgOGL | "
        "QtApplication and OpenGL shared viewer shared pointers must be "
        "initialized");
  }

  m_dbg_ogl_qapp_ = qapp;
  m_dbg_ogl_viewer_ = viewer;

  if (!canvas)
  {
    m_dbg_ogl_canvas_ = m_dbg_ogl_viewer_->getCanvas("global map");
  }
  else
  {
    m_dbg_ogl_canvas_ = canvas;
    m_dbg_ogl_canvas_name_ = canvas->name();
  }

  m_dbg_ogl_canvas_->_setup();
}

void OgbaAppROS1::updateDbgOPGLCanvas()
{
  if (!m_param_.m_ogba_debug)
  {
    throw std::runtime_error(
        "OgbaAppROS1::updateDbgOPGLCanvas | "
        "debug mode not enabled (only w/ dbg enabled the OGL viewer can be "
        "setup)");
  }
  if ((!m_dbg_ogl_qapp_) || (!m_dbg_ogl_viewer_) || (!m_dbg_ogl_canvas_))
  {
    throw std::runtime_error(
        "OgbaAppROS1::updateDbgOPGLCanvas | "
        "missing setup of the OpenGL-based debug visualization");
  }

  for (auto variable : m_graph_->variables())
  {
    /* const VariableType* v = static_cast<const
    VariableType*>(variable.second);

    m_dbg_ogl_canvas_->pushColor();
    m_dbg_ogl_canvas_->setColor(srrg2_core::ColorPalette::color4fDarkRed(0.5));
    m_dbg_ogl_canvas_->pushMatrix();
    m_dbg_ogl_canvas_->multMatrix(
        srrg2_core::geometry3d::get3dFrom2dPose(v->estimate()).matrix());
    m_dbg_ogl_canvas_->putSphere(0.05);
    m_dbg_ogl_canvas_->popMatrix();
    m_dbg_ogl_canvas_->popAttribute(); */

    const VariableType* v = static_cast<const VariableType*>(variable.second);

    v->_drawImpl(m_dbg_ogl_canvas_);
  }

  for (auto fact : m_graph_->factors())
  {
    const FactorType* f = static_cast<const FactorType*>(fact.second);

    srrg2_core::Vector3f segments[2];

    const VariableType* v_from =
        static_cast<const VariableType*>(f->variable(0));
    const VariableType* v_to = static_cast<const VariableType*>(f->variable(1));

    if (!v_from || !v_to)
    {
      continue;
    }

    const EstimateType& from = v_from->estimate();
    const EstimateType& to = v_to->estimate();

    segments[0] << from.translation().x(), from.translation().y(), 0.f;
    segments[1] << to.translation().x(), to.translation().y(), 0.f;

    m_dbg_ogl_canvas_->putSegment(2, segments, 0);
  }

  m_dbg_ogl_canvas_->flush();

  /* if (!m_global_map_)
  {
    m_dbg_ogl_canvas_->flush();
    return;
  }

  m_dbg_ogl_canvas_->pushColor();
  m_dbg_ogl_canvas_->setColor(srrg2_core::ColorPalette::color3fBlack());
  m_dbg_ogl_canvas_->pushMatrix();
  m_dbg_ogl_canvas_->multMatrix(
      srrg2_core::geometry3d::get3dFrom2dPose(m_v_fixed_->estimate()).matrix());

  srrg2_core::Point3fVectorCloud pts;

  pts.resize(m_global_map_->size());

  int idx = 0;

  for (const auto& pt : *m_global_map_)
  {
    if (pt.m_status != inesctec_mrdt_slam_distmap_2d::pcl::PointStatus::kValid)
    {
      continue;
    }

    pts[idx].coordinates().x() = pt.coordinates().x();
    pts[idx].coordinates().y() = pt.coordinates().y();
    pts[idx].coordinates().z() = 0;

    ++idx;
  }

  pts.resize(idx);

  m_dbg_ogl_canvas_->putPoints(pts);

  m_dbg_ogl_canvas_->popMatrix();
  m_dbg_ogl_canvas_->popAttribute();

  m_dbg_ogl_canvas_->flush(); */
}

}  // namespace inesctec_mrdt_slam_distmap_2d_ros
