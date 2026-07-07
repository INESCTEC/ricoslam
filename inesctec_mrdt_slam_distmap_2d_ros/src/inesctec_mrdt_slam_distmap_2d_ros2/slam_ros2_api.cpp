#include "inesctec_mrdt_slam_distmap_2d_ros/slam_ros2_api.hpp"

#include <exception>
#include <filesystem>
#include <string>

// ROS
#if defined(ROS_DISTRO_foxy)
#include <tf2/exceptions.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/create_timer_ros.h>
#else
#include <tf2/exceptions.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/create_timer_ros.hpp>
#endif

// SLAM
#include <inesctec_mrdt_slam_distmap_2d/data_structures/instances.hpp>
#include <inesctec_mrdt_slam_distmap_2d/geometry/geometry_2d.hpp>
#include <inesctec_mrdt_slam_distmap_2d/srrg2_solver/instances.hpp>
#include <inesctec_mrdt_slam_distmap_2d/system_utils/filesystem.hpp>

namespace inesctec_mrdt_slam_distmap_2d_ros
{

using namespace inesctec_mrdt_slam_distmap_2d;

SLAMROS2API::SLAMROS2API(const std::string& name,
                         const rclcpp::NodeOptions& options)
    : rclcpp::Node(name, options), m_is_first_msg_(true)
{
  // Register types for BOSS serialization
  srrg2_core::point_cloud_registerTypes();
  srrg2_solver::inesctec_mrdt_slam_distmap_2d_data_structures_registerTypes();
  srrg2_solver::inesctec_mrdt_slam_distmap_2d_srrg2_solver_registerTypes();

  m_tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  m_tf2_buffer_->setCreateTimerInterface(
      std::make_shared<tf2_ros::CreateTimerROS>(
          this->get_node_base_interface(), this->get_node_timers_interface()));

  readParam();
  printParam();

  m_slam_ = std::make_shared<SLAMType>(m_param_.m_slam_param);

  std::ostringstream param_print("");
  param_print << m_slam_->getParam();

  RCLCPP_INFO(this->get_logger(),
              "[%s][SLAM] ...checking tracker loaded parametrisation:...\n%s",
              this->get_name(), param_print.str().c_str());

  if (m_slam_->getTracker()->getMergingCriteriaPtr())
  {
    m_srv_trigger_merging_ = this->create_service<std_srvs::srv::Trigger>(
        "~/trigger_merging",
        std::bind(&SLAMROS2API::srvTriggerMergingCriteria, this,
                  std::placeholders::_1, std::placeholders::_2));
  }

  if (m_slam_->getTracker()->getSplittingCriteriaPtr())
  {
    m_srv_trigger_splitting_ = this->create_service<std_srvs::srv::Trigger>(
        "~/trigger_splitting",
        std::bind(&SLAMROS2API::srvTriggerSplittingCriteria, this,
                  std::placeholders::_1, std::placeholders::_2));
  }

  m_srv_save_graph_ =
      this->create_service<inesctec_mrdt_slam_distmap_2d_ros::srv::SetFilename>(
          "~/save_graph",
          std::bind(&SLAMROS2API::srvSaveGraph, this, std::placeholders::_1,
                    std::placeholders::_2));
  m_srv_save_corrected_pose_ =
      this->create_service<inesctec_mrdt_slam_distmap_2d_ros::srv::SetFilename>(
          "~/save_corrected_pose",
          std::bind(&SLAMROS2API::srvSaveCorrectedPose, this,
                    std::placeholders::_1, std::placeholders::_2));
}

void SLAMROS2API::readParam()
{
  if (!this->has_parameter("base_frame"))
  {
    throw std::runtime_error("SLAMROS2API::readParam | base_frame missing...");
  }

  m_param_.m_base_frame =
      this->get_parameter("base_frame").get_value<std::string>();

  // ----- EXTERNAL ODOMETRY ---------------------------------------------------

  this->get_parameter_or<bool>("ext_odom_use", m_param_.m_ext_odom_use, false);

  if (m_param_.m_ext_odom_use)
  {
    this->get_parameter_or<std::string>("ext_odom_impl",
                                        m_param_.m_ext_odom_impl, "topic");

    if ((m_param_.m_ext_odom_impl.compare("topic") != 0) &&
        (m_param_.m_ext_odom_impl.compare("tf") != 0))
    {
      throw std::runtime_error(
          "SLAMROS2API::readParam | unknown ext_odom_impl value: " +
          m_param_.m_ext_odom_impl + " (topic|tf)");
    }
    if ((m_param_.m_ext_odom_impl.compare("tf") == 0) &&
        !(this->has_parameter("ext_odom_frame")))
    {
      throw std::runtime_error(
          "SLAMROS2API::readParam | ext_odom_frame missing when "
          "ext_odom_impl set to tf mode");
    }

    this->get_parameter_or<std::string>("ext_odom_frame",
                                        m_param_.m_ext_odom_frame, "");
    this->get_parameter_or<bool>("ext_odom_pub_tf", m_param_.m_ext_odom_pub_tf,
                                 false);
    this->get_parameter_or<bool>("ext_odom_pub_msg",
                                 m_param_.m_ext_odom_pub_msg, false);

    if (m_param_.m_ext_odom_pub_tf)
    {
      this->get_parameter_or<bool>("ext_odom_invert_tf",
                                   m_param_.m_ext_odom_invert_tf, false);
    }
    else
    {
      m_param_.m_ext_odom_invert_tf = false;
    }

    if ((m_param_.m_ext_odom_impl.compare("tf") == 0) &&
        m_param_.m_ext_odom_pub_tf)
    {
      throw std::runtime_error(
          "SLAMROS2API::readParam | ext_odom_impl TF implementation and "
          "ext_odom_pub_tf both enabled does not make any sense!");
    }
    if ((m_param_.m_ext_odom_impl.compare("topic") == 0) &&
        m_param_.m_ext_odom_pub_msg)
    {
      throw std::runtime_error(
          "SLAMROS2API::readParam | ext_odom_impl topic implementation and "
          "ext_odom_pub_msg both enabled does not make any sense!");
    }
  }
  else
  {
    m_param_.m_ext_odom_impl = "";
    m_param_.m_ext_odom_frame = "";
    m_param_.m_ext_odom_pub_tf = false;
    m_param_.m_ext_odom_pub_msg = false;
    m_param_.m_ext_odom_invert_tf = false;
  }

  // ----- LASER SENSOR --------------------------------------------------------

  this->get_parameter_or<bool>("laser_pub_tf_static",
                               m_param_.m_laser_pub_tf_static, false);

  if (this->has_parameter("laser_pose_x") ||
      this->has_parameter("laser_pose_y") ||
      this->has_parameter("laser_pose_th"))
  {
    if (!(this->has_parameter("laser_pose_x") &&
          this->has_parameter("laser_pose_y") &&
          this->has_parameter("laser_pose_th")))
    {
      throw std::runtime_error(
          "SLAMROS2API::readParam | pre-defined laser pose w.r.t. robot "
          "must be defined completely (laser_pose_x & laser_pose_y & "
          "laser_pose_th)");
    }
    if (!this->has_parameter("laser_frame") && m_param_.m_laser_pub_tf_static)
    {
      throw std::runtime_error(
          "SLAMROS2API::readParam | pre-defined laser pose w.r.t. robot "
          "defined but no laser_frame defined for the TF system (when laser "
          "pose set, tracker publishes the laser odometry w.r.t. sensor, not "
          "the robot base frame)");
    }

    m_param_.m_laser_tf_static_yaml = true;

    m_param_.m_laser_pose.x() =
        this->get_parameter("laser_pose_x").get_value<float>();
    m_param_.m_laser_pose.y() =
        this->get_parameter("laser_pose_y").get_value<float>();
    m_param_.m_laser_pose.z() =
        this->get_parameter("laser_pose_th").get_value<float>();

    m_param_.m_laser_pose.z() = geometry::degToRad(m_param_.m_laser_pose.z());

    this->get_parameter_or<std::string>("laser_frame", m_param_.m_laser_frame,
                                        "");
  }
  else
  {
    m_param_.m_laser_tf_static_yaml = false;
    m_param_.m_laser_pose.setZero();
    m_param_.m_laser_frame = "";
  }

  if (m_param_.m_laser_pub_tf_static && !m_param_.m_laser_tf_static_yaml)
  {
    throw std::runtime_error(
        "SLAMROS2API::readParam | pre-defined laser pose w.r.t. robot must "
        "be defined completely (laser_pose_x & laser_pose_y & laser_pose_th) "
        "when laser_pub_tf_static enabled");
  }

  // ----- SLAM ROS API --------------------------------------------------------

  this->get_parameter_or<bool>("slam_pub_pose_tf", m_param_.m_slam_pub_pose_tf,
                               true);
  this->get_parameter_or<bool>("slam_pub_pose_msg",
                               m_param_.m_slam_pub_pose_msg, false);
  this->get_parameter_or<std::string>("slam_pose_frame",
                                      m_param_.m_slam_pose_frame, "map");

  if (m_param_.m_slam_pub_pose_tf)
  {
    this->get_parameter_or<bool>("slam_pose_invert_tf",
                                 m_param_.m_slam_pose_invert_tf, false);
  }
  else
  {
    m_param_.m_slam_pose_invert_tf = false;
  }

  this->get_parameter_or<bool>("slam_pub_odom_tf", m_param_.m_slam_pub_odom_tf,
                               false);
  this->get_parameter_or<bool>("slam_pub_odom_msg",
                               m_param_.m_slam_pub_odom_msg, false);
  this->get_parameter_or<std::string>("slam_odom_frame",
                                      m_param_.m_slam_odom_frame, "odom_laser");

  if (m_param_.m_slam_pub_odom_tf)
  {
    this->get_parameter_or<bool>("slam_odom_invert_tf",
                                 m_param_.m_slam_odom_invert_tf, false);
  }
  else
  {
    m_param_.m_slam_odom_invert_tf = false;
  }

  if (m_param_.m_laser_tf_static_yaml &&
      (!m_param_.m_slam_odom_invert_tf || !m_param_.m_slam_pose_invert_tf))
  {
    throw std::runtime_error(
        "SLAMROS2API::readParam | pre-defined laser pose w.r.t. robot "
        "defined but slam_odom_invert_tf and/or slam_pose_invert_tf are "
        "disabled (when laser pose set, SLAM tracker publishes the laser "
        "odometry / pose w.r.t. sensor, not the robot base frame)");
  }
  if ((m_param_.m_ext_odom_impl.compare("tf") == 0) &&
      m_param_.m_slam_pub_odom_tf && !m_param_.m_slam_odom_invert_tf)
  {
    throw std::runtime_error(
        "SLAMROS2API::readParam | ext_odom_impl TF implementation and "
        "slam_pub_odom_tf enabled but slam_odom_invert_tf disabled (breaks "
        "the TF tree structure of a frame having a single parent and multiple "
        "child frames)");
  }

  // ----- SLAM OFFLINE CORRECTED POSE -----------------------------------------

  this->get_parameter_or<bool>("slam_log_pose_corrected",
                               m_param_.m_slam_param.m_log_pose_corrected,
                               false);

  // ----- SLAM BACKEND SOLVER -------------------------------------------------

  int solver_dbg_ogl_interval;

  this->get_parameter_or<bool>("slam_solver_debug",
                               m_param_.m_slam_param.m_solver_debug, false);
  this->get_parameter_or<bool>("slam_solver_verbose",
                               m_param_.m_slam_param.m_solver_verbose, false);
  this->get_parameter_or<float>("slam_solver_damping",
                                m_param_.m_slam_param.m_solver_damping, 1.0f);
  this->get_parameter_or<int>("slam_solver_max_iterations",
                              m_param_.m_slam_param.m_solver_max_iterations,
                              100);
  this->get_parameter_or<std::string>(
      "slam_solver_criteria_type", m_param_.m_slam_param.m_solver_criteria_type,
      "stepnorm");
  this->get_parameter_or<float>("slam_solver_criteria_epsilon",
                                m_param_.m_slam_param.m_solver_criteria_epsilon,
                                1e-5f);
  this->get_parameter_or<int>("slam_solver_dbg_ogl_interval",
                              solver_dbg_ogl_interval, 0);
  this->get_parameter_or<std::string>(
      "slam_solver_dbg_ogl_mode", m_param_.m_slam_param.m_solver_dbg_ogl_mode,
      "");

  m_param_.m_slam_param.m_solver_dbg_ogl_interval = solver_dbg_ogl_interval;

  this->get_parameter_or<bool>("slam_factor_info_use_tracker",
                               m_param_.m_slam_param.m_factor_info_use_tracker,
                               true);

  if (!m_param_.m_slam_param.m_factor_info_use_tracker)
  {
    float slam_factor_info_x, slam_factor_info_y, slam_factor_info_th;

    this->get_parameter_or<float>("slam_factor_info_x", slam_factor_info_x,
                                  1.0f);
    this->get_parameter_or<float>("slam_factor_info_y", slam_factor_info_y,
                                  1.0f);
    this->get_parameter_or<float>("slam_factor_info_th", slam_factor_info_th,
                                  10.0f);

    m_param_.m_slam_param.m_factor_info_mat.setIdentity();

    m_param_.m_slam_param.m_factor_info_mat(0, 0) = slam_factor_info_x;
    m_param_.m_slam_param.m_factor_info_mat(1, 1) = slam_factor_info_y;
    m_param_.m_slam_param.m_factor_info_mat(2, 2) = slam_factor_info_th;
  }

  this->get_parameter_or<bool>("slam_factor_local_map_update",
                               m_param_.m_slam_param.m_factor_local_map_update,
                               false);

  // ----- SLAM LOOP CLOSURE ---------------------------------------------------

  this->get_parameter_or<bool>("slam_loop_enable",
                               m_param_.m_slam_param.m_loop_enable, true);

  if (m_param_.m_slam_param.m_loop_enable)
  {
    this->get_parameter_or<bool>(
        "slam_loop_use_same_place_guess",
        m_param_.m_slam_param.m_loop_use_same_place_guess, false);
    this->get_parameter_or<bool>(
        "slam_loop_use_pose_graph_guess",
        m_param_.m_slam_param.m_loop_use_pose_graph_guess, true);
    this->get_parameter_or<int>("slam_loop_min_graph_len",
                                m_param_.m_slam_param.m_loop_min_graph_len, 5);
    this->get_parameter_or<float>("slam_loop_max_dist",
                                  m_param_.m_slam_param.m_loop_max_dist, 2.5f);
  }

  // ----- SLAM RELOCALIZATION -------------------------------------------------

  this->get_parameter_or<bool>("slam_relocalization_enable",
                               m_param_.m_slam_param.m_relocalization_enable,
                               false);

  // ----- TRACKER -------------------------------------------------------------

  TrackerTypeParam tracker_param;

  this->get_parameter_or<bool>("laser_invert_scan",
                               tracker_param.m_laser_invert_scan, false);

  this->get_parameter_or<bool>("tracker_use_ext_odom_as_init",
                               tracker_param.m_use_ext_odom_as_init, false);
  this->get_parameter_or<bool>("tracker_use_ext_odom_as_prior",
                               tracker_param.m_use_ext_odom_as_prior, false);

  if ((tracker_param.m_use_ext_odom_as_init ||
       tracker_param.m_use_ext_odom_as_prior) &&
      !m_param_.m_ext_odom_use)
  {
    throw std::runtime_error(
        "SLAMROS2API::readParam | ext_odom_use must be "
        "enabled if the tracker is using external odometry as initialization "
        "or as a prior factor...");
  }

  // ----- TRACKER::SPLITTING CRITERIA -----------------------------------------

  if (this->has_parameter("tracker_split_criteria_type"))
  {
    std::string tracker_split_criteria_type =
        this->get_parameter("tracker_split_criteria_type")
            .get_value<std::string>();

    if (tracker_split_criteria_type.compare("empty") == 0)
    {
      SplittingCriteriaEmptyTypePtr criteria =
          std::make_shared<SplittingCriteriaEmptyType>();

      tracker_param.m_splitting_criteria =
          std::dynamic_pointer_cast<SplittingCriteriaType>(criteria);
    }
    else if (tracker_split_criteria_type.compare("rotation") == 0)
    {
      SplittingCriteriaRotationTypePtr criteria =
          std::make_shared<SplittingCriteriaRotationType>();

      this->get_parameter_or<float>("tracker_split_criteria_min_travel_heading",
                                    criteria->m_min_travel_heading, 30.0f);

      criteria->m_min_travel_heading =
          geometry::degToRad(criteria->m_min_travel_heading);

      tracker_param.m_splitting_criteria =
          std::dynamic_pointer_cast<SplittingCriteriaType>(criteria);
    }
    else if (tracker_split_criteria_type.compare("translation") == 0)
    {
      SplittingCriteriaTranslationTypePtr criteria =
          std::make_shared<SplittingCriteriaTranslationType>();

      this->get_parameter_or<float>("tracker_split_criteria_min_travel_dist",
                                    criteria->m_min_travel_dist, 0.50f);

      tracker_param.m_splitting_criteria =
          std::dynamic_pointer_cast<SplittingCriteriaType>(criteria);
    }
    else if (tracker_split_criteria_type.compare("viewpoint") == 0)
    {
      SplittingCriteriaViewpointTypePtr criteria =
          std::make_shared<SplittingCriteriaViewpointType>();

      this->get_parameter_or<float>("tracker_split_criteria_min_travel_heading",
                                    criteria->m_min_travel_heading, 30.0f);
      this->get_parameter_or<float>("tracker_split_criteria_min_travel_dist",
                                    criteria->m_min_travel_dist, 0.50f);

      criteria->m_min_travel_heading =
          geometry::degToRad(criteria->m_min_travel_heading);

      tracker_param.m_splitting_criteria =
          std::dynamic_pointer_cast<SplittingCriteriaType>(criteria);
    }
    else if (tracker_split_criteria_type.compare("gridcell") == 0)
    {
      SplittingCriteriaGridCellTypePtr criteria =
          std::make_shared<SplittingCriteriaGridCellType>();

      this->get_parameter_or<float>("tracker_split_criteria_cell_size",
                                    criteria->m_cell_size_2, 2.5f);

      criteria->m_cell_size_2 *= 0.5f;

      tracker_param.m_splitting_criteria =
          std::dynamic_pointer_cast<SplittingCriteriaType>(criteria);
    }
    else if (tracker_split_criteria_type.compare("inliers") == 0)
    {
      SplittingCriteriaInliersRatioTypePtr criteria =
          std::make_shared<SplittingCriteriaInliersRatioType>();

      this->get_parameter_or<float>("tracker_split_criteria_min_travel_dist",
                                    criteria->m_min_travel_dist, 0.10f);
      this->get_parameter_or<float>("tracker_split_criteria_min_travel_heading",
                                    criteria->m_min_travel_heading, 5.0f);
      this->get_parameter_or<float>("tracker_split_criteria_max_ratio_inliers",
                                    criteria->m_max_ratio_inliers, 0.60f);
      this->get_parameter_or<float>("tracker_split_criteria_max_travel_dist",
                                    criteria->m_max_travel_dist, -1.0f);
      this->get_parameter_or<float>("tracker_split_criteria_max_travel_heading",
                                    criteria->m_max_travel_heading, -1.0f);

      criteria->m_min_travel_heading =
          geometry::degToRad(criteria->m_min_travel_heading);
      criteria->m_max_travel_heading =
          geometry::degToRad(criteria->m_max_travel_heading);

      tracker_param.m_splitting_criteria =
          std::dynamic_pointer_cast<SplittingCriteriaType>(criteria);
    }
    else if (tracker_split_criteria_type.compare("time") == 0)
    {
      SplittingCriteriaTimeTypePtr criteria =
          std::make_shared<SplittingCriteriaTimeType>();

      this->get_parameter_or<double>("tracker_split_criteria_min_time_interval",
                                     criteria->m_min_time_interval, -1.0);
      this->get_parameter_or<float>("tracker_split_criteria_max_ratio_inliers",
                                    criteria->m_max_ratio_inliers, 0.60f);

      tracker_param.m_splitting_criteria =
          std::dynamic_pointer_cast<SplittingCriteriaType>(criteria);
    }
    else
    {
      throw std::runtime_error(
          "SLAMROS2API::readParam | unknown tracker_split_criteria_type "
          "value: " +
          tracker_split_criteria_type +
          " (empty|rotation|translation|viewpoint|gridcell|inliers|time)");
    }
  }
  else
  {
    tracker_param.m_splitting_criteria = nullptr;
  }

  // ----- TRACKER::MERGING CRITERIA -------------------------------------------

  if (this->has_parameter("tracker_merge_criteria_type"))
  {
    std::string tracker_merge_criteria_type;

    tracker_merge_criteria_type =
        this->get_parameter("tracker_merge_criteria_type")
            .get_value<std::string>();

    if (tracker_merge_criteria_type.compare("empty") == 0)
    {
      MergingCriteriaEmptyTypePtr criteria =
          std::make_shared<MergingCriteriaEmptyType>();

      tracker_param.m_merging_criteria =
          std::dynamic_pointer_cast<MergingCriteriaType>(criteria);
    }
    else if (tracker_merge_criteria_type.compare("rotation") == 0)
    {
      MergingCriteriaRotationTypePtr criteria =
          std::make_shared<MergingCriteriaRotationType>();

      this->get_parameter_or<float>("tracker_merge_criteria_min_travel_heading",
                                    criteria->m_min_travel_heading, 30.0f);

      criteria->m_min_travel_heading =
          geometry::degToRad(criteria->m_min_travel_heading);

      tracker_param.m_merging_criteria =
          std::dynamic_pointer_cast<MergingCriteriaType>(criteria);
    }
    else if (tracker_merge_criteria_type.compare("translation") == 0)
    {
      MergingCriteriaTranslationTypePtr criteria =
          std::make_shared<MergingCriteriaTranslationType>();

      this->get_parameter_or<float>("tracker_merge_criteria_min_travel_dist",
                                    criteria->m_min_travel_dist, 0.50f);

      tracker_param.m_merging_criteria =
          std::dynamic_pointer_cast<MergingCriteriaType>(criteria);
    }
    else if (tracker_merge_criteria_type.compare("viewpoint") == 0)
    {
      MergingCriteriaViewpointTypePtr criteria =
          std::make_shared<MergingCriteriaViewpointType>();

      this->get_parameter_or<float>("tracker_merge_criteria_min_travel_heading",
                                    criteria->m_min_travel_heading, 30.0f);
      this->get_parameter_or<float>("tracker_merge_criteria_min_travel_dist",
                                    criteria->m_min_travel_dist, 0.50f);

      criteria->m_min_travel_heading =
          geometry::degToRad(criteria->m_min_travel_heading);

      tracker_param.m_merging_criteria =
          std::dynamic_pointer_cast<MergingCriteriaType>(criteria);
    }
    else if (tracker_merge_criteria_type.compare("gridcell") == 0)
    {
      MergingCriteriaGridCellTypePtr criteria =
          std::make_shared<MergingCriteriaGridCellType>();

      this->get_parameter_or<float>("tracker_merge_criteria_cell_size",
                                    criteria->m_cell_size_2, 2.5f);

      criteria->m_cell_size_2 *= 0.5f;

      tracker_param.m_merging_criteria =
          std::dynamic_pointer_cast<MergingCriteriaType>(criteria);
    }
    else if (tracker_merge_criteria_type.compare("inliers") == 0)
    {
      MergingCriteriaInliersRatioTypePtr criteria =
          std::make_shared<MergingCriteriaInliersRatioType>();

      this->get_parameter_or<float>("tracker_merge_criteria_min_travel_dist",
                                    criteria->m_min_travel_dist, 0.10f);
      this->get_parameter_or<float>("tracker_merge_criteria_min_travel_heading",
                                    criteria->m_min_travel_heading, 5.0f);
      this->get_parameter_or<float>("tracker_merge_criteria_max_ratio_inliers",
                                    criteria->m_max_ratio_inliers, 0.80f);
      this->get_parameter_or<float>("tracker_merge_criteria_max_travel_dist",
                                    criteria->m_max_travel_dist, -1.0f);
      this->get_parameter_or<float>("tracker_merge_criteria_max_travel_heading",
                                    criteria->m_max_travel_heading, -1.0f);

      criteria->m_min_travel_heading =
          geometry::degToRad(criteria->m_min_travel_heading);
      criteria->m_max_travel_heading =
          geometry::degToRad(criteria->m_max_travel_heading);

      tracker_param.m_merging_criteria =
          std::dynamic_pointer_cast<MergingCriteriaType>(criteria);
    }
    else if (tracker_merge_criteria_type.compare("time") == 0)
    {
      MergingCriteriaTimeTypePtr criteria =
          std::make_shared<MergingCriteriaTimeType>();

      this->get_parameter_or<double>("tracker_merge_criteria_min_time_interval",
                                     criteria->m_min_time_interval, -1.0);
      this->get_parameter_or<float>("tracker_merge_criteria_max_ratio_inliers",
                                    criteria->m_max_ratio_inliers, 0.60f);

      tracker_param.m_merging_criteria =
          std::dynamic_pointer_cast<MergingCriteriaType>(criteria);
    }
    else
    {
      throw std::runtime_error(
          "SLAMROS2API::readParam | unknown tracker_merge_criteria_type "
          "value: " +
          tracker_merge_criteria_type +
          " (empty|rotation|translation|viewpoint|gridcell|inliers|time)");
    }
  }
  else
  {
    tracker_param.m_merging_criteria = nullptr;
  }

  // ----- TRACKER::MERGER -----------------------------------------------------

  if (tracker_param.m_merging_criteria)
  {
    MergerTypeParam merger_param;

    this->get_parameter_or<float>("tracker_merger_ray_min_dist_through",
                                  merger_param.m_ray_min_dist_through, 0.20f);
    this->get_parameter_or<float>("tracker_merger_ray_min_dist_add_new",
                                  merger_param.m_ray_min_dist_add_new, 0.20f);
    this->get_parameter_or<bool>("tracker_merger_enable_avg",
                                 merger_param.m_enable_avg, false);

    if (merger_param.m_enable_avg)
    {
      if (!this->has_parameter("tracker_merger_weight_old") ||
          !this->has_parameter("tracker_merger_weight_new"))
      {
        throw std::runtime_error(
            "SLAMROS2API::readParam | "
            "tracker_merger_weight_old and tracker_merger_weight_new must be "
            "defined if tracker_merger_enable_avg is enabled...");
      }

      merger_param.m_weight_old =
          this->get_parameter("tracker_merger_weight_old").get_value<float>();
      merger_param.m_weight_new =
          this->get_parameter("tracker_merger_weight_new").get_value<float>();
    }
    else
    {
      merger_param.m_weight_old = -1.f;
      merger_param.m_weight_new = -1.f;
    }

    tracker_param.m_merger = std::make_shared<MergerType>(merger_param);
  }

  // ----- TRACKER::ALIGNER ----------------------------------------------------

  std::string tracker_aligner_type;

  this->get_parameter_or<std::string>("tracker_aligner_type",
                                      tracker_aligner_type, "point2point");

  AlignerTypeParamSolver tracker_solver;
  AlignerTypeParamDistMap tracker_dist_map;

  int tracker_dbg_ogl_interval;

  this->get_parameter_or<bool>("tracker_aligner_debug", tracker_solver.m_debug,
                               false);
  this->get_parameter_or<bool>("tracker_aligner_solver_verbose",
                               tracker_solver.m_solver_verbose, false);
  this->get_parameter_or<float>("tracker_aligner_solver_damping",
                                tracker_solver.m_solver_damping, 1.0f);
  this->get_parameter_or<int>("tracker_aligner_solver_max_iterations",
                              tracker_solver.m_solver_max_iterations, 25);
  this->get_parameter_or<std::string>("tracker_aligner_solver_criteria_type",
                                      tracker_solver.m_solver_criteria_type,
                                      "stepnorm");
  this->get_parameter_or<float>("tracker_aligner_solver_criteria_epsilon",
                                tracker_solver.m_solver_criteria_epsilon,
                                1e-5f);
  this->get_parameter_or<std::string>("tracker_aligner_solver_robustifier_type",
                                      tracker_solver.m_solver_robustifier_type,
                                      "cauchy");

  if (!tracker_solver.m_solver_robustifier_type.empty())
  {
    if (!this->has_parameter(
            "tracker_aligner_solver_robustifier_num_iterations_coarse"))
    {
      if (!this->has_parameter("tracker_aligner_solver_robustifier_threshold"))
      {
        throw std::runtime_error(
            "SLAMROS2API::readParam | "
            "tracker_aligner_solver_robustifier_threshold must be defined when "
            "fine + coarse matching is not used");
      }
      if (this->has_parameter(
              "tracker_aligner_solver_robustifier_threshold_coarse") ||
          this->has_parameter(
              "tracker_aligner_solver_robustifier_threshold_fine"))
      {
        throw std::runtime_error(
            "SLAMROS2API::readParam | "
            "when tracker_aligner_solver_robustifier_threshold defined, no "
            "fine (tracker_aligner_solver_robustifier_threshold_fine) or "
            "coarse (tracker_aligner_solver_robustifier_threshold_coarse) "
            "thresholds must be defined...");
      }

      tracker_solver.m_solver_robustifier_num_iterations_coarse = 0;

      tracker_solver.m_solver_robustifier_threshold_coarse =
          this->get_parameter("tracker_aligner_solver_robustifier_threshold")
              .get_value<float>();
      tracker_solver.m_solver_robustifier_threshold_fine =
          this->get_parameter("tracker_aligner_solver_robustifier_threshold")
              .get_value<float>();
    }
    else
    {
      if (!this->has_parameter(
              "tracker_aligner_solver_robustifier_threshold_coarse") ||
          !this->has_parameter(
              "tracker_aligner_solver_robustifier_threshold_fine"))
      {
        throw std::runtime_error(
            "SLAMROS2API::readParam | "
            "fine (tracker_aligner_solver_robustifier_threshold_fine) and "
            "coarse (tracker_aligner_solver_robustifier_threshold_coarse) "
            "thresholds must be defined when fine + coarse matching is "
            "enabled");
      }

      tracker_solver.m_solver_robustifier_num_iterations_coarse =
          this->get_parameter(
                  "tracker_aligner_solver_robustifier_num_iterations_coarse")
              .get_value<int>();
      tracker_solver.m_solver_robustifier_threshold_coarse =
          this->get_parameter(
                  "tracker_aligner_solver_robustifier_threshold_coarse")
              .get_value<float>();
      tracker_solver.m_solver_robustifier_threshold_fine =
          this->get_parameter(
                  "tracker_aligner_solver_robustifier_threshold_fine")
              .get_value<float>();
    }
  }
  else
  {
    tracker_solver.m_solver_robustifier_num_iterations_coarse = 0;
    tracker_solver.m_solver_robustifier_threshold_coarse = 0.f;
    tracker_solver.m_solver_robustifier_threshold_fine = 0.f;
  }

  this->get_parameter_or<float>("tracker_aligner_failure_ratio_inliers",
                                tracker_solver.m_failure_ratio_inliers, 0.35f);
  this->get_parameter_or<float>("tracker_aligner_failure_chi_inliers",
                                tracker_solver.m_failure_chi_inliers, -1.0f);
  this->get_parameter_or<int>("tracker_aligner_dbg_ogl_interval",
                              tracker_dbg_ogl_interval, 0);
  this->get_parameter_or<std::string>("tracker_aligner_dbg_ogl_mode",
                                      tracker_solver.m_dbg_ogl_mode, "");
  this->get_parameter_or<bool>("tracker_aligner_dbg_ogl_show_correspondences",
                               tracker_solver.m_dbg_ogl_show_correspondences,
                               false);
  this->get_parameter_or<bool>("tracker_aligner_dbg_ogl_show_normals",
                               tracker_solver.m_dbg_ogl_show_normals, false);

  tracker_solver.m_dbg_ogl_interval = tracker_dbg_ogl_interval;

  this->get_parameter_or<float>("tracker_aligner_dist_map_res",
                                tracker_dist_map.m_dist_map_res, 0.01f);
  this->get_parameter_or<float>("tracker_aligner_dist_map_size",
                                tracker_dist_map.m_dist_map_size, 10.0f);
  this->get_parameter_or<float>("tracker_aligner_dist_map_max_dist",
                                tracker_dist_map.m_dist_map_max_dist, 0.25f);

  this->get_parameter_or<bool>(
      "tracker_solver_regularize_correspondences",
      tracker_dist_map.m_solver_regularize_correspondences, false);

  if (tracker_aligner_type.compare("point2point") == 0)
  {
    AlignerPt2PtTypePtr aligner =
        std::make_shared<AlignerPt2PtType>(tracker_solver, tracker_dist_map);

    tracker_param.m_aligner = std::dynamic_pointer_cast<AlignerType>(aligner);
  }
  else if (tracker_aligner_type.compare("point2plane") == 0)
  {
    AlignerPt2PlnTypePtr aligner =
        std::make_shared<AlignerPt2PlnType>(tracker_solver, tracker_dist_map);

    tracker_param.m_aligner = std::dynamic_pointer_cast<AlignerType>(aligner);
  }
  else if (tracker_aligner_type.compare("icp_point2point") == 0)
  {
    AlignerICPPt2PtTypePtr aligner =
        std::make_shared<AlignerICPPt2PtType>(tracker_solver, tracker_dist_map);

    tracker_param.m_aligner = std::dynamic_pointer_cast<AlignerType>(aligner);
  }
  else if (tracker_aligner_type.compare("icp_point2plane") == 0)
  {
    AlignerICPPt2PlnTypePtr aligner = std::make_shared<AlignerICPPt2PlnType>(
        tracker_solver, tracker_dist_map);

    tracker_param.m_aligner = std::dynamic_pointer_cast<AlignerType>(aligner);
  }
  else
  {
    throw std::runtime_error(
        "SLAMROS2API::readParam | unknown tracker_aligner_type value: " +
        tracker_aligner_type +
        " (point2point|point2plane|icp_point2point|icp_point2plane)");
  }

  // ----- TRACKER::PREPROCESS -------------------------------------------------

  this->get_parameter_or<bool>("tracker_enable_voxelization",
                               tracker_param.m_enable_voxelization, false);

  if (this->has_parameter("tracker_voxel_scale"))
  {
    tracker_param.m_voxel_scale =
        this->get_parameter("tracker_voxel_scale").get_value<float>();
  }
  else
  {
    // will default internally to distance map resolution
    tracker_param.m_voxel_scale = Scalar(-1);
  }

  if ((tracker_aligner_type.compare("point2plane") == 0) ||
      (this->has_parameter("tracker_normals_computator_min_pts")) ||
      (this->has_parameter("tracker_normals_computator_radius")))
  {
    tracker_param.m_enable_normals = true;
    this->get_parameter_or<int>("tracker_normals_computator_min_pts",
                                tracker_param.m_normals_computator_min_pts, 5);
    this->get_parameter_or<float>("tracker_normals_computator_radius",
                                  tracker_param.m_normals_computator_radius,
                                  0.1f);
  }
  else
  {
    tracker_param.m_enable_normals = false;
  }

  this->get_parameter_or<float>("tracker_range_min", tracker_param.m_range_min,
                                -1.f);
  this->get_parameter_or<float>("tracker_range_max", tracker_param.m_range_max,
                                -1.f);

  // ----- SLAM TRACKER INITIALIZATION -----------------------------------------

  m_param_.m_slam_param.m_tracker =
      std::make_shared<TrackerType>(tracker_param);

  // ----- LOOP CLOSURE ALIGNER ------------------------------------------------

  if (!this->has_parameter("slam_loop_aligner_type") &&
      !this->has_parameter("slam_loop_aligner_debug") &&
      !this->has_parameter("slam_loop_aligner_solver_verbose") &&
      !this->has_parameter("slam_loop_aligner_solver_damping") &&
      !this->has_parameter("slam_loop_aligner_solver_max_iterations") &&
      !this->has_parameter("slam_loop_aligner_solver_criteria_type") &&
      !this->has_parameter("slam_loop_aligner_solver_criteria_epsilon") &&
      !this->has_parameter("slam_loop_aligner_solver_robustifier_type") &&
      !this->has_parameter(
          "slam_loop_aligner_solver_robustifier_num_iterations_coarse") &&
      !this->has_parameter(
          "slam_loop_aligner_solver_robustifier_threshold_coarse") &&
      !this->has_parameter(
          "slam_loop_aligner_solver_robustifier_threshold_fine") &&
      !this->has_parameter("slam_loop_aligner_failure_ratio_inliers") &&
      !this->has_parameter("slam_loop_aligner_failure_chi_inliers") &&
      !this->has_parameter("slam_loop_aligner_dbg_ogl_interval") &&
      !this->has_parameter("slam_loop_aligner_dbg_ogl_mode") &&
      !this->has_parameter("slam_loop_aligner_dbg_ogl_show_correspondences") &&
      !this->has_parameter("slam_loop_aligner_dbg_ogl_show_normals") &&
      !this->has_parameter("slam_loop_aligner_dist_map_res") &&
      !this->has_parameter("slam_loop_aligner_dist_map_size") &&
      !this->has_parameter("slam_loop_aligner_dist_map_max_dist") &&
      !this->has_parameter(
          "slam_loop_aligner_solver_regularize_correspondences"))
  {
    m_param_.m_slam_param.m_loop_aligner =
        m_param_.m_slam_param.m_tracker->getAlignerPtr();

    /**
     * @todo check if this is really stable...
     */

    return;
  }

  //! NOTE: the following creates a specific aligner for loop closure!

  std::string loop_aligner_type;

  this->get_parameter_or<std::string>("slam_loop_aligner_type",
                                      loop_aligner_type, tracker_aligner_type);

  AlignerTypeParamSolver loop_aligner_solver;
  AlignerTypeParamDistMap loop_aligner_dist_map;

  int dbg_ogl_interval;

  this->get_parameter_or<bool>("slam_loop_aligner_debug",
                               loop_aligner_solver.m_debug, false);
  this->get_parameter_or<bool>("slam_loop_aligner_solver_verbose",
                               loop_aligner_solver.m_solver_verbose, false);
  this->get_parameter_or<float>("slam_loop_aligner_solver_damping",
                                loop_aligner_solver.m_solver_damping,
                                tracker_solver.m_solver_damping);
  this->get_parameter_or<int>("slam_loop_aligner_solver_max_iterations",
                              loop_aligner_solver.m_solver_max_iterations,
                              tracker_solver.m_solver_max_iterations);
  this->get_parameter_or<std::string>(
      "slam_loop_aligner_solver_criteria_type",
      loop_aligner_solver.m_solver_criteria_type,
      tracker_solver.m_solver_criteria_type);
  this->get_parameter_or<float>("slam_loop_aligner_solver_criteria_epsilon",
                                loop_aligner_solver.m_solver_criteria_epsilon,
                                tracker_solver.m_solver_criteria_epsilon);
  this->get_parameter_or<std::string>(
      "slam_loop_aligner_solver_robustifier_type",
      loop_aligner_solver.m_solver_robustifier_type,
      tracker_solver.m_solver_robustifier_type);

  if (!loop_aligner_solver.m_solver_robustifier_type.empty())
  {
    this->get_parameter_or<int>(
        "slam_loop_aligner_solver_robustifier_num_iterations_coarse",
        loop_aligner_solver.m_solver_robustifier_num_iterations_coarse,
        tracker_solver.m_solver_robustifier_num_iterations_coarse);

    if (loop_aligner_solver.m_solver_robustifier_num_iterations_coarse <= 0)
    {
      if (this->has_parameter(
              "slam_loop_aligner_solver_robustifier_threshold_coarse") ||
          this->has_parameter(
              "slam_loop_aligner_solver_robustifier_threshold_fine"))
      {
        throw std::runtime_error(
            "SLAMROS2API::readParam | "
            "when slam_loop_aligner_solver_robustifier_threshold defined, no "
            "fine (slam_loop_aligner_solver_robustifier_threshold_fine) or "
            "coarse (slam_loop_aligner_solver_robustifier_threshold_coarse) "
            "thresholds must be defined...");
      }

      loop_aligner_solver.m_solver_robustifier_num_iterations_coarse = 0;

      this->get_parameter_or<float>(
          "slam_loop_aligner_solver_robustifier_threshold",
          loop_aligner_solver.m_solver_robustifier_threshold_coarse,
          tracker_solver.m_solver_robustifier_threshold_fine);
      this->get_parameter_or<float>(
          "slam_loop_aligner_solver_robustifier_threshold",
          loop_aligner_solver.m_solver_robustifier_threshold_fine,
          tracker_solver.m_solver_robustifier_threshold_fine);
    }
    else
    {
      if (!this->has_parameter(
              "slam_loop_aligner_solver_robustifier_threshold_coarse") ||
          !this->has_parameter(
              "slam_loop_aligner_solver_robustifier_threshold_fine"))
      {
        throw std::runtime_error(
            "SLAMROS2API::readParam | "
            "fine (slam_loop_aligner_solver_robustifier_threshold_fine) and "
            "coarse (slam_loop_aligner_solver_robustifier_threshold_coarse) "
            "thresholds must be defined when fine + coarse matching is "
            "enabled");
      }

      this->get_parameter_or<float>(
          "slam_loop_aligner_solver_robustifier_threshold_coarse",
          loop_aligner_solver.m_solver_robustifier_threshold_coarse,
          tracker_solver.m_solver_robustifier_threshold_coarse);
      this->get_parameter_or<float>(
          "slam_loop_aligner_solver_robustifier_threshold_fine",
          loop_aligner_solver.m_solver_robustifier_threshold_fine,
          tracker_solver.m_solver_robustifier_threshold_fine);
    }
  }
  else
  {
    loop_aligner_solver.m_solver_robustifier_num_iterations_coarse = 0;
    loop_aligner_solver.m_solver_robustifier_threshold_coarse = 0.f;
    loop_aligner_solver.m_solver_robustifier_threshold_fine = 0.f;
  }

  this->get_parameter_or<float>("slam_loop_aligner_failure_ratio_inliers",
                                loop_aligner_solver.m_failure_ratio_inliers,
                                tracker_solver.m_failure_ratio_inliers);
  this->get_parameter_or<float>("slam_loop_aligner_failure_chi_inliers",
                                loop_aligner_solver.m_failure_chi_inliers,
                                tracker_solver.m_failure_chi_inliers);
  this->get_parameter_or<int>("slam_loop_aligner_dbg_ogl_interval",
                              dbg_ogl_interval, 0);
  this->get_parameter_or<std::string>("slam_loop_aligner_dbg_ogl_mode",
                                      loop_aligner_solver.m_dbg_ogl_mode, "");
  this->get_parameter_or<bool>(
      "slam_loop_aligner_dbg_ogl_show_correspondences",
      loop_aligner_solver.m_dbg_ogl_show_correspondences,
      tracker_solver.m_dbg_ogl_show_correspondences);
  this->get_parameter_or<bool>("slam_loop_aligner_dbg_ogl_show_normals",
                               loop_aligner_solver.m_dbg_ogl_show_normals,
                               tracker_solver.m_dbg_ogl_show_normals);

  loop_aligner_solver.m_dbg_ogl_interval = dbg_ogl_interval;

  this->get_parameter_or<float>("slam_loop_aligner_dist_map_res",
                                loop_aligner_dist_map.m_dist_map_res,
                                tracker_dist_map.m_dist_map_res);
  this->get_parameter_or<float>("slam_loop_aligner_dist_map_size",
                                loop_aligner_dist_map.m_dist_map_size,
                                tracker_dist_map.m_dist_map_size);
  this->get_parameter_or<float>("slam_loop_aligner_dist_map_max_dist",
                                loop_aligner_dist_map.m_dist_map_max_dist,
                                tracker_dist_map.m_dist_map_max_dist);

  this->get_parameter_or<bool>(
      "slam_loop_aligner_solver_regularize_correspondences",
      loop_aligner_dist_map.m_solver_regularize_correspondences,
      tracker_dist_map.m_solver_regularize_correspondences);

  if (loop_aligner_type.compare("point2point") == 0)
  {
    AlignerPt2PtTypePtr aligner = std::make_shared<AlignerPt2PtType>(
        loop_aligner_solver, loop_aligner_dist_map);

    m_param_.m_slam_param.m_loop_aligner =
        std::dynamic_pointer_cast<AlignerType>(aligner);
  }
  else if (loop_aligner_type.compare("point2plane") == 0)
  {
    AlignerPt2PlnTypePtr aligner = std::make_shared<AlignerPt2PlnType>(
        loop_aligner_solver, loop_aligner_dist_map);

    m_param_.m_slam_param.m_loop_aligner =
        std::dynamic_pointer_cast<AlignerType>(aligner);
  }
  else if (tracker_aligner_type.compare("icp_point2point") == 0)
  {
    AlignerICPPt2PtTypePtr aligner = std::make_shared<AlignerICPPt2PtType>(
        loop_aligner_solver, loop_aligner_dist_map);

    m_param_.m_slam_param.m_loop_aligner =
        std::dynamic_pointer_cast<AlignerType>(aligner);
  }
  else if (tracker_aligner_type.compare("icp_point2plane") == 0)
  {
    AlignerICPPt2PlnTypePtr aligner = std::make_shared<AlignerICPPt2PlnType>(
        loop_aligner_solver, loop_aligner_dist_map);

    m_param_.m_slam_param.m_loop_aligner =
        std::dynamic_pointer_cast<AlignerType>(aligner);
  }
  else
  {
    throw std::runtime_error(
        "SLAMROS2API::readParam | unknown loop_aligner_type value: " +
        loop_aligner_type +
        " (point2point|point2plane|icp_point2point|icp_point2plane)");
  }
}

void SLAMROS2API::printParam()
{
  RCLCPP_INFO(this->get_logger(), "[%s] base frame id: %s", this->get_name(),
              m_param_.m_base_frame.c_str());

  // ----- EXTERNAL ODOMETRY ---------------------------------------------------

  RCLCPP_INFO(this->get_logger(), "[%s] external odom          : %s",
              this->get_name(),
              m_param_.m_ext_odom_use ? "enabled" : "not enabled");
  RCLCPP_INFO(
      this->get_logger(), "[%s] external odom impl     : %s", this->get_name(),
      m_param_.m_ext_odom_use ? m_param_.m_ext_odom_impl.c_str() : "N/A");
  RCLCPP_INFO(
      this->get_logger(), "[%s] external odom frame id : %s", this->get_name(),
      m_param_.m_ext_odom_use ? m_param_.m_ext_odom_impl.compare("tf") == 0
                                    ? m_param_.m_ext_odom_frame.c_str()
                                    : "N/A (read from ext odom msg)"
                              : "N/A");
  RCLCPP_INFO(
      this->get_logger(), "[%s] external odom pub tf   : %s", this->get_name(),
      m_param_.m_ext_odom_use && m_param_.m_ext_odom_impl.compare("tf") != 0
          ? m_param_.m_ext_odom_pub_tf ? "enabled" : "not enabled"
          : "N/A");
  RCLCPP_INFO(
      this->get_logger(), "[%s] external odom pub msg  : %s", this->get_name(),
      m_param_.m_ext_odom_use && m_param_.m_ext_odom_impl.compare("topic") != 0
          ? m_param_.m_ext_odom_pub_msg ? "enabled" : "not enabled"
          : "N/A");
  RCLCPP_INFO(this->get_logger(), "[%s] external odom invert tf: %s",
              this->get_name(),
              m_param_.m_ext_odom_pub_tf
                  ? m_param_.m_ext_odom_invert_tf ? "enabled" : "not enabled"
                  : "N/A");

  // ----- LASER SENSOR --------------------------------------------------------

  RCLCPP_INFO(this->get_logger(), "[%s][LASER] invert scan (roll/pitch): %s",
              this->get_name(),
              m_param_.m_slam_param.m_tracker->getParam().m_laser_invert_scan
                  ? "enabled"
                  : "not enabled");

  if (m_param_.m_laser_pub_tf_static)
  {
    RCLCPP_INFO(this->get_logger(),
                "[%s][LASER] publish static TF       : enabled (%s w.r.t. %s)",
                this->get_name(), m_param_.m_laser_frame.c_str(),
                m_param_.m_base_frame.c_str());
  }
  else
  {
    RCLCPP_INFO(this->get_logger(),
                "[%s][LASER] publish static TF       : not enabled",
                this->get_name());
  }

  // ----- SLAM ROS API --------------------------------------------------------

  RCLCPP_INFO(this->get_logger(), "[%s][SLAM] pub robot pose tf   : %s",
              this->get_name(),
              m_param_.m_slam_pub_pose_tf ? "enabled" : "not enabled");
  RCLCPP_INFO(this->get_logger(), "[%s][SLAM] pub robot pose msg  : %s",
              this->get_name(),
              m_param_.m_slam_pub_pose_msg ? "enabled" : "not enabled");
  RCLCPP_INFO(this->get_logger(), "[%s][SLAM] robot pose frame id : %s",
              this->get_name(), m_param_.m_slam_pose_frame.c_str());

  RCLCPP_INFO(
      this->get_logger(), "[%s][SLAM] invert robot pose tf: %s",
      this->get_name(),
      m_param_.m_slam_pub_pose_tf
          ? m_param_.m_slam_pose_invert_tf
                ? m_param_.m_laser_tf_static_yaml
                      ? "enabled (robot pose / map frame w.r.t. sensor frame)"
                      : "enabled (robot pose / map frame w.r.t. base frame)"
                : "not enabled (base frame w.r.t. robot pose / map frame)"
          : "N/A");

  RCLCPP_INFO(this->get_logger(), "[%s][SLAM] pub laser odom tf   : %s",
              this->get_name(),
              m_param_.m_slam_pub_odom_tf ? "enabled" : "not enabled");
  RCLCPP_INFO(this->get_logger(), "[%s][SLAM] pub laser odom msg  : %s",
              this->get_name(),
              m_param_.m_slam_pub_odom_msg ? "enabled" : "not enabled");
  RCLCPP_INFO(this->get_logger(), "[%s][SLAM] laser odom frame id : %s",
              this->get_name(), m_param_.m_slam_odom_frame.c_str());

  RCLCPP_INFO(this->get_logger(), "[%s][SLAM] invert laser odom tf: %s",
              this->get_name(),
              m_param_.m_slam_pub_odom_tf
                  ? m_param_.m_slam_odom_invert_tf
                        ? m_param_.m_laser_tf_static_yaml
                              ? "enabled (laser odom frame w.r.t. sensor frame)"
                              : "enabled (laser odom frame w.r.t. base frame)"
                        : "not enabled (base frame w.r.t. laser odom frame)"
                  : "N/A");

  // ----- SLAM OFFLINE CORRECTED POSE -----------------------------------------

  RCLCPP_INFO(
      this->get_logger(),
      "[%s][SLAM] log offline corrected pose (w/ loop closures correction): %s",
      this->get_name(),
      m_param_.m_slam_param.m_log_pose_corrected ? "enabled" : "not enabled");

  // ----- SLAM BACKEND SOLVER -------------------------------------------------

  RCLCPP_INFO(this->get_logger(),
              "[%s][SLAM][GRAPH] OpenGL-based dbg          : %s",
              this->get_name(),
              m_param_.m_slam_param.m_solver_debug ? "enabled" : "not enabled");
  RCLCPP_INFO(
      this->get_logger(), "[%s][SLAM][GRAPH] solver verbose            : %s",
      this->get_name(),
      m_param_.m_slam_param.m_solver_verbose ? "enabled" : "not enabled");
  RCLCPP_INFO(this->get_logger(),
              "[%s][SLAM][GRAPH] solver damping            : %f",
              this->get_name(), m_param_.m_slam_param.m_solver_damping);
  RCLCPP_INFO(this->get_logger(),
              "[%s][SLAM][GRAPH] solver maximum iterations : %d",
              this->get_name(), m_param_.m_slam_param.m_solver_max_iterations);
  RCLCPP_INFO(this->get_logger(),
              "[%s][SLAM][GRAPH] solver termination type   : %s",
              this->get_name(),
              !m_param_.m_slam_param.m_solver_criteria_type.empty()
                  ? m_param_.m_slam_param.m_solver_criteria_type.c_str()
                  : "N/A");
  RCLCPP_INFO(
      this->get_logger(), "[%s][SLAM][GRAPH] solver termination epsilon: %s",
      this->get_name(),
      !m_param_.m_slam_param.m_solver_criteria_type.empty()
          ? std::to_string(m_param_.m_slam_param.m_solver_criteria_epsilon)
                .c_str()
          : "N/A");
  RCLCPP_INFO(this->get_logger(),
              "[%s][SLAM][GRAPH] Open-GL dbg mode          : %s",
              this->get_name(),
              m_param_.m_slam_param.m_solver_debug
                  ? m_param_.m_slam_param.m_solver_dbg_ogl_mode.c_str()
                  : "N/A");
  RCLCPP_INFO(
      this->get_logger(),
      "[%s][SLAM][GRAPH] Open-GL dbg sleep         : %s (ms)", this->get_name(),
      m_param_.m_slam_param.m_solver_debug
          ? std::to_string(m_param_.m_slam_param.m_solver_dbg_ogl_interval)
                .c_str()
          : "N/A");

  std::ostringstream param_info_mat_print("");
  param_info_mat_print << m_param_.m_slam_param.m_factor_info_mat;

  RCLCPP_INFO(this->get_logger(),
              "[%s][SLAM][GRAPH] factor info matrix        :\n%s",
              this->get_name(),
              !m_param_.m_slam_param.m_factor_info_use_tracker
                  ? param_info_mat_print.str().c_str()
                  : "H matrix from the tracker's aligner solver");

  RCLCPP_INFO(this->get_logger(),
              "[%s][SLAM][GRAPH] factor automatic update   : %s",
              this->get_name(),
              m_param_.m_slam_param.m_factor_local_map_update ? "enabled"
                                                              : "not enabled");

  // ----- SLAM LOOP CLOSURE ---------------------------------------------------

  RCLCPP_INFO(this->get_logger(),
              "[%s][SLAM][LOOP] loop closure detection          : %s",
              this->get_name(),
              m_param_.m_slam_param.m_loop_enable ? "enabled" : "not enabled");

  RCLCPP_INFO(
      this->get_logger(),
      "[%s][SLAM][LOOP] use same place as guess (0,0,th): %s", this->get_name(),
      m_param_.m_slam_param.m_loop_enable
          ? m_param_.m_slam_param.m_loop_use_same_place_guess ? "enabled"
                                                              : "not enabled"
          : "N/A");
  RCLCPP_INFO(
      this->get_logger(),
      "[%s][SLAM][LOOP] use pose graph as guess (x,y,th): %s", this->get_name(),
      m_param_.m_slam_param.m_loop_enable
          ? m_param_.m_slam_param.m_loop_use_pose_graph_guess ? "enabled"
                                                              : "not enabled"
          : "N/A");
  RCLCPP_INFO(
      this->get_logger(),
      "[%s][SLAM][LOOP] hint minimum graph length       : %s", this->get_name(),
      m_param_.m_slam_param.m_loop_enable
          ? std::to_string(m_param_.m_slam_param.m_loop_min_graph_len).c_str()
          : "N/A");
  RCLCPP_INFO(
      this->get_logger(),
      "[%s][SLAM][LOOP] hint maximum pose distance      : %s", this->get_name(),
      m_param_.m_slam_param.m_loop_enable
          ? std::to_string(m_param_.m_slam_param.m_loop_max_dist).c_str()
          : "N/A");

  bool is_print_loop_aligner_param = true;

  if (m_param_.m_slam_param.m_loop_aligner ==
      m_param_.m_slam_param.m_tracker->getAlignerPtr())
  {
    is_print_loop_aligner_param = false;

    RCLCPP_INFO(
        this->get_logger(),
        "[%s][SLAM][LOOP] loop aligner                    : same as the "
        "tracker",
        this->get_name());
  }
  else if (dynamic_cast<AlignerPt2PtType*>(
               m_param_.m_slam_param.m_loop_aligner.get()))
  {
    RCLCPP_INFO(
        this->get_logger(),
        "[%s][SLAM][LOOP] loop aligner type               : point2point "
        "(distance map-based)",
        this->get_name());
  }
  else if (dynamic_cast<AlignerPt2PlnType*>(
               m_param_.m_slam_param.m_loop_aligner.get()))
  {
    RCLCPP_INFO(
        this->get_logger(),
        "[%s][SLAM][LOOP] loop aligner type               : point2plane "
        "(distance map-based)",
        this->get_name());
  }
  else if (dynamic_cast<AlignerICPPt2PtType*>(
               m_param_.m_slam_param.m_loop_aligner.get()))
  {
    RCLCPP_INFO(
        this->get_logger(),
        "[%s][SLAM][LOOP] loop aligner type               : icp_point2point "
        "(distance map-based)",
        this->get_name());
  }
  else if (dynamic_cast<AlignerICPPt2PlnType*>(
               m_param_.m_slam_param.m_loop_aligner.get()))
  {
    RCLCPP_INFO(
        this->get_logger(),
        "[%s][SLAM][LOOP] loop aligner type               : icp_point2plane "
        "(distance map-based)",
        this->get_name());
  }
  else
  {
    is_print_loop_aligner_param = false;

    RCLCPP_INFO(
        this->get_logger(),
        "[%s][SLAM][LOOP] loop aligner type               : ...unknown...",
        this->get_name());
  }

  if (is_print_loop_aligner_param)
  {
    std::ostringstream param_print("");
    param_print << m_param_.m_slam_param.m_loop_aligner->getParam()
                << m_param_.m_slam_param.m_loop_aligner->getDistMapParam();

    RCLCPP_INFO(this->get_logger(),
                "[%s][SLAM][LOOP] loop aligner parametrisaation   :\n%s",
                this->get_name(), param_print.str().c_str());
  }

  // ----- SLAM RELOCALIZATION -------------------------------------------------

  RCLCPP_INFO(this->get_logger(),
              "[%s][SLAM][RELOCALIZATION] relocalize in the pose graph: %s",
              this->get_name(),
              m_param_.m_slam_param.m_relocalization_enable ? "enabled"
                                                            : "not enabled");

  // ----- TRACKER -------------------------------------------------------------

  RCLCPP_INFO(this->get_logger(),
              "[%s][TRACKER] use external odom as init : %s", this->get_name(),
              m_param_.m_slam_param.m_tracker->getParam().m_use_ext_odom_as_init
                  ? "enabled"
                  : "not enabled");
  RCLCPP_INFO(
      this->get_logger(), "[%s][TRACKER] use external odom as prior: %s",
      this->get_name(),
      m_param_.m_slam_param.m_tracker->getParam().m_use_ext_odom_as_prior
          ? "enabled"
          : "not enabled");

  // ----- TRACKER::SPLITTING CRITERIA -----------------------------------------

  if (dynamic_cast<SplittingCriteriaEmptyType*>(
          m_param_.m_slam_param.m_tracker->getParam()
              .m_splitting_criteria.get()))
  {
    RCLCPP_INFO(this->get_logger(),
                "[%s][TRACKER] splitting criteria type: empty (only forcing)",
                this->get_name());
  }
  else if (dynamic_cast<SplittingCriteriaRotationType*>(
               m_param_.m_slam_param.m_tracker->getParam()
                   .m_splitting_criteria.get()))
  {
    SplittingCriteriaRotationType* criteria =
        dynamic_cast<SplittingCriteriaRotationType*>(
            m_param_.m_slam_param.m_tracker->getParam()
                .m_splitting_criteria.get());

    RCLCPP_INFO(this->get_logger(),
                "[%s][TRACKER] splitting criteria type              : rotation",
                this->get_name());
    RCLCPP_INFO(this->get_logger(),
                "[%s][TRACKER] splitting criteria min travel heading: %f (deg)",
                this->get_name(),
                geometry::radToDeg(criteria->m_min_travel_heading));
  }
  else if (dynamic_cast<SplittingCriteriaTranslationType*>(
               m_param_.m_slam_param.m_tracker->getParam()
                   .m_splitting_criteria.get()))
  {
    SplittingCriteriaTranslationType* criteria =
        dynamic_cast<SplittingCriteriaTranslationType*>(
            m_param_.m_slam_param.m_tracker->getParam()
                .m_splitting_criteria.get());

    RCLCPP_INFO(
        this->get_logger(),
        "[%s][TRACKER] splitting criteria type              : translation",
        this->get_name());
    RCLCPP_INFO(this->get_logger(),
                "[%s][TRACKER] splitting criteria min travel dist   : %f (m)",
                this->get_name(), criteria->m_min_travel_dist);
  }
  else if (dynamic_cast<SplittingCriteriaViewpointType*>(
               m_param_.m_slam_param.m_tracker->getParam()
                   .m_splitting_criteria.get()))
  {
    SplittingCriteriaViewpointType* criteria =
        dynamic_cast<SplittingCriteriaViewpointType*>(
            m_param_.m_slam_param.m_tracker->getParam()
                .m_splitting_criteria.get());

    RCLCPP_INFO(
        this->get_logger(),
        "[%s][TRACKER] splitting criteria type              : viewpoint",
        this->get_name());
    RCLCPP_INFO(this->get_logger(),
                "[%s][TRACKER] splitting criteria min travel dist   : %f (m)",
                this->get_name(), criteria->m_min_travel_dist);
    RCLCPP_INFO(this->get_logger(),
                "[%s][TRACKER] splitting criteria min travel heading: %f (deg)",
                this->get_name(),
                geometry::radToDeg(criteria->m_min_travel_heading));
  }
  else if (dynamic_cast<SplittingCriteriaGridCellType*>(
               m_param_.m_slam_param.m_tracker->getParam()
                   .m_splitting_criteria.get()))
  {
    SplittingCriteriaGridCellType* criteria =
        dynamic_cast<SplittingCriteriaGridCellType*>(
            m_param_.m_slam_param.m_tracker->getParam()
                .m_splitting_criteria.get());

    RCLCPP_INFO(this->get_logger(),
                "[%s][TRACKER] splitting criteria type          : gridcell",
                this->get_name());
    RCLCPP_INFO(this->get_logger(),
                "[%s][TRACKER] splitting criteria grid cell size: %f (m)",
                this->get_name(), criteria->m_cell_size_2 * 2.f);
  }
  else if (dynamic_cast<SplittingCriteriaInliersRatioType*>(
               m_param_.m_slam_param.m_tracker->getParam()
                   .m_splitting_criteria.get()))
  {
    SplittingCriteriaInliersRatioType* criteria =
        dynamic_cast<SplittingCriteriaInliersRatioType*>(
            m_param_.m_slam_param.m_tracker->getParam()
                .m_splitting_criteria.get());

    RCLCPP_INFO(
        this->get_logger(),
        "[%s][TRACKER] splitting criteria type              : inliers ratio",
        this->get_name());
    RCLCPP_INFO(this->get_logger(),
                "[%s][TRACKER] splitting criteria min travel dist   : %f (m)",
                this->get_name(), criteria->m_min_travel_dist);
    RCLCPP_INFO(this->get_logger(),
                "[%s][TRACKER] splitting criteria min travel heading: %f (deg)",
                this->get_name(),
                geometry::radToDeg(criteria->m_min_travel_heading));
    RCLCPP_INFO(
        this->get_logger(),
        "[%s][TRACKER] splitting criteria max ratio inliers : %f (0..1)",
        this->get_name(), criteria->m_max_ratio_inliers);
    RCLCPP_INFO(this->get_logger(),
                "[%s][TRACKER] splitting criteria max travel dist   : %s (m)",
                this->get_name(),
                criteria->m_max_travel_dist > 0.f
                    ? std::to_string(criteria->m_max_travel_dist).c_str()
                    : "not enabled");
    RCLCPP_INFO(
        this->get_logger(),
        "[%s][TRACKER] splitting criteria max travel heading: %s (deg)",
        this->get_name(),
        criteria->m_max_travel_heading > 0.f
            ? std::to_string(geometry::radToDeg(criteria->m_max_travel_heading))
                  .c_str()
            : "not enabled");
  }
  else if (dynamic_cast<SplittingCriteriaTimeType*>(
               m_param_.m_slam_param.m_tracker->getParam()
                   .m_splitting_criteria.get()))
  {
    SplittingCriteriaTimeType* criteria =
        dynamic_cast<SplittingCriteriaTimeType*>(
            m_param_.m_slam_param.m_tracker->getParam()
                .m_splitting_criteria.get());

    RCLCPP_INFO(this->get_logger(),
                "[%s][TRACKER] splitting criteria type             : time",
                this->get_name());
    RCLCPP_INFO(this->get_logger(),
                "[%s][TRACKER] splitting criteria min time interval: %lf (s)",
                this->get_name(), criteria->m_min_time_interval);
    RCLCPP_INFO(this->get_logger(),
                "[%s][TRACKER] splitting criteria max ratio inliers: %s (0..1)",
                this->get_name(),
                criteria->m_max_ratio_inliers > 0.f
                    ? std::to_string(criteria->m_max_ratio_inliers).c_str()
                    : "not enabled");
  }
  else if (!m_param_.m_slam_param.m_tracker->getParam().m_splitting_criteria)
  {
    RCLCPP_INFO(this->get_logger(),
                "[%s][TRACKER] splitting criteria type  : N/A",
                this->get_name());
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(),
                 "[%s][TRACKER] splitting criteria type  : ...unknown...",
                 this->get_name());
  }

  // ----- TRACKER::MERGING CRITERIA -------------------------------------------

  if (dynamic_cast<MergingCriteriaEmptyType*>(
          m_param_.m_slam_param.m_tracker->getParam().m_merging_criteria.get()))
  {
    RCLCPP_INFO(this->get_logger(),
                "[%s][TRACKER] merging criteria type: empty (only forcing)",
                this->get_name());
  }
  else if (dynamic_cast<MergingCriteriaRotationType*>(
               m_param_.m_slam_param.m_tracker->getParam()
                   .m_merging_criteria.get()))
  {
    MergingCriteriaRotationType* criteria =
        dynamic_cast<MergingCriteriaRotationType*>(
            m_param_.m_slam_param.m_tracker->getParam()
                .m_merging_criteria.get());

    RCLCPP_INFO(this->get_logger(),
                "[%s][TRACKER] merging criteria type              : rotation",
                this->get_name());
    RCLCPP_INFO(this->get_logger(),
                "[%s][TRACKER] merging criteria min travel heading: %f (deg)",
                this->get_name(),
                geometry::radToDeg(criteria->m_min_travel_heading));
  }
  else if (dynamic_cast<MergingCriteriaTranslationType*>(
               m_param_.m_slam_param.m_tracker->getParam()
                   .m_merging_criteria.get()))
  {
    MergingCriteriaTranslationType* criteria =
        dynamic_cast<MergingCriteriaTranslationType*>(
            m_param_.m_slam_param.m_tracker->getParam()
                .m_merging_criteria.get());

    RCLCPP_INFO(
        this->get_logger(),
        "[%s][TRACKER] merging criteria type              : translation",
        this->get_name());
    RCLCPP_INFO(this->get_logger(),
                "[%s][TRACKER] merging criteria min travel dist   : %f (m)",
                this->get_name(), criteria->m_min_travel_dist);
  }
  else if (dynamic_cast<MergingCriteriaViewpointType*>(
               m_param_.m_slam_param.m_tracker->getParam()
                   .m_merging_criteria.get()))
  {
    MergingCriteriaViewpointType* criteria =
        dynamic_cast<MergingCriteriaViewpointType*>(
            m_param_.m_slam_param.m_tracker->getParam()
                .m_merging_criteria.get());

    RCLCPP_INFO(this->get_logger(),
                "[%s][TRACKER] merging criteria type              : viewpoint",
                this->get_name());
    RCLCPP_INFO(this->get_logger(),
                "[%s][TRACKER] merging criteria min travel dist   : %f (m)",
                this->get_name(), criteria->m_min_travel_dist);
    RCLCPP_INFO(this->get_logger(),
                "[%s][TRACKER] merging criteria min travel heading: %f (deg)",
                this->get_name(),
                geometry::radToDeg(criteria->m_min_travel_heading));
  }
  else if (dynamic_cast<MergingCriteriaGridCellType*>(
               m_param_.m_slam_param.m_tracker->getParam()
                   .m_merging_criteria.get()))
  {
    MergingCriteriaGridCellType* criteria =
        dynamic_cast<MergingCriteriaGridCellType*>(
            m_param_.m_slam_param.m_tracker->getParam()
                .m_merging_criteria.get());

    RCLCPP_INFO(this->get_logger(),
                "[%s][TRACKER] merging criteria type          : gridcell",
                this->get_name());
    RCLCPP_INFO(this->get_logger(),
                "[%s][TRACKER] merging criteria grid cell size: %f (m)",
                this->get_name(), criteria->m_cell_size_2 * 2.f);
  }
  else if (dynamic_cast<MergingCriteriaInliersRatioType*>(
               m_param_.m_slam_param.m_tracker->getParam()
                   .m_merging_criteria.get()))
  {
    MergingCriteriaInliersRatioType* criteria =
        dynamic_cast<MergingCriteriaInliersRatioType*>(
            m_param_.m_slam_param.m_tracker->getParam()
                .m_merging_criteria.get());

    RCLCPP_INFO(
        this->get_logger(),
        "[%s][TRACKER] merging criteria type              : inliers ratio",
        this->get_name());
    RCLCPP_INFO(this->get_logger(),
                "[%s][TRACKER] merging criteria min travel dist   : %f (m)",
                this->get_name(), criteria->m_min_travel_dist);
    RCLCPP_INFO(this->get_logger(),
                "[%s][TRACKER] merging criteria min travel heading: %f (deg)",
                this->get_name(),
                geometry::radToDeg(criteria->m_min_travel_heading));
    RCLCPP_INFO(this->get_logger(),
                "[%s][TRACKER] merging criteria max ratio inliers : %f (0..1)",
                this->get_name(), criteria->m_max_ratio_inliers);
    RCLCPP_INFO(this->get_logger(),
                "[%s][TRACKER] merging criteria max travel dist   : %s (m)",
                this->get_name(),
                criteria->m_max_travel_dist > 0.f
                    ? std::to_string(criteria->m_max_travel_dist).c_str()
                    : "not enabled");
    RCLCPP_INFO(
        this->get_logger(),
        "[%s][TRACKER] merging criteria max travel heading: %s (deg)",
        this->get_name(),
        criteria->m_max_travel_heading > 0.f
            ? std::to_string(geometry::radToDeg(criteria->m_max_travel_heading))
                  .c_str()
            : "not enabled");
  }
  else if (dynamic_cast<MergingCriteriaTimeType*>(
               m_param_.m_slam_param.m_tracker->getParam()
                   .m_merging_criteria.get()))
  {
    MergingCriteriaTimeType* criteria = dynamic_cast<MergingCriteriaTimeType*>(
        m_param_.m_slam_param.m_tracker->getParam().m_merging_criteria.get());

    RCLCPP_INFO(this->get_logger(),
                "[%s][TRACKER] merging criteria type             : time",
                this->get_name());
    RCLCPP_INFO(this->get_logger(),
                "[%s][TRACKER] merging criteria min time interval: %lf (s)",
                this->get_name(), criteria->m_min_time_interval);
    RCLCPP_INFO(this->get_logger(),
                "[%s][TRACKER] merging criteria max ratio inliers: %s (0..1)",
                this->get_name(),
                criteria->m_max_ratio_inliers > 0.f
                    ? std::to_string(criteria->m_max_ratio_inliers).c_str()
                    : "not enabled");
  }
  else if (!m_param_.m_slam_param.m_tracker->getParam().m_merging_criteria)
  {
    RCLCPP_INFO(this->get_logger(),
                "[%s][TRACKER] merging criteria type  : N/A", this->get_name());
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(),
                 "[%s][TRACKER] merging criteria type  : ...unknown...",
                 this->get_name());
  }

  // ----- TRACKER::MERGER -----------------------------------------------------

  if (m_param_.m_slam_param.m_tracker->getParam().m_merging_criteria)
  {
    RCLCPP_INFO(this->get_logger(),
                "[%s][TRACKER] merger ray min dist see through : %f",
                this->get_name(),
                m_param_.m_slam_param.m_tracker->getParam()
                    .m_merger->getParam()
                    .m_ray_min_dist_through);
    RCLCPP_INFO(this->get_logger(),
                "[%s][TRACKER] merger ray min dist add new pt  : %f",
                this->get_name(),
                m_param_.m_slam_param.m_tracker->getParam()
                    .m_merger->getParam()
                    .m_ray_min_dist_add_new);
    RCLCPP_INFO(this->get_logger(),
                "[%s][TRACKER] merger ray average measurements : %s",
                this->get_name(),
                m_param_.m_slam_param.m_tracker->getParam()
                        .m_merger->getParam()
                        .m_enable_avg
                    ? "enabled"
                    : "not enabled");
    RCLCPP_INFO(this->get_logger(),
                "[%s][TRACKER] merger ray average weight old pt: %s",
                this->get_name(),
                m_param_.m_slam_param.m_tracker->getParam()
                        .m_merger->getParam()
                        .m_enable_avg
                    ? std::to_string(m_param_.m_slam_param.m_tracker->getParam()
                                         .m_merger->getParam()
                                         .m_weight_old)
                          .c_str()
                    : "N/A");
    RCLCPP_INFO(this->get_logger(),
                "[%s][TRACKER] merger ray average weight new pt: %s",
                this->get_name(),
                m_param_.m_slam_param.m_tracker->getParam()
                        .m_merger->getParam()
                        .m_enable_avg
                    ? std::to_string(m_param_.m_slam_param.m_tracker->getParam()
                                         .m_merger->getParam()
                                         .m_weight_new)
                          .c_str()
                    : "N/A");
  }

  // ----- TRACKER::ALIGNER ----------------------------------------------------

  bool is_tracker_aligner_valid = true;

  if (dynamic_cast<AlignerPt2PtType*>(
          m_param_.m_slam_param.m_tracker->getParam().m_aligner.get()))
  {
    RCLCPP_INFO(this->get_logger(),
                "[%s][TRACKER] aligner type           : point2point (distance "
                "map-based)",
                this->get_name());
  }
  else if (dynamic_cast<AlignerPt2PlnType*>(
               m_param_.m_slam_param.m_tracker->getParam().m_aligner.get()))
  {
    RCLCPP_INFO(this->get_logger(),
                "[%s][TRACKER] aligner type           : point2plane (distance "
                "map-based)",
                this->get_name());
  }
  else if (dynamic_cast<AlignerICPPt2PtType*>(
               m_param_.m_slam_param.m_tracker->getParam().m_aligner.get()))
  {
    RCLCPP_INFO(
        this->get_logger(),
        "[%s][TRACKER] aligner type           : icp_point2point (distance "
        "map-based)",
        this->get_name());
  }
  else if (dynamic_cast<AlignerICPPt2PlnType*>(
               m_param_.m_slam_param.m_tracker->getParam().m_aligner.get()))
  {
    RCLCPP_INFO(
        this->get_logger(),
        "[%s][TRACKER] aligner type           : icp_point2plane (distance "
        "map-based)",
        this->get_name());
  }
  else
  {
    is_tracker_aligner_valid = false;

    RCLCPP_ERROR(this->get_logger(),
                 "[%s][TRACKER] aligner type           : ...unknown...",
                 this->get_name());
  }

  if (is_tracker_aligner_valid)
  {
    std::ostringstream param_print("");
    param_print
        << m_param_.m_slam_param.m_tracker->getParam().m_aligner->getParam()
        << m_param_.m_slam_param.m_tracker->getParam()
               .m_aligner->getDistMapParam();

    RCLCPP_INFO(this->get_logger(),
                "[%s][TRACKER] aligner parametrisation:\n%s", this->get_name(),
                param_print.str().c_str());
  }

  // ----- TRACKER::PREPROCESS -------------------------------------------------

  RCLCPP_INFO(this->get_logger(),
              "[%s][TRACKER] preprocess enable voxelization   : %s",
              this->get_name(),
              m_param_.m_slam_param.m_tracker->getParam().m_enable_voxelization
                  ? "enabled"
                  : "not enabled");
  RCLCPP_INFO(
      this->get_logger(), "[%s][TRACKER] preprocess voxelization scale (m): %s",
      this->get_name(),
      !m_param_.m_slam_param.m_tracker->getParam().m_enable_voxelization
          ? "N/A"
          : (m_param_.m_slam_param.m_tracker->getParam().m_voxel_scale <= 0
                 ? "distance map resolution"
                 : std::to_string(m_param_.m_slam_param.m_tracker->getParam()
                                      .m_voxel_scale)
                       .c_str()));

  RCLCPP_INFO(
      this->get_logger(),
      "[%s][TRACKER] preprocess normals computation                    : %s",
      this->get_name(),
      m_param_.m_slam_param.m_tracker->getParam().m_enable_normals
          ? "enabled"
          : "not enabled");
  RCLCPP_INFO(
      this->get_logger(),
      "[%s][TRACKER] preprocess normals neighborhood minimum points    : %s",
      this->get_name(),
      !m_param_.m_slam_param.m_tracker->getParam().m_enable_normals
          ? "N/A"
          : std::to_string(m_param_.m_slam_param.m_tracker->getParam()
                               .m_normals_computator_min_pts)
                .c_str());
  RCLCPP_INFO(
      this->get_logger(),
      "[%s][TRACKER] preprocess normals nearest neighborhood radius (m): %s",
      this->get_name(),
      !m_param_.m_slam_param.m_tracker->getParam().m_enable_normals
          ? "N/A"
          : std::to_string(m_param_.m_slam_param.m_tracker->getParam()
                               .m_normals_computator_radius)
                .c_str());
  RCLCPP_INFO(
      this->get_logger(),
      "[%s][TRACKER] preprocess laser minimum range threshold       (m): %s",
      this->get_name(),
      m_param_.m_slam_param.m_tracker->getParam().m_range_min > 0
          ? std::to_string(
                m_param_.m_slam_param.m_tracker->getParam().m_range_min)
                .c_str()
          : "read from msg.range_min");
  RCLCPP_INFO(
      this->get_logger(),
      "[%s][TRACKER] preprocess laser maximum range threshold       (m): %s",
      this->get_name(),
      m_param_.m_slam_param.m_tracker->getParam().m_range_max > 0
          ? std::to_string(
                m_param_.m_slam_param.m_tracker->getParam().m_range_max)
                .c_str()
          : "read from msg.range_max");
}

void SLAMROS2API::subScan(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr scan)
{
  /* if (!m_param_.m_ext_odom_use)
  {
    RCLCPP_INFO(this->get_logger(), "[%s][SCAN] %lf", this->get_name(),
                rclcpp::Time(scan->header.stamp).seconds());
  } */

  if (m_is_first_msg_)
  {
    m_is_first_msg_ = false;

    m_scan_stamp_prev_ = rclcpp::Time(scan->header.stamp).seconds();

    m_param_.m_laser_frame = scan->header.frame_id;
  }
  else
  {
    if (!checkDataLossAndCausality(*scan))
    {
      return;
    }
  }

  if (!m_is_tf_init_)
  {
    if (!setupSensorInRobot(scan->header.frame_id))
    {
      return;
    }

    m_is_tf_init_ = true;
  }

  LaserScanMsgTypePtr data = std::make_shared<LaserScanMsgType>(
      inesctec_mrdt_slam_distmap_2d::slam::msg::Header(
          rclcpp::Time(scan->header.stamp).seconds(), scan->header.frame_id),
      scan->angle_min, scan->angle_max, scan->angle_increment,
      scan->time_increment, scan->scan_time, scan->range_min, scan->range_max,
      scan->ranges, scan->intensities);

  EstimateType ext_odom_t = EstimateType::Identity();

  if (m_param_.m_ext_odom_use)
  {
    try
    {
      geometry_msgs::msg::TransformStamped ext_odom =
          m_tf2_buffer_->lookupTransform(m_param_.m_ext_odom_frame,
                                         scan->header.frame_id,
                                         scan->header.stamp);

      ext_odom_t = geometry::v2t(geometry::Vector3_<Scalar>(
          ext_odom.transform.translation.x, ext_odom.transform.translation.y,
          tf2::getYaw(ext_odom.transform.rotation)));

      /* RCLCPP_INFO(this->get_logger(), "[%s][SCAN] %lf \t [ODOM TF] %lf",
                  this->get_name(), rclcpp::Time(scan->header.stamp).seconds(),
                  rclcpp::Time(ext_odom.header.stamp).seconds()); */

      if (m_param_.m_ext_odom_pub_msg)
      {
        pubExtOdom(ext_odom);
      }
    }
    catch (const tf2::TransformException& e)
    {
      RCLCPP_WARN(this->get_logger(),
                  "[%s] m_tf_buffer_.lookupTransform exception thrown (%s)",
                  this->get_name(), e.what());
      return;
    }
  }

  m_slam_->processScan(data, ext_odom_t);

  if (m_slam_->getTracker()->getStatus() == TrackerType::Status::Error)
  {
    RCLCPP_WARN(
        this->get_logger(),
        "[%s] Tracker_::processPointCloud | keyframe lost... unsucessful "
        "match...",
        this->get_name());
  }

  if ((m_param_.m_slam_pub_odom_tf) || (m_param_.m_slam_pub_odom_msg))
  {
    pubOdom(scan->header);
  }

  if ((m_param_.m_slam_pub_pose_tf) || (m_param_.m_slam_pub_pose_msg))
  {
    pubPose(scan->header);
  }

  if (m_slam_->getParam().m_solver_debug)
  {
    m_slam_->updateDbgOPGLCanvas();
  }

  /* std::cout << "local: "
            << geometry::t2v(m_tracker_->getRobotInLocalMap()).transpose()
            << std::endl;
  std::cout << "odom : "
            << geometry::t2v(m_tracker_->getRobotInOdom()).transpose()
            << std::endl; */
}

void SLAMROS2API::subOdomScan(
    const nav_msgs::msg::Odometry::ConstSharedPtr odom,
    const sensor_msgs::msg::LaserScan::ConstSharedPtr scan)
{
  /* RCLCPP_INFO(this->get_logger(), "[%s][SCAN] %lf \t [ODOM] %lf",
              this->get_name(), rclcpp::Time(scan->header.stamp).seconds(),
              rclcpp::Time(odom->header.stamp).seconds()); */

  if (m_is_first_msg_)
  {
    m_is_first_msg_ = false;

    m_scan_stamp_prev_ = rclcpp::Time(scan->header.stamp).seconds();

    m_param_.m_ext_odom_frame = odom->header.frame_id;
    m_param_.m_laser_frame = scan->header.frame_id;
  }
  else
  {
    if (!checkDataLossAndCausality(*scan))
    {
      return;
    }
  }

  if (!m_is_tf_init_)
  {
    if (!setupSensorInRobot(scan->header.frame_id))
    {
      return;
    }

    m_is_tf_init_ = true;
  }

  LaserScanMsgTypePtr data = std::make_shared<LaserScanMsgType>(
      inesctec_mrdt_slam_distmap_2d::slam::msg::Header(
          rclcpp::Time(scan->header.stamp).seconds(), scan->header.frame_id),
      scan->angle_min, scan->angle_max, scan->angle_increment,
      scan->time_increment, scan->scan_time, scan->range_min, scan->range_max,
      scan->ranges, scan->intensities);

  EstimateType ext_odom_t = EstimateType::Identity();

  if (m_param_.m_ext_odom_use)
  {
    ext_odom_t = geometry::v2t(geometry::Vector3_<Scalar>(
        odom->pose.pose.position.x, odom->pose.pose.position.y,
        tf2::getYaw(odom->pose.pose.orientation)));

    if (m_param_.m_ext_odom_pub_tf)
    {
      pubExtOdomTF(*odom);
    }
  }

  m_slam_->processScan(data, ext_odom_t);

  if (m_slam_->getTracker()->getStatus() == TrackerType::Status::Error)
  {
    RCLCPP_WARN(
        this->get_logger(),
        "[%s] Tracker_::processPointCloud | keyframe lost... unsucessful "
        "match...",
        this->get_name());
  }

  if ((m_param_.m_slam_pub_odom_tf) || (m_param_.m_slam_pub_odom_msg))
  {
    pubOdom(scan->header);
  }

  if ((m_param_.m_slam_pub_pose_tf) || (m_param_.m_slam_pub_pose_msg))
  {
    pubPose(scan->header);
  }

  if (m_slam_->getParam().m_solver_debug)
  {
    m_slam_->updateDbgOPGLCanvas();
  }

  /* std::cout << "local: "
            << geometry::t2v(m_tracker_->getRobotInLocalMap()).transpose()
            << std::endl;
  std::cout << "odom : "
            << geometry::t2v(m_tracker_->getRobotInOdom()).transpose()
            << std::endl; */
}

bool SLAMROS2API::setupSensorInRobot(const std::string& scan_frame)
{
  if (m_param_.m_laser_tf_static_yaml)
  {
    m_slam_->getParam().m_tracker->setSensorInRobot(
        geometry::v2t(m_param_.m_laser_pose.cast<Scalar>()));
  }
  else
  {
    geometry_msgs::msg::TransformStamped sensor_in_robot;

    try
    {
      sensor_in_robot = m_tf2_buffer_->lookupTransform(
          m_param_.m_base_frame, scan_frame, tf2::TimePointZero);
    }
    catch (const tf2::TransformException& e)
    {
      RCLCPP_WARN(
          this->get_logger(),
          "[%s] m_tf_buffer_.lookupTransform exception thrown when setting "
          "up the ssensor (%s) w.r.t. robot (%s) transformation (%s)",
          this->get_name(), scan_frame.c_str(), m_param_.m_base_frame.c_str(),
          e.what());
      return false;
    }

    m_slam_->getParam().m_tracker->setSensorInRobot(
        geometry::v2t(geometry::Vector3_<Scalar>(
            static_cast<Scalar>(sensor_in_robot.transform.translation.x),
            static_cast<Scalar>(sensor_in_robot.transform.translation.y),
            static_cast<Scalar>(
                tf2::getYaw(sensor_in_robot.transform.rotation)))));
  }

  RCLCPP_INFO(
      this->get_logger(),
      "[%s] sensor (%s) w.r.t. (%s) frame transformation: "
      "( %f m , %f m , %f deg )",
      this->get_name(), scan_frame.c_str(), m_param_.m_base_frame.c_str(),
      m_slam_->getParam().m_tracker->getSensorInRobot().translation().x(),
      m_slam_->getParam().m_tracker->getSensorInRobot().translation().y(),
      geometry::radToDeg(
          geometry::t2v(m_slam_->getParam().m_tracker->getSensorInRobot())
              .z()));

  return true;
}

bool SLAMROS2API::checkDataLossAndCausality(
    const sensor_msgs::msg::LaserScan& scan)
{
  double scan_stamp_curr = rclcpp::Time(scan.header.stamp).seconds();

  if (scan_stamp_curr <= m_scan_stamp_prev_)
  {
    RCLCPP_ERROR(this->get_logger(),
                 "[%s] tracker causality assumption broken! you are looking "
                 "into the past... (t_curr: %lf vs t_prev: %lf)",
                 this->get_name(), scan_stamp_curr, m_scan_stamp_prev_);
    return false;
  }

  m_scan_stamp_prev_ = scan_stamp_curr;

  return true;
}

bool SLAMROS2API::srvTriggerMergingCriteria(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  if (m_slam_->getTracker()->getMergingCriteriaPtr())
  {
    m_slam_->getTracker()->getMergingCriteria()->forceTrigger();

    res->success = true;
    res->message = "";
  }
  else
  {
    res->success = false;
    res->message =
        "SLAMROS2API::srvTriggerMergingCriteria | merging criteria not set";
  }

  return res->success;
}

bool SLAMROS2API::srvTriggerSplittingCriteria(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  if (m_slam_->getTracker()->getSplittingCriteriaPtr())
  {
    m_slam_->getTracker()->getSplittingCriteria()->forceTrigger();

    res->success = true;
    res->message = "";
  }
  else
  {
    res->success = false;
    res->message =
        "SLAMROS2API::srvTriggerSplittingCriteria | splitting criteria not "
        "set";
  }

  return res->success;
}

bool SLAMROS2API::srvSaveGraph(
    const std::shared_ptr<
        inesctec_mrdt_slam_distmap_2d_ros::srv::SetFilename::Request>
        req,
    std::shared_ptr<
        inesctec_mrdt_slam_distmap_2d_ros::srv::SetFilename::Response>
        res)
{
  std::string filename("");

  if (!req->filename.empty())
  {
    filename = req->filename;

    try
    {
      std::filesystem::path filename_path(filename);

      if (filename_path.extension().string().compare(".json") != 0)
      {
        res->success = false;
        res->message =
            "SLAMROS2API::srvSaveGraph | only supports JSON extension";

        RCLCPP_ERROR(
            this->get_logger(),
            "[%s] something occurred when saving the factor graph... (filename "
            "only supports JSON extension)",
            this->get_name());

        return false;
      }

      system_utils::validateAndCreatePath(filename);
    }
    catch (const std::filesystem::filesystem_error& e)
    {
      res->success = false;
      res->message = "SLAMROS2API::srvSaveGraph | Error resolving paths for '" +
                     filename + "'; error: " + e.what();

      RCLCPP_ERROR(
          this->get_logger(),
          "[%s] something occurred when resolving paths for '%s'; error: %s)",
          this->get_name(), filename.c_str(), e.what());

      return false;
    }
    catch (const std::exception& e)
    {
      res->success = false;
      res->message =
          "SLAMROS2API::srvSaveGraph | Error when processing paths for map "
          "file; error: " +
          std::string(e.what());

      RCLCPP_ERROR(
          this->get_logger(),
          "[%s] something occurred when processing paths for map file (%s)",
          this->get_name(), e.what());

      return false;
    }
    catch (...)
    {
      res->success = false;
      res->message =
          "SLAMROS2API::srvSaveGraph | Unknown error when processing "
          "paths for map file";

      RCLCPP_ERROR(
          this->get_logger(),
          "[%s] ...unknown error when processing paths for map file...",
          this->get_name());

      return false;
    }
  }
  else
  {
    filename = "map.json";
  }

  std::filesystem::path current_path = std::filesystem::current_path();

  RCLCPP_INFO(this->get_logger(),
              "[%s] SLAMROS2API::srvSaveGraph | current path: %s",
              this->get_name(), current_path.c_str());
  RCLCPP_INFO(this->get_logger(),
              "[%s] SLAMROS2API::srvSaveGraph | map filename: %s",
              this->get_name(), filename.c_str());

  try
  {
    m_slam_->saveGraph(filename);
  }
  catch (const std::exception& e)
  {
    res->success = false;
    res->message = std::string(e.what());

    RCLCPP_ERROR(this->get_logger(),
                 "[%s] something occurred when saving the factor graph... (%s)",
                 this->get_name(), e.what());

    return false;
  }

  res->success = true;
  res->message.clear();

  return true;
}

bool SLAMROS2API::srvSaveCorrectedPose(
    const std::shared_ptr<
        inesctec_mrdt_slam_distmap_2d_ros::srv::SetFilename::Request>
        req,
    std::shared_ptr<
        inesctec_mrdt_slam_distmap_2d_ros::srv::SetFilename::Response>
        res)
{
  std::string filename("");

  if (!req->filename.empty())
  {
    filename = req->filename;

    try
    {
      system_utils::validateAndCreatePath(filename);
    }
    catch (const std::filesystem::filesystem_error& e)
    {
      res->success = false;
      res->message =
          "SLAMROS2API::srvSaveCorrectedPose | Error resolving paths for '" +
          filename + "'; error: " + e.what();

      RCLCPP_ERROR(
          this->get_logger(),
          "[%s] something occurred when resolving paths for '%s'; error: %s)",
          this->get_name(), filename.c_str(), e.what());

      return false;
    }
    catch (const std::exception& e)
    {
      res->success = false;
      res->message =
          "SLAMROS2API::srvSaveCorrectedPose | Error when processing paths for "
          "robot offline corrected pose file; error: " +
          std::string(e.what());

      RCLCPP_ERROR(
          this->get_logger(),
          "[%s] something occurred when processing paths for robot offline "
          "corrected pose file (%s)",
          this->get_name(), e.what());

      return false;
    }
    catch (...)
    {
      res->success = false;
      res->message =
          "SLAMROS2API::srvSaveCorrectedPose | Unknown error when processing "
          "paths for robot offline corrected pose file";

      RCLCPP_ERROR(
          this->get_logger(),
          "[%s] ...unknown error when processing paths for robot offline "
          "corrected pose file...",
          this->get_name());

      return false;
    }
  }
  else
  {
    filename = "ricoslam_slam_pose_corrected_offline.tum";
  }

  std::filesystem::path current_path = std::filesystem::current_path();

  RCLCPP_INFO(this->get_logger(),
              "[%s] SLAMROS2API::srvSaveCorrectedPose | current path : %s",
              this->get_name(), current_path.c_str());
  RCLCPP_INFO(this->get_logger(),
              "[%s] SLAMROS2API::srvSaveCorrectedPose | pose filename: %s",
              this->get_name(), filename.c_str());

  try
  {
    auto sensor_pose_history = m_slam_->getRobotPoseHistory();

    std::ofstream log_file_pose_history(filename);

    if (!log_file_pose_history.is_open())
    {
      res->success = false;
      res->message =
          std::string("SLAMROS2API::srvSaveCorrectedPose | file (" + filename +
                      ") for corrected sensor pose data not opened");

      RCLCPP_ERROR(
          this->get_logger(),
          "[%s] something occurred when saving the corrected pose data... "
          "(file %s for corrected sensor pose data not opened)",
          this->get_name(), filename.c_str());

      return false;
    }

    for (const auto& sensor_pose_tf : sensor_pose_history)
    {
      srrg2_solver::VariableBase* var =
          m_slam_->getGraph()->variable(sensor_pose_tf.m_local_map_id);

      if (!var)
      {
        throw std::runtime_error("GridMapper::updateMap | graph variable " +
                                 std::to_string(sensor_pose_tf.m_local_map_id) +
                                 " not found when preprocessing the scans");
      }

      srrg2_solver::VariableSE2Base* v =
          dynamic_cast<srrg2_solver::VariableSE2Base*>(var);

      if (!v)
      {
        throw std::runtime_error(
            "GridMapper::updateMap | invalid variable dynamic cast to "
            "VariableSE2Base when preprocessing the scans");
      }

      const auto sensor_pose_t =
          v->estimate() * sensor_pose_tf.m_pose_in_local_map;
      /* * sensor_pose_tf.m_sensor_in_robot; */

      EstimateVectorType sensor_pose_v = geometry::t2v(sensor_pose_t);
      Eigen::Quaternionf sensor_pose_q(geometry::a2q(sensor_pose_v.z()));

      log_file_pose_history
          << std::fixed << std::setprecision(9) << sensor_pose_tf.m_t << " "
          << sensor_pose_v.x() << " " << sensor_pose_v.y() << " " << 0 << " "
          << sensor_pose_q.x() << " " << sensor_pose_q.y() << " "
          << sensor_pose_q.z() << " " << sensor_pose_q.w() << std::endl;
    }

    if (log_file_pose_history.is_open())
    {
      log_file_pose_history.close();
    }
  }
  catch (const std::exception& e)
  {
    res->success = false;
    res->message = std::string(e.what());

    RCLCPP_ERROR(
        this->get_logger(),
        "[%s] something occurred when saving the corrected pose data... (%s)",
        this->get_name(), e.what());

    return false;
  }

  res->success = true;
  res->message.clear();

  return true;
}

}  // namespace inesctec_mrdt_slam_distmap_2d_ros
