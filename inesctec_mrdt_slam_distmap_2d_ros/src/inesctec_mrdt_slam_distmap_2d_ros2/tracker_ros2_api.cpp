#include "inesctec_mrdt_slam_distmap_2d_ros/tracker_ros2_api.hpp"

#include <exception>
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

// Tracker C++ Implementation
#include <inesctec_mrdt_slam_distmap_2d/data_structures/instances.hpp>
#include <inesctec_mrdt_slam_distmap_2d/geometry/geometry_2d.hpp>
#include <inesctec_mrdt_slam_distmap_2d/srrg2_solver/instances.hpp>

namespace inesctec_mrdt_slam_distmap_2d_ros
{

using namespace inesctec_mrdt_slam_distmap_2d;

TrackerROS2API::TrackerROS2API(const std::string& name,
                               const rclcpp::NodeOptions& options)
    : rclcpp::Node(name, options), m_is_first_msg_(true)
{
  // Register types for BOSS serialization
  srrg2_solver::inesctec_mrdt_slam_distmap_2d_data_structures_registerTypes();
  srrg2_solver::inesctec_mrdt_slam_distmap_2d_srrg2_solver_registerTypes();

  m_tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  m_tf2_buffer_->setCreateTimerInterface(
      std::make_shared<tf2_ros::CreateTimerROS>(
          this->get_node_base_interface(), this->get_node_timers_interface()));

  readParam();
  printParam();

  m_tracker_ = std::make_shared<TrackerType>(m_param_.m_tracker_param);

  std::ostringstream param_print("");
  param_print << m_tracker_->getParam();

  RCLCPP_INFO(
      this->get_logger(),
      "[%s][TRACKER] ...checking tracker loaded parametrisation:...\n%s",
      this->get_name(), param_print.str().c_str());

  if (m_tracker_->getMergingCriteriaPtr())
  {
    m_srv_trigger_merging_ = this->create_service<std_srvs::srv::Trigger>(
        "~/trigger_merging",
        std::bind(&TrackerROS2API::srvTriggerMergingCriteria, this,
                  std::placeholders::_1, std::placeholders::_2));
  }

  if (m_tracker_->getSplittingCriteriaPtr())
  {
    m_srv_trigger_splitting_ = this->create_service<std_srvs::srv::Trigger>(
        "~/trigger_splitting",
        std::bind(&TrackerROS2API::srvTriggerSplittingCriteria, this,
                  std::placeholders::_1, std::placeholders::_2));
  }
}

void TrackerROS2API::readParam()
{
  if (!this->has_parameter("base_frame"))
  {
    throw std::runtime_error(
        "TrackerROS2API::readParam | base_frame missing...");
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
          "TrackerROS2API::readParam | unknown ext_odom_impl value: " +
          m_param_.m_ext_odom_impl + " (topic|tf)");
    }
    if ((m_param_.m_ext_odom_impl.compare("tf") == 0) &&
        !(this->has_parameter("ext_odom_frame")))
    {
      throw std::runtime_error(
          "TrackerROS2API::readParam | ext_odom_frame missing when "
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
          "TrackerROS2API::readParam | ext_odom_impl TF implementation and "
          "ext_odom_pub_tf both enabled does not make any sense!");
    }
    if ((m_param_.m_ext_odom_impl.compare("topic") == 0) &&
        m_param_.m_ext_odom_pub_msg)
    {
      throw std::runtime_error(
          "TrackerROS2API::readParam | ext_odom_impl topic implementation and "
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

  this->get_parameter_or<bool>(
      "laser_invert_scan", m_param_.m_tracker_param.m_laser_invert_scan, false);
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
          "TrackerROS2API::readParam | pre-defined laser pose w.r.t. robot "
          "must be defined completely (laser_pose_x & laser_pose_y & "
          "laser_pose_th)");
    }
    if (!this->has_parameter("laser_frame") && m_param_.m_laser_pub_tf_static)
    {
      throw std::runtime_error(
          "TrackerROS2API::readParam | pre-defined laser pose w.r.t. robot "
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
        "TrackerROS2API::readParam | pre-defined laser pose w.r.t. robot must "
        "be defined completely (laser_pose_x & laser_pose_y & laser_pose_th) "
        "when laser_pub_tf_static enabled");
  }

  // ----- TRACKER -------------------------------------------------------------

  this->get_parameter_or<bool>("tracker_pub_tf", m_param_.m_tracker_pub_tf,
                               true);
  this->get_parameter_or<bool>("tracker_pub_odom_msg",
                               m_param_.m_tracker_pub_odom_msg, false);
  this->get_parameter_or<std::string>("tracker_odom_frame",
                                      m_param_.m_tracker_odom_frame, "odom");

  if (m_param_.m_tracker_pub_tf)
  {
    this->get_parameter_or<bool>("tracker_odom_invert_tf",
                                 m_param_.m_tracker_odom_invert_tf, false);
  }
  else
  {
    m_param_.m_tracker_odom_invert_tf = false;
  }

  if (m_param_.m_laser_tf_static_yaml && !m_param_.m_tracker_odom_invert_tf)
  {
    throw std::runtime_error(
        "TrackerROS2API::readParam | pre-defined laser pose w.r.t. robot "
        "defined but tracker_odom_invert_tf disabled (when laser "
        "pose set, tracker publishes the laser odometry w.r.t. sensor, not "
        "the robot base frame)");
  }
  if ((m_param_.m_ext_odom_impl.compare("tf") == 0) &&
      m_param_.m_tracker_pub_tf && !m_param_.m_tracker_odom_invert_tf)
  {
    throw std::runtime_error(
        "TrackerROS2API::readParam | ext_odom_impl TF implementation and "
        "tracker_pub_tf enabled but tracker_odom_invert_tf disabled (breaks "
        "the TF tree structure of a frame having a single parent and multiple "
        "child frames)");
  }

  this->get_parameter_or<bool>("tracker_use_ext_odom_as_init",
                               m_param_.m_tracker_param.m_use_ext_odom_as_init,
                               false);
  this->get_parameter_or<bool>("tracker_use_ext_odom_as_prior",
                               m_param_.m_tracker_param.m_use_ext_odom_as_prior,
                               false);

  if ((m_param_.m_tracker_param.m_use_ext_odom_as_init ||
       m_param_.m_tracker_param.m_use_ext_odom_as_prior) &&
      !m_param_.m_ext_odom_use)
  {
    throw std::runtime_error(
        "TrackerROS2API::readParam | ext_odom_use must be "
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

      m_param_.m_tracker_param.m_splitting_criteria =
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

      m_param_.m_tracker_param.m_splitting_criteria =
          std::dynamic_pointer_cast<SplittingCriteriaType>(criteria);
    }
    else if (tracker_split_criteria_type.compare("translation") == 0)
    {
      SplittingCriteriaTranslationTypePtr criteria =
          std::make_shared<SplittingCriteriaTranslationType>();

      this->get_parameter_or<float>("tracker_split_criteria_min_travel_dist",
                                    criteria->m_min_travel_dist, 0.50f);

      m_param_.m_tracker_param.m_splitting_criteria =
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

      m_param_.m_tracker_param.m_splitting_criteria =
          std::dynamic_pointer_cast<SplittingCriteriaType>(criteria);
    }
    else if (tracker_split_criteria_type.compare("gridcell") == 0)
    {
      SplittingCriteriaGridCellTypePtr criteria =
          std::make_shared<SplittingCriteriaGridCellType>();

      this->get_parameter_or<float>("tracker_split_criteria_cell_size",
                                    criteria->m_cell_size_2, 2.5f);

      criteria->m_cell_size_2 *= 0.5f;

      m_param_.m_tracker_param.m_splitting_criteria =
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

      m_param_.m_tracker_param.m_splitting_criteria =
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

      m_param_.m_tracker_param.m_splitting_criteria =
          std::dynamic_pointer_cast<SplittingCriteriaType>(criteria);
    }
    else
    {
      throw std::runtime_error(
          "TrackerROS2API::readParam | unknown tracker_split_criteria_type "
          "value: " +
          tracker_split_criteria_type +
          " (empty|rotation|translation|viewpoint|gridcell|inliers|time)");
    }
  }
  else
  {
    m_param_.m_tracker_param.m_splitting_criteria = nullptr;
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

      m_param_.m_tracker_param.m_merging_criteria =
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

      m_param_.m_tracker_param.m_merging_criteria =
          std::dynamic_pointer_cast<MergingCriteriaType>(criteria);
    }
    else if (tracker_merge_criteria_type.compare("translation") == 0)
    {
      MergingCriteriaTranslationTypePtr criteria =
          std::make_shared<MergingCriteriaTranslationType>();

      this->get_parameter_or<float>("tracker_merge_criteria_min_travel_dist",
                                    criteria->m_min_travel_dist, 0.50f);

      m_param_.m_tracker_param.m_merging_criteria =
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

      m_param_.m_tracker_param.m_merging_criteria =
          std::dynamic_pointer_cast<MergingCriteriaType>(criteria);
    }
    else if (tracker_merge_criteria_type.compare("gridcell") == 0)
    {
      MergingCriteriaGridCellTypePtr criteria =
          std::make_shared<MergingCriteriaGridCellType>();

      this->get_parameter_or<float>("tracker_merge_criteria_cell_size",
                                    criteria->m_cell_size_2, 2.5f);

      criteria->m_cell_size_2 *= 0.5f;

      m_param_.m_tracker_param.m_merging_criteria =
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

      m_param_.m_tracker_param.m_merging_criteria =
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

      m_param_.m_tracker_param.m_merging_criteria =
          std::dynamic_pointer_cast<MergingCriteriaType>(criteria);
    }
    else
    {
      throw std::runtime_error(
          "TrackerROS2API::readParam | unknown tracker_merge_criteria_type "
          "value: " +
          tracker_merge_criteria_type +
          " (empty|rotation|translation|viewpoint|gridcell|inliers|time)");
    }
  }
  else
  {
    m_param_.m_tracker_param.m_merging_criteria = nullptr;
  }

  // ----- TRACKER::MERGER -----------------------------------------------------

  if (m_param_.m_tracker_param.m_merging_criteria)
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
            "TrackerROS2API::readParam | "
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

    m_param_.m_tracker_param.m_merger =
        std::make_shared<MergerType>(merger_param);
  }

  // ----- TRACKER::ALIGNER ----------------------------------------------------

  std::string tracker_aligner_type;

  this->get_parameter_or<std::string>("tracker_aligner_type",
                                      tracker_aligner_type, "point2point");

  AlignerTypeParamSolver param_solver;
  AlignerTypeParamDistMap param_dist_map;

  int dbg_ogl_interval;

  this->get_parameter_or<bool>("tracker_aligner_debug", param_solver.m_debug,
                               false);
  this->get_parameter_or<bool>("tracker_aligner_solver_verbose",
                               param_solver.m_solver_verbose, false);
  this->get_parameter_or<float>("tracker_aligner_solver_damping",
                                param_solver.m_solver_damping, 1.0f);
  this->get_parameter_or<int>("tracker_aligner_solver_max_iterations",
                              param_solver.m_solver_max_iterations, 25);
  this->get_parameter_or<std::string>("tracker_aligner_solver_criteria_type",
                                      param_solver.m_solver_criteria_type,
                                      "stepnorm");
  this->get_parameter_or<float>("tracker_aligner_solver_criteria_epsilon",
                                param_solver.m_solver_criteria_epsilon, 1e-5f);
  this->get_parameter_or<std::string>("tracker_aligner_solver_robustifier_type",
                                      param_solver.m_solver_robustifier_type,
                                      "cauchy");

  if (!param_solver.m_solver_robustifier_type.empty())
  {
    if (!this->has_parameter(
            "tracker_aligner_solver_robustifier_num_iterations_coarse"))
    {
      if (!this->has_parameter("tracker_aligner_solver_robustifier_threshold"))
      {
        throw std::runtime_error(
            "TrackerROS2API::readParam | "
            "tracker_aligner_solver_robustifier_threshold must be defined when "
            "fine + coarse matching is not used");
      }
      if (this->has_parameter(
              "tracker_aligner_solver_robustifier_threshold_coarse") ||
          this->has_parameter(
              "tracker_aligner_solver_robustifier_threshold_fine"))
      {
        throw std::runtime_error(
            "TrackerROS2API::readParam | "
            "when tracker_aligner_solver_robustifier_threshold defined, no "
            "fine (tracker_aligner_solver_robustifier_threshold_fine) or "
            "coarse (tracker_aligner_solver_robustifier_threshold_coarse) "
            "thresholds must be defined...");
      }

      param_solver.m_solver_robustifier_num_iterations_coarse = 0;

      param_solver.m_solver_robustifier_threshold_coarse =
          this->get_parameter("tracker_aligner_solver_robustifier_threshold")
              .get_value<float>();
      param_solver.m_solver_robustifier_threshold_fine =
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
            "TrackerROS2API::readParam | "
            "fine (tracker_aligner_solver_robustifier_threshold_fine) and "
            "coarse (tracker_aligner_solver_robustifier_threshold_coarse) "
            "thresholds must be defined when fine + coarse matching is "
            "enabled");
      }

      param_solver.m_solver_robustifier_num_iterations_coarse =
          this->get_parameter(
                  "tracker_aligner_solver_robustifier_num_iterations_coarse")
              .get_value<int>();
      param_solver.m_solver_robustifier_threshold_coarse =
          this->get_parameter(
                  "tracker_aligner_solver_robustifier_threshold_coarse")
              .get_value<float>();
      param_solver.m_solver_robustifier_threshold_fine =
          this->get_parameter(
                  "tracker_aligner_solver_robustifier_threshold_fine")
              .get_value<float>();
    }
  }
  else
  {
    param_solver.m_solver_robustifier_num_iterations_coarse = 0;
    param_solver.m_solver_robustifier_threshold_coarse = 0.f;
    param_solver.m_solver_robustifier_threshold_fine = 0.f;
  }

  this->get_parameter_or<float>("tracker_aligner_failure_ratio_inliers",
                                param_solver.m_failure_ratio_inliers, 0.35f);
  this->get_parameter_or<float>("tracker_aligner_failure_chi_inliers",
                                param_solver.m_failure_chi_inliers, -1.0f);
  this->get_parameter_or<int>("tracker_aligner_dbg_ogl_interval",
                              dbg_ogl_interval, 0);
  this->get_parameter_or<std::string>("tracker_aligner_dbg_ogl_mode",
                                      param_solver.m_dbg_ogl_mode, "");
  this->get_parameter_or<bool>("tracker_aligner_dbg_ogl_show_correspondences",
                               param_solver.m_dbg_ogl_show_correspondences,
                               false);
  this->get_parameter_or<bool>("tracker_aligner_dbg_ogl_show_normals",
                               param_solver.m_dbg_ogl_show_normals, false);

  param_solver.m_dbg_ogl_interval = dbg_ogl_interval;

  this->get_parameter_or<float>("tracker_aligner_dist_map_res",
                                param_dist_map.m_dist_map_res, 0.01f);
  this->get_parameter_or<float>("tracker_aligner_dist_map_size",
                                param_dist_map.m_dist_map_size, 10.0f);
  this->get_parameter_or<float>("tracker_aligner_dist_map_max_dist",
                                param_dist_map.m_dist_map_max_dist, 0.25f);

  this->get_parameter_or<bool>(
      "tracker_solver_regularize_correspondences",
      param_dist_map.m_solver_regularize_correspondences, false);

  if (tracker_aligner_type.compare("point2point") == 0)
  {
    AlignerPt2PtTypePtr aligner =
        std::make_shared<AlignerPt2PtType>(param_solver, param_dist_map);

    m_param_.m_tracker_param.m_aligner =
        std::dynamic_pointer_cast<AlignerType>(aligner);
  }
  else if (tracker_aligner_type.compare("point2plane") == 0)
  {
    AlignerPt2PlnTypePtr aligner =
        std::make_shared<AlignerPt2PlnType>(param_solver, param_dist_map);

    m_param_.m_tracker_param.m_aligner =
        std::dynamic_pointer_cast<AlignerType>(aligner);
  }
  else if (tracker_aligner_type.compare("icp_point2point") == 0)
  {
    AlignerICPPt2PtTypePtr aligner =
        std::make_shared<AlignerICPPt2PtType>(param_solver, param_dist_map);

    m_param_.m_tracker_param.m_aligner =
        std::dynamic_pointer_cast<AlignerType>(aligner);
  }
  else if (tracker_aligner_type.compare("icp_point2plane") == 0)
  {
    AlignerICPPt2PlnTypePtr aligner =
        std::make_shared<AlignerICPPt2PlnType>(param_solver, param_dist_map);

    m_param_.m_tracker_param.m_aligner =
        std::dynamic_pointer_cast<AlignerType>(aligner);
  }
  else
  {
    throw std::runtime_error(
        "TrackerROS2API::readParam | unknown tracker_aligner_type value: " +
        tracker_aligner_type +
        " (point2point|point2plane|icp_point2point|icp_point2plane)");
  }

  // ----- TRACKER::PREPROCESS -------------------------------------------------

  this->get_parameter_or<bool>("tracker_enable_voxelization",
                               m_param_.m_tracker_param.m_enable_voxelization,
                               false);

  if (this->has_parameter("tracker_voxel_scale"))
  {
    m_param_.m_tracker_param.m_voxel_scale =
        this->get_parameter("tracker_voxel_scale").get_value<float>();
  }
  else
  {
    // will default internally to distance map resolution
    m_param_.m_tracker_param.m_voxel_scale = Scalar(-1);
  }

  if ((tracker_aligner_type.compare("point2plane") == 0) ||
      (this->has_parameter("tracker_normals_computator_min_pts")) ||
      (this->has_parameter("tracker_normals_computator_radius")))
  {
    m_param_.m_tracker_param.m_enable_normals = true;
    this->get_parameter_or<int>(
        "tracker_normals_computator_min_pts",
        m_param_.m_tracker_param.m_normals_computator_min_pts, 5);
    this->get_parameter_or<float>(
        "tracker_normals_computator_radius",
        m_param_.m_tracker_param.m_normals_computator_radius, 0.1f);
  }
  else
  {
    m_param_.m_tracker_param.m_enable_normals = false;
  }

  this->get_parameter_or<float>("tracker_range_min",
                                m_param_.m_tracker_param.m_range_min, -1.f);
  this->get_parameter_or<float>("tracker_range_max",
                                m_param_.m_tracker_param.m_range_max, -1.f);
}

void TrackerROS2API::printParam()
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

  RCLCPP_INFO(
      this->get_logger(), "[%s][LASER] invert scan (roll/pitch): %s",
      this->get_name(),
      m_param_.m_tracker_param.m_laser_invert_scan ? "enabled" : "not enabled");

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

  // ----- TRACKER -------------------------------------------------------------

  RCLCPP_INFO(this->get_logger(), "[%s][TRACKER] pub laser odom tf   : %s",
              this->get_name(),
              m_param_.m_tracker_pub_tf ? "enabled" : "not enabled");
  RCLCPP_INFO(this->get_logger(), "[%s][TRACKER] pub laser odom msg  : %s",
              this->get_name(),
              m_param_.m_tracker_pub_odom_msg ? "enabled" : "not enabled");
  RCLCPP_INFO(this->get_logger(), "[%s][TRACKER] invert laser odom tf: %s",
              this->get_name(),
              m_param_.m_tracker_pub_tf
                  ? m_param_.m_tracker_odom_invert_tf
                        ? m_param_.m_laser_tf_static_yaml
                              ? "enabled (laser odom frame w.r.t. sensor frame)"
                              : "enabled (laser odom frame w.r.t. base frame)"
                        : "not enabled (base frame w.r.t. laser odom frame)"
                  : "N/A");
  RCLCPP_INFO(this->get_logger(), "[%s][TRACKER] laser odom frame id : %s",
              this->get_name(), m_param_.m_tracker_odom_frame.c_str());
  RCLCPP_INFO(this->get_logger(),
              "[%s][TRACKER] use external odom as init : %s", this->get_name(),
              m_param_.m_tracker_param.m_use_ext_odom_as_init ? "enabled"
                                                              : "not enabled");
  RCLCPP_INFO(this->get_logger(),
              "[%s][TRACKER] use external odom as prior: %s", this->get_name(),
              m_param_.m_tracker_param.m_use_ext_odom_as_prior ? "enabled"
                                                               : "not enabled");

  // ----- TRACKER::SPLITTING CRITERIA -----------------------------------------

  if (dynamic_cast<SplittingCriteriaEmptyType*>(
          m_param_.m_tracker_param.m_splitting_criteria.get()))
  {
    RCLCPP_INFO(this->get_logger(),
                "[%s][TRACKER] splitting criteria type: empty (only forcing)",
                this->get_name());
  }
  else if (dynamic_cast<SplittingCriteriaRotationType*>(
               m_param_.m_tracker_param.m_splitting_criteria.get()))
  {
    SplittingCriteriaRotationType* criteria =
        dynamic_cast<SplittingCriteriaRotationType*>(
            m_param_.m_tracker_param.m_splitting_criteria.get());

    RCLCPP_INFO(this->get_logger(),
                "[%s][TRACKER] splitting criteria type              : rotation",
                this->get_name());
    RCLCPP_INFO(this->get_logger(),
                "[%s][TRACKER] splitting criteria min travel heading: %f (deg)",
                this->get_name(),
                geometry::radToDeg(criteria->m_min_travel_heading));
  }
  else if (dynamic_cast<SplittingCriteriaTranslationType*>(
               m_param_.m_tracker_param.m_splitting_criteria.get()))
  {
    SplittingCriteriaTranslationType* criteria =
        dynamic_cast<SplittingCriteriaTranslationType*>(
            m_param_.m_tracker_param.m_splitting_criteria.get());

    RCLCPP_INFO(
        this->get_logger(),
        "[%s][TRACKER] splitting criteria type              : translation",
        this->get_name());
    RCLCPP_INFO(this->get_logger(),
                "[%s][TRACKER] splitting criteria min travel dist   : %f (m)",
                this->get_name(), criteria->m_min_travel_dist);
  }
  else if (dynamic_cast<SplittingCriteriaViewpointType*>(
               m_param_.m_tracker_param.m_splitting_criteria.get()))
  {
    SplittingCriteriaViewpointType* criteria =
        dynamic_cast<SplittingCriteriaViewpointType*>(
            m_param_.m_tracker_param.m_splitting_criteria.get());

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
               m_param_.m_tracker_param.m_splitting_criteria.get()))
  {
    SplittingCriteriaGridCellType* criteria =
        dynamic_cast<SplittingCriteriaGridCellType*>(
            m_param_.m_tracker_param.m_splitting_criteria.get());

    RCLCPP_INFO(this->get_logger(),
                "[%s][TRACKER] splitting criteria type          : gridcell",
                this->get_name());
    RCLCPP_INFO(this->get_logger(),
                "[%s][TRACKER] splitting criteria grid cell size: %f (m)",
                this->get_name(), criteria->m_cell_size_2 * 2.f);
  }
  else if (dynamic_cast<SplittingCriteriaInliersRatioType*>(
               m_param_.m_tracker_param.m_splitting_criteria.get()))
  {
    SplittingCriteriaInliersRatioType* criteria =
        dynamic_cast<SplittingCriteriaInliersRatioType*>(
            m_param_.m_tracker_param.m_splitting_criteria.get());

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
               m_param_.m_tracker_param.m_splitting_criteria.get()))
  {
    SplittingCriteriaTimeType* criteria =
        dynamic_cast<SplittingCriteriaTimeType*>(
            m_param_.m_tracker_param.m_splitting_criteria.get());

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
  else if (!m_param_.m_tracker_param.m_splitting_criteria)
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
          m_param_.m_tracker_param.m_merging_criteria.get()))
  {
    RCLCPP_INFO(this->get_logger(),
                "[%s][TRACKER] merging criteria type: empty (only forcing)",
                this->get_name());
  }
  else if (dynamic_cast<MergingCriteriaRotationType*>(
               m_param_.m_tracker_param.m_merging_criteria.get()))
  {
    MergingCriteriaRotationType* criteria =
        dynamic_cast<MergingCriteriaRotationType*>(
            m_param_.m_tracker_param.m_merging_criteria.get());

    RCLCPP_INFO(this->get_logger(),
                "[%s][TRACKER] merging criteria type              : rotation",
                this->get_name());
    RCLCPP_INFO(this->get_logger(),
                "[%s][TRACKER] merging criteria min travel heading: %f (deg)",
                this->get_name(),
                geometry::radToDeg(criteria->m_min_travel_heading));
  }
  else if (dynamic_cast<MergingCriteriaTranslationType*>(
               m_param_.m_tracker_param.m_merging_criteria.get()))
  {
    MergingCriteriaTranslationType* criteria =
        dynamic_cast<MergingCriteriaTranslationType*>(
            m_param_.m_tracker_param.m_merging_criteria.get());

    RCLCPP_INFO(
        this->get_logger(),
        "[%s][TRACKER] merging criteria type              : translation",
        this->get_name());
    RCLCPP_INFO(this->get_logger(),
                "[%s][TRACKER] merging criteria min travel dist   : %f (m)",
                this->get_name(), criteria->m_min_travel_dist);
  }
  else if (dynamic_cast<MergingCriteriaViewpointType*>(
               m_param_.m_tracker_param.m_merging_criteria.get()))
  {
    MergingCriteriaViewpointType* criteria =
        dynamic_cast<MergingCriteriaViewpointType*>(
            m_param_.m_tracker_param.m_merging_criteria.get());

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
               m_param_.m_tracker_param.m_merging_criteria.get()))
  {
    MergingCriteriaGridCellType* criteria =
        dynamic_cast<MergingCriteriaGridCellType*>(
            m_param_.m_tracker_param.m_merging_criteria.get());

    RCLCPP_INFO(this->get_logger(),
                "[%s][TRACKER] merging criteria type          : gridcell",
                this->get_name());
    RCLCPP_INFO(this->get_logger(),
                "[%s][TRACKER] merging criteria grid cell size: %f (m)",
                this->get_name(), criteria->m_cell_size_2 * 2.f);
  }
  else if (dynamic_cast<MergingCriteriaInliersRatioType*>(
               m_param_.m_tracker_param.m_merging_criteria.get()))
  {
    MergingCriteriaInliersRatioType* criteria =
        dynamic_cast<MergingCriteriaInliersRatioType*>(
            m_param_.m_tracker_param.m_merging_criteria.get());

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
               m_param_.m_tracker_param.m_merging_criteria.get()))
  {
    MergingCriteriaTimeType* criteria = dynamic_cast<MergingCriteriaTimeType*>(
        m_param_.m_tracker_param.m_merging_criteria.get());

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
  else if (!m_param_.m_tracker_param.m_merging_criteria)
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

  if (m_param_.m_tracker_param.m_merging_criteria)
  {
    RCLCPP_INFO(
        this->get_logger(),
        "[%s][TRACKER] merger ray min dist see through : %f", this->get_name(),
        m_param_.m_tracker_param.m_merger->getParam().m_ray_min_dist_through);
    RCLCPP_INFO(
        this->get_logger(),
        "[%s][TRACKER] merger ray min dist add new pt  : %f", this->get_name(),
        m_param_.m_tracker_param.m_merger->getParam().m_ray_min_dist_add_new);
    RCLCPP_INFO(this->get_logger(),
                "[%s][TRACKER] merger ray average measurements : %s",
                this->get_name(),
                m_param_.m_tracker_param.m_merger->getParam().m_enable_avg
                    ? "enabled"
                    : "not enabled");
    RCLCPP_INFO(
        this->get_logger(),
        "[%s][TRACKER] merger ray average weight old pt: %s", this->get_name(),
        m_param_.m_tracker_param.m_merger->getParam().m_enable_avg
            ? std::to_string(
                  m_param_.m_tracker_param.m_merger->getParam().m_weight_old)
                  .c_str()
            : "N/A");
    RCLCPP_INFO(
        this->get_logger(),
        "[%s][TRACKER] merger ray average weight new pt: %s", this->get_name(),
        m_param_.m_tracker_param.m_merger->getParam().m_enable_avg
            ? std::to_string(
                  m_param_.m_tracker_param.m_merger->getParam().m_weight_new)
                  .c_str()
            : "N/A");
  }

  // ----- TRACKER::ALIGNER ----------------------------------------------------

  bool is_tracker_aligner_valid = true;

  if (dynamic_cast<AlignerPt2PtType*>(m_param_.m_tracker_param.m_aligner.get()))
  {
    RCLCPP_INFO(this->get_logger(),
                "[%s][TRACKER] aligner type           : point2point (distance "
                "map-based)",
                this->get_name());
  }
  else if (dynamic_cast<AlignerPt2PlnType*>(
               m_param_.m_tracker_param.m_aligner.get()))
  {
    RCLCPP_INFO(this->get_logger(),
                "[%s][TRACKER] aligner type           : point2plane (distance "
                "map-based)",
                this->get_name());
  }
  else if (dynamic_cast<AlignerICPPt2PtType*>(
               m_param_.m_tracker_param.m_aligner.get()))
  {
    RCLCPP_INFO(
        this->get_logger(),
        "[%s][TRACKER] aligner type           : icp_point2point (distance "
        "map-based)",
        this->get_name());
  }
  else if (dynamic_cast<AlignerICPPt2PlnType*>(
               m_param_.m_tracker_param.m_aligner.get()))
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
    param_print << m_param_.m_tracker_param.m_aligner->getParam()
                << m_param_.m_tracker_param.m_aligner->getDistMapParam();

    RCLCPP_INFO(this->get_logger(),
                "[%s][TRACKER] aligner parametrisation:\n%s", this->get_name(),
                param_print.str().c_str());
  }

  // ----- TRACKER::PREPROCESS -------------------------------------------------

  RCLCPP_INFO(this->get_logger(),
              "[%s][TRACKER] preprocess enable voxelization   : %s",
              this->get_name(),
              m_param_.m_tracker_param.m_enable_voxelization ? "enabled"
                                                             : "not enabled");
  RCLCPP_INFO(
      this->get_logger(), "[%s][TRACKER] preprocess voxelization scale (m): %s",
      this->get_name(),
      !m_param_.m_tracker_param.m_enable_voxelization
          ? "N/A"
          : (m_param_.m_tracker_param.m_voxel_scale <= 0
                 ? "distance map resolution"
                 : std::to_string(m_param_.m_tracker_param.m_voxel_scale)
                       .c_str()));

  RCLCPP_INFO(
      this->get_logger(),
      "[%s][TRACKER] preprocess normals computation                    : %s",
      this->get_name(),
      m_param_.m_tracker_param.m_enable_normals ? "enabled" : "not enabled");
  RCLCPP_INFO(
      this->get_logger(),
      "[%s][TRACKER] preprocess normals neighborhood minimum points    : %s",
      this->get_name(),
      !m_param_.m_tracker_param.m_enable_normals
          ? "N/A"
          : std::to_string(
                m_param_.m_tracker_param.m_normals_computator_min_pts)
                .c_str());
  RCLCPP_INFO(
      this->get_logger(),
      "[%s][TRACKER] preprocess normals nearest neighborhood radius (m): %s",
      this->get_name(),
      !m_param_.m_tracker_param.m_enable_normals
          ? "N/A"
          : std::to_string(m_param_.m_tracker_param.m_normals_computator_radius)
                .c_str());
  RCLCPP_INFO(
      this->get_logger(),
      "[%s][TRACKER] preprocess laser minimum range threshold       (m): %s",
      this->get_name(),
      m_param_.m_tracker_param.m_range_min > 0
          ? std::to_string(m_param_.m_tracker_param.m_range_min).c_str()
          : "read from msg.range_min");
  RCLCPP_INFO(
      this->get_logger(),
      "[%s][TRACKER] preprocess laser maximum range threshold       (m): %s",
      this->get_name(),
      m_param_.m_tracker_param.m_range_max > 0
          ? std::to_string(m_param_.m_tracker_param.m_range_max).c_str()
          : "read from msg.range_max");
}

void TrackerROS2API::subScan(
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

  m_tracker_->processScan(data, ext_odom_t);

  if (m_tracker_->getStatus() == TrackerType::Status::Error)
  {
    RCLCPP_WARN(
        this->get_logger(),
        "[%s] Tracker_::processPointCloud | keyframe lost... unsucessful "
        "match...",
        this->get_name());
  }

  if ((m_param_.m_tracker_pub_tf) || (m_param_.m_tracker_pub_odom_msg))
  {
    pubOdom(scan->header);
  }

  /* std::cout << "local: "
            << geometry::t2v(m_tracker_->getRobotInLocalMap()).transpose()
            << std::endl;
  std::cout << "odom : "
            << geometry::t2v(m_tracker_->getRobotInOdom()).transpose()
            << std::endl; */
}

void TrackerROS2API::subOdomScan(
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

  m_tracker_->processScan(data, ext_odom_t);

  if (m_tracker_->getStatus() == TrackerType::Status::Error)
  {
    RCLCPP_WARN(
        this->get_logger(),
        "[%s] Tracker_::processPointCloud | keyframe lost... unsucessful "
        "match...",
        this->get_name());
  }

  if ((m_param_.m_tracker_pub_tf) || (m_param_.m_tracker_pub_odom_msg))
  {
    pubOdom(scan->header);
  }

  /* std::cout << "local: "
            << geometry::t2v(m_tracker_->getRobotInLocalMap()).transpose()
            << std::endl;
  std::cout << "odom : "
            << geometry::t2v(m_tracker_->getRobotInOdom()).transpose()
            << std::endl; */
}

bool TrackerROS2API::setupSensorInRobot(const std::string& scan_frame)
{
  if (m_param_.m_laser_tf_static_yaml)
  {
    m_tracker_->setSensorInRobot(
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

    m_tracker_->setSensorInRobot(geometry::v2t(geometry::Vector3_<Scalar>(
        static_cast<Scalar>(sensor_in_robot.transform.translation.x),
        static_cast<Scalar>(sensor_in_robot.transform.translation.y),
        static_cast<Scalar>(tf2::getYaw(sensor_in_robot.transform.rotation)))));
  }

  RCLCPP_INFO(
      this->get_logger(),
      "[%s] sensor (%s) w.r.t. (%s) frame transformation: "
      "( %f m , %f m , %f deg )",
      this->get_name(), scan_frame.c_str(), m_param_.m_base_frame.c_str(),
      m_tracker_->getSensorInRobot().translation().x(),
      m_tracker_->getSensorInRobot().translation().y(),
      geometry::radToDeg(geometry::t2v(m_tracker_->getSensorInRobot()).z()));

  return true;
}

bool TrackerROS2API::checkDataLossAndCausality(
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

bool TrackerROS2API::srvTriggerMergingCriteria(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  if (m_tracker_->getMergingCriteriaPtr())
  {
    m_tracker_->getMergingCriteria()->forceTrigger();

    res->success = true;
    res->message = "";
  }
  else
  {
    res->success = false;
    res->message =
        "TrackerROS2API::srvTriggerMergingCriteria | merging criteria not set";
  }

  return res->success;
}

bool TrackerROS2API::srvTriggerSplittingCriteria(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  if (m_tracker_->getSplittingCriteriaPtr())
  {
    m_tracker_->getSplittingCriteria()->forceTrigger();

    res->success = true;
    res->message = "";
  }
  else
  {
    res->success = false;
    res->message =
        "TrackerROS2API::srvTriggerSplittingCriteria | splitting criteria not "
        "set";
  }

  return res->success;
}

}  // namespace inesctec_mrdt_slam_distmap_2d_ros
