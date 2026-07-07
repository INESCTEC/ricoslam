#pragma once

#include <iostream>
#include <memory>

// ROS
#include <geometry_msgs/TransformStamped.h>
#include <inesctec_mrdt_slam_distmap_2d_ros/SetFilename.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/buffer.h>

// SLAM C++ Implementation
#include <inesctec_mrdt_slam_distmap_2d/geometry/defs.hpp>
#include <inesctec_mrdt_slam_distmap_2d/slam/aligner.hpp>
#include <inesctec_mrdt_slam_distmap_2d/slam/aligner_distance_map_icp_point_plane.hpp>
#include <inesctec_mrdt_slam_distmap_2d/slam/aligner_distance_map_icp_point_point.hpp>
#include <inesctec_mrdt_slam_distmap_2d/slam/aligner_distance_map_point_plane.hpp>
#include <inesctec_mrdt_slam_distmap_2d/slam/aligner_distance_map_point_point.hpp>
#include <inesctec_mrdt_slam_distmap_2d/slam/merger.hpp>
#include <inesctec_mrdt_slam_distmap_2d/slam/slam.hpp>
#include <inesctec_mrdt_slam_distmap_2d/slam/tracker.hpp>
#include <inesctec_mrdt_slam_distmap_2d/srrg2_solver/all_types.hpp>

namespace inesctec_mrdt_slam_distmap_2d_ros
{

class SLAMROS1API
{
 public:

  using ThisType = SLAMROS1API;

  using Scalar = float;
  using VariableType =
      srrg2_solver::VariableSE2DistanceMapStaticPointNormal2fVectorCloudRight;

  using EstimateType = VariableType::EstimateType;
  using EstimateVectorType = Eigen::Matrix<Scalar, 3, 1>;

  using SLAMType = inesctec_mrdt_slam_distmap_2d::slam::SLAM_<VariableType>;
  using SLAMTypePtr = std::shared_ptr<SLAMType>;
  using SLAMTypeParam = SLAMType::Param;

  using TrackerType =
      inesctec_mrdt_slam_distmap_2d::slam::Tracker_<VariableType>;
  using TrackerTypePtr = std::shared_ptr<TrackerType>;
  using TrackerTypeParam = TrackerType::Param;

  using SplittingCriteriaType =
      inesctec_mrdt_slam_distmap_2d::slam::TriggeringCriteria_<EstimateType>;
  using SplittingCriteriaEmptyType =
      inesctec_mrdt_slam_distmap_2d::slam::TriggeringCriteriaEmpty_<
          EstimateType>;
  using SplittingCriteriaRotationType =
      inesctec_mrdt_slam_distmap_2d::slam::TriggeringCriteriaRotation_<
          EstimateType>;
  using SplittingCriteriaTranslationType =
      inesctec_mrdt_slam_distmap_2d::slam::TriggeringCriteriaTranslation_<
          EstimateType>;
  using SplittingCriteriaViewpointType =
      inesctec_mrdt_slam_distmap_2d::slam::TriggeringCriteriaViewpoint_<
          EstimateType>;
  using SplittingCriteriaGridCellType =
      inesctec_mrdt_slam_distmap_2d::slam::TriggeringCriteriaGridCell_<
          EstimateType>;
  using SplittingCriteriaInliersRatioType =
      inesctec_mrdt_slam_distmap_2d::slam::TriggeringCriteriaInliersRatio_<
          EstimateType>;
  using SplittingCriteriaTimeType =
      inesctec_mrdt_slam_distmap_2d::slam::TriggeringCriteriaTime_<
          EstimateType>;

  using SplittingCriteriaTypePtr = std::shared_ptr<SplittingCriteriaType>;
  using SplittingCriteriaEmptyTypePtr =
      std::shared_ptr<SplittingCriteriaEmptyType>;
  using SplittingCriteriaRotationTypePtr =
      std::shared_ptr<SplittingCriteriaRotationType>;
  using SplittingCriteriaTranslationTypePtr =
      std::shared_ptr<SplittingCriteriaTranslationType>;
  using SplittingCriteriaViewpointTypePtr =
      std::shared_ptr<SplittingCriteriaViewpointType>;
  using SplittingCriteriaGridCellTypePtr =
      std::shared_ptr<SplittingCriteriaGridCellType>;
  using SplittingCriteriaInliersRatioTypePtr =
      std::shared_ptr<SplittingCriteriaInliersRatioType>;
  using SplittingCriteriaTimeTypePtr =
      std::shared_ptr<SplittingCriteriaTimeType>;

  using MergingCriteriaType =
      inesctec_mrdt_slam_distmap_2d::slam::TriggeringCriteria_<EstimateType>;
  using MergingCriteriaEmptyType =
      inesctec_mrdt_slam_distmap_2d::slam::TriggeringCriteriaEmpty_<
          EstimateType>;
  using MergingCriteriaRotationType =
      inesctec_mrdt_slam_distmap_2d::slam::TriggeringCriteriaRotation_<
          EstimateType>;
  using MergingCriteriaTranslationType =
      inesctec_mrdt_slam_distmap_2d::slam::TriggeringCriteriaTranslation_<
          EstimateType>;
  using MergingCriteriaViewpointType =
      inesctec_mrdt_slam_distmap_2d::slam::TriggeringCriteriaViewpoint_<
          EstimateType>;
  using MergingCriteriaGridCellType =
      inesctec_mrdt_slam_distmap_2d::slam::TriggeringCriteriaGridCell_<
          EstimateType>;
  using MergingCriteriaInliersRatioType =
      inesctec_mrdt_slam_distmap_2d::slam::TriggeringCriteriaInliersRatio_<
          EstimateType>;
  using MergingCriteriaTimeType =
      inesctec_mrdt_slam_distmap_2d::slam::TriggeringCriteriaTime_<
          EstimateType>;

  using MergingCriteriaTypePtr = std::shared_ptr<MergingCriteriaType>;
  using MergingCriteriaEmptyTypePtr = std::shared_ptr<MergingCriteriaEmptyType>;
  using MergingCriteriaRotationTypePtr =
      std::shared_ptr<MergingCriteriaRotationType>;
  using MergingCriteriaTranslationTypePtr =
      std::shared_ptr<MergingCriteriaTranslationType>;
  using MergingCriteriaViewpointTypePtr =
      std::shared_ptr<MergingCriteriaViewpointType>;
  using MergingCriteriaGridCellTypePtr =
      std::shared_ptr<MergingCriteriaGridCellType>;
  using MergingCriteriaInliersRatioTypePtr =
      std::shared_ptr<MergingCriteriaInliersRatioType>;
  using MergingCriteriaTimeTypePtr = std::shared_ptr<MergingCriteriaTimeType>;

  using MergerType = SLAMType::TrackerType::MergerType;
  using MergerTypePtr = std::shared_ptr<MergerType>;
  using MergerTypeParam = MergerType::Param;

  using AlignerBaseType = inesctec_mrdt_slam_distmap_2d::slam::AlignerBase;
  using AlignerBaseTypePtr = std::shared_ptr<AlignerBaseType>;

  using AlignerType =
      inesctec_mrdt_slam_distmap_2d::slam::Aligner_<VariableType>;
  using AlignerTypePtr = std::shared_ptr<AlignerType>;

  using AlignerPt2PtType =
      inesctec_mrdt_slam_distmap_2d::slam::AlignerDistanceMapPointPoint_<
          VariableType>;
  using AlignerPt2PtTypePtr = std::shared_ptr<AlignerPt2PtType>;

  using AlignerPt2PlnType =
      inesctec_mrdt_slam_distmap_2d::slam::AlignerDistanceMapPointPlane_<
          VariableType>;
  using AlignerPt2PlnTypePtr = std::shared_ptr<AlignerPt2PlnType>;

  using AlignerICPPt2PtType =
      inesctec_mrdt_slam_distmap_2d::slam::AlignerDistanceMapICPPointPoint_<
          VariableType>;
  using AlignerICPPt2PtTypePtr = std::shared_ptr<AlignerICPPt2PtType>;

  using AlignerICPPt2PlnType =
      inesctec_mrdt_slam_distmap_2d::slam::AlignerDistanceMapICPPointPlane_<
          VariableType>;
  using AlignerICPPt2PlnTypePtr = std::shared_ptr<AlignerICPPt2PlnType>;

  using AlignerTypeParamSolver = AlignerType::Param;
  using AlignerTypeParamDistMap = AlignerType::DistMapParam;

  using PointCloudType = AlignerType::PointCloudType;
  using PointCloudTypePtr = AlignerType::PointCloudTypePtr;

  using LaserScanMsgType = TrackerType::LaserScanMsgType;
  using LaserScanMsgTypePtr = TrackerType::LaserScanMsgTypePtr;

 public:

  struct Param
  {
    std::string m_base_frame =
        "";  //!< base frame id of the robot in the tf system

    bool m_ext_odom_use = false;  //!< enable the usage of the external odometry
                                  //!< data (e.g. wheel encoders)
    std::string m_ext_odom_impl =
        "";  //!< external odometry implementation (topic|tf)
    std::string m_ext_odom_frame =
        "";  //!< external odometry frame id in the tf system (read from ROS
             //!< param when tf beign used or ext odometry published in tf)
    bool m_ext_odom_pub_tf = false;   //!< publish ext odometry in tf system
    bool m_ext_odom_pub_msg = false;  //!< publish ext odometry as Odometry msg
    bool m_ext_odom_invert_tf = false;  //!< invert ext odometry tf (global
                                        //!< ext odom w.r.t. base frame)

    std::string m_laser_frame = "";      //!< laser frame id in the tf system
    bool m_laser_pub_tf_static = false;  //!< pub laser tf set in ROS YAML param
    bool m_laser_tf_static_yaml =
        false;  //!< read from ROS param if any laser pose parameter set
    inesctec_mrdt_slam_distmap_2d::geometry::Vector3_<float>
        m_laser_pose;  //!< laser pose w.r.t. base frame

    bool m_slam_pub_pose_tf = true;  //!< publish robot pose in the tf system
    bool m_slam_pub_pose_msg =
        true;  //!< publish pose as PoseWithCovarianceStamped msg
    bool m_slam_pose_invert_tf =
        false;  //!< invert robot pose tf (robot pose / map w.r.t. base frame)
    bool m_slam_pub_odom_tf =
        true;  //!< publish laser odometry in the tf system
    bool m_slam_pub_odom_msg =
        true;  //!< publish laser odometry as an Odometry msg
    bool m_slam_odom_invert_tf =
        true;  //!< invert laser odometry tf (laser odom w.r.t. base frame)
    std::string m_slam_pose_frame =
        "map";  //!< robot pose / map frame id in the tf system
    std::string m_slam_odom_frame =
        "odom_laser";  //!< laser odometry frame id in the tf system

    SLAMTypeParam m_slam_param;  //!< slam parametrisation

  };  // struct SLAMROS1API::Param

 protected:

  Param m_param_;

  const std::string& m_name_;

  ros::NodeHandle m_nh_;
  ros::NodeHandle m_nh_priv_;

  tf2_ros::Buffer m_tf2_buffer_;

  bool m_is_first_msg_ = true;
  bool m_is_tf_init_ = false;

  uint32_t m_scan_seq_prev_;
  double m_scan_stamp_prev_;

  SLAMTypePtr m_slam_ = nullptr;

  ros::ServiceServer m_srv_trigger_merging_;
  ros::ServiceServer m_srv_trigger_splitting_;
  ros::ServiceServer m_srv_save_graph_;
  ros::ServiceServer m_srv_save_corrected_pose_;

 public:

  SLAMROS1API();

  inline const Param& getParam() const { return m_param_; }
  inline SLAMType* getSLAM() { return m_slam_.get(); }
  inline const SLAMTypePtr& getSLAMPtr() const { return m_slam_; }

  inline bool isOpenGLDbg() const
  {
    assert(m_slam_ && "SLAMROS1API::isOpenGLDbg | slam not set");

    bool is_opengl_dbg = false;

    if (m_slam_->getParam().m_tracker)
    {
      assert(m_slam_->getParam().m_tracker->getParam().m_aligner &&
             "SLAMROS1API::isOpenGLDbg | tracker set but no aligner");

      if (m_slam_->getParam()
              .m_tracker->getParam()
              .m_aligner->getParam()
              .m_debug)
      {
        is_opengl_dbg = true;
      }
    }

    if (m_slam_->getParam().m_loop_enable)
    {
      assert(m_slam_->getParam().m_loop_aligner &&
             "SLAMROS1API::isOpenGLDbg | loop closure detection enabled but "
             "no aligner");

      if (m_slam_->getParam().m_loop_aligner->getParam().m_debug)
      {
        is_opengl_dbg = true;
      }
    }

    is_opengl_dbg = is_opengl_dbg || m_slam_->getParam().m_solver_debug;

    return is_opengl_dbg;
  }

 public:

  void subScan(const sensor_msgs::LaserScan::ConstPtr& scan);
  void subOdomScan(const nav_msgs::Odometry::ConstPtr& odom,
                   const sensor_msgs::LaserScan::ConstPtr& scan);

 protected:

  void readParam();
  void printParam();

  bool setupSensorInRobot(const std::string& scan_frame);

  bool checkDataLossAndCausality(const sensor_msgs::LaserScan::ConstPtr& scan);

  virtual void pubOdom(const std_msgs::Header&) = 0;
  virtual void pubPose(const std_msgs::Header&) = 0;
  virtual void pubExtOdom(const geometry_msgs::TransformStamped&) = 0;
  virtual void pubExtOdomTF(const nav_msgs::Odometry::ConstPtr&) = 0;

  bool srvTriggerMergingCriteria(std_srvs::Trigger::Request& req,
                                 std_srvs::Trigger::Response& res);
  bool srvTriggerSplittingCriteria(std_srvs::Trigger::Request& req,
                                   std_srvs::Trigger::Response& res);

  bool srvSaveGraph(
      inesctec_mrdt_slam_distmap_2d_ros::SetFilename::Request& req,
      inesctec_mrdt_slam_distmap_2d_ros::SetFilename::Response& res);
  bool srvSaveCorrectedPose(
      inesctec_mrdt_slam_distmap_2d_ros::SetFilename::Request& req,
      inesctec_mrdt_slam_distmap_2d_ros::SetFilename::Response& res);

};  // class SLAMROS1API

}  // namespace inesctec_mrdt_slam_distmap_2d_ros
