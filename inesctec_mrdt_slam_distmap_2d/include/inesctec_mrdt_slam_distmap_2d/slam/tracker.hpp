#pragma once

#include <algorithm>
#include <exception>
#include <memory>

#include "inesctec_mrdt_slam_distmap_2d/pcl/normal_computator.hpp"
#include "inesctec_mrdt_slam_distmap_2d/slam/aligner.hpp"
#include "inesctec_mrdt_slam_distmap_2d/slam/aligner_distance_map_icp_point_plane.hpp"
#include "inesctec_mrdt_slam_distmap_2d/slam/aligner_distance_map_icp_point_point.hpp"
#include "inesctec_mrdt_slam_distmap_2d/slam/aligner_distance_map_point_plane.hpp"
#include "inesctec_mrdt_slam_distmap_2d/slam/aligner_distance_map_point_point.hpp"
#include "inesctec_mrdt_slam_distmap_2d/slam/merger.hpp"
#include "inesctec_mrdt_slam_distmap_2d/slam/msg/laser_scan.hpp"
#include "inesctec_mrdt_slam_distmap_2d/slam/triggering_criteria.hpp"
#include "inesctec_mrdt_slam_distmap_2d/srrg2_solver/all_types.hpp"
#include "inesctec_mrdt_slam_distmap_2d/srrg2_solver/instances.hpp"

namespace inesctec_mrdt_slam_distmap_2d
{

namespace slam
{

/**
 * @brief tracker generic class
 * @tparam VariableSE2DistanceMapRightType_ distance map-based variable type
 *                                          (must be right-sided
 *                                          pertubation-based!)
 */
template <typename VariableSE2DistanceMapRightType_>
class Tracker_
{
 public:

  using ThisType = Tracker_<VariableSE2DistanceMapRightType_>;

  using VariableType = VariableSE2DistanceMapRightType_;
  using VariableTypePtr = std::shared_ptr<VariableType>;
  using EstimateType = typename VariableType::EstimateType;
  using Scalar = typename EstimateType::Scalar;

  using AlignerBaseType = Aligner_<VariableType>;
  using AlignerBaseTypePtr = std::shared_ptr<AlignerBaseType>;
  using AlignerBaseTypeStats = typename AlignerBaseType::Stats;
  using PointCloudType = typename AlignerBaseType::PointCloudType;
  using PointCloudTypePtr = typename AlignerBaseType::PointCloudTypePtr;

  using DistanceMapType =
      typename VariableType::MeasurementOwnerType::DistanceMapType;

  using LaserScanMsgType = msg::LaserScan;
  using LaserScanMsgTypePtr = msg::LaserScanPtr;

  using EstimateTypeInfo = typename AlignerBaseType::EstimateTypeInfo;

  using MeasurementTypeElement =
      typename AlignerBaseType::MeasurementTypeElement;

  using MergerType = Merger_<EstimateType, DistanceMapType, PointCloudType>;
  using MergerTypePtr = std::shared_ptr<MergerType>;

  using SplittingCriteriaType = TriggeringCriteria_<EstimateType>;
  using SplittingCriteriaTypePtr = std::shared_ptr<SplittingCriteriaType>;
  using MergingCriteriaType = TriggeringCriteria_<EstimateType>;
  using MergingCriteriaTypePtr = std::shared_ptr<MergingCriteriaType>;

  using OGLAppType = typename AlignerBaseType::OGLAppType;
  using OGLAppTypePtr = typename AlignerBaseType::OGLAppTypePtr;
  using OGLViewerType = typename AlignerBaseType::OGLViewerType;
  using OGLViewerTypePtr = typename AlignerBaseType::OGLViewerTypePtr;
  using OGLCanvasType = typename AlignerBaseType::OGLCanvasType;
  using OGLCanvasTypePtr = typename AlignerBaseType::OGLCanvasTypePtr;

  using NormalComputatorType = pcl::NormalComputatorNearestPoints1D_<
      typename AlignerBaseType::PointCloudType>;
  using NormalComputatorTypePtr = std::shared_ptr<NormalComputatorType>;

 public:

  /**
   * @brief tracker parametrization
   */
  struct Param
  {
    bool m_laser_invert_scan =
        false;  //!< invert the laser scan data (when sensor upside down)

    bool m_use_ext_odom_as_init = false;  //!< use external odometry source as
                                          //!< initialization for the aligner
    bool m_use_ext_odom_as_prior =
        false;  //!< use external odometry source as a prior factor for the
                //!< alignment process (NYS)

    AlignerBaseTypePtr m_aligner = nullptr;  //!< aligner parametrization
    MergerTypePtr m_merger =
        nullptr;  //!< merger parametrization (requires a merging criteria!)
    SplittingCriteriaTypePtr m_splitting_criteria =
        nullptr;  //!< splitting criteria (when to change the tracker's local
                  //!< map)
    MergingCriteriaTypePtr m_merging_criteria =
        nullptr;  //!< merging criteria (when to merge the moving point cloud
                  //!< into local map)

    bool m_enable_voxelization = false;   //!< enable point cloud voxelization
    Scalar m_voxel_scale = Scalar(-1.0);  //!< voxel scale size (m) (-1 for
                                          //!< distance map resolution as
                                          //!< default)
    Scalar m_range_min = Scalar(-1.0);  //!< laser ray minimum range (m) (-1 for
                                        //!< msg.range_min as default)
    Scalar m_range_max = Scalar(-1.0);  //!< laser ray maximum range (m) (-1 for
                                        //!< msg.range_max as default)

    bool m_enable_normals = false;  //!< for point-to-plane error computation
    int m_normals_computator_min_pts =
        -1;  //!< minimum pts of nearest points neighborhood to compute normals
    Scalar m_normals_computator_radius =
        Scalar(-1);  //!< nearest neighborhood radius (m)

    /**
     * @brief default constructor
     */
    Param()
        : m_laser_invert_scan(false),
          m_use_ext_odom_as_init(false),
          m_use_ext_odom_as_prior(false),
          m_aligner(nullptr),
          m_merger(nullptr),
          m_splitting_criteria(nullptr),
          m_merging_criteria(nullptr),
          m_enable_voxelization(false),
          m_voxel_scale(Scalar(-1.0)),
          m_range_min(Scalar(-1.0)),
          m_range_max(Scalar(-1.0)),
          m_enable_normals(false),
          m_normals_computator_min_pts(-1),
          m_normals_computator_radius(Scalar(-1.0))
    {
    }

    /**
     * @brief tracker parametrization with aligner, merger, and splitting and
     *        merging criteria
     * @param[in] laser_invert_scan invert the laser scan data
     *                              (when sensor upside down)
     * @param[in] use_ext_odom_as_init use external odometry source as
     *                                 initialization for the aligner
     * @param[in] use_ext_odom_as_prior use external odometry source as a prior
     *                                  factor for the alignment process (NYS)
     * @param[in] aligner aligner implementation
     * @param[in] merger merger
     * @param[in] splitting_criteria splitting criteria implementation
     * @param[in] merging_criteria merging criteria implementation
     * @param[in] enable_voxelization enable point cloud voxelization
     * @param[in] voxel_scale voxel scale size (m) (-1 for distance map
     *                        resolution as default)
     * @param[in] range_min laser ray minimum range (m) (-1 for msg.range_min as
     *                      default)
     * @param[in] range_max laser ray maximum range (m) (-1 for msg.range_max as
     *                      default)
     * @param[in] enable_normals enable point-to-plane error computation
     * @param[in] normals_computator_min_pts minimum pts of nearest points
     *                                       neighborhood to compute normals
     * @param[in] normals_computator_radius nearest neighborhood radius (m)
     */
    Param(bool laser_invert_scan, bool use_ext_odom_as_init,
          bool use_ext_odom_as_prior, const AlignerBaseTypePtr& aligner,
          const MergerTypePtr& merger,
          const SplittingCriteriaTypePtr& splitting_criteria,
          const MergingCriteriaTypePtr& merging_criteria,
          bool enable_voxelization, Scalar voxel_scale, Scalar range_min,
          Scalar range_max, bool enable_normals, int normals_computator_min_pts,
          Scalar normals_computator_radius)
        : m_laser_invert_scan(laser_invert_scan),
          m_use_ext_odom_as_init(use_ext_odom_as_init),
          m_use_ext_odom_as_prior(use_ext_odom_as_prior),
          m_aligner(aligner),
          m_merger(merger),
          m_splitting_criteria(splitting_criteria),
          m_merging_criteria(merging_criteria),
          m_enable_voxelization(enable_voxelization),
          m_voxel_scale(voxel_scale),
          m_range_min(range_min),
          m_range_max(range_max),
          m_enable_normals(enable_normals),
          m_normals_computator_min_pts(normals_computator_min_pts),
          m_normals_computator_radius(normals_computator_radius)
    {
    }

    friend std::ostream& operator<<(std::ostream& os, const Param& param)
    {
      os << "- invert laser scan: "
         << (param.m_laser_invert_scan ? "enabled" : "not enabled")
         << std::endl;
      os << "- use external odometry src as initial guess: "
         << (param.m_use_ext_odom_as_init ? "enabled" : "not enabled")
         << std::endl;
      os << "- use external odometry src as prior factor : "
         << (param.m_use_ext_odom_as_prior ? "enabled" : "not enabled")
         << std::endl;
      if (param.m_aligner)
      {
        os << "- aligner parametrization:" << std::endl
           << "{" << std::endl
           << param.m_aligner->getParam() << "}" << std::endl;
      }
      else
      {
        os << "- aligner parametrization: not enabled" << std::endl;
      }
      if (param.m_splitting_criteria)
      {
        os << "- splitting criteria parametrization:" << std::endl
           << "{" << std::endl
           << *param.m_splitting_criteria << "}" << std::endl;
      }
      else
      {
        os << "- splitting criteria parametrization: not enabled" << std::endl;
      }
      if (param.m_merging_criteria)
      {
        os << "- merging criteria parametrization:" << std::endl
           << "{" << std::endl
           << *param.m_merging_criteria << "}" << std::endl;
      }
      else
      {
        os << "- merging criteria parametrization: not enabled" << std::endl;
      }
      if (param.m_merger)
      {
        os << "- merger parametrization          :" << std::endl
           << "{" << std::endl
           << param.m_merger->getParam() << "}" << std::endl;
      }
      else
      {
        os << "- merger parametrization          : not enabled" << std::endl;
      }
      os << "- pre-processing: " << std::endl
         << "  - point cloud voxelization: "
         << (param.m_enable_voxelization ? "enabled " : "not enabled");
      if (param.m_enable_voxelization)
      {
        if (param.m_voxel_scale <= 0)
        {
          os << "(voxel scale: distance map resolution)" << std::endl;
        }
        else
        {
          os << "(voxel scale: " << param.m_voxel_scale << " m)" << std::endl;
        }
      }
      else
      {
        os << std::endl;
      }
      os << "  - ray minimum range (m): ";
      if (param.m_range_min < 0)
      {
        os << "read from msg.range_min" << std::endl;
      }
      else
      {
        os << param.m_range_min << std::endl;
      }
      os << "  - ray maximum range (m): ";
      if (param.m_range_max <= 0)
      {
        os << "read from msg.range_max" << std::endl;
      }
      else
      {
        os << param.m_range_max << std::endl;
      }
      os << "  - normals computation: "
         << (param.m_enable_normals ? "enabled " : "not enabled ")
         << "(point-to-plane alignment)" << std::endl;
      if (param.m_enable_normals)
      {
        os << "    - neighborhood minimum points    : "
           << param.m_normals_computator_min_pts << std::endl
           << "    - nearest neighborhood radius (m): "
           << param.m_normals_computator_radius << std::endl;
      }

      return os;
    }

  };  // struct Tracker_<VariableSE2DistanceMapRightType_>::Param

  enum Status
  {
    Tracking = 0,  //!< tracking
    TrackingMeasurementSplit =
        1,  //!< tracking w/ splitting or reset on local map
    TrackingMeasurementMerged =
        2,             //!< tracking w/ measurement merged into local map
    Initializing = 3,  //!< initializing the tracker (local map creation)
    Initialized = 4,   //!< tracker initialized
    Error = 5,         //!< tracking error (failed alignment)
    ErrorReset = 6     //!< reset error
  };  // enum Tracker_<VariableSE2DistanceMapRightType_>::Status

 protected:

  Param m_param_;    //!< parametrization
  Status m_status_;  //!< status

  AlignerBaseTypeStats m_stats_;  //!< stats

  LaserScanMsgTypePtr m_scan_ = nullptr;  //!< latest laser scan message
  PointCloudTypePtr m_pc_moving_ =
      nullptr;  //!< moving point cloud (usually, the latest one received /
                //!< processed)
  VariableTypePtr m_local_map_ =
      nullptr;  //!< current local map used as reference for the tracker

  EstimateType m_robot_in_odom_ =
      EstimateType::Identity();  //!< robot pose w.r.t. odometry global
                                 //!< coordinate frame estimated by the tracker
  EstimateType m_robot_in_local_odom_ =
      EstimateType::Identity();  //!< robot pose w.r.t. local map
  EstimateTypeInfo m_robot_in_local_odom_info_ =
      EstimateTypeInfo::Identity();  //!< hessian matrix from aligner

  EstimateType m_robot_in_local_odom_prev_ =
      EstimateType::Identity();  //!< robot pose w.r.t. map (wo/ split)
  EstimateTypeInfo m_robot_in_local_odom_prev_info_ =
      EstimateTypeInfo::Identity();  //!< hessian matrix (wo/ split)

  EstimateType m_robot_in_ext_odom_ =
      EstimateType::Identity();  //!< robot pose w.r.t. external odometry source
  EstimateType m_robot_in_ext_odom_at_local_ =
      EstimateType::Identity();  //!< robot pose w.r.t. external odometry source
                                 //!< at the local map's coordinate frame
  EstimateType m_robot_in_ext_odom_on_local_ =
      EstimateType::Identity();  //!< robot pose w.r.t. external odometry source
                                 //!< relative to the local map's coordinate
                                 //!< frame (local external estimation)

  EstimateType m_sensor_in_robot_ =
      EstimateType::Identity();  //!< SE(2) transformation of the sensor w.r.t.
                                 //!< robot coordinate frame

  NormalComputatorTypePtr m_normals_computator_ =
      nullptr;  //!< normals computator for the point-to-plane alignment
                //!< implementation

 public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Construct a new Tracker_ object
   *        (Tracker status: Status::Initializing)
   * @param[in] param tracker parametrization
   */
  Tracker_(const Param& param = Param())
      : m_param_(param),
        m_status_(Status::Initializing),
        m_scan_(nullptr),
        m_pc_moving_(nullptr),
        m_local_map_(nullptr),
        m_robot_in_odom_(EstimateType::Identity()),
        m_robot_in_local_odom_(EstimateType::Identity()),
        m_robot_in_local_odom_info_(EstimateTypeInfo::Identity()),
        m_robot_in_local_odom_prev_(EstimateType::Identity()),
        m_robot_in_local_odom_prev_info_(EstimateTypeInfo::Identity()),
        m_robot_in_ext_odom_(EstimateType::Identity()),
        m_robot_in_ext_odom_at_local_(EstimateType::Identity()),
        m_robot_in_ext_odom_on_local_(EstimateType::Identity()),
        m_sensor_in_robot_(EstimateType::Identity()),
        m_normals_computator_(nullptr)
  {
    assertInit();
    init();
  }

  /**
   * @brief process the latest scan message from a 2D laser scanner
   *        (laser odometry-only OR initialization with external odometry
   *        AND/OR prior factor based on external odometry estimation)
   * @param[in] scan 2D laser scan message
   * @param[in] ext_odometry external odometry source (ony used depending on the
   *                         tracker's parametrization)
   */
  void processScan(const msg::LaserScanPtr& scan,
                   const EstimateType& ext_odometry = EstimateType::Identity())
  {
    m_scan_ = scan;

    preprocessScan();

    if (m_param_.m_use_ext_odom_as_init)
    {
      const EstimateType ext_odom_inc =
          m_robot_in_ext_odom_.inverse() * ext_odometry;

      m_robot_in_local_odom_ = m_robot_in_local_odom_ * ext_odom_inc;
    }

    m_robot_in_ext_odom_ = ext_odometry;

    preprocessPointCloud();

    m_robot_in_ext_odom_on_local_ =
        m_robot_in_ext_odom_at_local_.inverse() * m_robot_in_ext_odom_;

    if (m_status_ == Status::Initializing)
    {
      reset(Status::Initialized);
      resetLocalMap(EstimateType::Identity());
      return;
    }

    m_param_.m_aligner->setSource(m_pc_moving_, m_sensor_in_robot_);
    m_param_.m_aligner->setSourceInTarget(m_robot_in_local_odom_);

    if (m_param_.m_use_ext_odom_as_prior)
    {
      /**
       * @todo missing wheeled odometry propagation...
       * before doing the set prior, propage first the wheeled odometry
       * estimation and update accordingly, similar to predict step in EKF, the
       * covariance matrix should be the information matrix on the prior...
       */
      m_param_.m_aligner->setPriorEnabled(true);
      m_param_.m_aligner->setPrior(m_robot_in_ext_odom_on_local_);
    }
    else
    {
      m_param_.m_aligner->setPriorEnabled(false);
    }

    m_param_.m_aligner->compute();

    m_stats_ = m_param_.m_aligner->getStats();

    if (m_param_.m_aligner->getStatus() != AlignerBaseType::Status::Success)
    {
      std::cerr << "Tracker_::processPointCloud | keyframe lost (t: "
                << m_scan_->m_header.m_t << " s)... unsuccessful match... "
                << "( aligner status: " << m_param_.m_aligner->getStatus()
                << " ; " << m_stats_ << " )" << std::endl;
      reset(Status::Error);
      resetLocalMap(EstimateType::Identity());
      return;
    }

    m_status_ = Status::Tracking;

    m_robot_in_local_odom_ = m_param_.m_aligner->getSourceInTarget();
    m_robot_in_local_odom_info_ = m_param_.m_aligner->getH();

    m_robot_in_local_odom_prev_ = m_robot_in_local_odom_;
    m_robot_in_local_odom_prev_info_ = m_robot_in_local_odom_info_;

    if (m_param_.m_splitting_criteria)
    {
      m_param_.m_splitting_criteria->compute(
          *m_scan_, m_robot_in_local_odom_,
          m_param_.m_aligner->getStats().m_inliers_ratio);

      if (m_param_.m_splitting_criteria->isToTrigger())
      {
        reset(Status::TrackingMeasurementSplit);
        resetLocalMap(EstimateType::Identity());
        return;
      }
    }

    if (m_param_.m_merging_criteria)
    {
      m_param_.m_merging_criteria->compute(
          *m_scan_, m_robot_in_local_odom_,
          m_param_.m_aligner->getStats().m_inliers_ratio);

      if (m_param_.m_merging_criteria->isToTrigger())
      {
        m_status_ = Status::TrackingMeasurementMerged;
        m_param_.m_merger->merge(*m_pc_moving_, *m_scan_, m_sensor_in_robot_,
                                 m_robot_in_local_odom_,
                                 m_local_map_->measurement()->getPointCloud(),
                                 m_local_map_->measurement()->getDistanceMap());
        m_local_map_->measurement()->m_msgs->push_back(
            std::make_pair(m_scan_, m_robot_in_local_odom_));
        m_param_.m_merging_criteria->reset(*m_scan_, m_robot_in_local_odom_);
      }
    }

  }  // void Tracker_::processScan(const msg::LaserScanPtr&, const
     // EstimateType& = EstimateType::Identity())

  /**
   * @brief reset Tracker_ object to Status::Initializing or Status::Initialized
   *        status
   * @param[in] status tracker status
   */
  void reset(Status status = Status::Initializing)
  {
    m_local_map_ = std::make_shared<VariableType>();
    m_local_map_->setMeasurement(std::make_shared<MeasurementTypeElement>());
    m_local_map_->setSensorInRobot(m_sensor_in_robot_);
    m_local_map_->measurement()->m_sensor_in_robot = m_sensor_in_robot_;
    m_local_map_->measurement()->m_pc = m_pc_moving_;
    m_local_map_->measurement()->getDistanceMap()->setGoals(
        *m_local_map_->measurement()->m_pc,
        m_param_.m_aligner->getDistMapParam().m_dist_map_res,
        m_param_.m_aligner->getDistMapParam().m_dist_map_size,
        m_param_.m_aligner->getDistMapParam().m_dist_map_max_dist);
    m_local_map_->measurement()->m_msgs->push_back(
        std::make_pair(m_scan_, EstimateType::Identity()));

    m_param_.m_aligner->setTarget(m_local_map_.get());

    m_status_ = status;

  }  // void Tracker_::reset(Status = Status::Initializing)

  /**
   * @brief reset the local map on the Tracker_ based on a reference point cloud
   *        set by an external class
   * @param[in] pc_moving reference point cloud
   * @param[in] robot_in_local_odom new robot pose w.r.t. local map estimation
   */
  void resetLocalMap(
      const PointCloudTypePtr& pc_moving,
      const EstimateType& robot_in_local_odom = EstimateType::Identity())
  {
    m_pc_moving_ = pc_moving;

    m_local_map_->setMeasurement(std::make_shared<MeasurementTypeElement>());
    m_local_map_->setSensorInRobot(m_sensor_in_robot_);
    m_local_map_->measurement()->m_sensor_in_robot = m_sensor_in_robot_;
    m_local_map_->measurement()->m_pc = m_pc_moving_;
    m_local_map_->measurement()->getDistanceMap()->setGoals(
        *m_local_map_->measurement()->m_pc,
        m_param_.m_aligner->getDistMapParam().m_dist_map_res,
        m_param_.m_aligner->getDistMapParam().m_dist_map_size,
        m_param_.m_aligner->getDistMapParam().m_dist_map_max_dist);

    m_param_.m_aligner->setTarget(m_local_map_.get());

    resetLocalMap(robot_in_local_odom);
  }  // void Tracker_::resetLocalMap(const PointCloudTypePtr&, const
     // EstimateType& = EstimateType::Identity())

  /**
   * @brief reset the local map on the Tracker_ based on the measurement hold by
   *        an SE2 distance map-based solver variable set by an external class
   * @param[in] local_map reference local map SE2 variable
   * @param[in] robot_in_local_odom new robot pose w.r.t. local map estimation
   */
  void resetLocalMap(
      VariableType* const local_map,
      const EstimateType& robot_in_local_odom = EstimateType::Identity())
  {
    m_pc_moving_ = local_map->measurement()->getPointCloudPtr();

    m_local_map_->setSensorInRobot(m_sensor_in_robot_);

    if ((m_param_.m_aligner->getDistMapParam().m_dist_map_res !=
         local_map->measurement()->getDistanceMap()->getRes()) ||
        (m_param_.m_aligner->getDistMapParam().m_dist_map_size !=
         local_map->measurement()->getDistanceMap()->getMapSize()) ||
        (m_param_.m_aligner->getDistMapParam().m_dist_map_max_dist !=
         local_map->measurement()->getDistanceMap()->getMaxDist()))
    {
      m_local_map_->setMeasurement(std::make_shared<MeasurementTypeElement>());
      m_local_map_->measurement()->m_sensor_in_robot = m_sensor_in_robot_;
      m_local_map_->measurement()->m_pc = m_pc_moving_;
      m_local_map_->measurement()->getDistanceMap()->setGoals(
          *m_local_map_->measurement()->m_pc,
          m_param_.m_aligner->getDistMapParam().m_dist_map_res,
          m_param_.m_aligner->getDistMapParam().m_dist_map_size,
          m_param_.m_aligner->getDistMapParam().m_dist_map_max_dist);
      m_local_map_->measurement()->m_msgs = local_map->measurement()->m_msgs;
    }
    else
    {
      m_local_map_->setMeasurement(local_map->measurement());
    }

    m_param_.m_aligner->setTarget(m_local_map_.get());

    resetLocalMap(robot_in_local_odom);
  }  // void Tracker_::resetLocalMap(VariableType* const, const EstimateType& =
     // EstimateType::Identity())

  /**
   * @brief setup the debug OpenGL-based visualization for the tracker's
   *        internal aligner
   * @param[in] qapp QApplication shared object
   * @param[in] viewer shared viewer master
   * @param[in] canvas OpenGL canvas
   * @param[in] canvas_name OpenGL canvas name
   */
  void setupDbgOGL(const OGLAppTypePtr& qapp, const OGLViewerTypePtr& viewer,
                   const OGLCanvasTypePtr& canvas = nullptr,
                   const std::string& canvas_name = "Tracker")
  {
    assert(m_param_.m_aligner && "Tracker_::setupDbgOGL | aligner not set");

    m_param_.m_aligner->setupDbgOGL(qapp, viewer, canvas, canvas_name);
  }  // void Tracker_::setupDbgOGL(const OGLAppTypePtr&, const
     // OGLViewerTypePtr&, const std::string& = "Tracker")

 protected:

  /**
   * @brief resets internal SE2 transformations required to guarantee a
   *        continuous estimation for the tracker's laser odometry, while having
   *        a correct initial estimation with external odometry source if
   *        enabled
   * @param[in] robot_in_local_odom new robot pose w.r.t. local map estimation
   */
  void resetLocalMap(const EstimateType& robot_in_local_odom)
  {
    EstimateType new_robot_in_local_odom_inv = robot_in_local_odom.inverse();

    m_robot_in_ext_odom_at_local_ =
        m_robot_in_ext_odom_ * new_robot_in_local_odom_inv;

    m_robot_in_odom_ =
        m_robot_in_odom_ * m_robot_in_local_odom_ * new_robot_in_local_odom_inv;
    m_robot_in_local_odom_ = robot_in_local_odom;

    if (m_param_.m_splitting_criteria)
    {
      m_param_.m_splitting_criteria->reset(*m_scan_, EstimateType::Identity());
    }

    if (m_param_.m_merging_criteria)
    {
      m_param_.m_merging_criteria->reset(*m_scan_, m_robot_in_local_odom_);
    }
  }  // void Tracker_::resetLocalMap(const EstimateType&)

 public:

  inline const Param& getParam() const { return m_param_; }
  inline Status getStatus() const { return m_status_; }

  inline AlignerBaseType* getAligner() const
  {
    return m_param_.m_aligner.get();
  }
  inline SplittingCriteriaType* getSplittingCriteria() const
  {
    return m_param_.m_splitting_criteria.get();
  }
  inline MergingCriteriaType* getMergingCriteria() const
  {
    return m_param_.m_merging_criteria.get();
  }

  inline const AlignerBaseTypePtr& getAlignerPtr() const
  {
    return m_param_.m_aligner;
  }
  inline const SplittingCriteriaTypePtr& getSplittingCriteriaPtr() const
  {
    return m_param_.m_splitting_criteria;
  }
  inline const MergingCriteriaTypePtr& getMergingCriteriaPtr() const
  {
    return m_param_.m_merging_criteria;
  }

  inline const AlignerBaseTypeStats& getAlignerStats() const
  {
    return m_stats_;
  }

  inline VariableType* getLocalMap() const { return m_local_map_.get(); }
  inline const VariableTypePtr& getLocalMapPtr() const { return m_local_map_; }

  inline LaserScanMsgType* getScan() const { return m_scan_.get(); }
  inline const LaserScanMsgTypePtr& getScanPtr() const { return m_scan_; }

  inline PointCloudType* getMovingPc() const { return m_pc_moving_.get(); }
  inline PointCloudTypePtr getMovingPcPtr() const { return m_pc_moving_; }

  inline const EstimateType& getRobotInLocalMap() const
  {
    return m_robot_in_local_odom_;
  }
  inline const EstimateTypeInfo& getRobotInLocalMapInfo() const
  {
    return m_robot_in_local_odom_info_;
  }
  inline const EstimateType& getRobotInLocalMapPrev() const
  {
    return m_robot_in_local_odom_prev_;
  }
  inline const EstimateTypeInfo& getRobotInLocalMapPrevInfo() const
  {
    return m_robot_in_local_odom_prev_info_;
  }
  inline const EstimateType getRobotInOdom() const
  {
    return m_robot_in_odom_ * m_robot_in_local_odom_;
  }
  inline const EstimateType& getRobotInExtOdom() const
  {
    return m_robot_in_ext_odom_;
  }
  inline const EstimateType& getRobotInExtOdomOnLocalMap() const
  {
    return m_robot_in_ext_odom_on_local_;
  }
  inline const EstimateType& getRobotInExtOdomAtLocalMap() const
  {
    return m_robot_in_ext_odom_at_local_;
  }
  inline const EstimateType& getSensorInRobot() const
  {
    return m_sensor_in_robot_;
  }

  inline void setSensorInRobot(const EstimateType& sensor_in_robot)
  {
    m_sensor_in_robot_ = sensor_in_robot;
  }

 protected:

  void preprocessScan()
  {
    if (m_param_.m_laser_invert_scan)
    {
      m_scan_->invert();
    }

    m_pc_moving_ = std::make_shared<PointCloudType>();
    m_pc_moving_->resize(m_scan_->m_ranges.size());

    Scalar range_min =
        m_param_.m_range_min < 0 ? m_scan_->m_range_min : m_param_.m_range_min;
    Scalar range_max =
        m_param_.m_range_max > 0 ? m_param_.m_range_max : m_scan_->m_range_max;

    for (size_t idx = 0; idx < m_scan_->m_ranges.size(); idx++)
    {
      Scalar angle = m_scan_->m_angle_min +
                     static_cast<Scalar>(idx) * m_scan_->m_angle_inc;

      m_scan_->angleDistToPt(m_pc_moving_->operator[](idx).coordinates(), angle,
                             m_scan_->m_ranges[idx]);

      if ((m_scan_->m_ranges[idx] < range_min) ||
          (m_scan_->m_ranges[idx] > range_max))
      {
        m_pc_moving_->operator[](idx).m_status = pcl::PointStatus::kOutOfRange;
      }
      else
      {
        m_pc_moving_->operator[](idx).m_status = pcl::PointStatus::kValid;
      }
    }
  }

  void preprocessPointCloud()
  {
    assert(m_pc_moving_ &&
           "Tracker_::preprocessPointCloud | moving cloud not set");
    if (m_param_.m_enable_normals)
    {
      m_normals_computator_->computeNormals(*m_pc_moving_);
    }
    if (m_param_.m_enable_voxelization)
    {
      assert(m_param_.m_voxel_scale > 0 &&
             "Tracker_::preprocessPointCloud | voxel scale must be greater "
             "than 0");

      m_pc_moving_->voxelize(m_param_.m_voxel_scale);
    }
  }

 private:

  void assertInit()
  {
    if (!m_param_.m_aligner)
    {
      throw std::runtime_error("Tracker_::assertInit | aligner not assigned");
    }
    if ((m_param_.m_merging_criteria) && (!m_param_.m_merger))
    {
      throw std::runtime_error(
          "Tracker_::assertInit | "
          "merger not assigned, even though merging criteria assigned...");
    }
    if (m_param_.m_enable_normals)
    {
      if (m_param_.m_normals_computator_min_pts < 3)
      {
        throw std::runtime_error(
            "Tracker_::assertInit | "
            "minimum points for the normals computation neighborhood must be "
            "greater or equal to 3");
      }
      if (m_param_.m_normals_computator_radius <= 0)
      {
        throw std::runtime_error(
            "Tracker_::assertInit | "
            "neighborhood radius for the normals computation neighborhood must "
            "be greater than 0");
      }
    }
    if ((m_param_.m_range_min >= 0) && (m_param_.m_range_max >= 0))
    {
      if (m_param_.m_range_max <= m_param_.m_range_min)
      {
        throw std::runtime_error(
            "Tracker_::assertInit | "
            "when range_max and range_min enabled, range_max must be greater "
            "than range_min");
      }
    }
  }

  void init()
  {
    if (m_param_.m_enable_voxelization)
    {
      if (m_param_.m_voxel_scale <= Scalar(0))
      {
        m_param_.m_voxel_scale =
            m_param_.m_aligner->getDistMapParam().m_dist_map_res;
      }
    }
    if (m_param_.m_enable_normals)
    {
      m_normals_computator_ = std::make_shared<NormalComputatorType>();
      m_normals_computator_->m_min_pts = m_param_.m_normals_computator_min_pts;
      m_normals_computator_->m_radius = m_param_.m_normals_computator_radius;
    }
  }

};  // class Tracker_<VariableSE2DistanceMapRightType_>

}  // namespace slam

}  // namespace inesctec_mrdt_slam_distmap_2d
