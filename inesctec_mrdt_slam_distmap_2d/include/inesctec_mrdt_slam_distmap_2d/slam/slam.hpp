#pragma once

#include <srrg2_core/srrg_viewer/viewer_canvas.h>
#include <srrg2_core/srrg_viewer/viewer_core/color_palette.h>
#include <srrg2_qgl_viewport/viewer_core_shared_qgl.h>
#include <srrg2_solver/solver_core/factor.h>
#include <srrg2_solver/solver_core/factor_graph.h>
#include <srrg2_solver/solver_core/factor_graph_interface.h>
#include <srrg2_solver/solver_core/instances.h>
#include <srrg2_solver/solver_core/internals/linear_solvers/instances.h>
#include <srrg2_solver/solver_core/internals/linear_solvers/sparse_block_linear_solver_cholesky.h>
#include <srrg2_solver/solver_core/iteration_algorithm_gn.h>
#include <srrg2_solver/solver_core/solver.h>
#include <srrg2_solver/solver_core/variable.h>

#include <Eigen/StdVector>
#include <memory>
#include <vector>

#include "inesctec_mrdt_slam_distmap_2d/slam/loop_finder.hpp"
#include "inesctec_mrdt_slam_distmap_2d/slam/tracker.hpp"
#include "inesctec_mrdt_slam_distmap_2d/srrg2_solver/instances.hpp"
#include "inesctec_mrdt_slam_distmap_2d/srrg2_solver/se2_distance_map_pose_pose_geodesic_error_factor.hpp"
#include "inesctec_mrdt_slam_distmap_2d/srrg2_solver/variable_se2_distance_map.hpp"
#include "inesctec_mrdt_slam_distmap_2d/system_utils/filesystem.hpp"

namespace inesctec_mrdt_slam_distmap_2d
{

namespace slam
{

template <typename VariableSE2DistanceMapRightType_>
class SLAM_
{
 public:

  using ThisType = SLAM_<VariableSE2DistanceMapRightType_>;

  using TrackerType = Tracker_<VariableSE2DistanceMapRightType_>;
  using TrackerTypePtr = std::shared_ptr<TrackerType>;
  using TrackerTypeParam = typename TrackerType::Param;
  using TrackerTypeStatus = typename TrackerType::Status;

  using VariableType = typename TrackerType::VariableType;
  using VariableTypePtr = typename TrackerType::VariableTypePtr;
  using EstimateType = typename TrackerType::EstimateType;
  using Scalar = typename TrackerType::Scalar;
  using EstimateTypeInfo = typename TrackerType::EstimateTypeInfo;

  using FactorType =
      srrg2_solver::SE2DistanceMapPosePoseGeodesicErrorFactor_<VariableType>;
  using FactorInformationMatrix = typename FactorType::InformationMatrixType;

  using SolverType = srrg2_solver::Solver;
  using FactorGraphType = srrg2_solver::FactorGraph;
  using FactorGraphTypePtr = std::shared_ptr<FactorGraphType>;
  using IterationAlgorithm = srrg2_solver::IterationAlgorithmGN;
  using LinearSolver = srrg2_solver::SparseBlockLinearSolverCholesky;

  using LoopFinderType = LoopFinder_<ThisType>;
  using LoopFinderTypePtr = std::shared_ptr<LoopFinderType>;
  using LoopFinderTypeParam = typename LoopFinderType::Param;
  using LoopFinderGraphLenType = typename LoopFinderType::GraphLenType;

  using HintPtrSetType = typename LoopFinderType::HintPtrSetClosest;
  using HintPtrLoopsSetType =
      typename LoopFinderType::HintPtrSetHigherInliersRatio;
  using HintPtrShortLoopsSetType = typename LoopFinderType::HintPtrSetClosest;

  using AlignerBaseType = typename TrackerType::AlignerBaseType;
  using AlignerBaseTypePtr = typename TrackerType::AlignerBaseTypePtr;
  using AlignerBaseTypeStats = typename TrackerType::AlignerBaseTypeStats;
  using PointCloudType = typename TrackerType::PointCloudType;
  using PointCloudTypePtr = typename TrackerType::PointCloudTypePtr;
  using MeasurementTypeElement = typename TrackerType::MeasurementTypeElement;

  using LaserScanMsgType = typename TrackerType::LaserScanMsgType;
  using LaserScanMsgTypePtr = typename TrackerType::LaserScanMsgTypePtr;

  using OGLAppType = typename TrackerType::OGLAppType;
  using OGLAppTypePtr = typename TrackerType::OGLAppTypePtr;
  using OGLViewerType = typename TrackerType::OGLViewerType;
  using OGLViewerTypePtr = typename TrackerType::OGLViewerTypePtr;
  using OGLCanvasType = typename TrackerType::OGLCanvasType;
  using OGLCanvasTypePtr = typename TrackerType::OGLCanvasTypePtr;

  using OGLPointCloudTargetType = srrg2_core::PointNormal3fVectorCloud;

 public:

  /**
   * @brief SLAM parametrization
   */
  struct Param
  {
    bool m_log_pose_corrected;  //!< log offline corrected pose (loop closure)
    bool m_solver_debug;     //!< enable backend dbg mode (OpenGL visualization)
    bool m_solver_verbose;   //!< enable verbose mode of the back-end solver
    float m_solver_damping;  //!< damping parameter for the iteration algorithm
    int m_solver_max_iterations;  //!< maximum number of iterations for back-end
                                  //!< graph optimizer
    std::string m_solver_criteria_type;  //!< type of termination criteria
                                         //!< (simple, gradient, stepnorm)
                                         //!< (empty to force max iterations)
    float m_solver_criteria_epsilon;     //!< epsilon threshold for the solver's
                                         //!< termination criteria
    int64_t m_solver_dbg_ogl_interval;   //!< sleep interval between consecutive
                                         //!< visualizations (ms)
    std::string m_solver_dbg_ogl_mode;   //!< type of debug visualization mode
                                         //!< (iterationend|computeend)

    bool m_factor_info_use_tracker;  //!< use aligner H as info matrix
    FactorInformationMatrix m_factor_info_mat;  //!< default info matrix
    bool m_factor_local_map_update;  //!< update factors alignment on previous
                                     //!< keyframe if SLAM in local map mode
                                     //!< and loop closure enabled
                                     //!< (no merging criteria set!)

    bool m_loop_enable;  //!< enable loop closure (add loop in graph)
    bool m_loop_use_same_place_guess;  //!< test same place (0,0,th)
    bool m_loop_use_pose_graph_guess;  //!< test pose estimamte (x,y,th)
    LoopFinderGraphLenType
        m_loop_min_graph_len;  //!< loop hint minimum graph length
    Scalar m_loop_max_dist;    //!< initial guess SE2 max distance relative to
                               //!< robot pose
    AlignerBaseTypePtr m_loop_aligner;  //!< loop aligner

    bool m_relocalization_enable;  //!< enable relocalization existing nodes in
                                   //!< the pose graph

    TrackerTypePtr m_tracker;  //!< tracker

    /**
     * @brief default constructor
     * @note possibly, by default, the graph's factor information matrix should
     *       give higher importance to the heading, accordingly to Giorgio
     *       (and, indeed, brings better results compared to identity...)
     */
    Param()
        : m_log_pose_corrected(false),
          m_solver_debug(true),
          m_solver_verbose(false),
          m_solver_damping(1.0),
          m_solver_max_iterations(100),
          m_solver_criteria_type("stepnorm"),
          m_solver_criteria_epsilon(1e-5),
          m_solver_dbg_ogl_interval(0),
          m_solver_dbg_ogl_mode(""),
          m_factor_info_use_tracker(false),
          m_factor_info_mat(FactorInformationMatrix::Identity()),
          m_factor_local_map_update(false),
          m_loop_enable(true),
          m_loop_use_same_place_guess(true),
          m_loop_use_pose_graph_guess(true),
          m_loop_min_graph_len(5),
          m_loop_max_dist(2.50),
          m_loop_aligner(nullptr),
          m_relocalization_enable(false),
          m_tracker(nullptr)
    {
    }

    /**
     * @brief slam parametrization with loop aligner and tracker independent
     *        implementations
     * @param[in] log_pose_corrected log offline corrected pose (loop closure)
     * @param[in] solver_debug enable debug mode (OpenGL-based visualization)
     * @param[in] solver_verbose enable verbose mode of the solver (to show the
     *                           iteration stats)
     * @param[in] solver_damping damping parameter for the iteration algorithm
     *                           when computing the new state
     * @param[in] solver_max_iterations maximum number of iterations allowed
     *                                  for the solver
     * @param[in] solver_criteria_type type of termination criteria
     *                                 (simple, gradient, stepnorm)
     * @param[in] solver_criteria_epsilon epsilon threshold for the solver's
     *                                    termination criteria
     * @param[in] solver_dbg_ogl_interval sleep interval between consecutive
     *                                    visualizations (ms)
     * @param[in] solver_dbg_ogl_mode type of debug visualization mode
     *                                (iterationend|computeend)
     * @param[in] factor_info_use_tracker use aligner H as info matrix
     * @param[in] factor_info_mat default info matrix
     * @param[in] factor_local_map_update update factors alignment on previous
     *                                    keyframe if SLAM in local map mode
     *                                    and loop closure enabled
     *                                    (no merging criteria set!)
     * @param[in] loop_enable enable loop closure (add loop in graph)
     * @param[in] loop_use_same_place_guess test same place (0,0,th)
     * @param[in] loop_use_pose_graph_guess test pose estimamte (x,y,th)
     * @param[in] loop_min_graph_len loop hint minimum graph length
     * @param[in] loop_max_dist initial guess SE2 max distance relative to
     *                          robot pose
     * @param[in] loop_aligner loop aligner
     * @param[in] relocalization_enable enable relocalization existing nodes in
     *                                  the pose graph
     * @param[in] tracker tracker
     */
    Param(bool log_pose_corrected, bool solver_debug, bool solver_verbose,
          float solver_damping, int solver_max_iterations,
          std::string solver_criteria_type, float solver_criteria_epsilon,
          int64_t solver_dbg_ogl_interval, std::string solver_dbg_ogl_mode,
          bool factor_info_use_tracker, FactorInformationMatrix factor_info_mat,
          bool factor_local_map_update, bool loop_enable,
          bool loop_use_same_place_guess, bool loop_use_pose_graph_guess,
          LoopFinderGraphLenType loop_min_graph_len, float loop_max_dist,
          AlignerBaseTypePtr loop_aligner, bool relocalization_enable,
          TrackerTypePtr tracker)
        : m_log_pose_corrected(log_pose_corrected),
          m_solver_debug(solver_debug),
          m_solver_verbose(solver_verbose),
          m_solver_damping(solver_damping),
          m_solver_max_iterations(solver_max_iterations),
          m_solver_criteria_type(solver_criteria_type),
          m_solver_criteria_epsilon(solver_criteria_epsilon),
          m_solver_dbg_ogl_interval(solver_dbg_ogl_interval),
          m_solver_dbg_ogl_mode(solver_dbg_ogl_mode),
          m_factor_info_use_tracker(factor_info_use_tracker),
          m_factor_info_mat(factor_info_mat),
          m_factor_local_map_update(factor_local_map_update),
          m_loop_enable(loop_enable),
          m_loop_use_same_place_guess(loop_use_same_place_guess),
          m_loop_use_pose_graph_guess(loop_use_pose_graph_guess),
          m_loop_min_graph_len(loop_min_graph_len),
          m_loop_max_dist(loop_max_dist),
          m_loop_aligner(loop_aligner),
          m_relocalization_enable(relocalization_enable),
          m_tracker(tracker)
    {
    }

    friend std::ostream& operator<<(std::ostream& os, const Param& param)
    {
      os << "- log offline corrected pose (w/ loop closures correction): "
         << (param.m_log_pose_corrected ? "enabled" : "not enabled")
         << std::endl;
      os << "- back-end graph optimizer solver parametrization:" << std::endl;
      os << "  - OpenGL-based visualization dbg: "
         << (param.m_solver_debug ? "enabled" : "not enabled") << std::endl;
      os << "  - solver verbose enabled   : "
         << (param.m_solver_verbose ? "enabled" : "not enabled") << std::endl;
      os << "  - solver algorithm damping : " << param.m_solver_damping
         << std::endl;
      os << "  - solver maximum iterations: " << param.m_solver_max_iterations
         << std::endl;
      os << "  - solver criteria type   : "
         << (param.m_solver_criteria_type.empty()
                 ? "not enabled"
                 : param.m_solver_criteria_type)
         << std::endl;
      if (!param.m_solver_criteria_type.empty())
      {
        os << "  - solver criteria epsilon: " << param.m_solver_criteria_epsilon
           << std::endl;
      }
      if (param.m_solver_debug)
      {
        os << "  - dbg sleep interval between iterations (ms): "
           << param.m_solver_dbg_ogl_interval << std::endl;
        os << "  - OpenGL-based visualization debug mode     : "
           << param.m_solver_dbg_ogl_mode << std::endl;
      }
      else
      {
        os << "  - dbg sleep interval between iterations (ms): "
           << "not enabled" << std::endl;
        os << "  - OpenGL-based visualization debug mode     : "
           << "not enabled" << std::endl;
      }

      os << "- factor information matrix: ";
      if (param.m_factor_info_use_tracker)
      {
        os << "H matrix from alignment" << std::endl;
      }
      else
      {
        os << std::endl << param.m_factor_info_mat << std::endl;
      }

      os << "- factor automatic update  : "
         << (param.m_tracker->getMergingCriteriaPtr() && param.m_loop_enable
                 ? param.m_factor_local_map_update ? "enabled" : "not enabled"
                 : "N/A")
         << std::endl;

      os << "- loop closure: "
         << (param.m_loop_enable ? "enabled" : "not enabled") << std::endl;
      if (param.m_loop_enable)
      {
        os << "  - use same place as initial guess (0,0,th): "
           << (param.m_loop_use_same_place_guess ? "enabled" : "not enabled")
           << std::endl;
        os << "  - use pose graph as initial guess (x,y,th): "
           << (param.m_loop_use_pose_graph_guess ? "enabled" : "not enabled")
           << std::endl;
        os << "  - hint minimum graph length : " << param.m_loop_min_graph_len
           << std::endl;
        os << "  - hint maximum pose distance: " << param.m_loop_max_dist
           << std::endl;
        if (param.m_loop_aligner != param.m_tracker->getAlignerPtr())
        {
          os << "  - loop aligner parametrization:" << std::endl
             << "{" << std::endl
             << param.m_loop_aligner->getParam()
             << param.m_loop_aligner->getDistMapParam() << "}" << std::endl;
        }
        else
        {
          os << "  - loop aligner parametrization: same as the tracker"
             << std::endl;
        }
      }

      os << "- pose graph relocalization: "
         << (param.m_relocalization_enable ? "enabled" : "not enabled")
         << std::endl;

      if (param.m_tracker)
      {
        os << "- tracker parametrization:" << std::endl
           << "{" << std::endl
           << param.m_tracker->getParam() << "}" << std::endl;
      }
      else
      {
        os << "- tracker parametrization: not enabled" << std::endl;
      }

      return os;
    }
  };  // SLAM_<VariableSE2DistanceMapRightType_>::Param

  struct LocalPoseTfStamped
  {
   public:

    double m_t;  //!< timestamp (s)
    // EstimateType m_sensor_in_robot =
    //     EstimateType::Identity();  //!< sensor w.r.t. robot frame
    EstimateType m_pose_in_local_map =
        EstimateType::Identity();  //!< robot pose w.r.t. local map
    srrg2_solver::VariableBase::Id m_local_map_id = -1;  //!< local map graph id

   public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    LocalPoseTfStamped() = default;
    LocalPoseTfStamped(double t, /* const EstimateType& sensor_in_robot, */
                       const EstimateType& pose_in_local,
                       srrg2_solver::VariableBase::Id local_map_id)
        : m_t(t),
          // m_sensor_in_robot(sensor_in_robot),
          m_pose_in_local_map(pose_in_local),
          m_local_map_id(local_map_id)
    {
    }
    LocalPoseTfStamped(double t, /* EstimateType&& sensor_in_robot, */
                       EstimateType&& pose_in_local,
                       srrg2_solver::VariableBase::Id local_map_id)
        : m_t(t),
          // m_sensor_in_robot(std::move(sensor_in_robot)),
          m_pose_in_local_map(std::move(pose_in_local)),
          m_local_map_id(local_map_id)
    {
    }

  };  // SLAM_<VariableSE2DistanceMapRightType_>::LocalPoseTfStamped

 public:

  const Eigen::Vector4f OGLPointCloudMovingColorVar =
      srrg2_core::ColorPalette::color4fDarkRed(0.75f);

 protected:

  Param m_param_;  //!< parametrization

  SolverType m_solver_;         //!< back-end solver to optimize pose graph
  FactorGraphTypePtr m_graph_;  //!< factor graph

  VariableType* m_keyframe_;  //!< current keyframe in the global pose graph

  LoopFinderTypePtr m_loop_finder_;  //!< loop finder (near nodes+graph search)

  EstimateType m_robot_in_map_ =
      EstimateType::Identity();  //!< robot pose w.r.t. global map / world frame

  OGLAppTypePtr m_dbg_ogl_qapp_;       //!< Qt application required for the SRRG
                                       //!< ViewerCoreSharedQGL viz
  OGLViewerTypePtr m_dbg_ogl_viewer_;  //!< ViewerCoreSharedQGL handler
  OGLCanvasTypePtr m_dbg_ogl_canvas_;  //!< viz canvas to show tracking results
  std::string m_dbg_ogl_canvas_name_;  //!< viz canvas title name (to be used in
                                       //!< the ViewerCoreSharedQGL)

  std::vector<LocalPoseTfStamped, Eigen::aligned_allocator<LocalPoseTfStamped>>
      m_pose_history_;  //!< robot pose history (if sensor needed, set tf 0,0,0)

 public:

  SLAM_(const Param& param = Param())
      : m_param_(param),
        m_graph_(std::make_shared<FactorGraphType>()),
        m_keyframe_(nullptr),
        m_loop_finder_(nullptr),
        m_robot_in_map_(EstimateType::Identity())
  {
    assertInit();
    init();
  }

  /**
   * @brief process the latest scan message from a 2D laser scanner
   * @param[in] scan 2D laser scan message
   * @param[in] ext_odometry external odometry source (ony used depending on the
   *                         tracker's parametrization)
   */
  void processScan(const LaserScanMsgTypePtr& scan,
                   const EstimateType& ext_odometry = EstimateType::Identity())
  {
    m_param_.m_tracker->processScan(scan, ext_odometry);

    if (m_param_.m_tracker->getStatus() == TrackerTypeStatus::Initialized)
    {
      initFixedKeyframe();
      goto log_pose_corrected;
    }
    else if ((m_param_.m_tracker->getStatus() == TrackerTypeStatus::Error) ||
             (m_param_.m_tracker->getStatus() == TrackerTypeStatus::ErrorReset))
    {
      addOdomKeyframe();
      goto log_pose_corrected;
    }
    else if (m_param_.m_tracker->getStatus() ==
             TrackerTypeStatus::TrackingMeasurementSplit)
    {
      addOdomKeyframe();
    }

    m_robot_in_map_ =
        m_keyframe_->estimate() * m_param_.m_tracker->getRobotInLocalMap();

    if (m_param_.m_loop_enable)
    {
      m_loop_finder_->compute();

      HintPtrSetType* hints = m_param_.m_relocalization_enable
                                  ? m_loop_finder_->getShortLoops()
                                  : m_loop_finder_->getHints();

      HintPtrLoopsSetType loops;
      HintPtrShortLoopsSetType short_loops;

      for (auto& hint : *hints)
      {
        EstimateType pose_graph_guess =
            hint->m_local_map->estimate().inverse() * m_robot_in_map_;

        EstimateType same_place_guess = geometry::v2t(
            Eigen::Vector3f(0, 0, geometry::t2v(pose_graph_guess).z()));

        hint->m_chi_inliers = -1.f;
        hint->m_inliers_ratio = -1.f;

        m_param_.m_loop_aligner->setSource(
            m_param_.m_tracker->getMovingPcPtr(),
            m_param_.m_tracker->getSensorInRobot());
        m_param_.m_loop_aligner->setTarget(hint->m_local_map);
        m_param_.m_loop_aligner->setPriorEnabled(false);

        if (m_param_.m_loop_use_pose_graph_guess)
        {
          m_param_.m_loop_aligner->setSourceInTarget(pose_graph_guess);
          m_param_.m_loop_aligner->compute();

          if (m_param_.m_loop_aligner->getStatus() ==
              AlignerBaseType::Status::Success)
          {
            hint->m_initial_guess =
                m_param_.m_loop_aligner->getSourceInTarget();
            hint->m_info_mat = m_param_.m_factor_info_use_tracker
                                   ? m_param_.m_loop_aligner->getH()
                                   : m_param_.m_factor_info_mat;
            hint->m_chi_inliers =
                m_param_.m_loop_aligner->getStats().m_chi_inliers;
            hint->m_inliers_ratio =
                m_param_.m_loop_aligner->getStats().m_inliers_ratio;

            if (hint->m_graph_len >=
                m_loop_finder_->getParam().m_loop_min_graph_len)
            {
              loops.emplace(hint);
            }

            if (m_param_.m_relocalization_enable)
            {
              short_loops.emplace(hint);
            }

            continue;
          }
        }

        if (m_param_.m_loop_use_same_place_guess)
        {
          m_param_.m_loop_aligner->setSourceInTarget(same_place_guess);
          m_param_.m_loop_aligner->compute();

          if (m_param_.m_loop_aligner->getStatus() ==
              AlignerBaseType::Status::Success)
          {
            hint->m_initial_guess =
                m_param_.m_loop_aligner->getSourceInTarget();
            hint->m_info_mat = m_param_.m_factor_info_use_tracker
                                   ? m_param_.m_loop_aligner->getH()
                                   : m_param_.m_factor_info_mat;
            hint->m_chi_inliers =
                m_param_.m_loop_aligner->getStats().m_chi_inliers;
            hint->m_inliers_ratio =
                m_param_.m_loop_aligner->getStats().m_inliers_ratio;

            if (hint->m_graph_len >=
                m_loop_finder_->getParam().m_loop_min_graph_len)
            {
              loops.emplace(hint);
            }

            if (m_param_.m_relocalization_enable)
            {
              short_loops.emplace(hint);
            }

            continue;
          }
        }
      }

      for (const auto& loop : loops)
      {
        addFactor(loop->m_local_map, m_keyframe_,
                  loop->m_initial_guess *
                      m_param_.m_tracker->getRobotInLocalMap().inverse(),
                  loop->m_info_mat);
      }

      if (!loops.empty())
      {
        m_solver_.compute();
        m_robot_in_map_ =
            m_keyframe_->estimate() * m_param_.m_tracker->getRobotInLocalMap();
      }

      if (m_param_.m_relocalization_enable)
      {
        if (!short_loops.empty())
        {
          const auto& loop = *short_loops.begin();

          const geometry::Vector3_<Scalar> delta_to_loop = geometry::t2v(
              loop->m_local_map->estimate().inverse() * m_robot_in_map_);

          Scalar delta_to_loop_dist = delta_to_loop.head(2).norm();
          Scalar delta_to_local_dist =
              m_param_.m_tracker->getRobotInLocalMap().translation().norm();

          if (delta_to_loop_dist < delta_to_local_dist)
          {
            m_param_.m_tracker->resetLocalMap(loop->m_local_map,
                                              loop->m_initial_guess);

            updateKeyframeFactors(m_keyframe_, false);
            updateKeyframeFactors(loop->m_local_map, true);

            m_keyframe_ = loop->m_local_map;
            m_robot_in_map_ = m_keyframe_->estimate() *
                              m_param_.m_tracker->getRobotInLocalMap();
          }
        }
      }
    }

  log_pose_corrected:

    if (m_param_.m_log_pose_corrected)
    {
      m_pose_history_.emplace_back(LocalPoseTfStamped(
          scan->m_header.m_t, /* m_param_.m_tracker->getSensorInRobot(), */
          m_param_.m_tracker->getRobotInLocalMap(), m_keyframe_->graphId()));
    }
  }  // void SLAM_::processScan(const LaserScanMsgTypePtr&, const
     // EstimateType& = EstimateType::Identity())

  /**
   * @brief initialize the first keyframe in the graph
   *        (fixate its status to avoid undetermined pose graph optimization)
   */
  void initFixedKeyframe()
  {
    assert(m_param_.m_tracker->getLocalMap() &&
           "SLAM_::initFixedKeyframe | local map undefined");
    assert(m_param_.m_tracker->getLocalMap()->measurement() &&
           "SLAM_::initFixedKeyframe | local map measurement undefined");

    VariableTypePtr fixed_keyframe = std::make_shared<VariableType>();

    fixed_keyframe->setMeasurement(
        m_param_.m_tracker->getLocalMap()->measurement());
    fixed_keyframe->setSensorInRobot(m_param_.m_tracker->getSensorInRobot());
    fixed_keyframe->setEstimate(EstimateType::Identity());
    fixed_keyframe->setStatus(srrg2_solver::VariableBase::Status::Fixed);

    m_graph_->addVariable(fixed_keyframe);

    m_keyframe_ = fixed_keyframe.get();
    m_robot_in_map_ = EstimateType::Identity();
  }  // void SLAM_::initFixedKeyframe()

  /**
   * @brief add an odometry-based keyframe based on the alignment computed by
   *        the tracker between the current and the previous keyframes
   *        (before splitting / occuring alignment errors)
   */
  void addOdomKeyframe()
  {
    assert(m_param_.m_tracker->getLocalMap() &&
           "SLAM_::addOdomKeyframe | local map undefined");
    assert(m_param_.m_tracker->getLocalMap()->measurement() &&
           "SLAM_::addOdomKeyframe | local map measurement undefined");

    VariableType* const v1 = m_keyframe_;

    VariableTypePtr new_keyframe = std::make_shared<VariableType>();

    new_keyframe = std::make_shared<VariableType>();
    new_keyframe->setMeasurement(
        m_param_.m_tracker->getLocalMap()->measurement());
    new_keyframe->setSensorInRobot(m_param_.m_tracker->getSensorInRobot());
    new_keyframe->setEstimate(v1->estimate() *
                              m_param_.m_tracker->getRobotInLocalMapPrev());
    new_keyframe->setStatus(srrg2_solver::VariableBase::Status::Active);

    m_graph_->addVariable(new_keyframe);

    VariableType* const v2 = new_keyframe.get();

    EstimateType v2_in_v1 = v1->estimate().inverse() * v2->estimate();

    addFactor(v1, v2, v2_in_v1,
              m_param_.m_factor_info_use_tracker
                  ? m_param_.m_tracker->getAligner()->getH()
                  : m_param_.m_factor_info_mat);

    updateKeyframeFactors(m_keyframe_, true);

    m_keyframe_ = new_keyframe.get();
  }  // void SLAM_::addOdomKeyframe()

  /**
   * @brief add a SE2 pose geosedic factor to the graph between two keyframes
   * @param[in] v1 'from' SE2 variable
   * @param[in] v2 'to' SE2 variable
   * @param[in] v2_in_v1 SE2 transformation measurement to set in the factor
   * @param[in] v2_in_v1_info information matrix for SE2 pose geosedic factor
   */
  void addFactor(VariableType* const v1, VariableType* const v2,
                 const EstimateType& v2_in_v1,
                 const EstimateTypeInfo& v2_in_v1_info)
  {
    std::shared_ptr<FactorType> f = std::make_shared<FactorType>();

    f->setVariableId(0, v1->graphId());
    f->setVariableId(1, v2->graphId());
    f->setMeasurement(v2_in_v1);
    f->setInformationMatrix(v2_in_v1_info);

    m_graph_->addFactor(f);
  }  // void SLAM_::addFactor(VariableType* const, VariableType* const, const
     // EstimateType&)

  void updateKeyframeFactors(VariableType* const v, bool update_graph = false)
  {
    if (!m_param_.m_factor_local_map_update)
    {
      return;
    }

    for (auto f :
         static_cast<srrg2_solver::FactorGraphInterface*>(m_graph_.get())
             ->factors(static_cast<srrg2_solver::VariableBase*>(v)))
    {
      if (!f->enabled())
      {
        continue;
      }

      VariableType* v0 = static_cast<VariableType*>(f->variable(0));
      VariableType* v1 = static_cast<VariableType*>(f->variable(1));

      m_param_.m_loop_aligner->setSource(v1);
      m_param_.m_loop_aligner->setTarget(v0);
      m_param_.m_loop_aligner->setPriorEnabled(false);
      m_param_.m_loop_aligner->setSourceInTarget(v0->estimate().inverse() *
                                                 v1->estimate());
      m_param_.m_loop_aligner->compute();

      static_cast<FactorType*>(f)->setMeasurement(
          m_param_.m_loop_aligner->getSourceInTarget());
    }

    if (update_graph)
    {
      m_solver_.compute();
    }
  }  // void SLAM_::updateKeyframeFactors(VariableType* const, bool = false)

  void saveGraph(const std::string filename)
  {
    if (filename.empty())
    {
      return;
    }

    system_utils::validateAndCreatePath(filename);

    m_graph_->write(filename);
  }  // void SLAM_::saveGraph(const std::string)

  /**
   * @brief setup the debug OpenGL-based visualization for slam internals
   * @param[in] qapp QApplication shared object
   * @param[in] viewer shared viewer master
   * @param[in] canvas OpenGL canvas
   * @param[in] canvas_name OpenGL SLAM canvas name
   */
  void setupDbgOGL(const OGLAppTypePtr& qapp, const OGLViewerTypePtr& viewer,
                   const OGLCanvasTypePtr& canvas = nullptr,
                   const std::string& canvas_name = "SLAM")
  {
    if (!qapp || !viewer)
    {
      throw std::runtime_error(
          "SLAM_::setupDbgOGL | "
          "QtApplication and OpenGL shared viewer shared pointers must be "
          "initialized");
    }

    m_dbg_ogl_qapp_ = qapp;
    m_dbg_ogl_viewer_ = viewer;
    m_dbg_ogl_canvas_name_ = canvas_name;

    if (m_param_.m_solver_debug)
    {
      if (!canvas)
      {
        m_dbg_ogl_canvas_ =
            m_dbg_ogl_viewer_->getCanvas(m_dbg_ogl_canvas_name_);
      }
      else
      {
        m_dbg_ogl_canvas_ = canvas;
        m_dbg_ogl_canvas_name_ = canvas->name();
      }
      m_dbg_ogl_canvas_->_setup();
    }

    if (m_param_.m_tracker->getParam().m_aligner->getParam().m_debug)
    {
      m_param_.m_tracker->getParam().m_aligner->setupDbgOGL(
          qapp, viewer, nullptr, m_dbg_ogl_canvas_name_ + " - Aligner");
    }

    if ((m_param_.m_loop_aligner) &&
        (m_param_.m_loop_aligner != m_param_.m_tracker->getParam().m_aligner))
    {
      if (m_param_.m_loop_aligner->getParam().m_debug)
      {
        m_param_.m_loop_aligner->setupDbgOGL(
            qapp, viewer, nullptr, m_dbg_ogl_canvas_name_ + " - Loop Aligner");
      }
    }
  }  // void SLAM_::setupDbgOGL(const OGLAppTypePtr&, const OGLViewerTypePtr&,
     // const std::string& = "SLAM")

  /**
   * @brief update the debug OpenGL-based visualization canvas
   * @param[in] update_canvas update canvas if true (if not, you can call
   *                          another updateDbgOPGLCanvas with the same canvas
   *                          to draw over it)
   */
  void updateDbgOPGLCanvas(bool update_canvas = true)
  {
    if (!m_param_.m_solver_debug)
    {
      throw std::runtime_error(
          "SLAM_::updateDbgOPGLCanvas | "
          "debug mode not enabled (only w/ dbg enabled the OGL viewer can be "
          "setup)");
    }
    if ((!m_dbg_ogl_qapp_) || (!m_dbg_ogl_viewer_) || (!m_dbg_ogl_canvas_))
    {
      throw std::runtime_error(
          "Aligner_::updateDbgOPGLCanvas | "
          "missing setup of the OpenGL-based debug visualization");
    }
    if ((m_param_.m_loop_aligner) &&
        (m_param_.m_loop_aligner != m_param_.m_tracker->getParam().m_aligner))
    {
      if (m_param_.m_loop_aligner->getParam().m_debug)
      {
        m_param_.m_loop_aligner->updateDbgOPGLCanvas(true);
      }
    }

    for (auto variable : m_graph_->variables())
    {
      const VariableType* v = static_cast<const VariableType*>(variable.second);

      v->_drawImpl(m_dbg_ogl_canvas_);
    }

    for (auto factor : m_graph_->factors())
    {
      const FactorType* f = static_cast<FactorType*>(factor.second);

      f->_drawImpl(m_dbg_ogl_canvas_);
    }

    m_dbg_ogl_canvas_->pushColor();
    m_dbg_ogl_canvas_->setColor(OGLPointCloudMovingColorVar);
    m_dbg_ogl_canvas_->pushMatrix();
    m_dbg_ogl_canvas_->multMatrix(
        srrg2_core::geometry3d::get3dFrom2dPose(getRobotInMap()).matrix());
    m_dbg_ogl_canvas_->putReferenceSystem(0.10);
    m_dbg_ogl_canvas_->popMatrix();
    m_dbg_ogl_canvas_->popAttribute();

    m_dbg_ogl_viewer_->lookAtPosition2D(m_dbg_ogl_canvas_name_,
                                        m_robot_in_map_.translation().x(),
                                        m_robot_in_map_.translation().y());

    if (update_canvas)
    {
      m_dbg_ogl_canvas_->flush();
    }
  }  // void SLAM_::updateDbgOPGLCanvas(bool = true)

 public:

  inline const Param& getParam() const { return m_param_; }
  inline const SolverType& getSolver() const { return m_solver_; }
  inline FactorGraphType* getGraph() { return m_graph_.get(); }
  inline const FactorGraphTypePtr& getGraphPtr() const { return m_graph_; }
  inline VariableType* getKeyframe() const { return m_keyframe_; }
  inline const LoopFinderType* getLoopFinder() const
  {
    return m_loop_finder_.get();
  }
  inline const LoopFinderTypePtr getLoopFinderPtr() const
  {
    return m_loop_finder_;
  }
  inline const EstimateType& getRobotInMap() const { return m_robot_in_map_; }
  inline TrackerType* getTracker() { return m_param_.m_tracker.get(); }
  inline const TrackerTypePtr getTrackerPtr() { return m_param_.m_tracker; }

  inline const std::vector<LocalPoseTfStamped,
                           Eigen::aligned_allocator<LocalPoseTfStamped>>&
  getRobotPoseHistory() const
  {
    return m_pose_history_;
  }

 private:

  void assertInit()
  {
    // Back-end parametrization

    if (m_param_.m_solver_damping < 0)
    {
      throw std::runtime_error(
          "SLAM_::assertInit | solver algorithm "
          "damping value must be greater or equal to 0");
    }
    if (m_param_.m_solver_max_iterations <= 0)
    {
      throw std::runtime_error(
          "SLAM_::assertInit | solver algorithm "
          "maximum iterations must be greater than 0");
    }
    if ((!m_param_.m_solver_criteria_type.empty()) &&
        (m_param_.m_solver_criteria_type.compare("simple") != 0) &&
        (m_param_.m_solver_criteria_type.compare("gradient") != 0) &&
        (m_param_.m_solver_criteria_type.compare("stepnorm") != 0))
    {
      throw std::runtime_error(
          "SLAM_::assertInit | invalid solver "
          "termination criteria type (simple, gradient, stepnorm)");
    }
    if ((!m_param_.m_solver_criteria_type.empty()) &&
        (m_param_.m_solver_criteria_epsilon < 0))
    {
      throw std::runtime_error(
          "SLAM_::assertInit | epsilon for the "
          "termination criteria must be greater or equal to 0 if termination "
          "criteria type enabled");
    }
    if ((m_param_.m_solver_debug) && (m_param_.m_solver_dbg_ogl_interval < 0))
    {
      throw std::runtime_error(
          "SLAM_::assertInit | sleep interval "
          "between events must be equal or greater than 0");
    }
    if ((m_param_.m_solver_debug) &&
        (m_param_.m_solver_dbg_ogl_mode.compare("iterationend") != 0) &&
        (m_param_.m_solver_dbg_ogl_mode.compare("computeend") != 0))
    {
      throw std::runtime_error(
          "SLAM_::assertInit | invalid OpenGL-based "
          "debug mode (iterationend, computeend)");
    }

    // Tracker

    if (!m_param_.m_tracker)
    {
      throw std::runtime_error("SLAM_::assertInit | tracker not assigned");
    }

    // Loop closure detection

    if (m_param_.m_loop_enable)
    {
      if (m_param_.m_loop_max_dist <= 0)
      {
        throw std::runtime_error(
            "SLAM_::assertInit | loop maximum distance must be greater "
            "than 0 (m)");
      }
      if (m_param_.m_loop_min_graph_len < 1)
      {
        throw std::runtime_error(
            "SLAM_::assertInit | loop minimum graph length must be greater "
            "than 0 (#edges)");
      }
      if (!m_param_.m_loop_aligner)
      {
        throw std::runtime_error(
            "SLAM_::assertInit | loop aligner not assigned");
      }
      if (!m_param_.m_tracker->getSplittingCriteriaPtr())
      {
        throw std::runtime_error(
            "SLAM_::assertInit | loop closure detection enabled but no "
            "splitting criteria defined for the tracker");
      }
    }

    if ((m_param_.m_factor_local_map_update) &&
        ((!m_param_.m_tracker->getMergingCriteriaPtr()) ||
         (!m_param_.m_loop_enable)))
    {
      throw std::runtime_error(
          "SLAM_::assertInit | automatic update of the previous local map "
          "factors alignment enabled but no merging criteria defined for the "
          "tracker and/or loop closure detection disabled");
    }

    // Relocalization

    if ((!m_param_.m_loop_enable) && (m_param_.m_relocalization_enable))
    {
      throw std::runtime_error(
          "SLAM_::assertInit | relocalization requires that loop closure "
          "detection is enabled!");
    }
  }

  void init()
  {
    using SimpleCriteriaType = srrg2_solver::SimpleTerminationCriteria;
    using RelativeGradientChiCriteriaType =
        srrg2_solver::RelativeGradientChiTerminationCriteria;
    using PerturbationNormCriteriaType =
        srrg2_solver::PerturbationNormTerminationCriteria;

    using SimpleCriteriaTypePtr = std::shared_ptr<SimpleCriteriaType>;
    using RelativeGradientChiCriteriaTypePtr =
        std::shared_ptr<RelativeGradientChiCriteriaType>;
    using PerturbationNormCriteriaTypePtr =
        std::shared_ptr<PerturbationNormCriteriaType>;

    // Back-end parametrization

    m_solver_.param_verbose.setValue(m_param_.m_solver_verbose);
    if (m_param_.m_solver_verbose)
    {
      m_solver_.param_actions.pushBack(
          std::make_shared<srrg2_solver::SolverVerboseLastAction>());
    }

    m_solver_.param_algorithm.setValue(std::make_shared<IterationAlgorithm>());
    dynamic_cast<IterationAlgorithm*>(m_solver_.param_algorithm.value().get())
        ->param_damping.setValue(m_param_.m_solver_damping);
    m_solver_.param_max_iterations.setValue({m_param_.m_solver_max_iterations});
    m_solver_.param_linear_solver.setValue(std::make_shared<LinearSolver>());

    if (m_param_.m_solver_criteria_type.empty())
    {
      m_solver_.param_termination_criteria.setValue(nullptr);
    }
    else if (m_param_.m_solver_criteria_type.compare("simple") == 0)
    {
      SimpleCriteriaTypePtr simple_criteria =
          std::make_shared<SimpleCriteriaType>();

      simple_criteria->param_epsilon.setValue(
          m_param_.m_solver_criteria_epsilon);
      simple_criteria->setName(m_param_.m_solver_criteria_type);

      m_solver_.param_termination_criteria.setValue(simple_criteria);
    }
    else if (m_param_.m_solver_criteria_type.compare("gradient") == 0)
    {
      RelativeGradientChiCriteriaTypePtr gradient_criteria =
          std::make_shared<RelativeGradientChiCriteriaType>();

      gradient_criteria->param_epsilon.setValue(
          m_param_.m_solver_criteria_epsilon);
      gradient_criteria->setName(m_param_.m_solver_criteria_type);

      m_solver_.param_termination_criteria.setValue(gradient_criteria);
    }
    else if (m_param_.m_solver_criteria_type.compare("stepnorm") == 0)
    {
      PerturbationNormCriteriaTypePtr norm_criteria =
          std::make_shared<PerturbationNormCriteriaType>();

      norm_criteria->param_epsilon.setValue(m_param_.m_solver_criteria_epsilon);
      norm_criteria->setName(m_param_.m_solver_criteria_type);

      m_solver_.param_termination_criteria.setValue(norm_criteria);
    }

    if ((m_param_.m_solver_debug) && (m_param_.m_solver_dbg_ogl_interval >= 0))
    {
      if (m_param_.m_solver_dbg_ogl_mode.compare("iterationend") == 0)
      {
        srrg2_solver::SolverGenericAtIterationEndActionPtr iterationend =
            std::make_shared<srrg2_solver::SolverGenericAtIterationEndAction>();

        iterationend->m_sleep_ms = m_param_.m_solver_dbg_ogl_interval;
        iterationend->m_action =
            std::bind(&ThisType::updateDbgOPGLCanvas, this, true);

        m_solver_.param_actions.pushBack(iterationend);
      }
      else if (m_param_.m_solver_dbg_ogl_mode.compare("computeend") == 0)
      {
        srrg2_solver::SolverGenericAtComputeEndActionPtr computeend =
            std::make_shared<srrg2_solver::SolverGenericAtComputeEndAction>();

        computeend->m_sleep_ms = m_param_.m_solver_dbg_ogl_interval;
        computeend->m_action =
            std::bind(&ThisType::updateDbgOPGLCanvas, this, true);

        m_solver_.param_actions.pushBack(computeend);
      }
    }

    m_solver_.setGraph(m_graph_);

    // Loop closure detection

    if (m_param_.m_loop_enable)
    {
      m_loop_finder_ = std::make_shared<LoopFinderType>(LoopFinderTypeParam(
          m_param_.m_loop_min_graph_len, m_param_.m_loop_max_dist));
      m_loop_finder_->setSLAM(this);
    }
  }

};  // class SLAM_<VariableSE2DistanceMapRightType_>

}  // namespace slam

}  // namespace inesctec_mrdt_slam_distmap_2d
