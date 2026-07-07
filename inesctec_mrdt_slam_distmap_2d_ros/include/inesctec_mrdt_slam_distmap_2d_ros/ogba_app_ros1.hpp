#pragma once

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <inesctec_mrdt_slam_distmap_2d/data_structures/tum_pose.hpp>
#include <inesctec_mrdt_slam_distmap_2d/geometry/geometry_2d.hpp>
#include <inesctec_mrdt_slam_distmap_2d/pcl/normal_computator.hpp>
#include <inesctec_mrdt_slam_distmap_2d/pcl/point.hpp>
#include <inesctec_mrdt_slam_distmap_2d/slam/aligner.hpp>
#include <inesctec_mrdt_slam_distmap_2d/slam/aligner_distance_map_icp_point_plane.hpp>
#include <inesctec_mrdt_slam_distmap_2d/slam/aligner_distance_map_icp_point_point.hpp>
#include <inesctec_mrdt_slam_distmap_2d/slam/aligner_distance_map_point_plane.hpp>
#include <inesctec_mrdt_slam_distmap_2d/slam/aligner_distance_map_point_point.hpp>
#include <inesctec_mrdt_slam_distmap_2d/slam/merger.hpp>
#include <inesctec_mrdt_slam_distmap_2d/slam/msg/laser_scan.hpp>
#include <inesctec_mrdt_slam_distmap_2d/slam/triggering_criteria.hpp>
#include <inesctec_mrdt_slam_distmap_2d/srrg2_solver/se2_distance_map_pose_pose_bundle_error_factor.hpp>
#include <inesctec_mrdt_slam_distmap_2d/srrg2_solver/se2_distance_map_pose_pose_bundle_plane_error_factor.hpp>
#include <inesctec_mrdt_slam_distmap_2d/srrg2_solver/variable_se2_distance_map.hpp>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace inesctec_mrdt_slam_distmap_2d_ros
{

class OgbaAppROS1
{
 public:

  using ROSBagType = rosbag::Bag;
  using ROSBagTypePtr = std::shared_ptr<ROSBagType>;
  using ROSViewType = rosbag::View;
  using ROSViewTypePtr = std::shared_ptr<ROSViewType>;

  using TUMPoseType = inesctec_mrdt_slam_distmap_2d::data_structures::TUMPose;

  using LaserScanMsgType = inesctec_mrdt_slam_distmap_2d::slam::msg::LaserScan;
  using LaserScanMsgTypePtr =
      inesctec_mrdt_slam_distmap_2d::slam::msg::LaserScanPtr;

  using SolverType = srrg2_solver::Solver;
  using FactorGraphType = srrg2_solver::FactorGraph;
  using FactorGraphTypePtr = srrg2_solver::FactorGraphPtr;

  /**
   * @note put here the factor type to test offline global bundle adjustment
   *
   * - point2point? standard point2fvectorcloud or pointnormal2fvectorcloud
   * - point2plane? only pointnormal2fvectorcloud factors
   */

  using FactorType = srrg2_solver::
      SE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRightPosePoseBundlePlaneWithSensorErrorFactorCorrespondenceFreeDriven;
  using FactorTypePtr = std::shared_ptr<FactorType>;

  using VariableType = FactorType::VariableType;
  using VariableTypePtr = std::shared_ptr<VariableType>;
  using VariableBaseType = VariableType::BaseVariableType;

  using EstimateType = VariableType::EstimateType;
  using Scalar = EstimateType::Scalar;
  using EstimateVectorType =
      inesctec_mrdt_slam_distmap_2d::geometry::Vector3_<Scalar>;

  using MeasurementType =
      VariableType::MeasurementOwnerType::MeasurementObjType;
  using DistanceMapType = VariableType::MeasurementOwnerType::DistanceMapType;
  using DistanceMapTypePtr =
      VariableType::MeasurementOwnerType::DistanceMapTypePtr;
  using PointCloudType = VariableType::MeasurementOwnerType::PointCloudType;
  using PointCloudTypePtr =
      VariableType::MeasurementOwnerType::PointCloudTypePtr;
  using PointType = PointCloudType::PointType;

  using CriteriaType =
      inesctec_mrdt_slam_distmap_2d::slam::TriggeringCriteria_<EstimateType>;
  using CriteriaEmptyType =
      inesctec_mrdt_slam_distmap_2d::slam::TriggeringCriteriaEmpty_<
          EstimateType>;
  using CriteriaRotationType =
      inesctec_mrdt_slam_distmap_2d::slam::TriggeringCriteriaRotation_<
          EstimateType>;
  using CriteriaTranslationType =
      inesctec_mrdt_slam_distmap_2d::slam::TriggeringCriteriaTranslation_<
          EstimateType>;
  using CriteriaViewpointType =
      inesctec_mrdt_slam_distmap_2d::slam::TriggeringCriteriaViewpoint_<
          EstimateType>;
  using CriteriaGridCellType =
      inesctec_mrdt_slam_distmap_2d::slam::TriggeringCriteriaGridCell_<
          EstimateType>;
  using CriteriaInliersRatioType =
      inesctec_mrdt_slam_distmap_2d::slam::TriggeringCriteriaInliersRatio_<
          EstimateType>;
  using CriteriaTimeType =
      inesctec_mrdt_slam_distmap_2d::slam::TriggeringCriteriaTime_<
          EstimateType>;

  using CriteriaTypePtr = std::shared_ptr<CriteriaType>;
  using CriteriaEmptyTypePtr = std::shared_ptr<CriteriaEmptyType>;
  using CriteriaRotationTypePtr = std::shared_ptr<CriteriaRotationType>;
  using CriteriaTranslationTypePtr = std::shared_ptr<CriteriaTranslationType>;
  using CriteriaViewpointTypePtr = std::shared_ptr<CriteriaViewpointType>;
  using CriteriaGridCellTypePtr = std::shared_ptr<CriteriaGridCellType>;
  using CriteriaInliersRatioTypePtr = std::shared_ptr<CriteriaInliersRatioType>;
  using CriteriaTimeTypePtr = std::shared_ptr<CriteriaTimeType>;

  using GlobalMapAlignerType =
      inesctec_mrdt_slam_distmap_2d::slam::AlignerDistanceMapPointPoint_<
          srrg2_solver::
              VariableSE2DistanceMapNanoStaticPointNormal2fVectorCloudRight>;
  using GlobalMapAlignerTypePtr = std::shared_ptr<GlobalMapAlignerType>;

  using CovisibilityAlignerType =
      inesctec_mrdt_slam_distmap_2d::slam::AlignerDistanceMapPointPoint_<
          VariableType>;
  using CovisibilityAlignerTypePtr = std::shared_ptr<CovisibilityAlignerType>;

  using MergerType = inesctec_mrdt_slam_distmap_2d::slam::Merger_<
      EstimateType, DistanceMapType, PointCloudType>;
  using MergerTypePtr = std::shared_ptr<MergerType>;

  using OGLAppType = QApplication;
  using OGLAppTypePtr = std::shared_ptr<OGLAppType>;
  using OGLViewerType = srrg2_qgl_viewport::ViewerCoreSharedQGLCustom;
  using OGLViewerTypePtr = std::shared_ptr<OGLViewerType>;
  using OGLCanvasType = srrg2_core::ViewerCanvas;
  using OGLCanvasTypePtr = std::shared_ptr<OGLCanvasType>;

 public:

  /**
   * @brief parametrization struct
   */
  struct Param
  {
    std::vector<std::string> m_filenames_bags;  //!< ROS 1 bags to process
    std::string m_filename_log;                 //!< log filename (TUM format)
    std::string m_scan_topic;                   //!< 2D laser data topic name
    std::vector<double> m_sensor_in_robot_val;  //!< sensor w.r.t. robot
    bool m_has_duration;     //!< if duration from start time set in opts
    double m_time_start;     //!< start time (s) into the bag files (0..)
    double m_time_duration;  //!< duration (s) to only process (0..)

    std::string m_ogba_yaml;  //!< YAML file to configure OGBA

    bool m_laser_invert_scan;  //!< invert the laser scan data (when sensor
                               //!< upside down)

    bool m_enable_voxelization;  //!< enable point cloud voxelization
    float m_voxel_scale;         //!< voxel scale size (m) (-1 for distance map
                                 //!< resolution as default)
    float m_range_min;  //!< laser ray minimum range (m) (-1 for msg.range_min
                        //!< as default)
    float m_range_max;  //!< laser ray maximum range (m) (-1 for msg.range_max
                        //!< as default)

    bool m_enable_normals;             //!< for point-to-plane error computation
    int m_normals_computator_min_pts;  //!< minimum pts of nearest points
                                       //!< neighborhood to compute normals
    float m_normals_computator_radius;  //!< nearest neighborhood radius (m)

    float
        m_dist_map_res;  //!< distance map resolution of the distance map (m/px)
    float m_dist_map_size;  //!< distance map fixate the size (square width /
                            //!< 2) of the distance map (m) (-1 if you want to
                            //!< assume the maximum range in the point cloud
                            //!< used to setGoals)
    float m_dist_map_max_dist;  //!< distance map maximum distance considered
                                //!< to compute the distance map (m) (-1 if you
                                //!< want to compute the whole distance map)

    // std::string m_ogba_type;  //!< aligner type (point2point|point2plane)
    bool m_ogba_debug;  //!< enable debug mode (OpenGL-based visualization)
    bool m_ogba_solver_verbose;  //!< enable verbose mode of the solver (to show
                                 //!< the iteration stats)
    float m_ogba_solver_damping;  //!< damping parameter for the iteration
                                  //!< algorithm when computing the new state
    int m_ogba_solver_max_iterations;  //!< maximum number of iterations allowed
                                       //!< for the solver
    std::string
        m_ogba_solver_criteria_type;       //!< type of termination criteria
                                           //!< (simple, gradient, stepnorm)
                                           //!< (empty to force max iterations)
    float m_ogba_solver_criteria_epsilon;  //!< epsilon threshold for the
                                           //!< solver's termination criteria
    std::string m_ogba_solver_robustifier_type;  //!< type of robustifier
                                                 //!< (saturated, cauchy, clamp,
                                                 //!< huber) (empty not used)
    float m_ogba_solver_robustifier_threshold;   //!< robustifier threshold to
                                                 //!< apply to the error factors
    int64_t
        m_ogba_solver_dbg_ogl_interval;  //!< sleep interval between consecutive
                                         //!< visualizations (ms)
    std::string m_ogba_solver_dbg_ogl_mode;  //!< type of debug visualization
                                             //!< mode (iterationend|computeend)

    CriteriaTypePtr m_graph_criteria = nullptr;

    float m_covisibility_ratio_inliers;
    float m_covisibility_nn_distance;
    float m_covisibility_max_distance;
    int64_t m_covisibility_dbg_ogl_interval;

    MergerTypePtr m_global_merger = nullptr;

    Param()
        : m_filenames_bags({}),
          m_filename_log(""),
          m_scan_topic(""),
          m_sensor_in_robot_val({}),
          m_has_duration(false),
          m_time_start(-1.0),
          m_time_duration(-1.0),
          m_ogba_yaml(""),
          m_laser_invert_scan(false),
          m_enable_voxelization(false),
          m_voxel_scale(-1.f),
          m_range_min(-1.f),
          m_range_max(-1.f),
          m_enable_normals(false),
          m_normals_computator_min_pts(-1),
          m_normals_computator_radius(-1.f),
          m_dist_map_res(0.01f),
          m_dist_map_size(-1.f),
          m_dist_map_max_dist(0.25f),
          m_ogba_debug(true),
          m_ogba_solver_verbose(true),
          m_ogba_solver_damping(1.f),
          m_ogba_solver_max_iterations(100),
          m_ogba_solver_criteria_type("stepnorm"),
          m_ogba_solver_criteria_epsilon(1e-5),
          m_ogba_solver_robustifier_type("cauchy"),
          m_ogba_solver_robustifier_threshold(0.01f),
          m_ogba_solver_dbg_ogl_interval(0),
          m_ogba_solver_dbg_ogl_mode("iterationend"),
          m_graph_criteria(nullptr),
          m_covisibility_ratio_inliers(0.60f),
          m_covisibility_nn_distance(0.25f),
          m_covisibility_max_distance(5.f),
          m_covisibility_dbg_ogl_interval(0),
          m_global_merger(nullptr)

    {
    }
  };

  /**
   * @brief laser scan message associated with the sensor itself w.r.t. robot
   *        SE2 transformation and an initial pose for the robot base
   */
  struct LaserScanMsgWithInitialPoseTf
  {
   public:

    LaserScanMsgTypePtr m_scan = nullptr;  //!< laser scan message
    EstimateType m_sensor_in_robot =
        EstimateType::Identity();  //!< sensor w.r.t. robot frame
    EstimateType m_initial_pose = EstimateType::Identity();  //!< SE2 pose

    LaserScanMsgWithInitialPoseTf() = default;
    LaserScanMsgWithInitialPoseTf(const LaserScanMsgTypePtr& scan,
                                  const EstimateType& sensor_in_robot,
                                  const EstimateType& initial_pose)
        : m_scan(scan),
          m_sensor_in_robot(sensor_in_robot),
          m_initial_pose(initial_pose)
    {
    }
  };

 protected:

  Param m_param_;  //!< parametrization

  std::vector<ROSBagTypePtr> m_bags_;  //!< ROS 1 bags

  ROSViewTypePtr m_bags_view_full_ = nullptr;  //!< ROS bag view (all data)
  ROSViewTypePtr m_bags_view_ = nullptr;       //!< ROS bag view (only scan)

  double m_bags_t_initial_ = -1.;  //!< initial start time (s) from ROS bags
  double m_bags_t_finish_ = -1.;   //!< finish start time (s) from ROS bags

  std::vector<TUMPoseType> m_log_data_;  //!< TUM data (robot poses)

  EstimateVectorType m_sensor_in_robot_v_ = EstimateVectorType::Zero();
  EstimateType m_sensor_in_robot_t_ = EstimateType::Identity();

  srrg2_solver::Solver m_solver_;
  FactorGraphTypePtr m_graph_ = nullptr;

  PointCloudTypePtr m_global_map_ = nullptr;
  GlobalMapAlignerTypePtr m_global_aligner_ = nullptr;

  OGLAppTypePtr m_dbg_ogl_qapp_;       //!< Qt application required for the SRRG
                                       //!< ViewerCoreSharedQGL viz
  OGLViewerTypePtr m_dbg_ogl_viewer_;  //!< ViewerCoreSharedQGL handler
  OGLCanvasTypePtr
      m_dbg_ogl_canvas_;  //!< viz canvas to show the tracking results
  std::string m_dbg_ogl_canvas_name_;  //!< viz canvas title name (to be used in
                                       //!< the ViewerCoreSharedQGL)

 public:

  OgbaAppROS1(const Param& param = Param());
  ~OgbaAppROS1();

  void clear();
  void init();
  void print();

  void getPoses();
  void computeCovisibilityGraph();
  void compute();

  void setupDbgOGL(const OGLAppTypePtr& qapp = nullptr,
                   const OGLViewerTypePtr& viewer = nullptr,
                   const OGLCanvasTypePtr& canvas = nullptr);
  void updateDbgOPGLCanvas();

 public:

  inline void setParam(const Param& param)
  {
    assertInit(param);
    m_param_ = param;
    init();
    print();
  }
  inline const Param& getParam() const { return m_param_; }
  inline const std::vector<TUMPoseType>& getTUMData() const
  {
    return m_log_data_;
  }
  inline const EstimateType& getSensorInRobot() const
  {
    return m_sensor_in_robot_t_;
  }

 protected:

  void loadOGBAConfig();

  template <typename OtherPointCloudType>
  void preprocessScan(LaserScanMsgType& scan, OtherPointCloudType& pc)
  {
    if (m_param_.m_laser_invert_scan)
    {
      scan.invert();
    }

    pc.resize(scan.m_ranges.size());

    float range_min =
        m_param_.m_range_min < 0 ? scan.m_range_min : m_param_.m_range_min;
    float range_max =
        m_param_.m_range_max > 0 ? m_param_.m_range_max : scan.m_range_max;

    for (size_t idx = 0; idx < scan.m_ranges.size(); idx++)
    {
      float angle =
          scan.m_angle_min + static_cast<float>(idx) * scan.m_angle_inc;

      scan.angleDistToPt(pc[idx].coordinates(), angle, scan.m_ranges[idx]);

      if ((scan.m_ranges[idx] < range_min) || (scan.m_ranges[idx] > range_max))
      {
        pc[idx].m_status =
            inesctec_mrdt_slam_distmap_2d::pcl::PointStatus::kOutOfRange;
      }
      else
      {
        pc[idx].m_status =
            inesctec_mrdt_slam_distmap_2d::pcl::PointStatus::kValid;
      }
    }

    if (m_param_.m_enable_normals)
    {
      inesctec_mrdt_slam_distmap_2d::pcl::NormalComputatorNearestPoints1D_<
          OtherPointCloudType>
          normals_computator;

      normals_computator.m_min_pts = m_param_.m_normals_computator_min_pts;
      normals_computator.m_radius = m_param_.m_normals_computator_radius;

      normals_computator.computeNormals(pc);
    }

    if (m_param_.m_enable_voxelization)
    {
      pc.voxelize(m_param_.m_voxel_scale);
    }
  }

  template <typename OtherVariableType>
  OtherVariableType* addVariable(int64_t graph_id, VariableType::Status status,
                                 const EstimateType& initial_pose,
                                 const EstimateType& sensor_in_robot,
                                 const LaserScanMsgTypePtr& scan,
                                 const PointCloudTypePtr& pc)
  {
    std::shared_ptr<OtherVariableType> v =
        std::make_shared<OtherVariableType>();

    v->setGraphId(graph_id++);
    v->setStatus(status);
    v->setEstimate(initial_pose);
    v->setSensorInRobot(sensor_in_robot);

    v->setMeasurement(
        std::make_shared<typename OtherVariableType::MeasurementOwnerType::
                             MeasurementObjType>());

    v->measurement()->m_msgs->push_back(
        std::make_pair(scan, EstimateType::Identity()));
    v->measurement()->m_pc = pc;
    v->measurement()->m_dist_map->setGoals(
        *v->measurement()->m_pc, m_param_.m_dist_map_res,
        m_param_.m_dist_map_size, m_param_.m_dist_map_max_dist);

    m_graph_->addVariable(v);

    return v.get();
  }

 private:

  static void assertInit(const Param& param);
};

}  // namespace inesctec_mrdt_slam_distmap_2d_ros
