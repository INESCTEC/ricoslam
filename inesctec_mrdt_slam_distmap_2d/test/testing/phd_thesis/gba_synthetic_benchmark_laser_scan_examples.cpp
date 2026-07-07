#include <srrg2_core/srrg_geometry/geometry2d.h>
#include <srrg2_core/srrg_geometry/geometry3d.h>
#include <srrg2_core/srrg_viewer/viewer_core/color_palette.h>
#include <srrg2_qgl_viewport/viewer_core_shared_qgl.h>
#include <srrg2_solver/solver_core/factor_graph.h>
#include <srrg2_solver/solver_core/instances.h>
#include <srrg2_solver/solver_core/internals/linear_solvers/instances.h>
#include <srrg2_solver/solver_core/internals/linear_solvers/sparse_block_linear_solver_cholesky_cholmod.h>
#include <srrg2_solver/solver_core/iteration_algorithm_gn.h>
#include <srrg2_solver/solver_core/solver.h>

#include <Eigen/StdVector>
#include <algorithm>
#include <chrono>
#include <filesystem>
#include <functional>
#include <inesctec_mrdt_slam_distmap_2d/geometry/geometry_2d.hpp>
#include <inesctec_mrdt_slam_distmap_2d/pcl/normal_computator.hpp>
#include <inesctec_mrdt_slam_distmap_2d/srrg2_qgl_viewport/viewer_core_shared_qgl_custom.hpp>
#include <inesctec_mrdt_slam_distmap_2d/srrg2_solver/instances.hpp>
#include <inesctec_mrdt_slam_distmap_2d/srrg2_solver/se2_distance_map_pose_pose_bundle_error_factor.hpp>
#include <inesctec_mrdt_slam_distmap_2d/srrg2_solver/se2_distance_map_pose_pose_bundle_plane_error_factor.hpp>
#include <inesctec_mrdt_slam_distmap_2d/srrg2_solver/solver_actions.hpp>
#include <inesctec_mrdt_slam_distmap_2d/srrg2_solver/variable_se2_distance_map.hpp>
#include <inesctec_mrdt_slam_distmap_2d/system_utils/ros_log_offline.hpp>
#include <inesctec_mrdt_slam_distmap_2d/test/laser_scan_examples.hpp>
#include <iostream>
#include <memory>
#include <random>
#include <thread>
#include <vector>

using namespace inesctec_mrdt_slam_distmap_2d;
using namespace slam;

using PointCloudType = pcl::PointNormal2fVectorCloud;
using PointCloudTypePtr = std::shared_ptr<PointCloudType>;

using EstimateType = geometry::Isometry2f;
using EstimateVectorType = geometry::Vector3f;
using EstimateInfoType = geometry::Matrix3f;

using TrajectoryType =
    std::vector<EstimateType, Eigen::aligned_allocator<EstimateType>>;

using FactorGraphType = srrg2_solver::FactorGraph;
using FactorGraphTypePtr = std::shared_ptr<srrg2_solver::FactorGraph>;

// using FactorTypep2p = srrg2_solver::
// SE2DistanceMapStaticPointNormal2fVectorCloudRightPosePoseBundleWithSensorErrorFactorCorrespondenceFreeDriven;
using FactorTypep2p = srrg2_solver::
    SE2DistanceMapNanoStaticPointNormal2fVectorCloudRightPosePoseBundleWithSensorErrorFactorCorrespondenceFreeDriven;

// using FactorTypep2pl = srrg2_solver::
// SE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRightPosePoseBundlePlaneWithSensorErrorFactorCorrespondenceFreeDriven;
using FactorTypep2pl = srrg2_solver::
    SE2DistanceMapNanoStaticPointNormal2fVectorCloudRightPosePoseBundlePlaneWithSensorErrorFactorCorrespondenceFreeDriven;

using NormalsComputatorType =
    pcl::NormalComputatorNearestPoints1D_<PointCloudType>;

using OGLAppType = QApplication;
using OGLAppTypePtr = std::shared_ptr<OGLAppType>;
using OGLViewerType = srrg2_qgl_viewport::ViewerCoreSharedQGLCustom;
using OGLViewerTypePtr = std::shared_ptr<OGLViewerType>;
using OGLCanvasType = srrg2_core::ViewerCanvas;
using OGLCanvasTypePtr = std::shared_ptr<OGLCanvasType>;

using PerturbationNormCriteriaType =
    srrg2_solver::PerturbationNormTerminationCriteria;
using PerturbationNormCriteriaTypePtr =
    std::shared_ptr<PerturbationNormCriteriaType>;

const char kLogFilenameAddInfoP2P[] = "m0_vox-n_noise-0.00m";
const char kLogFilenameAddInfoP2Pl[] = "m0_vox-n_noise-0.00m";

constexpr bool kEnableVoxelization = false;
constexpr float kPointCloudNoiseStd = 0.00f;

constexpr int kGridSize = 8;        //!< Manhattan grid (2*5+1)^2 poses
constexpr float kLoopRange = 1.1f;  //!< all-pairs factor search radius (m)

constexpr float kDistMapRes = .03f;
constexpr float kDistMapSize = -1.f;
constexpr float kDistMapMaxDist = 5.f;

constexpr bool kDebug = true;
constexpr bool kSolverVerbose = true;
constexpr float kSolverDamping = 1.f;
constexpr float kSolverCriteriaEpsilon = 1e-3f;
constexpr int kSolverMaxIterations = 200;

const float kAngRes = geometry::degToRad(0.25f);  //!< angular resolution (rad)
constexpr float kFoV = 2.f * M_PIf32;             //!< full 360° field-of-view
// constexpr float kFoV = (2.f * 3 / 4) * M_PIf32;  //!< full 360° field-of-view
constexpr float kMaxRange = 30.f;  //!< maximum range (m)

constexpr float kSqrSide = 10.f;  //!< half-side of the square room (m)

constexpr float kMaxRngX = 1.5f;
constexpr float kMaxRngY = 1.5f;
const float kMaxRngTh = geometry::degToRad(20.f);

const EstimateType kSensorInRobot = EstimateType::Identity();

constexpr int kNormalComputatorMinPts = 3;
constexpr float kNormalComputatorRadius = .15f;

constexpr unsigned int kRngSeedPose = 5489u;
constexpr unsigned int kRngSeedNoise = 5489u;

struct PoseError
{
  int64_t m_graph_id;

  float m_gt_x;
  float m_gt_y;
  float m_gt_th;

  float m_estimate_x;
  float m_estimate_y;
  float m_estimate_th;

  float m_error_x;
  float m_error_y;
  float m_error_th;

  float m_error_trans;
  float m_error_rot;
};

struct TestResult
{
  float m_rmse_trans;
  float m_rmse_x;
  float m_rmse_y;
  float m_rmse_rot;
  int m_num_poses;
  int m_num_factors;
};

template <typename ViewVariableType, typename ViewFactorType>
void viewGraph(const FactorGraphTypePtr& graph,
               const srrg2_core::ViewerCanvasPtr canvas = nullptr,
               int64_t sleep_ms = 100)
{
  if (!canvas)
  {
    std::cout << "...Debug not enabled..." << std::endl;
    return;
  }

  for (auto v_pair : graph->variables())
  {
    const ViewVariableType* v =
        static_cast<const ViewVariableType*>(v_pair.second);

    v->_drawImpl(canvas);
  }

  for (auto f_pair : graph->factors())
  {
    const ViewFactorType* f = static_cast<const ViewFactorType*>(f_pair.second);

    srrg2_core::Vector3f segments[2];

    const ViewVariableType* v_from =
        static_cast<const ViewVariableType*>(f->variable(0));
    const ViewVariableType* v_to =
        static_cast<const ViewVariableType*>(f->variable(1));

    if (!v_from || !v_to)
    {
      continue;
    }

    const EstimateType& from = v_from->estimate();
    const EstimateType& to = v_to->estimate();

    segments[0] << from.translation().x(), from.translation().y(), 0.f;
    segments[1] << to.translation().x(), to.translation().y(), 0.f;

    canvas->putSegment(2, segments, 0);
  }

  canvas->flush();

  std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
}

void printSummary(std::ostream& os, const std::string& label,
                  const TestResult& res);

void saveCSV(const std::string& path, const std::vector<PoseError>& errors);

void laserScanToPointCloud(PointCloudType& cloud, const msg::LaserScan& scan);

TestResult getTestResult(const std::vector<PoseError>& errors, int num_poses,
                         int num_factors);

/**
 * @brief get pose error
 *
 * it differs from the aligner synthetic benchmark because right now
 * the information matrix does not matter. thus, the error reference coordinate
 * frame does not need to be that target one.
 *
 * error = (gt)^-1 * est
 *
 * where:
 * - translation: R_gt * (t_est - t_gt)
 * - rotation   : (R_gt)^T * R_est = R (th_est - th_gt)
 *
 * in the aligner benchmark, the base frame for the error had to be the target /
 * reference one, given that the information matrix is an estimation w.r.t.
 * the target / reference coordinate frame of the estimate itself.
 *
 * error = est * (gt)^-1
 *
 * where:
 * - translation: t_est - R_est * (R_gt)^T * t_gt
 * - rotation   : R_est * (R_gt)^T = R (th_est - th_gt)
 *
 * chi^2 = (error)^T * info * error
 *
 * even though it may be "polemic", I think that this is the one way to evaluate
 * uncertainty estimation. Still, yeah, probably the error for the registration
 * accuracy analysis could use other equation different than chi^2...
 * but there is no time to change all that...
 *
 * @param[in] graph_id graph ID of the pose node
 * @param[in] gt_t SE(2) ground-truth of the pose w.r.t. world frame
 * @param[in] estimate_t SE(2) estimate from the gba solver
 * @return PoseError
 */
PoseError getPoseError(int64_t graph_id, const EstimateType& gt_t,
                       const EstimateType& estimate_t);

/**
 * @brief create a grid manhattan-like trajectory
 *
 * every pose faces four different orientations: 0°, +90°, -180°, and -90°.
 * depending on the parity of the cells indices (r,c), different orientations
 * are assigned to the poses:
 * - r=even, c=even: 0°
 * - r=even, c=odd : +90°
 * - r=odd , c=even: -180°
 * - r=odd , c=odd : -90°
 *
 * further improvements may be made to this function to really simulate the
 * traversal along a Manhattan-like grid, simulating the change of orientation
 * at each intersection.
 *
 * @param[out] trajectory sequence of Isometry SE(2) poses
 * @param[in] grid_size grid of [ -grid_size , grid_size ] ^ 2
 *                      with (grid_size*2+1)^2 poses
 */
void createGTTrajectoryManhattan(TrajectoryType& trajectory, int grid_size);

template <typename RunFactorType>
void runSolver(const TrajectoryType& trajectory, std::vector<PoseError>& errors,
               int& num_factors,
               const srrg2_core::ViewerCanvasPtr canvas = nullptr)
{
  // using RunFactorType = FactorTypep2p;
  using RunFactorTypePtr = std::shared_ptr<RunFactorType>;
  using RunVariableType = typename RunFactorType::VariableType;
  using RunVariableTypePtr = std::shared_ptr<RunVariableType>;
  using RunInformationMatrixType =
      typename RunFactorType::InformationMatrixType;
  using RunMeasurementType =
      typename RunVariableType::MeasurementOwnerType::MeasurementObjType;
  using RunDistanceMapType =
      typename RunVariableType::MeasurementOwnerType::DistanceMapType;
  using RunLaserScanMsgContainerType =
      typename RunVariableType::MeasurementOwnerType::LaserScanMsgContainerType;

  std::mt19937 rng_pose(kRngSeedPose);  // fixed seed for reproducibility
  std::mt19937 rng_noise(kRngSeedNoise);

  FactorGraphTypePtr graph = std::make_shared<FactorGraphType>();

  NormalsComputatorType normal_computator;

  normal_computator.m_min_pts = kNormalComputatorMinPts;
  normal_computator.m_radius = kNormalComputatorRadius;

  if (canvas)
  {
    canvas->_setup();
  }

  std::cout << "create graph nodes..." << std::endl;

  for (size_t idx = 0; idx < trajectory.size(); idx++)
  {
    if (idx % 10 == 0)
    {
      std::cout << "create graph nodes... ( " << idx << " / "
                << trajectory.size() << " )" << std::endl;
    }

    RunVariableTypePtr v = std::make_shared<RunVariableType>();

    v->setGraphId(idx);
    v->setEstimate(trajectory[idx]);
    v->setMeasurement(std::make_shared<RunMeasurementType>());
    v->setSensorInRobot(kSensorInRobot);

    v->measurement()->m_dist_map = std::make_shared<RunDistanceMapType>();
    v->measurement()->m_pc = std::make_shared<PointCloudType>();
    v->measurement()->m_msgs = std::make_shared<RunLaserScanMsgContainerType>();

    msg::LaserScanPtr scan = std::make_shared<msg::LaserScan>();

    test::getLaserScanMsgSquare(*scan, trajectory[idx], kAngRes, kFoV,
                                kMaxRange, kSqrSide, rng_noise,
                                kPointCloudNoiseStd);

    v->measurement()->m_msgs->push_back(
        std::make_pair(scan, Eigen::Isometry2f::Identity()));

    laserScanToPointCloud(*v->measurement()->m_pc, *scan);

    normal_computator.computeNormals(*v->measurement()->m_pc);

    if (kEnableVoxelization)
    {
      v->measurement()->m_pc->voxelize(kDistMapRes);
    }

    v->measurement()->getDistanceMap()->setGoals(
        *v->measurement()->m_pc, kDistMapRes, kDistMapSize, kDistMapMaxDist);

    graph->addVariable(v);
  }

  graph->variable(0)->setStatus(RunVariableType::Status::Fixed);

  std::cout << "graph ground-truth poses / nodes display..." << std::endl;
  viewGraph<RunVariableType, RunFactorType>(graph, canvas);

  num_factors = 0;

  for (srrg2_core::AbstractMapView_<srrg2_solver::VariableBase::Id,
                                    srrg2_solver::VariableBase*>::iterator
           v1_it = graph->variables().begin();
       v1_it != graph->variables().end(); ++v1_it)
  {
    RunVariableType* const v1 = dynamic_cast<RunVariableType*>(v1_it.value());

    for (srrg2_core::AbstractMapView_<srrg2_solver::VariableBase::Id,
                                      srrg2_solver::VariableBase*>::iterator
             v2_it = v1_it;
         v2_it != graph->variables().end(); ++v2_it)
    {
      RunVariableType* const v2 = dynamic_cast<RunVariableType*>(v2_it.value());

      if (v1->graphId() == v2->graphId())
      {
        continue;
      }

      if ((v1->estimate().translation() - v2->estimate().translation()).norm() >
          kLoopRange)
      {
        continue;
      }

      RunFactorTypePtr f = std::make_shared<RunFactorType>();

      f->setVariableId(0, v1->graphId());
      f->setVariableId(1, v2->graphId());
      f->setInformationMatrix(RunInformationMatrixType::Identity());
      f->setSensorInRobot(kSensorInRobot);

      f->setMoving(*v1->measurement()->getPointCloud());

      graph->addFactor(f);

      num_factors++;
    }
  }

  std::cout << "graph ground-truth poses / nodes + edges display..."
            << std::endl;
  viewGraph<RunVariableType, RunFactorType>(graph, canvas);

  std::cout << "\nPress Enter to continue the test...";
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

  std::uniform_real_distribution<float> dist_x(-kMaxRngX, kMaxRngX);
  std::uniform_real_distribution<float> dist_y(-kMaxRngY, kMaxRngY);
  std::uniform_real_distribution<float> dist_r(-kMaxRngTh, kMaxRngTh);

  for (auto v_pair : graph->variables())
  {
    RunVariableType* v = static_cast<RunVariableType*>(v_pair.second);

    if (v->status() == RunVariableType::Status::Fixed)
    {
      continue;
    }

    EstimateType pert = geometry::v2t(
        Eigen::Vector3f(dist_x(rng_pose), dist_y(rng_pose), dist_r(rng_pose)));
    EstimateType iso = v->estimate();

    v->setEstimate(iso * pert);
  }

  std::cout << "graph initial estimates (...with noise...)..." << std::endl;
  viewGraph<RunVariableType, RunFactorType>(graph, canvas);

  std::cout << "\nPress Enter to continue the test...";
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

  srrg2_solver::Solver solver;
  solver.setGraph(graph);
  solver.param_verbose.setValue(kSolverVerbose);

  PerturbationNormCriteriaTypePtr norm_criteria =
      std::make_shared<PerturbationNormCriteriaType>();

  norm_criteria->param_epsilon.setValue(kSolverCriteriaEpsilon);
  norm_criteria->setName("stepnorm");

  solver.param_termination_criteria.setValue(norm_criteria);
  solver.param_max_iterations.setValue({kSolverMaxIterations});
  solver.param_algorithm.setValue(
      std::make_shared<srrg2_solver::IterationAlgorithmGN>());
  dynamic_cast<srrg2_solver::IterationAlgorithmGN*>(
      solver.param_algorithm.value().get())
      ->param_damping.setValue(1.0f);
  solver.param_linear_solver.setValue(
      std::make_shared<srrg2_solver::SparseBlockLinearSolverCholesky>());

  if (kDebug)
  {
    srrg2_solver::SolverGenericAtIterationEndActionPtr view_graph_dynamic =
        std::make_shared<srrg2_solver::SolverGenericAtIterationEndAction>();

    view_graph_dynamic->m_action = [&graph, &canvas]()
    { viewGraph<RunVariableType, RunFactorType>(graph, canvas); };

    solver.param_actions.pushBack(view_graph_dynamic);
  }

  if (kSolverVerbose)
  {
    solver.param_actions.pushBack(
        std::make_shared<srrg2_solver::SolverVerboseAction>());
  }

  std::cout << "solver compute global bundle adjustment graph "
               "optimization (gba)..."
            << std::endl;
  solver.compute();

  std::cout << "final graph estimate (after gba compute optimization)..."
            << std::endl;
  viewGraph<RunVariableType, RunFactorType>(graph, canvas);

  errors.clear();
  errors.resize(graph->variables().size());

  size_t idx = -1;
  size_t error_idx = 0;

  for (auto v_pair : graph->variables())
  {
    idx++;

    RunVariableType* v = static_cast<RunVariableType*>(v_pair.second);

    if (v->status() == RunVariableType::Status::Fixed)
    {
      continue;
    }

    errors[error_idx++] =
        getPoseError(v->graphId(), trajectory.at(idx), v->estimate());
  }

  errors.resize(error_idx);
}

/**
 * @brief main function that executes the offline global bundle adjustment
 * (gba) benchmark with synthetic data
 *
 * @param[in] argc number of arguments
 * @param[in] argv argument strings
 * @return int
 */
int main(int argc, char** argv)
{
  if (argc < 2)
  {
    std::cerr << "Usage: " << argv[0] << " <output_dir>" << std::endl;
    return 1;
  }

  const std::string output_dir = argv[1];

  std::filesystem::create_directories(output_dir);

  TrajectoryType trajectory_gt;
  createGTTrajectoryManhattan(trajectory_gt, kGridSize);

  const int num_poses = static_cast<int>(trajectory_gt.size());

  std::cout << "=== SE(2) Distance Map Factor Pose-Error Benchmark ===\n"
            << "  scene          : square (half-side " << kSqrSide << " m)\n"
            << "  grid size      : " << kGridSize << " -> " << num_poses
            << " poses\n"
            << "  cloud noise (m): " << kPointCloudNoiseStd << "\n"
            << "  loop range  (m): " << kLoopRange << "\n"
            << "  dmap res    (m): " << kDistMapRes << "\n"
            << "  dmap size   (m): " << kDistMapSize << "\n"
            << "  dmap dmax   (m): " << kDistMapMaxDist << "\n"
            << "  voxelization   : "
            << (kEnableVoxelization ? "enabled" : "not enabled") << "\n"
            << "  angular res (º): " << geometry::radToDeg(kAngRes) << "\n"
            << "  fov         (º): " << geometry::radToDeg(kFoV) << "\n"
            << "  max range   (m): " << kMaxRange << "\n"
            << "  square side (m): " << kSqrSide << "\n"
            << "  perturb x   (m): " << kMaxRngX << "\n"
            << "  perturb y   (m): " << kMaxRngY << "\n"
            << "  perturb th  (º): " << geometry::radToDeg(kMaxRngTh) << "\n\n";

  auto run = [&](const srrg2_core::ViewerCanvasPtr canvas = nullptr)
  {
    // -- p2p test -------------------------------------------------------

    std::cout << "Running p2p test...\n";

    int p2p_num_factors = 0;

    std::vector<PoseError> p2p_errors;

    runSolver<FactorTypep2p>(trajectory_gt, p2p_errors, p2p_num_factors,
                             canvas);

    const TestResult p2p_res = getTestResult(
        p2p_errors, static_cast<int>(trajectory_gt.size()), p2p_num_factors);

    std::cout << "\nPress Enter to continue to p2pl test...";
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    // -- p2pl test ------------------------------------------------------

    std::cout << "Running p2pl test...\n";

    int p2pl_num_factors = 0;

    std::vector<PoseError> p2pl_errors;

    runSolver<FactorTypep2pl>(trajectory_gt, p2pl_errors, p2pl_num_factors,
                              canvas);

    const TestResult p2pl_res = getTestResult(
        p2pl_errors, static_cast<int>(trajectory_gt.size()), p2pl_num_factors);

    // -- Print summary --------------------------------------------------

    std::string dmap_p2p_str =
        std::string("dmap_p2p_") + kLogFilenameAddInfoP2P + "_square";
    std::string dmap_p2pl_str =
        std::string("dmap_p2pl_") + kLogFilenameAddInfoP2Pl + "_square";

    std::cout << "\n=== Results ===\n\n";
    printSummary(std::cout, dmap_p2p_str, p2p_res);
    printSummary(std::cout, dmap_p2pl_str, p2pl_res);

    // -- Save CSV files -------------------------------------------------

    saveCSV(output_dir + "/" + dmap_p2p_str + ".csv", p2p_errors);
    saveCSV(output_dir + "/" + dmap_p2pl_str + ".csv", p2pl_errors);

    // -- Save text summary ---------------------------------------------------
    const std::string summary_path =
        output_dir + "/" + dmap_p2p_str + "_" + dmap_p2pl_str + ".txt";

    std::ofstream sf(summary_path);
    if (sf.is_open())
    {
      sf << "=== SE(2) Distance Map Factor Pose-Error Benchmark ===\n"
         << "  scene          : square (half-side " << kSqrSide << " m)\n"
         << "  grid size      : " << kGridSize << " -> " << num_poses
         << " poses\n"
         << "  cloud noise (m): " << kPointCloudNoiseStd << "\n"
         << "  loop range  (m): " << kLoopRange << "\n"
         << "  dmap res    (m): " << kDistMapRes << "\n"
         << "  dmap size   (m): " << kDistMapSize << "\n"
         << "  dmap dmax   (m): " << kDistMapMaxDist << "\n"
         << "  voxelization   : "
         << (kEnableVoxelization ? "enabled" : "not enabled") << "\n"
         << "  angular res (º): " << geometry::radToDeg(kAngRes) << "\n"
         << "  fov         (º): " << geometry::radToDeg(kFoV) << "\n"
         << "  max range   (m): " << kMaxRange << "\n"
         << "  square side (m): " << kSqrSide << "\n"
         << "  perturb x   (m): " << kMaxRngX << "\n"
         << "  perturb y   (m): " << kMaxRngY << "\n"
         << "  perturb th  (º): " << geometry::radToDeg(kMaxRngTh) << "\n\n";
      printSummary(sf, dmap_p2p_str, p2p_res);
      printSummary(sf, dmap_p2pl_str, p2pl_res);
      sf.close();
    }

    std::cout << "Output written to: " << output_dir << "\n";
  };

  if (!kDebug)
  {
    run();
  }
  else
  {
    std::shared_ptr<QApplication> qapp =
        std::make_shared<QApplication>(argc, argv);
    std::shared_ptr<srrg2_qgl_viewport::ViewerCoreSharedQGLCustom> srrg_viewer =
        std::make_shared<srrg2_qgl_viewport::ViewerCoreSharedQGLCustom>(
            argc, argv, qapp.get(), BUFFER_SIZE_50MEGABYTE, 3, 25, false);

    auto srrg_canvas = srrg_viewer->getCanvas(
        "test_phd_thesis_gba_synthetic_benchmark_laser_cloud_examples");
    srrg_canvas->setName(
        "test_phd_thesis_gba_synthetic_benchmark_laser_cloud_examples");

    std::thread run_thread([&]() { run(srrg_canvas); });

    srrg_viewer->startViewerServer();

    run_thread.join();
  }

  return 0;
}

void createGTTrajectoryManhattan(TrajectoryType& trajectory, int grid_size)
{
  trajectory.clear();

  for (int r = -grid_size; r <= grid_size; r++)
  {
    for (int c = -grid_size; c <= grid_size; c++)
    {
      EstimateType pose;

      pose.setIdentity();
      pose.translation().x() = r;
      pose.translation().y() = c;

      if ((abs(r) % 2 == 0) && (abs(c) % 2 == 0))
        pose.linear() = srrg2_core::geometry2d::a2r<float>(0.f);
      else if ((abs(r) % 2 == 0) && (abs(c) % 2 == 1))
        pose.linear() = srrg2_core::geometry2d::a2r<float>(M_PIf32 * 0.5f);
      else if ((abs(r) % 2 == 1) && (abs(c) % 2 == 0))
        pose.linear() = srrg2_core::geometry2d::a2r<float>(-M_PIf32);
      else
        pose.linear() = srrg2_core::geometry2d::a2r<float>(-M_PIf32 * 0.5f);

      trajectory.emplace_back(pose);
    }
  }
}

PoseError getPoseError(int64_t graph_id, const EstimateType& gt_t,
                       const EstimateType& estimate_t)
{
  PoseError e;

  e.m_graph_id = graph_id;

  const EstimateVectorType gt_v = geometry::t2v(gt_t);
  const EstimateVectorType estimate_v = geometry::t2v(estimate_t);

  e.m_gt_x = gt_v.x();
  e.m_gt_y = gt_v.y();
  e.m_gt_th = gt_v.z();

  e.m_estimate_x = estimate_v.x();
  e.m_estimate_y = estimate_v.y();
  e.m_estimate_th = estimate_v.z();

  EstimateType error_t = gt_t.inverse() * estimate_t;
  EstimateVectorType error_v = geometry::t2v(error_t);

  e.m_error_x = error_v.x();
  e.m_error_y = error_v.y();
  e.m_error_th = error_v.z();

  e.m_error_trans = error_t.translation().norm();
  e.m_error_rot = std::fabs(error_v.z());

  return e;
}

TestResult getTestResult(const std::vector<PoseError>& errors, int num_poses,
                         int num_factors)
{
  const int N = static_cast<int>(errors.size());

  TestResult res;

  res.m_num_poses = num_poses;
  res.m_num_factors = num_factors;

  res.m_rmse_trans = 0;
  res.m_rmse_x = 0;
  res.m_rmse_y = 0;
  res.m_rmse_rot = 0;

  for (const auto& e : errors)
  {
    res.m_rmse_trans += e.m_error_trans * e.m_error_trans;
    res.m_rmse_x += e.m_error_x * e.m_error_x;
    res.m_rmse_y += e.m_error_y * e.m_error_y;
    res.m_rmse_rot += e.m_error_rot * e.m_error_rot;
  }

  res.m_rmse_trans = std::sqrt(res.m_rmse_trans / N);
  res.m_rmse_x = std::sqrt(res.m_rmse_x / N);
  res.m_rmse_y = std::sqrt(res.m_rmse_y / N);
  res.m_rmse_rot = std::sqrt(res.m_rmse_rot / N);

  return res;
}

void saveCSV(const std::string& path, const std::vector<PoseError>& errors)
{
  std::ofstream f(path);

  if (!f.is_open())
  {
    std::cerr << "[saveCSV] ERROR: cannot open " << path << "\n";
    return;
  }

  f << "graph_id,"
    << "gt_x,gt_y,gt_th,"
    << "est_x,est_y,est_th,"
    << "err_x,err_y,err_th,err_trans,err_rot\n";

  f << std::scientific
    << std::setprecision(std::numeric_limits<float>::max_digits10);

  for (const auto& e : errors)
  {
    f << e.m_graph_id << "," << e.m_gt_x << "," << e.m_gt_y << "," << e.m_gt_th
      << "," << e.m_estimate_x << "," << e.m_estimate_y << ","
      << e.m_estimate_th << "," << e.m_error_x << "," << e.m_error_y << ","
      << e.m_error_th << "," << e.m_error_trans << "," << e.m_error_rot << "\n";
  }

  f.close();
}

void printSummary(std::ostream& os, const std::string& label,
                  const TestResult& res)
{
  os << "[square] " << label << ":\n"
     << "  poses (non-fixed)   : " << res.m_num_poses << "\n"
     << "  factors             : " << res.m_num_factors << "\n"
     << "  err trans RMSE   (m): " << res.m_rmse_trans << "\n"
     << "  err x     RMSE   (m): " << res.m_rmse_x << "\n"
     << "  err y     RMSE   (m): " << res.m_rmse_y << "\n"
     << "  err rot   RMSE (deg): " << geometry::radToDeg(res.m_rmse_rot)
     << "\n\n";
}

void laserScanToPointCloud(PointCloudType& cloud, const msg::LaserScan& scan)
{
  cloud.clear();

  for (size_t idx = 0; idx < scan.m_ranges.size(); idx++)
  {
    const float range = scan.m_ranges[idx];

    if ((range < scan.m_range_min) || (range >= scan.m_range_max))
    {
      continue;
    }

    const float angle =
        scan.m_angle_min + static_cast<float>(idx) * scan.m_angle_inc;

    geometry::Vector2f pt;
    scan.angleDistToPt(pt, angle, range);

    pcl::PointNormal2f point;

    scan.angleDistToPt(point.coordinates(), angle, range);

    point.normal().setZero();

    cloud.emplace_back(point);
  }
}
