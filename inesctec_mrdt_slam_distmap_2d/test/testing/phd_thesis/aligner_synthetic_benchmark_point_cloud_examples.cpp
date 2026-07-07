#include <srrg2_core/srrg_viewer/viewer_canvas.h>

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <chrono>
#include <filesystem>
#include <functional>
#include <inesctec_mrdt_slam_distmap_2d/geometry/geometry_2d.hpp>
#include <inesctec_mrdt_slam_distmap_2d/pcl/normal_computator.hpp>
#include <inesctec_mrdt_slam_distmap_2d/slam/aligner_distance_map_icp_point_plane.hpp>
#include <inesctec_mrdt_slam_distmap_2d/slam/aligner_distance_map_icp_point_point.hpp>
#include <inesctec_mrdt_slam_distmap_2d/slam/aligner_distance_map_point_plane.hpp>
#include <inesctec_mrdt_slam_distmap_2d/slam/aligner_distance_map_point_point.hpp>
#include <inesctec_mrdt_slam_distmap_2d/srrg2_qgl_viewport/viewer_core_shared_qgl_custom.hpp>
#include <inesctec_mrdt_slam_distmap_2d/system_utils/ros_log_offline.hpp>
#include <inesctec_mrdt_slam_distmap_2d/test/point_cloud_examples.hpp>
#include <iostream>
#include <limits>
#include <random>
#include <thread>

using namespace inesctec_mrdt_slam_distmap_2d;

using PointCloudType = pcl::PointNormal2fVectorCloud;
using PointCloudTypePtr = std::shared_ptr<PointCloudType>;

using VariableTypeICPp2p =
    srrg2_solver::VariableSE2DistanceMapNanoStaticPointNormal2fVectorCloudRight;
using VariableTypeICPp2pl =
    srrg2_solver::VariableSE2DistanceMapNanoStaticPointNormal2fVectorCloudRight;
using VariableTypeDistMapp2p =
    srrg2_solver::VariableSE2DistanceMapStaticPointNormal2fVectorCloudRight;
using VariableTypeDistMapp2pl = srrg2_solver::
    VariableSE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRight;

using EstimateType = geometry::Isometry2f;
using EstimateVectorType = geometry::Vector3f;
using EstimateInfoType = geometry::Matrix3f;

using AlignerTypeICPp2p =
    slam::AlignerDistanceMapICPPointPoint_<VariableTypeICPp2p>;
using AlignerTypeICPp2pl =
    slam::AlignerDistanceMapICPPointPlane_<VariableTypeICPp2pl>;
using AlignerTypeDistMapp2p =
    slam::AlignerDistanceMapPointPoint_<VariableTypeDistMapp2p>;
using AlignerTypeDistMapp2pl =
    slam::AlignerDistanceMapPointPlane_<VariableTypeDistMapp2pl>;

using AlignerTypeBaseParamICPp2p = AlignerTypeICPp2p::BaseType::Param;
using AlignerTypeBaseParamICPp2pl = AlignerTypeICPp2pl::BaseType::Param;
using AlignerTypeBaseParamDistMapp2p = AlignerTypeDistMapp2p::BaseType::Param;
using AlignerTypeBaseParamDistMapp2pl = AlignerTypeDistMapp2pl::BaseType::Param;

using AlignerTypeParamICPp2p = AlignerTypeICPp2p::DistMapParam;
using AlignerTypeParamICPp2pl = AlignerTypeICPp2pl::DistMapParam;
using AlignerTypeParamDistMapp2p = AlignerTypeDistMapp2p::DistMapParam;
using AlignerTypeParamDistMapp2pl = AlignerTypeDistMapp2pl::DistMapParam;

using OGLAppType = AlignerTypeDistMapp2p::OGLAppType;
using OGLAppTypePtr = AlignerTypeDistMapp2p::OGLAppTypePtr;
using OGLViewerType = AlignerTypeDistMapp2p::OGLViewerType;
using OGLViewerTypePtr = AlignerTypeDistMapp2p::OGLViewerTypePtr;
using OGLCanvasType = AlignerTypeDistMapp2p::OGLCanvasType;
using OGLCanvasTypePtr = AlignerTypeDistMapp2p::OGLCanvasTypePtr;

using NormalsComputatorType =
    pcl::NormalComputatorNearestPoints1D_<PointCloudType>;

constexpr bool kModeManual = false;
constexpr bool kEnableVoxelization = false;

constexpr bool kParamDebug = false;
constexpr bool kParamSolverVerbose = false;
constexpr float kParamSolverDamping = 1.f;
constexpr int kParamSolverMaxIterations = 50;
constexpr char kParamSolverCriteriaType[] = "stepnorm";
constexpr float kParamSolverCriteriaEpsilon = 1e-5;
constexpr char kParamSolverRobustifierType[] = "";
constexpr int kParamSolverRobustifierNumIterationsCoarse = -1;
constexpr float kParamSolverRobustifierThresholdCoarse = -1.f;
constexpr float kParamSolverRobustifierThresholdFine = -1.f;
constexpr float kParamFailureRatioInliers = -1.f;
constexpr float kParamFailureChiInliers = -1.f;
constexpr int64_t kParamDbgOglInterval = 100;
constexpr char kParamDbgOglMode[] = "iterationend";
constexpr bool kParamDbgOglShowCorrespondences = true;
constexpr bool kParamDbgOglShowNormals = false;

constexpr float kDistMapParamDistMapRes = .03f;
constexpr float kDistMapParamDistMapSize = -1.f;
constexpr float kDistMapParamDistMapMaxDist = 5.f;
constexpr bool kDistMapParamSolverRegularizeCorrespondences = false;

constexpr int kParamNormalComputatorMinPts = 3;
constexpr float kParamNormalComputatorRadius = .15f;

struct BenchmarkConfig
{
  // Number of trials for the benchmark
  int m_num_trials = 100;   //!< number of trials (>= 100, to have top 5% & 1%)
  int m_cout_feedback = 1;  //!< feedback per iterations (m_num_trials >...> 0)

  // Parametrization of the random number generator limits for its uniform dist
  float m_circ_max_rng_x = 1.5f;  //!< max x rng (m) (circle)
  float m_circ_max_rng_y = 1.5f;  //!< max y rng (m) (circle)
  float m_circ_max_rng_th =
      geometry::degToRad(20.f);  //!< max th rng (rad) (circle)

  float m_sqr_max_rng_x = 1.5f;  //!< max x rng (m) (square)
  float m_sqr_max_rng_y = 1.5f;  //!< max y rng (m) (square)
  float m_sqr_max_rng_th =
      geometry::degToRad(20.f);  //!< max th rng (rad) (square)

  float m_corr_max_rng_x = 1.5f;  //!< max x rng (m) (corridor)
  float m_corr_max_rng_y = 0.5f;  //!< max y rng (m) (corridor)
  float m_corr_max_rng_th =
      geometry::degToRad(10.f);  //!< max th rng (rad) (corridor)

  float m_rand_max_rng_x = 0.15f;  //!< max x rng (m) (random)
  float m_rand_max_rng_y = 0.15f;  //!< max y rng (m) (random)
  float m_rand_max_rng_th =
      geometry::degToRad(5.f);  //!< max th rng (rad) (random)

  // Parametrization of the scenes (point cloud generation)
  float m_ang_res = geometry::degToRad(0.25f);
  float m_fov = 2 * M_PIf32;
  float m_max_range = 10.f;

  float m_rand_max_range = 10.f;
  float m_circ_radius = 8.f;
  float m_sqr_side = 10.f;
  int m_sqr_num_pts_side = 400;
  float m_corr_width = 5.f;
  float m_corr_length = 10.f;
  int m_corr_num_pts_side = 400;
  float m_noise_std = 0.03f;
};

struct Scene
{
  std::string m_name;
  std::function<void(PointCloudType&, PointCloudType&)> m_generate;
};

struct TrialResults
{
  int m_trial;

  float m_gt_x;
  float m_gt_y;
  float m_gt_th;

  float m_estimate_x;
  float m_estimate_y;
  float m_estimate_th;

  EstimateInfoType m_estimate_info;

  float m_error_x;
  float m_error_y;
  float m_error_th;

  float m_error_trans;
  float m_error_rot;

  double m_time_ms;
};

struct SceneResults
{
  float m_error_trans;
  float m_error_x;
  float m_error_y;
  float m_error_rot;
  float m_chi_sqr;
  float m_chi_sqr_mean95;
  double m_time_mean_ms;
  double m_time_std_ms;
  double m_time_min_ms;
  double m_time_max_ms;
};

struct AlignerSceneResults
{
  std::string m_aligner_type = "";
  std::string m_scene_name = "";
  std::vector<TrialResults> m_results = {};
  SceneResults m_res = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f};

  AlignerSceneResults(const std::string& aligner_type,
                      const std::string& scene_name)
      : m_aligner_type(aligner_type), m_scene_name(scene_name)
  {
  }
};

EstimateType getRandomPerturbation(float max_x, float max_y, float max_th,
                                   std::mt19937& rng);
TrialResults getTrialResults(int trial, const EstimateType& gt_t,
                             const EstimateType& estimate_t,
                             const EstimateInfoType& info);

SceneResults getSceneResults(const std::vector<TrialResults>& results);

void saveTrialResultsCSV(const std::vector<AlignerSceneResults>& results,
                         const std::string& output_dir);

void run(AlignerTypeICPp2p& icp_p2p, AlignerTypeICPp2pl& icp_p2pl,
         AlignerTypeDistMapp2p& dmap_p2p, AlignerTypeDistMapp2pl& dmap_p2pl,
         const BenchmarkConfig& cfg,
         const NormalsComputatorType& normal_computator, std::mt19937& rng_pose,
         std::mt19937& rng_noise, const std::string& output_dir);

template <typename AlignerType>
inline void runAligner(AlignerType& aligner, int idx,
                       const PointCloudTypePtr& fixed,
                       const PointCloudTypePtr& moving,
                       const EstimateType& gt_t, TrialResults& result)
{
  aligner.setSource(moving, EstimateType::Identity());
  aligner.setSourceInTarget(EstimateType::Identity());
  aligner.setTarget(fixed, EstimateType::Identity());
  aligner.setPriorEnabled(false);

  const auto t0 = std::chrono::high_resolution_clock::now();
  aligner.compute();
  const auto t1 = std::chrono::high_resolution_clock::now();

  EstimateType estimate_t = aligner.getSourceInTarget();
  EstimateInfoType estimate_info = aligner.getH();

  result = getTrialResults(idx, gt_t, estimate_t, estimate_info);
  result.m_time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
}

int main(int argc, char** argv)
{
  if (argc < 2)
  {
    std::cerr << "Usage: " << argv[0] << " <output_dir>" << std::endl;
    return 1;
  }

  const std::string output_dir = argv[1];

  BenchmarkConfig cfg;

  AlignerTypeBaseParamICPp2p icp_p2p_param(
      kParamDebug, kParamSolverVerbose, kParamSolverDamping,
      kParamSolverMaxIterations, kParamSolverCriteriaType,
      kParamSolverCriteriaEpsilon, kParamSolverRobustifierType,
      kParamSolverRobustifierNumIterationsCoarse,
      kParamSolverRobustifierThresholdCoarse,
      kParamSolverRobustifierThresholdFine, kParamFailureRatioInliers,
      kParamFailureChiInliers, kParamDbgOglInterval, kParamDbgOglMode,
      kParamDbgOglShowCorrespondences, kParamDbgOglShowNormals);
  AlignerTypeBaseParamICPp2pl icp_p2pl_param(
      kParamDebug, kParamSolverVerbose, kParamSolverDamping,
      kParamSolverMaxIterations, kParamSolverCriteriaType,
      kParamSolverCriteriaEpsilon, kParamSolverRobustifierType,
      kParamSolverRobustifierNumIterationsCoarse,
      kParamSolverRobustifierThresholdCoarse,
      kParamSolverRobustifierThresholdFine, kParamFailureRatioInliers,
      kParamFailureChiInliers, kParamDbgOglInterval, kParamDbgOglMode,
      kParamDbgOglShowCorrespondences, kParamDbgOglShowNormals);
  AlignerTypeBaseParamDistMapp2p dmap_p2p_param(
      kParamDebug, kParamSolverVerbose, kParamSolverDamping,
      kParamSolverMaxIterations, kParamSolverCriteriaType,
      kParamSolverCriteriaEpsilon, kParamSolverRobustifierType,
      kParamSolverRobustifierNumIterationsCoarse,
      kParamSolverRobustifierThresholdCoarse,
      kParamSolverRobustifierThresholdFine, kParamFailureRatioInliers,
      kParamFailureChiInliers, kParamDbgOglInterval, kParamDbgOglMode,
      kParamDbgOglShowCorrespondences, kParamDbgOglShowNormals);
  AlignerTypeBaseParamDistMapp2pl dmap_p2pl_param(
      kParamDebug, kParamSolverVerbose, kParamSolverDamping,
      kParamSolverMaxIterations, kParamSolverCriteriaType,
      kParamSolverCriteriaEpsilon, kParamSolverRobustifierType,
      kParamSolverRobustifierNumIterationsCoarse,
      kParamSolverRobustifierThresholdCoarse,
      kParamSolverRobustifierThresholdFine, kParamFailureRatioInliers,
      kParamFailureChiInliers, kParamDbgOglInterval, kParamDbgOglMode,
      kParamDbgOglShowCorrespondences, kParamDbgOglShowNormals);

  AlignerTypeParamICPp2p icp_p2p_param_dmaps(
      kDistMapParamDistMapRes, kDistMapParamDistMapSize,
      kDistMapParamDistMapMaxDist,
      kDistMapParamSolverRegularizeCorrespondences);
  AlignerTypeParamICPp2pl icp_p2pl_param_dmaps(
      kDistMapParamDistMapRes, kDistMapParamDistMapSize,
      kDistMapParamDistMapMaxDist,
      kDistMapParamSolverRegularizeCorrespondences);
  AlignerTypeParamDistMapp2p dmap_p2p_param_dmaps(
      kDistMapParamDistMapRes, kDistMapParamDistMapSize,
      kDistMapParamDistMapMaxDist,
      kDistMapParamSolverRegularizeCorrespondences);
  AlignerTypeParamDistMapp2pl dmap_p2pl_param_dmaps(
      kDistMapParamDistMapRes, kDistMapParamDistMapSize,
      kDistMapParamDistMapMaxDist,
      kDistMapParamSolverRegularizeCorrespondences);

  AlignerTypeICPp2p icp_p2p(icp_p2p_param, icp_p2p_param_dmaps);
  AlignerTypeICPp2pl icp_p2pl(icp_p2pl_param, icp_p2pl_param_dmaps);
  AlignerTypeDistMapp2p dmap_p2p(dmap_p2p_param, dmap_p2p_param_dmaps);
  AlignerTypeDistMapp2pl dmap_p2pl(dmap_p2pl_param, dmap_p2pl_param_dmaps);

  NormalsComputatorType normal_computator;

  normal_computator.m_min_pts = kParamNormalComputatorMinPts;
  normal_computator.m_radius = kParamNormalComputatorRadius;

  std::mt19937 rng_pose(5489u);  // fixed seed for reproducibility
  std::mt19937 rng_noise(5489u);

  if (!kParamDebug)
  {
    run(icp_p2p, icp_p2pl, dmap_p2p, dmap_p2pl, cfg, normal_computator,
        rng_pose, rng_noise, output_dir);
  }
  else
  {
    std::shared_ptr<QApplication> qapp =
        std::make_shared<QApplication>(argc, argv);
    std::shared_ptr<OGLViewerType> srrg_viewer =
        std::make_shared<OGLViewerType>(argc, argv, qapp.get(),
                                        BUFFER_SIZE_50MEGABYTE, 3, 25, false);
    OGLCanvasTypePtr canvas_icp_p2p = srrg_viewer->getCanvas("icp_p2p");
    OGLCanvasTypePtr canvas_icp_p2pl = srrg_viewer->getCanvas("icp_p2pl");
    OGLCanvasTypePtr canvas_dmap_p2p = srrg_viewer->getCanvas("dmap_p2p");
    OGLCanvasTypePtr canvas_dmap_p2pl = srrg_viewer->getCanvas("dmap_p2pl");

    canvas_icp_p2p->setName("icp_p2p");
    canvas_icp_p2pl->setName("icp_p2pl");
    canvas_dmap_p2p->setName("dmap_p2p");
    canvas_dmap_p2pl->setName("dmap_p2pl");

    std::thread benchmark_thread(
        [&]()
        {
          std::cout << "just sleeping for a little second... "
                       "debug must get ready!"
                    << std::endl;
          std::this_thread::sleep_for(std::chrono::seconds(1));
          std::cout << "!!! LET'S GOOOOOOOO !!!" << std::endl << std::endl;

          icp_p2p.setupDbgOGL(qapp, srrg_viewer, canvas_icp_p2p);
          icp_p2pl.setupDbgOGL(qapp, srrg_viewer, canvas_icp_p2pl);
          dmap_p2p.setupDbgOGL(qapp, srrg_viewer, canvas_dmap_p2p);
          dmap_p2pl.setupDbgOGL(qapp, srrg_viewer, canvas_dmap_p2pl);

          std::cout << "just sleeping again for a little second just for the "
                       "fun of it... debug must getting very very ready or not!"
                    << std::endl;
          std::this_thread::sleep_for(std::chrono::seconds(1));
          std::cout << "!!! LET'S GOOOOOOOO (again I now) !!!" << std::endl
                    << std::endl;

          canvas_icp_p2p->flush();
          canvas_icp_p2pl->flush();
          canvas_dmap_p2p->flush();
          canvas_dmap_p2pl->flush();

          run(icp_p2p, icp_p2pl, dmap_p2p, dmap_p2pl, cfg, normal_computator,
              rng_pose, rng_noise, output_dir);

          qapp->closeAllWindows();
        });

    srrg_viewer->startViewerServer();

    benchmark_thread.join();
  }
}

EstimateType getRandomPerturbation(float max_x, float max_y, float max_th,
                                   std::mt19937& rng)
{
  std::uniform_real_distribution<float> dist_x(-max_x, max_x);
  std::uniform_real_distribution<float> dist_y(-max_y, max_y);
  std::uniform_real_distribution<float> dist_r(-max_th, max_th);

  return geometry::v2t(Eigen::Vector3f(dist_x(rng), dist_y(rng), dist_r(rng)));
}

TrialResults getTrialResults(int trial, const EstimateType& gt_t,
                             const EstimateType& estimate_t,
                             const EstimateInfoType& info)
{
  TrialResults res;

  res.m_trial = trial;

  EstimateVectorType gt_v = geometry::t2v(gt_t);
  EstimateVectorType estimate_v = geometry::t2v(estimate_t);

  res.m_gt_x = gt_v.x();
  res.m_gt_y = gt_v.y();
  res.m_gt_th = gt_v.z();

  res.m_estimate_x = estimate_v.x();
  res.m_estimate_y = estimate_v.y();
  res.m_estimate_th = estimate_v.z();

  res.m_estimate_info = info;

  EstimateType error_t = estimate_t * gt_t.inverse();  //!< w.r.t. target frame
  EstimateVectorType error_v = geometry::t2v(error_t);

  res.m_error_x = error_v.x();
  res.m_error_y = error_v.y();
  res.m_error_th = error_v.z();

  res.m_error_trans = error_t.translation().norm();
  res.m_error_rot = std::fabs(error_v.z());

  return res;
}

SceneResults getSceneResults(const std::vector<TrialResults>& results)
{
  const int N = static_cast<int>(results.size());
  const int N_trim = N - static_cast<int>(std::floor(0.05f * N));

  double sum_t = 0.0;
  double sum_t2 = 0.0;

  SceneResults res;

  res.m_error_trans = 0.f;
  res.m_error_x = 0.f;
  res.m_error_y = 0.f;
  res.m_error_rot = 0.f;
  res.m_chi_sqr = 0.f;
  res.m_chi_sqr_mean95 = 0.f;
  res.m_time_min_ms = std::numeric_limits<double>::max();
  res.m_time_max_ms = 0.0;

  std::vector<float> chi_sqr_vals;

  chi_sqr_vals.reserve(N);

  for (const auto& trial : results)
  {
    res.m_error_trans += trial.m_error_trans * trial.m_error_trans;
    res.m_error_x += trial.m_error_x * trial.m_error_x;
    res.m_error_y += trial.m_error_y * trial.m_error_y;
    res.m_error_rot += trial.m_error_rot * trial.m_error_rot;

    Eigen::Vector3f error(trial.m_error_x, trial.m_error_y, trial.m_error_th);

    float chi_sqr_val = error.transpose() * trial.m_estimate_info * error;

    res.m_chi_sqr += chi_sqr_val;

    chi_sqr_vals.push_back(chi_sqr_val);

    sum_t += trial.m_time_ms;
    sum_t2 += trial.m_time_ms * trial.m_time_ms;

    res.m_time_min_ms = std::min(res.m_time_min_ms, trial.m_time_ms);
    res.m_time_max_ms = std::max(res.m_time_max_ms, trial.m_time_ms);
  }

  // Partial sort ascending: smallest N_trim values at front, discarding top 5%
  std::partial_sort(chi_sqr_vals.begin(), chi_sqr_vals.begin() + N_trim,
                    chi_sqr_vals.end());

  for (int i = 0; i < N_trim; ++i)
  {
    res.m_chi_sqr_mean95 += chi_sqr_vals[i];
  }

  res.m_error_trans = std::sqrt(res.m_error_trans / N);
  res.m_error_x = std::sqrt(res.m_error_x / N);
  res.m_error_y = std::sqrt(res.m_error_y / N);
  res.m_error_rot = std::sqrt(res.m_error_rot / N);
  res.m_chi_sqr /= N;

  if (N_trim > 0)
  {
    res.m_chi_sqr_mean95 /= N_trim;
  }
  else
  {
    res.m_chi_sqr_mean95 = 0.f;
  }

  res.m_time_mean_ms = sum_t / N;
  res.m_time_std_ms =
      std::sqrt(sum_t2 / N - res.m_time_mean_ms * res.m_time_mean_ms);

  return res;
}

void saveTrialResultsCSV(const std::vector<AlignerSceneResults>& results,
                         const std::string& output_dir)
{
  std::filesystem::create_directories(output_dir);

  for (const auto& asr : results)
  {
    const std::string filepath =
        output_dir + "/" + asr.m_aligner_type + "_" + asr.m_scene_name + ".csv";

    std::ofstream f(filepath);

    if (!f.is_open())
    {
      std::cerr << "[saveTrialResultsCSV] ERROR: could not open file: "
                << filepath << std::endl;
      continue;
    }

    f << "trial,"
      << "gt_x,gt_y,gt_th,"
      << "est_x,est_y,est_th,"
      << "info_00,info_01,info_02,"
      << "info_10,info_11,info_12,"
      << "info_20,info_21,info_22,"
      << "err_x,err_y,err_th,err_trans,err_rot,"
      << "time_ms" << std::endl;

    for (const auto& tr : asr.m_results)
    {
      f << tr.m_trial << ",";

      f << std::scientific
        << std::setprecision(std::numeric_limits<float>::max_digits10);

      f << tr.m_gt_x << "," << tr.m_gt_y << "," << tr.m_gt_th << ","
        << tr.m_estimate_x << "," << tr.m_estimate_y << "," << tr.m_estimate_th
        << ",";

      for (int r = 0; r < 3; ++r)
      {
        for (int c = 0; c < 3; ++c)
        {
          f << tr.m_estimate_info(r, c) << ",";
        }
      }

      f << tr.m_error_x << "," << tr.m_error_y << "," << tr.m_error_th << ","
        << tr.m_error_trans << "," << tr.m_error_rot << ",";

      f << std::setprecision(std::numeric_limits<double>::max_digits10)
        << tr.m_time_ms << std::endl;
    }
  }
}

void run(AlignerTypeICPp2p& icp_p2p, AlignerTypeICPp2pl& icp_p2pl,
         AlignerTypeDistMapp2p& dmap_p2p, AlignerTypeDistMapp2pl& dmap_p2pl,
         const BenchmarkConfig& cfg,
         const NormalsComputatorType& normal_computator, std::mt19937& rng_pose,
         std::mt19937& rng_noise, const std::string& output_dir)
{
  std::vector<Scene> scenes = {
      {"circle",
       [&](PointCloudType& fixed, PointCloudType& moving)
       {
         test::getLaser2DPointCloudCircle(
             fixed, cfg.m_circ_radius, cfg.m_ang_res, cfg.m_fov,
             cfg.m_max_range, rng_noise, cfg.m_noise_std);
         test::getLaser2DPointCloudCircle(
             moving, cfg.m_circ_radius, cfg.m_ang_res, cfg.m_fov,
             cfg.m_max_range, rng_noise, cfg.m_noise_std);
         normal_computator.computeNormals(fixed);
         normal_computator.computeNormals(moving);
       }},
      {"square",
       [&](PointCloudType& fixed, PointCloudType& moving)
       {
         test::getPointCloudSquare(fixed, cfg.m_sqr_side,
                                   cfg.m_sqr_num_pts_side, rng_noise,
                                   cfg.m_noise_std);
         test::getPointCloudSquare(moving, cfg.m_sqr_side,
                                   cfg.m_sqr_num_pts_side, rng_noise,
                                   cfg.m_noise_std);
         normal_computator.computeNormals(fixed);
         normal_computator.computeNormals(moving);
       }},
      {"corridor",
       [&](PointCloudType& fixed, PointCloudType& moving)
       {
         test::getPointCloudCorridor(fixed, cfg.m_corr_width, cfg.m_corr_length,
                                     cfg.m_sqr_num_pts_side, rng_noise, false,
                                     cfg.m_noise_std);
         test::getPointCloudCorridor(moving, cfg.m_corr_width,
                                     cfg.m_corr_length, cfg.m_sqr_num_pts_side,
                                     rng_noise, false, cfg.m_noise_std);
         normal_computator.computeNormals(fixed);
         normal_computator.computeNormals(moving);
       }},
      {"random",
       [&](PointCloudType& fixed, PointCloudType& moving)
       {
         test::getLaser2DPointCloudRandom(fixed, cfg.m_ang_res, cfg.m_fov,
                                          cfg.m_max_range, rng_noise);

         std::normal_distribution<float> noise(0.0f, cfg.m_noise_std);

         moving.resize(fixed.size());

         for (int idx = 0; idx < static_cast<int>(fixed.size()); idx++)
         {
           moving[idx].coordinates().x() =
               fixed[idx].coordinates().x() + noise(rng_noise);
           moving[idx].coordinates().y() =
               fixed[idx].coordinates().y() + noise(rng_noise);

           moving[idx].normal().setZero();
         }
       }},
      {"corridor_partial", [&](PointCloudType& fixed, PointCloudType& moving)
       {
         test::getPointCloudCorridor(fixed, cfg.m_corr_width, cfg.m_corr_length,
                                     cfg.m_sqr_num_pts_side, rng_noise, false,
                                     cfg.m_noise_std);
         test::getPointCloudCorridor(moving, cfg.m_corr_width, cfg.m_corr_width,
                                     cfg.m_sqr_num_pts_side, rng_noise, false,
                                     cfg.m_noise_std);
         normal_computator.computeNormals(fixed);
         normal_computator.computeNormals(moving);
       }}};

  std::vector<AlignerSceneResults> results;

  std::cout << "=== 2D Point Cloud Aligners Benchmark ===" << std::endl
            << "  scenes      : circle, square, corridor, random" << std::endl
            << "  trials/scene: " << cfg.m_num_trials << std::endl;

  for (auto& scene : scenes)
  {
    float max_rng_x, max_rng_y, max_rng_th;

    if (scene.m_name.compare("circle") == 0)
    {
      max_rng_x = cfg.m_circ_max_rng_x;
      max_rng_y = cfg.m_circ_max_rng_y;
      max_rng_th = cfg.m_circ_max_rng_th;
    }
    else if (scene.m_name.compare("square") == 0)
    {
      max_rng_x = cfg.m_sqr_max_rng_x;
      max_rng_y = cfg.m_sqr_max_rng_y;
      max_rng_th = cfg.m_sqr_max_rng_th;
    }
    else if ((scene.m_name.compare("corridor") == 0) ||
             (scene.m_name.compare("corridor_partial") == 0))
    {
      max_rng_x = cfg.m_corr_max_rng_x;
      max_rng_y = cfg.m_corr_max_rng_y;
      max_rng_th = cfg.m_corr_max_rng_th;
    }
    else if (scene.m_name.compare("random") == 0)
    {
      max_rng_x = cfg.m_rand_max_rng_x;
      max_rng_y = cfg.m_rand_max_rng_y;
      max_rng_th = cfg.m_rand_max_rng_th;
    }
    else
    {
      std::cout << std::endl
                << std::endl
                << "!!! ERROR UNKNOWN SCENE !!!" << std::endl;
      exit(-1);
    }

    AlignerSceneResults icp_p2p_results("icp_p2p", scene.m_name);
    AlignerSceneResults icp_p2pl_results("icp_p2pl", scene.m_name);
    AlignerSceneResults dmap_p2p_results("dmap_p2p", scene.m_name);
    AlignerSceneResults dmap_p2pl_results("dmap_p2pl", scene.m_name);

    std::cout << std::endl
              << "[benchmark] Scene: " << scene.m_name << " ("
              << cfg.m_num_trials << " trials)" << std::endl
              << "  max x translation: " << max_rng_x << " m" << std::endl
              << "  max y translation: " << max_rng_y << " m" << std::endl
              << "  max th rotation  : " << geometry::radToDeg(max_rng_th)
              << " deg" << std::endl;

    bool is_rnd_scene = scene.m_name.compare("random") == 0;

    for (int idx = 0; idx < cfg.m_num_trials; idx++)
    {
      EstimateType gt_t;

      if (kModeManual)
      {
        float input_x, input_y, input_th;

        std::cout << "  [trial " << (idx + 1) << "/" << cfg.m_num_trials << "] "
                  << "Enter perturbation (tx ty th): ";
        std::cin >> input_x >> input_y >> input_th;
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        std::cin.clear();

        gt_t = geometry::v2t(
            Eigen::Vector3f(input_x, input_y, geometry::degToRad(input_th)));
      }
      else
      {
        gt_t =
            getRandomPerturbation(max_rng_x, max_rng_y, max_rng_th, rng_pose);
      }

      EstimateType perturb_t = gt_t.inverse();

      EstimateVectorType perturb_v = geometry::t2v(perturb_t);
      EstimateVectorType gt_v = geometry::t2v(gt_t);

      PointCloudTypePtr pc_fixed = std::make_shared<PointCloudType>();
      PointCloudTypePtr pc_moving = std::make_shared<PointCloudType>();

      scene.m_generate(*pc_fixed, *pc_moving);

      if (kEnableVoxelization)
      {
        pc_fixed->voxelize(kDistMapParamDistMapRes);
        pc_moving->voxelize(kDistMapParamDistMapRes);
      }

      pc_moving->transformInPlace(perturb_t);

      if (is_rnd_scene)
      {
        TrialResults icp_p2p_res;
        TrialResults dmap_p2p_res;

        runAligner(icp_p2p, idx, pc_fixed, pc_moving, gt_t, icp_p2p_res);
        runAligner(dmap_p2p, idx, pc_fixed, pc_moving, gt_t, dmap_p2p_res);

        std::function<void(void)> print_feedback = [&]()
        {
          std::cout << "  [trial " << (idx + 1) << "/" << cfg.m_num_trials
                    << "]" << std::endl
                    << "    - [pertur] " << perturb_v.x() << " "
                    << perturb_v.y() << " " << geometry::radToDeg(perturb_v.z())
                    << std::endl
                    << "    - [gt] " << gt_v.x() << " " << gt_v.y() << " "
                    << geometry::radToDeg(gt_v.z()) << std::endl
                    << "    - [icp  pt2pt] err_t=" << icp_p2p_res.m_error_trans
                    << " m (trans: " << icp_p2p_res.m_error_x << " , "
                    << icp_p2p_res.m_error_y << ")    err_r="
                    << geometry::radToDeg(icp_p2p_res.m_error_rot) << " deg"
                    << " (estimate: " << icp_p2p_res.m_estimate_x << " "
                    << icp_p2p_res.m_estimate_y << " "
                    << geometry::radToDeg(icp_p2p_res.m_estimate_th)
                    << ") [time=" << icp_p2p_res.m_time_ms << " ms]"
                    << std::endl
                    << "    - [dmap pt2pt] err_t=" << dmap_p2p_res.m_error_trans
                    << " m (trans: " << dmap_p2p_res.m_error_x << " , "
                    << dmap_p2p_res.m_error_y << ")    err_r="
                    << geometry::radToDeg(dmap_p2p_res.m_error_rot) << " deg"
                    << " (estimate: " << dmap_p2p_res.m_estimate_x << " "
                    << dmap_p2p_res.m_estimate_y << " "
                    << geometry::radToDeg(dmap_p2p_res.m_estimate_th)
                    << ") [time=" << dmap_p2p_res.m_time_ms << " ms]"
                    << std::endl;
        };

        bool is_feedback_printed = false;

        if ((idx + 1) % cfg.m_cout_feedback == 0 || kModeManual)
        {
          print_feedback();
          is_feedback_printed = true;
        }

        if ((icp_p2p_res.m_error_trans > 2 * kDistMapParamDistMapRes) ||
            (dmap_p2p_res.m_error_trans > 2 * kDistMapParamDistMapRes) ||
            (icp_p2p_res.m_error_rot > geometry::degToRad(5.f)) ||
            (dmap_p2p_res.m_error_rot > geometry::degToRad(5.f)))
        {
          if (!is_feedback_printed)
          {
            print_feedback();
          }

          OFFLERROR("...SOME ALIGNER DID NOT CONVERGED...");
        }

        icp_p2p_results.m_results.push_back(std::move(icp_p2p_res));
        dmap_p2p_results.m_results.push_back(std::move(dmap_p2p_res));
      }
      else
      {
        TrialResults icp_p2p_res;
        TrialResults icp_p2pl_res;
        TrialResults dmap_p2p_res;
        TrialResults dmap_p2pl_res;

        runAligner(icp_p2p, idx, pc_fixed, pc_moving, gt_t, icp_p2p_res);
        runAligner(icp_p2pl, idx, pc_fixed, pc_moving, gt_t, icp_p2pl_res);
        runAligner(dmap_p2p, idx, pc_fixed, pc_moving, gt_t, dmap_p2p_res);
        runAligner(dmap_p2pl, idx, pc_fixed, pc_moving, gt_t, dmap_p2pl_res);

        std::function<void(void)> print_feedback = [&]()
        {
          std::cout
              << "  [trial " << (idx + 1) << "/" << cfg.m_num_trials << "]"
              << std::endl
              << "    - [pertur] " << perturb_v.x() << " " << perturb_v.y()
              << " " << geometry::radToDeg(perturb_v.z()) << std::endl
              << "    - [gt] " << gt_v.x() << " " << gt_v.y() << " "
              << geometry::radToDeg(gt_v.z()) << std::endl
              << "    - [icp  pt2pt] err_t=" << icp_p2p_res.m_error_trans
              << " m (trans: " << icp_p2p_res.m_error_x << " , "
              << icp_p2p_res.m_error_y
              << ")    err_r=" << geometry::radToDeg(icp_p2p_res.m_error_rot)
              << " deg"
              << " (estimate: " << icp_p2p_res.m_estimate_x << " "
              << icp_p2p_res.m_estimate_y << " "
              << geometry::radToDeg(icp_p2p_res.m_estimate_th)
              << ") [time=" << icp_p2p_res.m_time_ms << " ms]" << std::endl
              << "    - [icp  pt2pl] err_t=" << icp_p2pl_res.m_error_trans
              << " m (trans: " << icp_p2pl_res.m_error_x << " , "
              << icp_p2pl_res.m_error_y
              << ")    err_r=" << geometry::radToDeg(icp_p2pl_res.m_error_rot)
              << " deg"
              << " (estimate: " << icp_p2pl_res.m_estimate_x << " "
              << icp_p2pl_res.m_estimate_y << " "
              << geometry::radToDeg(icp_p2pl_res.m_estimate_th)
              << ") [time=" << icp_p2pl_res.m_time_ms << " ms]" << std::endl
              << "    - [dmap pt2pt] err_t=" << dmap_p2p_res.m_error_trans
              << " m (trans: " << dmap_p2p_res.m_error_x << " , "
              << dmap_p2p_res.m_error_y
              << ")    err_r=" << geometry::radToDeg(dmap_p2p_res.m_error_rot)
              << " deg"
              << " (estimate: " << dmap_p2p_res.m_estimate_x << " "
              << dmap_p2p_res.m_estimate_y << " "
              << geometry::radToDeg(dmap_p2p_res.m_estimate_th)
              << ") [time=" << dmap_p2p_res.m_time_ms << " ms]" << std::endl
              << "    - [dmap pt2pl] err_t=" << dmap_p2pl_res.m_error_trans
              << " m (trans: " << dmap_p2pl_res.m_error_x << " , "
              << dmap_p2pl_res.m_error_y
              << ")    err_r=" << geometry::radToDeg(dmap_p2pl_res.m_error_rot)
              << " deg"
              << " (estimate: " << dmap_p2pl_res.m_estimate_x << " "
              << dmap_p2pl_res.m_estimate_y << " "
              << geometry::radToDeg(dmap_p2pl_res.m_estimate_th)
              << ") [time=" << dmap_p2pl_res.m_time_ms << " ms]" << std::endl;
        };

        bool is_feedback_printed = false;

        if ((idx + 1) % cfg.m_cout_feedback == 0 || kModeManual)
        {
          print_feedback();
          is_feedback_printed = true;
        }

        if (scene.m_name.compare("circle") == 0)
        {
          if ((icp_p2p_res.m_error_trans > 2 * kDistMapParamDistMapRes) ||
              (icp_p2pl_res.m_error_trans > 2 * kDistMapParamDistMapRes) ||
              (dmap_p2p_res.m_error_trans > 2 * kDistMapParamDistMapRes) ||
              (dmap_p2pl_res.m_error_trans > 2 * kDistMapParamDistMapRes))
          {
            if (!is_feedback_printed)
            {
              print_feedback();
            }

            OFFLERROR("...SOME ALIGNER DID NOT CONVERGED...");
          }
        }
        else if ((scene.m_name.compare("random") == 0) ||
                 (scene.m_name.compare("square") == 0))
        {
          if ((icp_p2p_res.m_error_trans > 2 * kDistMapParamDistMapRes) ||
              (icp_p2pl_res.m_error_trans > 2 * kDistMapParamDistMapRes) ||
              (dmap_p2p_res.m_error_trans > 2 * kDistMapParamDistMapRes) ||
              (dmap_p2pl_res.m_error_trans > 2 * kDistMapParamDistMapRes) ||
              (icp_p2p_res.m_error_rot > geometry::degToRad(5.f)) ||
              (icp_p2pl_res.m_error_rot > geometry::degToRad(5.f)) ||
              (dmap_p2p_res.m_error_rot > geometry::degToRad(5.f)) ||
              (dmap_p2pl_res.m_error_rot > geometry::degToRad(5.f)))
          {
            if (!is_feedback_printed)
            {
              print_feedback();
            }

            OFFLERROR("...SOME ALIGNER DID NOT CONVERGED...");
          }
        }
        else if ((scene.m_name.compare("corridor") == 0) ||
                 (scene.m_name.compare("corridor_partial") == 0))
        {
          if ((icp_p2p_res.m_error_y > 2 * kDistMapParamDistMapRes) ||
              (icp_p2pl_res.m_error_y > 2 * kDistMapParamDistMapRes) ||
              (dmap_p2p_res.m_error_y > 2 * kDistMapParamDistMapRes) ||
              (dmap_p2pl_res.m_error_y > 2 * kDistMapParamDistMapRes) ||
              (icp_p2p_res.m_error_rot > geometry::degToRad(5.f)) ||
              (icp_p2pl_res.m_error_rot > geometry::degToRad(5.f)) ||
              (dmap_p2p_res.m_error_rot > geometry::degToRad(5.f)) ||
              (dmap_p2pl_res.m_error_rot > geometry::degToRad(5.f)))
          {
            if (!is_feedback_printed)
            {
              print_feedback();
            }

            OFFLERROR("...SOME ALIGNER DID NOT CONVERGED...");
          }
        }

        icp_p2p_results.m_results.push_back(std::move(icp_p2p_res));
        icp_p2pl_results.m_results.push_back(std::move(icp_p2pl_res));
        dmap_p2p_results.m_results.push_back(std::move(dmap_p2p_res));
        dmap_p2pl_results.m_results.push_back(std::move(dmap_p2pl_res));
      }
    }

    icp_p2p_results.m_res = getSceneResults(icp_p2p_results.m_results);
    icp_p2pl_results.m_res = getSceneResults(icp_p2pl_results.m_results);
    dmap_p2p_results.m_res = getSceneResults(dmap_p2p_results.m_results);
    dmap_p2pl_results.m_res = getSceneResults(dmap_p2pl_results.m_results);

    if (is_rnd_scene)
    {
      results.push_back(std::move(icp_p2p_results));
      results.push_back(std::move(dmap_p2p_results));
    }
    else
    {
      results.push_back(std::move(icp_p2p_results));
      results.push_back(std::move(icp_p2pl_results));
      results.push_back(std::move(dmap_p2p_results));
      results.push_back(std::move(dmap_p2pl_results));
    }
  }

  std::cout << std::endl
            << std::endl
            << "=== 2D Point Cloud Aligners Benchmark --- RESULTS ==="
            << std::endl
            << "  scenes      : circle, square, corridor, random" << std::endl
            << "  trials/scene: " << cfg.m_num_trials << std::endl
            << "  trials top95: "
            << cfg.m_num_trials -
                   static_cast<int>(std::floor(0.05f * cfg.m_num_trials))
            << std::endl
            << "  dmap res (m): " << kDistMapParamDistMapRes << std::endl
            << std::endl;

  for (const auto& res : results)
  {
    std::cout
        << "[" << res.m_scene_name << "] " << res.m_aligner_type << ":"
        << std::endl
        << "- err trans rmse (m): " << res.m_res.m_error_trans << std::endl
        << "- err x rmse     (m): " << res.m_res.m_error_x << std::endl
        << "- err y rmse     (m): " << res.m_res.m_error_y << std::endl
        << "- err rot rmse (deg): " << geometry::radToDeg(res.m_res.m_error_rot)
        << std::endl
        << "- mean chi sqr      : " << res.m_res.m_chi_sqr << std::endl
        << "- mean chi sqr 95%  : " << res.m_res.m_chi_sqr_mean95 << std::endl
        << "- time mean     (ms): " << res.m_res.m_time_mean_ms << std::endl
        << "- time std      (ms): " << res.m_res.m_time_std_ms << std::endl
        << "- time min      (ms): " << res.m_res.m_time_min_ms << std::endl
        << "- time max      (ms): " << res.m_res.m_time_max_ms << std::endl
        << std::endl
        << std::endl;
  }

  saveTrialResultsCSV(results, output_dir);
}
