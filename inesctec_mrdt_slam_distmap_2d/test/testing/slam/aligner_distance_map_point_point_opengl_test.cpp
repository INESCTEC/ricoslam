#include <Eigen/Eigenvalues>
#include <chrono>
#include <inesctec_mrdt_slam_distmap_2d/geometry/geometry_2d.hpp>
#include <inesctec_mrdt_slam_distmap_2d/slam/aligner_distance_map_point_point.hpp>
#include <inesctec_mrdt_slam_distmap_2d/test/point_cloud_examples.hpp>
#include <memory>
#include <mutex>
#include <random>
#include <thread>
#include <vector>

std::shared_ptr<srrg2_core::ViewerCanvas> srrg_canvas = nullptr;

using namespace inesctec_mrdt_slam_distmap_2d;

int main(int argc, char **argv)
{
  std::shared_ptr<QApplication> qapp =
      std::make_shared<QApplication>(argc, argv);
  std::shared_ptr<srrg2_qgl_viewport::ViewerCoreSharedQGLCustom> srrg_viewer =
      std::make_shared<srrg2_qgl_viewport::ViewerCoreSharedQGLCustom>(
          argc, argv, qapp.get(), BUFFER_SIZE_50MEGABYTE, 3, 25, false);

  srrg_canvas = srrg_viewer->getCanvas(
      "slam_distmap_2d_slam_aligner_distance_map_point_point_opengl");
  srrg_canvas->setName(
      "slam_distmap_2d_slam_aligner_distance_map_point_point_opengl");

  std::thread run_tests(
      [&qapp, &srrg_viewer]()
      {
        using VariableType = srrg2_solver::
            VariableSE2DistanceMapStaticPointNormal2fVectorCloudRight;
        using EstimateType = VariableType::EstimateType;
        using AlignerType =
            inesctec_mrdt_slam_distmap_2d::slam::AlignerDistanceMapPointPoint_<
                VariableType>;
        using AlignerBaseParamType = AlignerType::BaseType::Param;
        using AlignerParamType = AlignerType::DistMapParam;
        using FactorType = AlignerType::FactorType;
        using MovingPointCloudType = FactorType::MovingContainerType;
        using FixedPointCloudType = AlignerType::PointCloudType;

        using MovingPointCloudTypePtr = std::shared_ptr<MovingPointCloudType>;
        using FixedPointCloudTypePtr = std::shared_ptr<FixedPointCloudType>;

        std::random_device rd;
        std::mt19937 rng(rd());

        AlignerBaseParamType param;

        param.m_debug = true;
        param.m_solver_verbose = true;
        param.m_solver_damping = 1.f;
        param.m_solver_max_iterations = 25;
        param.m_solver_criteria_type = "stepnorm";
        param.m_solver_criteria_epsilon = 1e-5;
        param.m_solver_robustifier_type = "cauchy";
        param.m_solver_robustifier_num_iterations_coarse = 10;
        param.m_solver_robustifier_threshold_coarse = 1.f;
        param.m_solver_robustifier_threshold_fine = 0.25f;
        param.m_failure_ratio_inliers = 0.35f;
        param.m_failure_chi_inliers = -1.0f;
        param.m_dbg_ogl_interval = 250;
        param.m_dbg_ogl_mode = "iterationend";
        param.m_dbg_ogl_show_correspondences = true;
        param.m_dbg_ogl_show_normals = false;

        AlignerParamType distmap_param;

        distmap_param.m_dist_map_res = 0.025;
        distmap_param.m_dist_map_size = 10.0;
        distmap_param.m_dist_map_max_dist = 2.5;
        distmap_param.m_solver_regularize_correspondences = false;

        const float noise_std = distmap_param.m_dist_map_res;

        constexpr float max_range = 10.f;

        constexpr float circle_radius = 5.0;
        constexpr float square_side = 10.0;
        constexpr int square_side_pts = 250;
        constexpr float corridor_length = 10.0;
        constexpr float corridor_width = 5.0;

        float ang_res = geometry::degToRad(1.f);
        float fov = 2 * M_PIf32;

        AlignerType aligner(param, distmap_param);

        aligner.setupDbgOGL(qapp, srrg_viewer, srrg_canvas);

        std::cout << "AlignerDistanceMapPointPoint_<"
                     "VariableSE2DistanceMapStaticRight> Parametrization"
                  << std::endl
                  << param << distmap_param << std::endl;

        std::cout << "TEST "
                     "AlignerDistanceMapPointPoint_<"
                     "VariableSE2DistanceMapStaticRight>"
                  << std::endl
                  << "esc : exit the test (NEED TO INPUT ANOTHER LETTER AFTER "
                     "THE CANVAS "
                     "CLOSES!)"
                  << std::endl
                  << "w, s: forward linear motion" << std::endl
                  << "a, d: side linear motion" << std::endl
                  << "q, e: rotate" << std::endl
                  << "1   : compute solver" << std::endl
                  << "r   : random" << std::endl
                  << "c   : circle" << std::endl
                  << "x   : square" << std::endl
                  << "l   : corridor" << std::endl;

        FixedPointCloudTypePtr pc_fixed =
            std::make_shared<FixedPointCloudType>();
        MovingPointCloudTypePtr pc_moving =
            std::make_shared<MovingPointCloudType>();

        test::getLaser2DPointCloudCircle(*pc_fixed, circle_radius, ang_res, fov,
                                         max_range, rng, noise_std);
        test::getLaser2DPointCloudCircle(*pc_moving, circle_radius, ang_res,
                                         fov, max_range, rng, noise_std);

        aligner.setSource(pc_moving, EstimateType::Identity());
        aligner.setSourceInTarget(EstimateType::Identity());
        aligner.setTarget(pc_fixed, EstimateType::Identity());
        aligner.setPriorEnabled(false);

        char key = 0;

        EstimateType motion_gt = EstimateType::Identity();

        while (srrg_viewer->isRunning())
        {
          aligner.updateDbgOPGLCanvas(true);
          std::cin >> key;
          std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

          EstimateType motion = EstimateType::Identity();

          float dt = distmap_param.m_dist_map_res;
          float dr = inesctec_mrdt_slam_distmap_2d::geometry::degToRad(1.0f);

          if ((key == 'r') || (key == 'c') || (key == 'x') || (key == 'l'))
          {
            pc_fixed->clear();
            pc_moving->clear();
          }

          switch (key)
          {
            case 'w':
            {
              motion =
                  srrg2_core::geometry2d::v2t(srrg2_core::Vector3f(dt, 0, 0));
              break;
            }
            case 's':
            {
              motion =
                  srrg2_core::geometry2d::v2t(srrg2_core::Vector3f(-dt, 0, 0));
              break;
            }
            case 'a':
            {
              motion =
                  srrg2_core::geometry2d::v2t(srrg2_core::Vector3f(0, dt, 0));
              break;
            }
            case 'd':
            {
              motion =
                  srrg2_core::geometry2d::v2t(srrg2_core::Vector3f(0, -dt, 0));
              break;
            }
            case 'q':
            {
              motion =
                  srrg2_core::geometry2d::v2t(srrg2_core::Vector3f(0, 0, dr));
              break;
            }
            case 'e':
            {
              motion =
                  srrg2_core::geometry2d::v2t(srrg2_core::Vector3f(0, 0, -dr));
              break;
            }
            case 'r':
            {
              inesctec_mrdt_slam_distmap_2d::test::getLaser2DPointCloudRandom(
                  *pc_fixed, ang_res, fov, max_range, rng);
              inesctec_mrdt_slam_distmap_2d::test::getLaser2DPointCloudRandom(
                  *pc_moving, ang_res, fov, max_range, rng);
              break;
            }
            case 'c':
            {
              inesctec_mrdt_slam_distmap_2d::test::getLaser2DPointCloudCircle(
                  *pc_fixed, circle_radius, ang_res, fov, max_range, rng,
                  noise_std);
              inesctec_mrdt_slam_distmap_2d::test::getLaser2DPointCloudCircle(
                  *pc_moving, circle_radius, ang_res, fov, max_range, rng,
                  noise_std);
              break;
            }
            case 'x':
            {
              inesctec_mrdt_slam_distmap_2d::test::getPointCloudSquare(
                  *pc_fixed, square_side, square_side_pts, rng, noise_std);
              inesctec_mrdt_slam_distmap_2d::test::getPointCloudSquare(
                  *pc_moving, square_side, square_side_pts, rng, noise_std);
              break;
            }
            case 'l':
            {
              inesctec_mrdt_slam_distmap_2d::test::getPointCloudCorridor(
                  *pc_fixed, corridor_width, corridor_length, square_side_pts,
                  rng, false, noise_std);
              inesctec_mrdt_slam_distmap_2d::test::getPointCloudCorridor(
                  *pc_moving, corridor_width, corridor_length, square_side_pts,
                  rng, false, noise_std);
              break;
            }
            case '1':
            {
              aligner.compute();
              std::cout << std::endl
                        << std::endl
                        << "final estimation :"
                        << srrg2_core::geometry2d::t2v(
                               aligner.getSourceInTarget())
                               .transpose()
                        << std::endl;
              std::cout << "ground-truth     :"
                        << srrg2_core::geometry2d::t2v(motion_gt.inverse())
                               .transpose()
                        << std::endl;
              std::cout << "H mat            :" << std::endl;
              aligner.getSolver()->H().blockAt(0, 0)->print();

              Eigen::Matrix3f info_mat =
                  aligner.getSolver()->H().blockAt(0, 0)->toMatrixXf();
              Eigen::Matrix3f cov_mat = info_mat.inverse();

              Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver;

              eigen_solver.computeDirect(info_mat);

              std::cout << "H mat - eigen val: "
                        << eigen_solver.eigenvalues().real().transpose()
                        << std::endl;

              std::cout << "C mat            : " << std::endl
                        << cov_mat << std::endl;

              eigen_solver.computeDirect(cov_mat);

              std::cout << "C mat - eigen val: "
                        << eigen_solver.eigenvalues().real().transpose()
                        << std::endl
                        << std::endl;

              std::cout
                  << "TEST "
                     "AlignerDistanceMapPointPoint_<"
                     "VariableSE2DistanceMapStaticRight>"
                  << std::endl
                  << "esc : exit the test (NEED TO INPUT ANOTHER LETTER AFTER "
                     "THE CANVAS CLOSES!)"
                  << std::endl
                  << "w, s: forward linear motion" << std::endl
                  << "a, d: side linear motion" << std::endl
                  << "q, e: rotate" << std::endl
                  << "1   : compute solver" << std::endl
                  << "r   : random" << std::endl
                  << "c   : circle" << std::endl
                  << "x   : square" << std::endl
                  << "l   : corridor" << std::endl;
              break;
            }
          }

          if ((key == 'a') || (key == 's') || (key == 'd') || (key == 'w') ||
              (key == 'q') || (key == 'e'))
          {
            pc_moving->transformInPlace(motion);
            motion_gt = motion * motion_gt;
          }

          if ((key == 'r') || (key == 'c') || (key == 'x') || (key == 'l'))
          {
            motion_gt = EstimateType::Identity();
            aligner.setSource(pc_moving, motion_gt);
            aligner.setSourceInTarget(motion_gt);
            aligner.setTarget(pc_fixed, motion_gt);
          }
        }
      });

  srrg_viewer->startViewerServer();

  run_tests.join();

  return 0;
}  // int main(int, char**)
