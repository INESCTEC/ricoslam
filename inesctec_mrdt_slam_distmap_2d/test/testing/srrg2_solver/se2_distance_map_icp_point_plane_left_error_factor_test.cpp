#include <srrg2_core/srrg_geometry/geometry2d.h>
#include <srrg2_core/srrg_geometry/geometry3d.h>
#include <srrg2_core/srrg_viewer/viewer_core/color_palette.h>
#include <srrg2_qgl_viewport/viewer_core_shared_qgl.h>
#include <srrg2_solver/solver_core/factor_graph.h>
#include <srrg2_solver/solver_core/instances.h>
#include <srrg2_solver/solver_core/internals/linear_solvers/instances.h>
#include <srrg2_solver/solver_core/internals/linear_solvers/sparse_block_linear_solver_cholesky.h>
#include <srrg2_solver/solver_core/iteration_algorithm_gn.h>
#include <srrg2_solver/solver_core/solver.h>

#include <Eigen/Eigenvalues>
#include <chrono>
#include <inesctec_mrdt_slam_distmap_2d/data_structures/image_defs.hpp>
#include <inesctec_mrdt_slam_distmap_2d/geometry/geometry_2d.hpp>
#include <inesctec_mrdt_slam_distmap_2d/pcl/normal_computator.hpp>
#include <inesctec_mrdt_slam_distmap_2d/srrg2_qgl_viewport/viewer_core_shared_qgl_custom.hpp>
#include <inesctec_mrdt_slam_distmap_2d/srrg2_solver/instances.hpp>
#include <inesctec_mrdt_slam_distmap_2d/srrg2_solver/se2_distance_map_icp_point_plane_left_error_factor.hpp>
#include <inesctec_mrdt_slam_distmap_2d/srrg2_solver/variable_se2_distance_map.hpp>
#include <inesctec_mrdt_slam_distmap_2d/test/point_cloud_examples.hpp>
#include <memory>
#include <mutex>
#include <random>
#include <thread>
#include <vector>

using FactorGraphType = srrg2_solver::FactorGraph;

using FactorType = srrg2_solver::
    SE2DistanceMapStaticPointNormal2fVectorCloudLeftICPPointPlaneWithSensorErrorFactorCorrespondenceFreeDriven;
using VariableType = FactorType::VariableType;
using EstimateType = VariableType::EstimateType;
using InformationMatrixType = FactorType::InformationMatrixType;

using FactorGraphTypePtr = std::shared_ptr<srrg2_solver::FactorGraph>;
using VariableTypePtr = std::shared_ptr<VariableType>;
using FactorTypePtr = std::shared_ptr<FactorType>;

using MovingPointCloudType = FactorType::MovingContainerType;
using MovingPointType = FactorType::MovingItemType;

using MeasurementType = VariableType::MeasurementOwnerType::MeasurementObjType;
using DistanceMapType = VariableType::MeasurementOwnerType::DistanceMapType;
using DistanceMapTypePtr =
    VariableType::MeasurementOwnerType::DistanceMapTypePtr;
using FixedPointCloudType = VariableType::MeasurementOwnerType::PointCloudType;
using FixedPointType = FixedPointCloudType::PointType;

using MovingPointCloudTypePtr = std::shared_ptr<MovingPointCloudType>;
using FixedPointCloudTypePtr = std::shared_ptr<FixedPointCloudType>;

using NormalComputatorType =
    inesctec_mrdt_slam_distmap_2d::pcl::NormalComputatorNearestPoints1D_<
        MovingPointCloudType>;
using NormalComputatorTypePtr = std::shared_ptr<NormalComputatorType>;

std::shared_ptr<srrg2_core::ViewerCanvas> srrg_canvas = nullptr;

namespace testing
{

void viewGraph(const srrg2_core::ViewerCanvasPtr& canvas,
               const FactorGraphTypePtr& graph, int64_t sleep_ms = 50)
{
  using VwrPointCloudType = srrg2_core::PointNormal3fVectorCloud;

  while (!canvas->_setup())
  {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  for (auto var : graph->variables())
  {
    const VariableType* v = static_cast<const VariableType*>(var.second);

    VwrPointCloudType pts;
    pts.resize(v->measurement()->getPointCloud()->size());

    canvas->pushColor();
    canvas->setColor(srrg2_core::ColorPalette::color4fDarkRed(0.5));
    canvas->putSphere(0.05);
    canvas->putReferenceSystem(0.05);
    canvas->popAttribute();

    if (!v->measurement())
    {
      continue;
    }
    if (!v->measurement()->getPointCloudPtr())
    {
      continue;
    }
    if (v->measurement()->getPointCloud()->empty())
    {
      continue;
    }

    for (size_t idx = 0; idx < v->measurement()->getPointCloud()->size(); idx++)
    {
      pts[idx].coordinates().x() =
          v->measurement()->getPointCloud()->operator[](idx).coordinates().x();
      pts[idx].coordinates().y() =
          v->measurement()->getPointCloud()->operator[](idx).coordinates().y();
      pts[idx].coordinates().z() = 0;
    }

    canvas->pushColor();
    canvas->setColor(srrg2_core::ColorPalette::color3fBlack());
    canvas->putPoints(pts);
    canvas->popAttribute();
  }
  for (auto fact : graph->factors())
  {
    const FactorType* f = static_cast<const FactorType*>(fact.second);
    const VariableType* v = static_cast<const VariableType*>(f->variable(0));

    VwrPointCloudType pts;

    if (!f->getMoving())
    {
      continue;
    }
    if (f->getMoving()->empty())
    {
      continue;
    }

    pts.resize(f->getMoving()->size());

    for (size_t idx = 0; idx < v->measurement()->getPointCloud()->size(); idx++)
    {
      pts[idx].coordinates().x() =
          f->getMoving()->operator[](idx).coordinates().x();
      pts[idx].coordinates().y() =
          f->getMoving()->operator[](idx).coordinates().y();
      pts[idx].coordinates().z() = 0;

      pts[idx].normal().x() = f->getMoving()->operator[](idx).normal().x();
      pts[idx].normal().y() = f->getMoving()->operator[](idx).normal().y();
      pts[idx].normal().z() = 0;
    }

    canvas->pushColor();
    canvas->setColor(srrg2_core::ColorPalette::color3fRed());
    canvas->pushMatrix();
    canvas->multMatrix(
        srrg2_core::geometry3d::get3dFrom2dPose(v->estimate()).matrix());
    canvas->putPoints(pts);
    canvas->popMatrix();
    canvas->popAttribute();
  }
  canvas->flush();
  std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
}  // void viewGraph(const srrg2_core::ViewerCanvasPtr&, const
   // FactorGraphTypePtr&, int64_t = 50)

}  // namespace testing

int main(int argc, char** argv)
{
  std::shared_ptr<QApplication> qapp =
      std::make_shared<QApplication>(argc, argv);
  std::shared_ptr<srrg2_qgl_viewport::ViewerCoreSharedQGLCustom> srrg_viewer =
      std::make_shared<srrg2_qgl_viewport::ViewerCoreSharedQGLCustom>(
          argc, argv, qapp.get(), BUFFER_SIZE_50MEGABYTE, 3, 25, false);

  srrg_canvas = srrg_viewer->getCanvas(
      "srrg2_solver_se2_distance_map_point_plane_left_error_factor");
  srrg_canvas->setName(
      "srrg2_solver_se2_distance_map_point_plane_left_error_factor");

  std::thread run_tests(
      [&srrg_viewer]()
      {
        constexpr int num_iterations = 25;

        constexpr float res = 0.025f;
        constexpr float max_dist = 2.5f;
        constexpr float max_range = 10.f;
        constexpr float map_size = 10.f;

        constexpr float circle_radius = 5.0;
        constexpr float square_side = 10.0;
        constexpr int square_side_pts = 250;
        constexpr float corridor_length = square_side;
        constexpr float corridor_width = 5.0;

        std::random_device rd;
        std::mt19937 rng(rd());

        float noise_std = res;

        float ang_res = inesctec_mrdt_slam_distmap_2d::geometry::degToRad(1.0f);
        float fov = 2 * M_PIf32;

        std::cout << "TEST "
                     "SE2DistanceMapStaticPointPlaneLeftErrorFactorCorresponden"
                     "ceFreeDriven"
                  << std::endl
                  << "esc : exit the test (NEED TO INPUT ANOTHER LETTER AFTER "
                     "THE CANVAS "
                     "CLOSES!)"
                  << std::endl
                  << "w, s: forward linear motion" << std::endl
                  << "a, d: side linear motion" << std::endl
                  << "q, e: rotate" << std::endl
                  << "+, -: add / reduce noise (current noise: " << noise_std
                  << ")" << std::endl
                  << "1   : compute solver" << std::endl
                  << "c   : circle" << std::endl
                  << "x   : square" << std::endl
                  << "l   : corridor" << std::endl;

        FactorGraphTypePtr graph = std::make_shared<FactorGraphType>();
        VariableTypePtr variable = std::make_shared<VariableType>();
        FactorTypePtr factor = std::make_shared<FactorType>();

        variable->setEstimate(EstimateType::Identity());
        variable->setGraphId(0);
        variable->setMeasurement(std::make_shared<MeasurementType>());

        factor->setVariableId(0, variable->graphId());
        factor->setInformationMatrix(InformationMatrixType::Identity());

        graph->addVariable(variable);
        graph->addFactor(factor);

        srrg2_solver::Solver solver;

        solver.setGraph(graph);
        solver.param_verbose.setValue(true);
        solver.param_termination_criteria.setValue(nullptr);
        solver.param_max_iterations.setValue({num_iterations});

        solver.param_actions.pushBack(
            std::make_shared<srrg2_solver::SolverVerboseAction>());

        solver.param_algorithm.setValue(
            std::make_shared<srrg2_solver::IterationAlgorithmGN>());
        dynamic_cast<srrg2_solver::IterationAlgorithmGN*>(
            solver.param_algorithm.value().get())
            ->param_damping.setValue(1.0f);

        solver.param_linear_solver.setValue(
            std::make_shared<srrg2_solver::SparseBlockLinearSolverCholesky>());

        FixedPointCloudTypePtr pc_fixed =
            std::make_shared<FixedPointCloudType>();
        MovingPointCloudTypePtr pc_moving =
            std::make_shared<MovingPointCloudType>();

        inesctec_mrdt_slam_distmap_2d::test::getLaser2DPointCloudCircle(
            *pc_fixed, circle_radius, ang_res, fov, max_range, rng, noise_std);
        inesctec_mrdt_slam_distmap_2d::test::getLaser2DPointCloudCircle(
            *pc_moving, circle_radius, ang_res, fov, max_range, rng, noise_std);

        variable->measurement()->m_pc = pc_fixed;
        variable->measurement()->m_dist_map =
            std::make_shared<DistanceMapType>();
        variable->measurement()->getDistanceMap()->setGoals(*pc_fixed, res,
                                                            map_size, max_dist);

        NormalComputatorTypePtr normal_computator =
            std::make_shared<NormalComputatorType>();

        normal_computator->m_min_pts = 3;
        normal_computator->m_radius = 0.15f;

        normal_computator->computeNormals(*pc_fixed);
        normal_computator->computeNormals(*pc_moving);

        factor->setMoving(*pc_moving);

        char key = 0;

        EstimateType motion_gt = EstimateType::Identity();

        while (srrg_viewer->isRunning())
        {
          testing::viewGraph(srrg_canvas, graph, 250);

          std::cin >> key;
          std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

          EstimateType motion = EstimateType::Identity();

          float dt = res;
          float dr = inesctec_mrdt_slam_distmap_2d::geometry::degToRad(1.0f);

          if ((key == 'c') || (key == 'x') || (key == 'l'))
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
            case 'c':
            {
              inesctec_mrdt_slam_distmap_2d::test::getLaser2DPointCloudCircle(
                  *pc_fixed, circle_radius, ang_res, fov, max_range, rng,
                  noise_std);
              inesctec_mrdt_slam_distmap_2d::test::getLaser2DPointCloudCircle(
                  *pc_moving, circle_radius, ang_res, fov, max_range, rng,
                  noise_std);
              normal_computator->computeNormals(*pc_fixed);
              normal_computator->computeNormals(*pc_moving);
              break;
            }
            case 'x':
            {
              inesctec_mrdt_slam_distmap_2d::test::getPointCloudSquare(
                  *pc_fixed, square_side, square_side_pts, rng, noise_std);
              inesctec_mrdt_slam_distmap_2d::test::getPointCloudSquare(
                  *pc_moving, square_side, square_side_pts, rng, noise_std);
              normal_computator->computeNormals(*pc_fixed);
              normal_computator->computeNormals(*pc_moving);
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
              normal_computator->computeNormals(*pc_fixed);
              normal_computator->computeNormals(*pc_moving);
              break;
            }
            case '+':
            {
              noise_std += res * 0.25f;
              break;
            }
            case '-':
            {
              noise_std = std::max(noise_std - res * 0.25f, 0.f);
              break;
            }
            case '1':
            {
              solver.compute();
              std::cout << std::endl
                        << std::endl
                        << "final estimation:"
                        << srrg2_core::geometry2d::t2v(variable->estimate())
                               .transpose()
                        << std::endl
                        << std::endl;
              std::cout << "gt:" << std::endl
                        << srrg2_core::geometry2d::t2v(motion_gt.inverse())
                               .transpose()
                        << std::endl
                        << std::endl;
              std::cout << "H matrix:" << std::endl;
              solver.H().blockAt(0, 0)->print();

              Eigen::Matrix3f info_mat = solver.H().blockAt(0, 0)->toMatrixXf();
              Eigen::Matrix3f cov_mat = info_mat.inverse();

              Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver;

              eigen_solver.computeDirect(info_mat);

              std::cout << "H matrix - eigen values: "
                        << eigen_solver.eigenvalues().real().transpose()
                        << std::endl;

              std::cout << "Covariance matrix:" << std::endl
                        << cov_mat << std::endl;

              eigen_solver.computeDirect(cov_mat);

              std::cout << "Covariance matrix - eigen values: "
                        << eigen_solver.eigenvalues().real().transpose()
                        << std::endl
                        << std::endl;

              std::cout
                  << "TEST "
                     "SE2DistanceMapStaticPointPlaneLeftErrorFactorCorresponden"
                     "ceFreeDriven"
                  << std::endl
                  << "esc : exit the test (NEED TO INPUT ANOTHER LETTER AFTER "
                     "THE CANVAS CLOSES!)"
                  << std::endl
                  << "w, s: forward linear motion" << std::endl
                  << "a, d: side linear motion" << std::endl
                  << "q, e: rotate" << std::endl
                  << "+, -: add / reduce noise (current noise: " << noise_std
                  << ")" << std::endl
                  << "1   : compute solver" << std::endl
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

          if ((key == '+') || (key == '-'))
          {
            std::cout
                << "TEST "
                   "SE2DistanceMapStaticPointPlaneLeftErrorFactorCorrespondence"
                   "FreeDriven"
                << std::endl
                << "esc : exit the test (NEED TO INPUT ANOTHER LETTER AFTER "
                   "THE CANVAS CLOSES!)"
                << std::endl
                << "w, s: forward linear motion" << std::endl
                << "a, d: side linear motion" << std::endl
                << "q, e: rotate" << std::endl
                << "+, -: add / reduce noise (current noise: " << noise_std
                << ")" << std::endl
                << "1   : compute solver" << std::endl
                << "c   : circle" << std::endl
                << "x   : square" << std::endl
                << "l   : corridor" << std::endl;
          }

          if ((key == 'c') || (key == 'x') || (key == 'l'))
          {
            motion_gt = EstimateType::Identity();
            variable->setEstimate(EstimateType::Identity());
            variable->measurement()->getDistanceMap()->setGoals(
                *pc_fixed, res, map_size, max_dist);
            factor->setMoving(*pc_moving);
          }
        }
      });

  srrg_viewer->startViewerServer();

  run_tests.join();

  return 0;
}  // int main(int, char**)
