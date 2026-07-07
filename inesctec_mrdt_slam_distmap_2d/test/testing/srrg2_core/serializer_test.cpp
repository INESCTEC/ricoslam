#include <gtest/gtest.h>
#include <srrg2_core/srrg_boss/deserializer.h>
#include <srrg2_core/srrg_boss/serializable.h>
#include <srrg2_core/srrg_pcl/instances.h>
#include <srrg2_core/srrg_pcl/point_types.h>
#include <srrg2_core/srrg_pcl/point_types_data.h>
#include <srrg2_solver/solver_core/factor.h>
#include <srrg2_solver/solver_core/factor_graph.h>
#include <srrg2_solver/solver_core/instances.h>
#include <srrg2_solver/solver_core/variable.h>

#include <inesctec_mrdt_slam_distmap_2d/data_structures/all_types.hpp>
#include <inesctec_mrdt_slam_distmap_2d/data_structures/instances.hpp>
#include <inesctec_mrdt_slam_distmap_2d/geometry/geometry_2d.hpp>
#include <inesctec_mrdt_slam_distmap_2d/pcl/point_types.hpp>
#include <inesctec_mrdt_slam_distmap_2d/srrg2_solver/instances.hpp>
#include <inesctec_mrdt_slam_distmap_2d/srrg2_solver/measurement_owner_distance_map_point_cloud.hpp>
#include <inesctec_mrdt_slam_distmap_2d/srrg2_solver/se2_distance_map_pose_pose_geodesic_error_factor.hpp>
#include <inesctec_mrdt_slam_distmap_2d/srrg2_solver/variable_se2_distance_map.hpp>
#include <memory>

namespace testing
{

using namespace inesctec_mrdt_slam_distmap_2d;

using FactorGraphType = srrg2_solver::FactorGraph;
using FactorGraphTypePtr = std::shared_ptr<FactorGraphType>;

// using FactorType = srrg2_solver::
//     SE2DistanceMapStaticPointNormal2fVectorCloudRightPosePoseGeodesicErrorFactor;
using FactorType = srrg2_solver::
    SE2DistanceMapStaticPoint2fVectorCloudRightPosePoseGeodesicErrorFactor;
using FactorTypePtr = std::shared_ptr<FactorType>;
using InformationMatrix = FactorType::InformationMatrixType;

using VariableType = FactorType::VariableType;
using VariableTypePtr = std::shared_ptr<VariableType>;
using EstimateType = VariableType::EstimateType;

using MeasurementOwnerType = VariableType::MeasurementOwnerType;
using MeasurementType = MeasurementOwnerType::MeasurementObjType;
using PointCloudTypePtr = MeasurementOwnerType::PointCloudTypePtr;
using PointCloudType = MeasurementOwnerType::PointCloudType;
using PointType = PointCloudType::PointType;
using PointVectorType = PointType::VectorType;
using DistanceMapTypePtr = MeasurementOwnerType::DistanceMapTypePtr;
using DistanceMapType = MeasurementOwnerType::DistanceMapType;

TEST(srrg2_core_serializer, saveLoadGraph)
{
  constexpr float res = 0.01f;
  constexpr float map_size = 10.f;
  constexpr float max_dist = 1.0f;

  srrg2_solver::point_cloud_registerTypes();
  srrg2_solver::inesctec_mrdt_slam_distmap_2d_data_structures_registerTypes();
  srrg2_solver::inesctec_mrdt_slam_distmap_2d_srrg2_solver_registerTypes();

  std::string filename = std::string(__FILE__) + ".json";

  FactorGraphTypePtr graph = std::make_shared<FactorGraphType>();

  VariableTypePtr v1 = std::make_shared<VariableType>();

  v1->setMeasurement(std::make_shared<MeasurementType>());
  v1->setSensorInRobot(EstimateType::Identity());
  v1->setEstimate(EstimateType::Identity());
  v1->setStatus(srrg2_solver::VariableBase::Status::Fixed);

  v1->measurement()->m_sensor_in_robot = EstimateType::Identity();
  v1->measurement()->m_pc = std::make_shared<PointCloudType>();
  v1->measurement()->m_pc->resize(1);
  v1->measurement()->m_pc->at(0).coordinates().setZero();
  // v1->measurement()->m_pc->at(0).normal() = PointVectorType::Ones();
  v1->measurement()->m_dist_map->setGoals(*v1->measurement()->m_pc, res,
                                          map_size, max_dist);

  graph->addVariable(v1);

  VariableTypePtr v2 = std::make_shared<VariableType>();

  v2->setMeasurement(std::make_shared<MeasurementType>());
  v2->setSensorInRobot(EstimateType::Identity());
  v2->setEstimate(EstimateType::Identity());
  v2->setStatus(srrg2_solver::VariableBase::Status::Fixed);

  v2->measurement()->m_sensor_in_robot = EstimateType::Identity();
  v2->measurement()->m_pc = std::make_shared<PointCloudType>();
  v2->measurement()->m_pc->resize(1);
  v2->measurement()->m_pc->at(0).coordinates().setZero();
  // v2->measurement()->m_pc->at(0).normal() = PointVectorType::Ones();
  v2->measurement()->m_dist_map = std::make_shared<DistanceMapType>();
  v2->measurement()->m_dist_map->setGoals(*v2->measurement()->m_pc, res,
                                          map_size, max_dist);

  graph->addVariable(v2);

  FactorTypePtr f = std::make_shared<FactorType>();

  f->setVariableId(0, v1->graphId());
  f->setVariableId(1, v2->graphId());
  f->setMeasurement(EstimateType::Identity());

  f->setInformationMatrix(InformationMatrix::Identity());

  graph->addFactor(f);

  std::cout << "check graph to serialize..." << std::endl;
  std::cout << "- variables:" << std::endl;

  for (auto var : graph->variables())
  {
    const VariableType* v = static_cast<const VariableType*>(var.second);
    std::cout << "  - [" << v->graphId() << "] "
              << geometry::t2v(v->estimate()).transpose() << std::endl;
  }
  std::cout << "- factors:" << std::endl;
  for (auto fact : graph->factors())
  {
    const FactorType* f = static_cast<const FactorType*>(fact.second);
    std::cout << "  - [" << f->variable(0)->graphId() << ","
              << f->variable(1)->graphId() << "] "
              << geometry::t2v(f->measurement()).transpose() << std::endl;
  }

  std::cout << "writing graph to file " << filename << " ..." << std::endl;

  graph->write(filename);

  graph.reset();

  ASSERT_FALSE(graph);

  graph = srrg2_solver::FactorGraph::read(filename);

  EXPECT_EQ(graph->variables().size(), 2);
  EXPECT_EQ(graph->factors().size(), 1);

  std::cout << "- variables:" << std::endl;
  for (auto var : graph->variables())
  {
    const VariableType* v = static_cast<const VariableType*>(var.second);
    std::cout << "  - [" << v->graphId() << "] "
              << geometry::t2v(v->estimate()).transpose() << std::endl;

    ASSERT_TRUE(v->measurement());
    ASSERT_TRUE(v->measurement()->m_pc);
    ASSERT_TRUE(v->measurement()->m_dist_map);

    EXPECT_EQ(v->measurement()->m_pc->size(), 1);
    EXPECT_FLOAT_EQ(v->measurement()->m_dist_map->getRes(), res);
    EXPECT_FLOAT_EQ(v->measurement()->m_dist_map->getMapSize(), map_size);
    EXPECT_FLOAT_EQ(v->measurement()->m_dist_map->getMaxDist(), max_dist);

    EXPECT_TRUE(
        v->measurement()->m_sensor_in_robot.isApprox(EstimateType::Identity()));
    EXPECT_TRUE(v->estimate().isApprox(EstimateType::Identity()));

    /* data_structures::RGBImage idx_img;

    v->measurement()->m_dist_map->toImageIdx(idx_img);

    cv::imshow("setGoalsSimple: index", idx_img);
    cv::waitKey(0); */
  }
  std::cout << "- factors:" << std::endl;
  for (auto fact : graph->factors())
  {
    const FactorType* f = static_cast<const FactorType*>(fact.second);
    std::cout << "  - [" << f->variable(0)->graphId() << ","
              << f->variable(1)->graphId() << "] "
              << geometry::t2v(f->measurement()).transpose() << std::endl;

    EXPECT_TRUE(f->measurement().isApprox(EstimateType::Identity()));
  }

}  // TEST(srrg2_core_serializer, saveLoadGraph)

}  // namespace testing
