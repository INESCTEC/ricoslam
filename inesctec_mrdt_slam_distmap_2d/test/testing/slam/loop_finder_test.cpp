#include <gtest/gtest.h>
#include <srrg2_core/srrg_geometry/geometry2d.h>
#include <srrg2_solver/solver_core/factor_graph.h>
#include <srrg2_solver/solver_core/instances.h>
#include <srrg2_solver/solver_core/internals/linear_solvers/instances.h>
#include <srrg2_solver/solver_core/solver.h>
#include <srrg2_solver/variables_and_factors/types_2d/instances.h>

#include <Eigen/StdVector>
#include <exception>
#include <inesctec_mrdt_slam_distmap_2d/geometry/geometry_2d.hpp>
#include <inesctec_mrdt_slam_distmap_2d/slam/loop_finder.hpp>
#include <inesctec_mrdt_slam_distmap_2d/srrg2_solver/instances.hpp>
#include <inesctec_mrdt_slam_distmap_2d/srrg2_solver/se2_distance_map_pose_pose_geodesic_error_factor.hpp>
#include <inesctec_mrdt_slam_distmap_2d/srrg2_solver/variable_se2_distance_map.hpp>
#include <vector>

namespace testing
{

using namespace inesctec_mrdt_slam_distmap_2d;
using namespace slam;

using FactorGraphType = srrg2_solver::FactorGraph;
using FactorType = srrg2_solver::
    SE2DistanceMapStaticPoint2fVectorCloudRightPosePoseGeodesicErrorFactor;
using InformationMatrix = FactorType::InformationMatrixType;

using VariableType = FactorType::VariableType;
using EstimateType = VariableType::EstimateType;

using FactorGraphTypePtr = std::shared_ptr<srrg2_solver::FactorGraph>;
using VariableTypePtr = std::shared_ptr<VariableType>;
using FactorTypePtr = std::shared_ptr<FactorType>;

using FactorGraphInterfaceType = srrg2_solver::FactorGraphInterface;

using TrajectoryType =
    std::vector<EstimateType, Eigen::aligned_allocator<EstimateType>>;
using TrajectoryPoseIdType = std::vector<int>;

/**
 * @brief creaate a full-dense grid manhattan-like trajectory
 * @param[out] trajectory sequence of Isometry SE(2) poses
 * @param[out] trajectory_pose_idx pose idx to easy graph's variables access
 * @param[in] grid_size [ -grid_size , grid_size ] , [ -grid_size , grid_size ]
 *                      (must be lower than 9)
 */
void createGTTrajectoryManhattanFinder(
    TrajectoryType& trajectory, TrajectoryPoseIdType& trajectory_pose_idx,
    int grid_size)
{
  trajectory.clear();
  trajectory_pose_idx.clear();

  if (grid_size > 10)
  {
    throw std::invalid_argument(
        "createGTTrajectoryManhattan | grid_size must be lower or equal to 10 "
        "for this unit test...");
  }

  for (int r = 0; r < grid_size; r++)
  {
    for (int c = 0; c < grid_size; c++)
    {
      EstimateType pose;

      pose.setIdentity();
      pose.translation().x() = r;
      pose.translation().y() = c;
      pose.linear().setIdentity();

      trajectory.emplace_back(pose);
      trajectory_pose_idx.emplace_back(r * 10 + c);
    }
  }
}  // void createGTTrajectoryManhattan(TrajectoryType&, int)

/**
 * @brief create a graph by adding factors if a pair of variables are within an
 *        certain range / distance
 * @param[in] trajectory sequence of Isometry SE(2) poses
 * @param[in] trajectory_pose_idx pose idx to easy graph's variables access
 * @param[in] grid_size [ -grid_size , grid_size ] , [ -grid_size , grid_size ]
 *                      (must be lower than 9)
 * @param[in,out] graph factor graph
 * @param[in] loop_range loop search radius
 */
void createGraphTrajectoryFinder(
    const TrajectoryType& trajectory,
    const TrajectoryPoseIdType& trajectory_pose_idx,
    const FactorGraphTypePtr& graph, float loop_range)
{
  for (size_t idx = 0; idx < trajectory.size(); idx++)
  {
    VariableTypePtr v = std::make_shared<VariableType>();

    v->setGraphId(trajectory_pose_idx[idx]);
    v->setEstimate(trajectory[idx]);
    v->setStatus(VariableType::Status::Fixed);

    graph->addVariable(v);
  }

  FactorGraphInterfaceType* interface =
      dynamic_cast<FactorGraphInterfaceType*>(graph.get());

  for (const auto& v1 : graph->variables())
  {
    for (const auto& v2 : graph->variables())
    {
      if (v1.second->graphId() == v2.second->graphId())
      {
        continue;
      }

      auto does_f_v1_v2_exist = [=]() -> bool
      {
        for (auto f : interface->factors(v1.second))
        {
          for (int idx = 0; idx < f->numVariables(); ++idx)
          {
            if (f->variable(idx)->graphId() == v2.second->graphId())
            {
              return true;
              break;
            }
          }
        }
        return false;
      };

      if (does_f_v1_v2_exist())
      {
        continue;
      }

      if ((static_cast<VariableType*>(v1.second)->estimate().translation() -
           static_cast<VariableType*>(v2.second)->estimate().translation())
              .norm() > loop_range)
      {
        continue;
      }

      FactorTypePtr f = std::make_shared<FactorType>();

      f->setVariableId(0, v1.second->graphId());
      f->setVariableId(1, v2.second->graphId());

      graph->addFactor(f);
    }
  }
}  // void createGraphTrajectory(const TrajectoryType& trajectory, const
   // TrajectoryPoseIdType&, const FactorGraphTypePtr&, float)

/**
 * @brief loop finder SLAM test class (mock object let's say)
 */
class LoopFinderTestSLAM
{
 public:

  using FactorGraphType = srrg2_solver::FactorGraph;
  using FactorGraphTypePtr = std::shared_ptr<FactorGraphType>;

  using FactorType = srrg2_solver::
      SE2DistanceMapStaticPoint2fVectorCloudRightPosePoseGeodesicErrorFactor;
  using FactorInformationMatrix = typename FactorType::InformationMatrixType;

  using VariableType = typename FactorType::VariableType;
  using EstimateType = typename VariableType::EstimateType;

 protected:

  FactorGraphTypePtr m_graph_ = std::make_shared<FactorGraphType>();
  VariableType* m_local_map_;
  EstimateType m_robot_in_map_;
  EstimateType m_robot_in_local_map_;

 public:

  inline srrg2_solver::FactorGraphInterface* getGraph() const
  {
    return m_graph_.get();
  }
  inline const FactorGraphTypePtr& getGraphPtr() const { return m_graph_; }
  inline VariableType* getLocalMap() const { return m_local_map_; }
  inline VariableType* getKeyframe() const { return m_local_map_; }
  inline const EstimateType& getRobotInMap() const { return m_robot_in_map_; }
  inline const EstimateType& getRobotInLocalMap() const
  {
    return m_robot_in_local_map_;
  }
  inline const LoopFinderTestSLAM* getTracker() const { return this; }

  inline void setLocalMap(VariableType* local_map) { m_local_map_ = local_map; }
  inline void setRobotInMap(const EstimateType& robot_in_map)
  {
    m_robot_in_map_ = robot_in_map;
  }
  inline void setRobotInLocalMap(const EstimateType& robot_in_local_map)
  {
    m_robot_in_local_map_ = robot_in_local_map;
  }
};  // class LoopFinderTestSLAM

using LoopFinderTest = LoopFinder_<LoopFinderTestSLAM>;
using LoopFinderTestParam = LoopFinderTest::Param;
using LoopHintType = LoopFinderTest::Hint;
using LoopHintTypePtr = LoopFinderTest::HintPtr;

TEST(slam_loop_finder, hintComparatorClosest)
{
  using HintPtrSetComparator = LoopFinderTest::HintPtrComparatorClosest;
  using HintPtrSetType = LoopFinderTest::HintPtrSet_<HintPtrSetComparator>;

  HintPtrSetType loop_hints_container;

  VariableType v_1;
  VariableType v_2;
  VariableType v_3;

  v_1.setGraphId(1);
  v_2.setGraphId(2);
  v_3.setGraphId(3);

  LoopHintTypePtr hint_1 = std::make_shared<LoopHintType>(
      &v_1, geometry::v2t(Eigen::Vector3f(0.1f, 0.1f, 0.0f)),
      InformationMatrix::Identity(), 5, 0.3f, 0.7f);
  LoopHintTypePtr hint_2 = std::make_shared<LoopHintType>(
      &v_2, geometry::v2t(Eigen::Vector3f(0.4f, 0.4f, 0.0f)),
      InformationMatrix::Identity(), 5, 0.3f, 0.7f);
  LoopHintTypePtr hint_3 = std::make_shared<LoopHintType>(
      &v_3, geometry::v2t(Eigen::Vector3f(0.25f, 0.25f, 0.0f)),
      InformationMatrix::Identity(), 5, 0.3f, 0.7f);

  loop_hints_container.insert(hint_1);
  loop_hints_container.insert(hint_2);
  loop_hints_container.insert(hint_3);

  auto it = loop_hints_container.begin();

  EXPECT_TRUE((*it++) == hint_1);
  EXPECT_TRUE((*it++) == hint_3);
  EXPECT_TRUE((*it++) == hint_2);
}  // TEST(slam_loop_finder, hintComparatorClosest)

TEST(slam_loop_finder, hintComparatorLongerLength)
{
  using HintPtrSetComparator = LoopFinderTest::HintPtrComparatorLongerLength;
  using HintPtrSetType = LoopFinderTest::HintPtrSet_<HintPtrSetComparator>;

  HintPtrSetType loop_hints_container;

  VariableType v_1;
  VariableType v_2;
  VariableType v_3;
  VariableType v_4;

  v_1.setGraphId(1);
  v_2.setGraphId(2);
  v_3.setGraphId(3);
  v_4.setGraphId(4);

  LoopHintTypePtr hint_1 = std::make_shared<LoopHintType>(
      &v_1, geometry::v2t(Eigen::Vector3f(0.1f, 0.1f, 0.0f)),
      InformationMatrix::Identity(), 7, 0.3f, 0.7f);
  LoopHintTypePtr hint_2 = std::make_shared<LoopHintType>(
      &v_2, geometry::v2t(Eigen::Vector3f(0.1f, 0.1f, 0.0f)),
      InformationMatrix::Identity(), 3, 0.3f, 0.7f);
  LoopHintTypePtr hint_3 = std::make_shared<LoopHintType>(
      &v_3, geometry::v2t(Eigen::Vector3f(0.5f, 0.5f, 0.0f)),
      InformationMatrix::Identity(), 4, 0.3f, 0.7f);
  LoopHintTypePtr hint_4 = std::make_shared<LoopHintType>(
      &v_4, geometry::v2t(Eigen::Vector3f(0.4f, 0.4f, 0.0f)),
      InformationMatrix::Identity(), 4, 0.3f, 0.7f);

  loop_hints_container.insert(hint_1);
  loop_hints_container.insert(hint_2);
  loop_hints_container.insert(hint_3);
  loop_hints_container.insert(hint_4);

  auto it = loop_hints_container.begin();

  EXPECT_TRUE((*it++) == hint_1);
  EXPECT_TRUE((*it++) == hint_4);
  EXPECT_TRUE((*it++) == hint_3);
  EXPECT_TRUE((*it++) == hint_2);
}  // TEST(slam_loop_finder, hintComparatorLongerLength)

TEST(slam_loop_finder, hintComparatorLowerChiInliers)
{
  using HintPtrSetComparator = LoopFinderTest::HintPtrComparatorLowerChiInliers;
  using HintPtrSetType = LoopFinderTest::HintPtrSet_<HintPtrSetComparator>;

  HintPtrSetType loop_hints_container;

  VariableType v_1;
  VariableType v_2;
  VariableType v_3;

  v_1.setGraphId(1);
  v_2.setGraphId(2);
  v_3.setGraphId(3);

  LoopHintTypePtr hint_1 = std::make_shared<LoopHintType>(
      &v_1, geometry::v2t(Eigen::Vector3f(0.1f, 0.1f, 0.0f)),
      InformationMatrix::Identity(), 5, 0.3f, 0.7f);
  LoopHintTypePtr hint_2 = std::make_shared<LoopHintType>(
      &v_2, geometry::v2t(Eigen::Vector3f(0.1f, 0.1f, 0.0f)),
      InformationMatrix::Identity(), 5, 0.2f, 0.7f);
  LoopHintTypePtr hint_3 = std::make_shared<LoopHintType>(
      &v_3, geometry::v2t(Eigen::Vector3f(0.1f, 0.1f, 0.0f)),
      InformationMatrix::Identity(), 5, 0.1f, 0.7f);

  loop_hints_container.insert(hint_1);
  loop_hints_container.insert(hint_2);
  loop_hints_container.insert(hint_3);

  auto it = loop_hints_container.begin();

  EXPECT_TRUE((*it++) == hint_3);
  EXPECT_TRUE((*it++) == hint_2);
  EXPECT_TRUE((*it++) == hint_1);
}  // TEST(slam_loop_finder, hintComparatorLowerChiInliers)

TEST(slam_loop_finder, hintComparatorHigherInliersRatio)
{
  using HintPtrSetComparator =
      LoopFinderTest::HintPtrComparatorHigherInliersRatio;
  using HintPtrSetType = LoopFinderTest::HintPtrSet_<HintPtrSetComparator>;

  HintPtrSetType loop_hints_container;

  VariableType v_1;
  VariableType v_2;
  VariableType v_3;

  v_1.setGraphId(1);
  v_2.setGraphId(2);
  v_3.setGraphId(3);

  LoopHintTypePtr hint_1 = std::make_shared<LoopHintType>(
      &v_1, geometry::v2t(Eigen::Vector3f(0.1f, 0.1f, 0.0f)),
      InformationMatrix::Identity(), 5, 0.3f, 0.80f);
  LoopHintTypePtr hint_2 = std::make_shared<LoopHintType>(
      &v_2, geometry::v2t(Eigen::Vector3f(0.1f, 0.1f, 0.0f)),
      InformationMatrix::Identity(), 5, 0.3f, 0.99f);
  LoopHintTypePtr hint_3 = std::make_shared<LoopHintType>(
      &v_3, geometry::v2t(Eigen::Vector3f(0.1f, 0.1f, 0.0f)),
      InformationMatrix::Identity(), 5, 0.3f, 0.7f);

  loop_hints_container.insert(hint_1);
  loop_hints_container.insert(hint_2);
  loop_hints_container.insert(hint_3);

  auto it = loop_hints_container.begin();

  EXPECT_TRUE((*it++) == hint_2);
  EXPECT_TRUE((*it++) == hint_1);
  EXPECT_TRUE((*it++) == hint_3);
}  // TEST(slam_loop_finder, hintComparatorHigherInliersRatio)

TEST(slam_loop_finder, constructor)
{
  ASSERT_NO_THROW(LoopFinderTest loop_finder);
  LoopFinderTest loop_finder;

  EXPECT_NO_THROW(LoopFinderTest loop_finder(LoopFinderTestParam(1, 2.5)));
  EXPECT_NO_THROW(loop_finder.setParam(LoopFinderTestParam(1, 2.5)));
  EXPECT_NO_THROW(LoopFinderTest loop_finder(LoopFinderTestParam(5, 2.5)));
  EXPECT_NO_THROW(loop_finder.setParam(LoopFinderTestParam(5, 2.5)));
  EXPECT_ANY_THROW(LoopFinderTest loop_finder(LoopFinderTestParam(0, 2.5)));
  EXPECT_ANY_THROW(loop_finder.setParam(LoopFinderTestParam(0, 2.5)));
  EXPECT_ANY_THROW(LoopFinderTest loop_finder(LoopFinderTestParam(1, 0.f)));
  EXPECT_ANY_THROW(loop_finder.setParam(LoopFinderTestParam(1, 0.f)));
  EXPECT_ANY_THROW(LoopFinderTest loop_finder(LoopFinderTestParam(-1, -1.f)));
  EXPECT_ANY_THROW(loop_finder.setParam(LoopFinderTestParam(-1, -1.f)));
}  // TEST(slam_loop_finder, constructor)

TEST(slam_loop_finder, compute)
{
  constexpr int grid_size = 9;

  TrajectoryType trajectory;
  TrajectoryPoseIdType trajectory_idx;

  createGTTrajectoryManhattanFinder(trajectory, trajectory_idx, grid_size);

  LoopFinderTestSLAM slam;

  createGraphTrajectoryFinder(trajectory, trajectory_idx, slam.getGraphPtr(),
                              1.5f);

  LoopFinderTest loop_finder(LoopFinderTestParam(1, 2.9));

  slam.setLocalMap(dynamic_cast<VariableType*>(slam.getGraph()->variable(44)));
  slam.setRobotInMap(slam.getLocalMap()->estimate());
  slam.setRobotInLocalMap(EstimateType::Identity());

  loop_finder.setSLAM(&slam);
  loop_finder.compute();

  EXPECT_EQ(loop_finder.getShortLoops()->size(), 24);  // 5x5 mat except id:44
  EXPECT_EQ(loop_finder.getHints()->size(), 24);       // 5x5 mat except id:44

  for (auto& short_loop : *loop_finder.getShortLoops())
  {
    EXPECT_TRUE(short_loop->m_local_map);
    EXPECT_TRUE(short_loop->m_initial_guess.translation().norm() <=
                loop_finder.getParam().m_loop_max_dist);
    EXPECT_TRUE(short_loop->m_graph_len >=
                loop_finder.getParam().m_loop_min_graph_len);
  }

  for (auto& hint : *loop_finder.getHints())
  {
    EXPECT_TRUE(hint->m_local_map);
    EXPECT_TRUE(hint->m_initial_guess.translation().norm() <=
                loop_finder.getParam().m_loop_max_dist);
    EXPECT_TRUE(hint->m_graph_len >=
                loop_finder.getParam().m_loop_min_graph_len);
  }

  loop_finder.setParam(LoopFinderTestParam(2, 2.9));

  loop_finder.setSLAM(&slam);
  loop_finder.compute();

  EXPECT_EQ(loop_finder.getShortLoops()->size(), 24);  // 5x5 mat except id:44
  EXPECT_EQ(loop_finder.getHints()->size(),
            16);  // 5x5 except inner 3x3 mat

  std::cout << "slam.setLocalMap(dynamic_cast<VariableType*>(slam.getGraph()->"
               "variable(44)))"
            << std::endl
            << "loop_finder(LoopFinderTestParam(2, 2.9))" << std::endl
            << "short loops:" << std::endl;

  for (auto& short_loop : *loop_finder.getShortLoops())
  {
    EXPECT_TRUE(short_loop->m_local_map);
    EXPECT_TRUE(short_loop->m_initial_guess.translation().norm() <=
                loop_finder.getParam().m_loop_max_dist);
    std::cout << "- (id:" << short_loop->m_local_map->graphId() << " , ["
              << geometry::t2v(short_loop->m_initial_guess).transpose()
              << "] , len:" << short_loop->m_graph_len << ")" << std::endl;
  }

  std::cout << "hints:" << std::endl;

  for (auto& hint : *loop_finder.getHints())
  {
    EXPECT_TRUE(hint->m_local_map);
    EXPECT_TRUE(hint->m_initial_guess.translation().norm() <=
                loop_finder.getParam().m_loop_max_dist);
    EXPECT_TRUE(hint->m_graph_len >=
                loop_finder.getParam().m_loop_min_graph_len);
    std::cout << "- (id:" << hint->m_local_map->graphId() << " , ["
              << geometry::t2v(hint->m_initial_guess).transpose()
              << "] , len:" << hint->m_graph_len << ")" << std::endl;
  }
}  // TEST(slam_loop_finder, compute)

}  // namespace testing
