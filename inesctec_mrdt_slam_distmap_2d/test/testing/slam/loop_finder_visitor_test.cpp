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
#include <inesctec_mrdt_slam_distmap_2d/slam/loop_finder_visitor.hpp>
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
using VariableType = FactorType::VariableType;
using EstimateType = VariableType::EstimateType;
using InformationMatrixType = FactorType::InformationMatrixType;

using FactorGraphTypePtr = std::shared_ptr<srrg2_solver::FactorGraph>;
using VariableTypePtr = std::shared_ptr<VariableType>;
using FactorTypePtr = std::shared_ptr<FactorType>;

using FactorGraphInterfaceType = srrg2_solver::FactorGraphInterface;

using LoopFinderVisitorType = LoopFinderVisitor;

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
void createGTTrajectoryManhattan(TrajectoryType& trajectory,
                                 TrajectoryPoseIdType& trajectory_pose_idx,
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
 * @param[in,out] graph factor graph
 * @param[in] loop_range loop search radius
 */
void createGraphTrajectory(const TrajectoryType& trajectory,
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

TEST(slam_loop_finder_visitor, constructor)
{
  EXPECT_NO_FATAL_FAILURE(LoopFinderVisitor visitor);
  EXPECT_NO_FATAL_FAILURE(LoopFinderVisitor visitor(-1));
  EXPECT_NO_FATAL_FAILURE(LoopFinderVisitor visitor(5));
}  // TEST(slam_loop_finder_visitor, constructor)

TEST(slam_loop_finder_visitor, param)
{
  LoopFinderVisitor visitor_1;
  LoopFinderVisitor visitor_2(-1);
  LoopFinderVisitor visitor_3(5);

  EXPECT_TRUE(visitor_1.m_max_depth < 0);
  EXPECT_EQ(visitor_2.m_max_depth, -1);
  EXPECT_EQ(visitor_3.m_max_depth, 5);
}  // TEST(slam_loop_finder_visitor, param)

TEST(slam_loop_finder_visitor, simpleGridGraphVisitorAll)
{
  constexpr int grid_size = 5;

  TrajectoryType trajectory;
  TrajectoryPoseIdType trajectory_idx;

  createGTTrajectoryManhattan(trajectory, trajectory_idx, grid_size);

  FactorGraphTypePtr graph = std::make_shared<FactorGraphType>();

  createGraphTrajectory(trajectory, trajectory_idx, graph, 1.5f);

  std::cout << "Manhattan-like Trajectory:" << std::endl
            << "- #poses: " << graph->variables().size() << std::endl
            << "- #factors: " << graph->factors().size() << std::endl;

  LoopFinderVisitor visitor;

  ASSERT_TRUE(visitor.m_max_depth < 0);

  for (auto it : graph->variables())
  {
    srrg2_solver::VariableBase* var =
        dynamic_cast<srrg2_solver::VariableBase*>(it.second);

    visitor.clear();
    visitor.setGraph(graph.get());
    visitor.addSource(var);

    EXPECT_NO_FATAL_FAILURE(visitor.compute());
    EXPECT_EQ(visitor.getEntries().size(), grid_size * grid_size);
    EXPECT_EQ(visitor.getSources().size(), 1);
    EXPECT_EQ(visitor.getGraphSlices().size(), 1);

    for (auto& e : visitor.getEntries().getEntries())
    {
      EXPECT_EQ(e.second.m_graph_slice_idx, 0);
    }
  }

  visitor.clear();
  visitor.setGraph(graph.get());
  visitor.addSource(graph->variable(22));

  EXPECT_NO_FATAL_FAILURE(visitor.compute());
  EXPECT_EQ(visitor.getEntries().size(), grid_size * grid_size);
  EXPECT_EQ(visitor.getSources().size(), 1);
  EXPECT_EQ(visitor.getGraphSlices().size(), 1);

  for (int r = 0; r < grid_size; r++)
  {
    for (int c = 0; c < grid_size; c++)
    {
      int64_t idx = r * 10 + c;

      LoopFinderEntry* e = visitor.getEntries().at(idx);

      EXPECT_TRUE(e);

      if (e)
      {
        EXPECT_TRUE(e->m_variable != nullptr);
        EXPECT_TRUE(e->m_parent != nullptr);
        EXPECT_TRUE(e->m_cost >= 0);

        if (r == 0)
        {
          std::cout << " ";
        }

        std::cout << e->m_variable->graphId()
                  << " , p: " << e->m_parent->graphId() << " , c: " << e->m_cost
                  << " -;- ";
      }
    }
    std::cout << std::endl;
  }

  visitor.printPath(graph->variable((grid_size - 1) * 10 + (grid_size - 1)));
  std::cout << std::endl;

  visitor.printPath(graph->variable(22),
                    graph->variable((grid_size - 1) * 10 + (grid_size - 1)));
  std::cout << std::endl;

  visitor.printGraphSlices();
}  // TEST(slam_loop_finder_visitor, simpleGridGraphVisitorAll)

TEST(slam_loop_finder_visitor, simpleGridGraphVisitorAllMoreThanOneSource)
{
  constexpr int grid_size = 7;

  TrajectoryType trajectory;
  TrajectoryPoseIdType trajectory_idx;

  createGTTrajectoryManhattan(trajectory, trajectory_idx, grid_size);

  FactorGraphTypePtr graph = std::make_shared<FactorGraphType>();

  createGraphTrajectory(trajectory, trajectory_idx, graph, 1.5f);

  std::cout << "Manhattan-like Trajectory:" << std::endl
            << "- #poses: " << graph->variables().size() << std::endl
            << "- #factors: " << graph->factors().size() << std::endl;

  LoopFinderVisitor visitor;

  ASSERT_TRUE(visitor.m_max_depth < 0);

  for (auto it : graph->variables())
  {
    srrg2_solver::VariableBase* var =
        dynamic_cast<srrg2_solver::VariableBase*>(it.second);

    visitor.clear();
    visitor.setGraph(graph.get());
    visitor.addSource(var);

    EXPECT_NO_FATAL_FAILURE(visitor.compute());
    EXPECT_EQ(visitor.getEntries().size(), grid_size * grid_size);
    EXPECT_EQ(visitor.getSources().size(), 1);
    EXPECT_EQ(visitor.getGraphSlices().size(), 1);

    for (auto& e : visitor.getEntries().getEntries())
    {
      EXPECT_EQ(e.second.m_graph_slice_idx, 0);
    }
  }

  visitor.clear();
  visitor.setGraph(graph.get());
  visitor.addSource(graph->variable(22));
  visitor.addSource(graph->variable(44));

  EXPECT_NO_FATAL_FAILURE(visitor.compute());
  EXPECT_EQ(visitor.getEntries().size(), grid_size * grid_size);
  EXPECT_EQ(visitor.getSources().size(), 2);
  EXPECT_EQ(visitor.getGraphSlices().size(), 1);

  for (int r = 0; r < grid_size; r++)
  {
    for (int c = 0; c < grid_size; c++)
    {
      int64_t idx = r * 10 + c;

      LoopFinderEntry* e = visitor.getEntries().at(idx);

      EXPECT_TRUE(e);

      if (e)
      {
        EXPECT_TRUE(e->m_variable != nullptr);
        EXPECT_TRUE(e->m_parent != nullptr);
        EXPECT_TRUE(e->m_cost >= 0);

        if (r == 0)
        {
          std::cout << " ";
        }

        std::cout << e->m_variable->graphId()
                  << " , p: " << e->m_parent->graphId() << " , c: " << e->m_cost
                  << " -;- ";
      }
    }
    std::cout << std::endl;
  }

  visitor.printPath(graph->variable((grid_size - 1) * 10 + (grid_size - 1)));
  std::cout << std::endl;

  visitor.printPath(graph->variable(22),
                    graph->variable((grid_size - 1) * 10 + (grid_size - 1)));
  std::cout << std::endl;

  visitor.printPath(graph->variable(44),
                    graph->variable((grid_size - 1) * 10 + (grid_size - 1)));
  std::cout << std::endl;

  visitor.printGraphSlices();
}  // TEST(slam_loop_finder_visitor, simpleGridGraphVisitorAllMoreThanOneSource)

TEST(slam_loop_finder_visitor, simpleGridGraphVisitorMaxDepth)
{
  constexpr int grid_size = 9;

  TrajectoryType trajectory;
  TrajectoryPoseIdType trajectory_idx;

  createGTTrajectoryManhattan(trajectory, trajectory_idx, grid_size);

  FactorGraphTypePtr graph = std::make_shared<FactorGraphType>();

  createGraphTrajectory(trajectory, trajectory_idx, graph, 1.5f);

  std::cout << "Manhattan-like Trajectory:" << std::endl
            << "- #poses: " << graph->variables().size() << std::endl
            << "- #factors: " << graph->factors().size() << std::endl;

  LoopFinderVisitor visitor(2);

  ASSERT_EQ(visitor.m_max_depth, 2);

  for (auto it : graph->variables())
  {
    srrg2_solver::VariableBase* var =
        dynamic_cast<srrg2_solver::VariableBase*>(it.second);

    visitor.clear();
    visitor.setGraph(graph.get());
    visitor.addSource(var);

    EXPECT_NO_FATAL_FAILURE(visitor.compute());
    EXPECT_TRUE(static_cast<int>(visitor.getEntries().size()) <=
                (visitor.m_max_depth * 2 + 1) * (visitor.m_max_depth * 2 + 1));
    EXPECT_EQ(visitor.getGraphSlices().size(), 1);

    for (auto& e : visitor.getEntries().getEntries())
    {
      EXPECT_EQ(e.second.m_graph_slice_idx, 0);
    }
  }

  visitor.clear();
  visitor.setGraph(graph.get());
  visitor.addSource(graph->variable(33));
  visitor.addSource(graph->variable(88));

  EXPECT_NO_FATAL_FAILURE(visitor.compute());
  EXPECT_EQ(visitor.getGraphSlices().size(), 2);
  EXPECT_EQ(visitor.getSources().size(), 2);

  for (int r = 0; r < grid_size; r++)
  {
    for (int c = 0; c < grid_size; c++)
    {
      int64_t idx = r * 10 + c;

      LoopFinderEntry* e = visitor.getEntries().at(idx);

      if (e)
      {
        EXPECT_TRUE(e->m_variable != nullptr);
        EXPECT_TRUE(e->m_parent != nullptr);
        EXPECT_TRUE(e->m_cost >= 0);

        if (r == 0)
        {
          std::cout << " ";
        }

        std::cout << e->m_variable->graphId()
                  << " , p: " << e->m_parent->graphId() << " , c: " << e->m_cost
                  << " -;- ";
      }
      else
      {
        std::cout << r * 10 + c << " , p: NA , c: NA -;- ";
      }
    }
    std::cout << std::endl;
  }

  // visitor.printPath(graph->variable((grid_size - 1) * 10 + (grid_size - 1)));
  // std::cout << std::endl;

  // visitor.printPath(graph->variable(22),
  //                   graph->variable((grid_size - 1) * 10 + (grid_size - 1)));
  // std::cout << std::endl;

  // visitor.printPath(graph->variable(44),
  //                   graph->variable((grid_size - 1) * 10 + (grid_size - 1)));
  // std::cout << std::endl;

  visitor.printGraphSlices();
}  // TEST(slam_loop_finder_visitor, simpleGridGraphVisitorMaxDepth)

TEST(slam_loop_finder_visitor, simpleGridGraphVisitorAllWithTarget)
{
  constexpr int grid_size = 9;

  TrajectoryType trajectory;
  TrajectoryPoseIdType trajectory_idx;

  createGTTrajectoryManhattan(trajectory, trajectory_idx, grid_size);

  FactorGraphTypePtr graph = std::make_shared<FactorGraphType>();

  createGraphTrajectory(trajectory, trajectory_idx, graph, 1.5f);

  std::cout << "Manhattan-like Trajectory:" << std::endl
            << "- #poses: " << graph->variables().size() << std::endl
            << "- #factors: " << graph->factors().size() << std::endl;

  LoopFinderVisitor visitor;

  ASSERT_TRUE(visitor.m_max_depth < 0);

  visitor.clear();
  visitor.setGraph(graph.get());
  visitor.addSource(graph->variable(86));
  visitor.addSource(graph->variable(68));
  visitor.addTarget(graph->variable(34));

  EXPECT_NO_FATAL_FAILURE(visitor.compute());
  EXPECT_NE(visitor.getEntries().size(), grid_size * grid_size);
  // target definition should stop the expansion of the graph visitor!!!...
  EXPECT_EQ(visitor.getSources().size(), 2);
  EXPECT_EQ(visitor.getTargets().size(), 1);

  for (int r = 0; r < grid_size; r++)
  {
    for (int c = 0; c < grid_size; c++)
    {
      int64_t idx = r * 10 + c;

      LoopFinderEntry* e = visitor.getEntries().at(idx);

      if (e)
      {
        EXPECT_TRUE(e->m_variable != nullptr);
        EXPECT_TRUE(e->m_parent != nullptr);
        EXPECT_TRUE(e->m_cost >= 0);

        if (r == 0)
        {
          std::cout << " ";
        }

        std::cout << e->m_variable->graphId()
                  << " , p: " << e->m_parent->graphId() << " , c: " << e->m_cost
                  << " -;- ";
      }
      else
      {
        std::cout << r * 10 + c << " , p: NA , c: NA -;- ";
      }
    }
    std::cout << std::endl;
  }
}  // TEST(slam_loop_finder_visitor, simpleGridGraphVisitorAllWithTarget)

}  // namespace testing
