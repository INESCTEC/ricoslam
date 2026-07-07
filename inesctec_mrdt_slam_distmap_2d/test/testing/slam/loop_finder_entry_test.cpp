#include <gtest/gtest.h>
#include <srrg2_solver/solver_core/factor.h>
#include <srrg2_solver/solver_core/factor_graph.h>
#include <srrg2_solver/solver_core/variable.h>
#include <srrg2_solver/variables_and_factors/types_2d/variable_se2.h>

#include <inesctec_mrdt_slam_distmap_2d/slam/loop_finder_entry.hpp>

namespace testing
{

using namespace inesctec_mrdt_slam_distmap_2d;
using namespace slam;

TEST(slam_loop_finder_entry, constructor)
{
  LoopFinderEntry entry_1;

  EXPECT_EQ(entry_1.m_variable, nullptr);
  EXPECT_EQ(entry_1.m_parent, nullptr);
  EXPECT_FALSE(entry_1.m_variable);
  EXPECT_FALSE(entry_1.m_parent);
  EXPECT_EQ(entry_1.m_graph_slice_idx, -1);
  EXPECT_EQ(entry_1.m_cost, std::numeric_limits<int>::max());

  srrg2_solver::VariableSE2Right variable;
  variable.setGraphId(1);

  LoopFinderEntry entry_2(&variable);

  EXPECT_EQ(entry_2.m_variable, &variable);
  EXPECT_EQ(entry_2.m_parent, nullptr);
  EXPECT_TRUE(entry_2.m_variable);
  EXPECT_FALSE(entry_2.m_parent);
  EXPECT_EQ(entry_2.m_graph_slice_idx, -1);
  EXPECT_EQ(entry_2.m_cost, std::numeric_limits<int>::max());
}  // TEST(slam_loop_finder_entry, constructor)

TEST(slam_loop_finder_entry, queue)
{
  srrg2_solver::VariableSE2Right v_1;
  srrg2_solver::VariableSE2Right v_2;
  srrg2_solver::VariableSE2Right v_3;

  v_1.setGraphId(1);
  v_2.setGraphId(2);
  v_3.setGraphId(3);

  LoopFinderEntryQueue queue;

  LoopFinderEntry entry_1(&v_1);
  LoopFinderEntry entry_2(&v_2);
  LoopFinderEntry entry_3(&v_3);

  ASSERT_EQ(entry_1.m_variable->graphId(), v_1.graphId());
  ASSERT_EQ(entry_2.m_variable->graphId(), v_2.graphId());
  ASSERT_EQ(entry_3.m_variable->graphId(), v_3.graphId());

  queue.emplace(entry_1);
  queue.emplace(entry_2);
  queue.emplace(entry_3);

  /**
   * @brief cost not initialized, should prioritize older entries in the graph
   *        (assuming a continuously growing graph ID counter...)
   */
  EXPECT_EQ(queue.top().m_variable->graphId(), entry_1.m_variable->graphId());
  queue.pop();
  EXPECT_EQ(queue.top().m_variable->graphId(), entry_2.m_variable->graphId());
  queue.pop();
  EXPECT_EQ(queue.top().m_variable->graphId(), entry_3.m_variable->graphId());
  queue.pop();

  entry_1.m_cost = 3;
  entry_2.m_cost = 4;
  entry_3.m_cost = 2;

  queue.emplace(entry_1);
  queue.emplace(entry_2);
  queue.emplace(entry_3);

  /**
   * @brief cost not initialized, should prioritize cost
   */
  EXPECT_EQ(queue.top().m_variable->graphId(), entry_3.m_variable->graphId());
  queue.pop();
  EXPECT_EQ(queue.top().m_variable->graphId(), entry_1.m_variable->graphId());
  queue.pop();
  EXPECT_EQ(queue.top().m_variable->graphId(), entry_2.m_variable->graphId());
  queue.pop();

  entry_1.m_cost = 3;
  entry_2.m_cost = 2;
  entry_3.m_cost = 3;

  queue.emplace(entry_1);
  queue.emplace(entry_2);
  queue.emplace(entry_3);

  /**
   * @brief cost not initialized, should prioritize cost and older entries
   */
  EXPECT_EQ(queue.top().m_variable->graphId(), entry_2.m_variable->graphId());
  queue.pop();
  EXPECT_EQ(queue.top().m_variable->graphId(), entry_1.m_variable->graphId());
  queue.pop();
  EXPECT_EQ(queue.top().m_variable->graphId(), entry_3.m_variable->graphId());
  queue.pop();
}  // TEST(slam_loop_finder_entry, queue)

}  // namespace testing
