#include <gtest/gtest.h>
#include <srrg2_solver/solver_core/factor.h>
#include <srrg2_solver/solver_core/factor_graph.h>
#include <srrg2_solver/solver_core/variable.h>
#include <srrg2_solver/variables_and_factors/types_2d/variable_se2.h>

#include <inesctec_mrdt_slam_distmap_2d/slam/loop_finder_entry_container.hpp>

namespace testing
{

using namespace inesctec_mrdt_slam_distmap_2d;
using namespace slam;

TEST(slam_loop_finder_entry_container, add)
{
  srrg2_solver::VariableSE2Right v_1;
  srrg2_solver::VariableSE2Right v_2;
  srrg2_solver::VariableSE2Right v_3;

  v_1.setGraphId(1);
  v_2.setGraphId(2);
  v_3.setGraphId(3);

  LoopFinderEntryContainer entries;

  EXPECT_EQ(entries.size(), 0);

  entries.add(LoopFinderEntry(&v_1));
  entries.add(LoopFinderEntry(&v_2));
  entries.add(LoopFinderEntry(&v_3));

  EXPECT_EQ(entries.size(), 3);

  entries.add(LoopFinderEntry(&v_3));
  entries.add(LoopFinderEntry(&v_3));

  EXPECT_EQ(entries.size(), 3);

  entries.clear();

  EXPECT_EQ(entries.size(), 0);
}  // TEST(slam_loop_finder_entry_container, add)

TEST(slam_loop_finder_entry_container, at)
{
  srrg2_solver::VariableSE2Right v_1;
  srrg2_solver::VariableSE2Right v_2;
  srrg2_solver::VariableSE2Right v_3;

  v_1.setGraphId(1);
  v_2.setGraphId(2);
  v_3.setGraphId(3);

  LoopFinderEntryContainer entries;

  EXPECT_EQ(entries.size(), 0);
  EXPECT_FALSE(entries.at(v_1.graphId()));
  EXPECT_FALSE(entries.at(v_2.graphId()));
  EXPECT_FALSE(entries.at(v_3.graphId()));

  entries.add(LoopFinderEntry(&v_1));
  EXPECT_TRUE(entries.at(v_1.graphId()));
  EXPECT_FALSE(entries.at(v_2.graphId()));
  EXPECT_FALSE(entries.at(v_3.graphId()));
  EXPECT_EQ(entries.at(v_1.graphId())->m_variable->graphId(), v_1.graphId());
  EXPECT_EQ(entries.at(v_1.graphId())->m_variable, &v_1);

  entries.add(LoopFinderEntry(&v_2));
  EXPECT_TRUE(entries.at(v_1.graphId()));
  EXPECT_TRUE(entries.at(v_2.graphId()));
  EXPECT_FALSE(entries.at(v_3.graphId()));
  EXPECT_EQ(entries.at(v_1.graphId())->m_variable->graphId(), v_1.graphId());
  EXPECT_EQ(entries.at(v_2.graphId())->m_variable->graphId(), v_2.graphId());
  EXPECT_EQ(entries.at(v_1.graphId())->m_variable, &v_1);
  EXPECT_EQ(entries.at(v_2.graphId())->m_variable, &v_2);

  entries.add(LoopFinderEntry(&v_3));
  EXPECT_TRUE(entries.at(v_1.graphId()));
  EXPECT_TRUE(entries.at(v_2.graphId()));
  EXPECT_TRUE(entries.at(v_3.graphId()));
  EXPECT_EQ(entries.at(v_1.graphId())->m_variable->graphId(), v_1.graphId());
  EXPECT_EQ(entries.at(v_2.graphId())->m_variable->graphId(), v_2.graphId());
  EXPECT_EQ(entries.at(v_3.graphId())->m_variable->graphId(), v_3.graphId());
  EXPECT_EQ(entries.at(v_1.graphId())->m_variable, &v_1);
  EXPECT_EQ(entries.at(v_2.graphId())->m_variable, &v_2);
  EXPECT_EQ(entries.at(v_3.graphId())->m_variable, &v_3);

  EXPECT_EQ(entries.size(), 3);

  entries.add(LoopFinderEntry(&v_3));
  entries.add(LoopFinderEntry(&v_3));

  EXPECT_EQ(entries.size(), 3);
  EXPECT_TRUE(entries.at(v_1.graphId()));
  EXPECT_TRUE(entries.at(v_2.graphId()));
  EXPECT_TRUE(entries.at(v_3.graphId()));
  EXPECT_EQ(entries.at(v_1.graphId())->m_variable->graphId(), v_1.graphId());
  EXPECT_EQ(entries.at(v_2.graphId())->m_variable->graphId(), v_2.graphId());
  EXPECT_EQ(entries.at(v_3.graphId())->m_variable->graphId(), v_3.graphId());
  EXPECT_EQ(entries.at(v_1.graphId())->m_variable, &v_1);
  EXPECT_EQ(entries.at(v_2.graphId())->m_variable, &v_2);
  EXPECT_EQ(entries.at(v_3.graphId())->m_variable, &v_3);

  entries.clear();

  EXPECT_EQ(entries.size(), 0);
}  // TEST(slam_loop_finder_entry_container, at)

TEST(slam_loop_finder_entry_container, isInit)
{
  srrg2_solver::VariableSE2Right v_1;
  srrg2_solver::VariableSE2Right v_2;
  srrg2_solver::VariableSE2Right v_3;

  v_1.setGraphId(1);
  v_2.setGraphId(2);
  v_3.setGraphId(3);

  LoopFinderEntryContainer entries;

  EXPECT_EQ(entries.size(), 0);
  EXPECT_FALSE(entries.isInit(v_1.graphId()));
  EXPECT_FALSE(entries.isInit(v_2.graphId()));
  EXPECT_FALSE(entries.isInit(v_3.graphId()));

  entries.add(LoopFinderEntry(&v_1));
  EXPECT_TRUE(entries.isInit(v_1.graphId()));
  EXPECT_FALSE(entries.isInit(v_2.graphId()));
  EXPECT_FALSE(entries.isInit(v_3.graphId()));

  entries.add(LoopFinderEntry(&v_2));
  EXPECT_TRUE(entries.isInit(v_1.graphId()));
  EXPECT_TRUE(entries.isInit(v_2.graphId()));
  EXPECT_FALSE(entries.isInit(v_3.graphId()));

  entries.add(LoopFinderEntry(&v_3));
  EXPECT_TRUE(entries.isInit(v_1.graphId()));
  EXPECT_TRUE(entries.isInit(v_2.graphId()));
  EXPECT_TRUE(entries.isInit(v_3.graphId()));

  EXPECT_EQ(entries.size(), 3);

  entries.add(LoopFinderEntry(&v_3));
  entries.add(LoopFinderEntry(&v_3));

  EXPECT_EQ(entries.size(), 3);

  EXPECT_TRUE(entries.isInit(v_1.graphId()));
  EXPECT_TRUE(entries.isInit(v_2.graphId()));
  EXPECT_TRUE(entries.isInit(v_3.graphId()));

  entries.clear();

  EXPECT_EQ(entries.size(), 0);
  EXPECT_FALSE(entries.isInit(v_1.graphId()));
  EXPECT_FALSE(entries.isInit(v_2.graphId()));
  EXPECT_FALSE(entries.isInit(v_3.graphId()));
}  // TEST(slam_loop_finder_entry_container, isInit)

}  // namespace testing