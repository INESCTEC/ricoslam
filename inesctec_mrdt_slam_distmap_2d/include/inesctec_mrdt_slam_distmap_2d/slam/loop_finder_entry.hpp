#pragma once

#include <srrg2_solver/solver_core/variable.h>

#include <limits>
#include <queue>

namespace inesctec_mrdt_slam_distmap_2d
{

namespace slam
{

/**
 * @brief graph visitor entry for loop finder (expansion from source nodes =
 *        loop closure hints until the expansion length is greater than a
 *        certain threshold, slices identification upon expansion)
 */
struct LoopFinderEntry
{
 public:

  using SliceIdxType = int32_t;
  using CostType = int;

 public:

  srrg2_solver::VariableBase* m_variable = nullptr;  //!< graph variable
  srrg2_solver::VariableBase* m_parent =
      nullptr;  //!< variable from which current cost was computed

  SliceIdxType m_graph_slice_idx = -1;  //!< graph slice index (-1 not assigned)

  CostType m_cost = std::numeric_limits<CostType>::max();  //!< current cost

 public:

  LoopFinderEntry()
      : m_variable(nullptr),
        m_parent(nullptr),
        m_graph_slice_idx(-1),
        m_cost(std::numeric_limits<CostType>::max())
  {
  }  // LoopFinderEntry()

  LoopFinderEntry(srrg2_solver::VariableBase* variable)
      : m_variable(variable),
        m_parent(nullptr),
        m_graph_slice_idx(-1),
        m_cost(std::numeric_limits<CostType>::max())
  {
  }  // LoopFinderEntry(srrg2_solver::VariableBase*)

  /**
   * @brief operator< definition for priority queues
   *        (current implementation prioritizes lower costs and older entries,
   *        the latter for possibly longer loops)
   * @param[in] other other loop finder entry
   * @return true other entry has lower cost or is an older graph entry
   *              (assuming lower graph IDs as older entries)
   * @return false otherwise
   */
  inline bool operator<(const LoopFinderEntry& other) const
  {
    if (this->m_cost > other.m_cost)
    {
      return true;
    }
    if (this->m_cost < other.m_cost)
    {
      return false;
    }
    return (this->m_variable->graphId() > other.m_variable->graphId());
  }  // inline bool operator<(const LoopFinderEntry&) const
};  // struct LoopFinderEntry

using LoopFinderEntryQueue = std::priority_queue<LoopFinderEntry>;

}  // namespace slam

}  // namespace inesctec_mrdt_slam_distmap_2d
