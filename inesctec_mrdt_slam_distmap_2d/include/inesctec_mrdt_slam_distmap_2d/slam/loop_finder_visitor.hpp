#pragma once

#include <srrg2_solver/solver_core/factor_graph_interface.h>

#include <map>
#include <set>
#include <unordered_set>

#include "inesctec_mrdt_slam_distmap_2d/slam/loop_finder_entry_container.hpp"

namespace inesctec_mrdt_slam_distmap_2d
{

namespace slam
{

/**
 * @brief graph visitor for the loop finder class, where the sources may be the
 *        loop hints based on distance or the current local map variable, and
 *        the targets may be the current local map
 *        (if defined, computation stops when all targets are found)
 *
 * maximum graph depth should be the same threshold as the minimum graph length
 * to consider a loop closure and ONLY one source as the current local map
 * variable (optimizes graph visitor searching time), disadvantage is not
 * computing the graph length...
 *
 * another possibility is having the sources as the loop hints, considering a
 * maximum depth if needed to stop searching, and a target entries container to
 * consider another stopping policy (when all targets are found upon expansion
 * and computation, visitor stops!)
 */
class LoopFinderVisitor
{
 public:

  /**
   * @brief slices entry comparator to sort the entries of each slice in the
   *        sorted container by lower cost and older / lower graph IDs
   */
  struct SlicesEntryComparator
  {
    inline bool operator()(LoopFinderEntry* const a,
                           LoopFinderEntry* const b) const
    {
      if (a->m_cost < b->m_cost)
      {
        return true;
      }

      return (a->m_variable->graphId() < b->m_variable->graphId());
    }
  };  // struct LoopFinderVisitor::SlicesEntryComparator

 public:

  using EntryCostType = typename LoopFinderEntry::CostType;
  using SliceIdxType = typename LoopFinderEntry::SliceIdxType;
  using SlicesType =
      std::map<SliceIdxType, std::set<LoopFinderEntry*, SlicesEntryComparator>>;

  using LoopFinderTargetQueueList =
      std::unordered_set<srrg2_solver::VariableBase::Id>;

 public:

  EntryCostType m_max_depth = -1;  //!< maximum search depth in the graph
                                   //!< (-1 to search through all graph)

 protected:

  srrg2_solver::FactorGraphInterface* m_graph_ = nullptr;  //!< factor graph

  LoopFinderEntryContainer m_entries_;        //!< graph visitor entries
  LoopFinderEntryContainer m_sources_;        //!< graph visitor source entries
  LoopFinderEntryContainer m_targets_;        //!< graph visitor target entries
  LoopFinderEntryQueue m_queue_;              //!< search queue
  LoopFinderTargetQueueList m_target_queue_;  //!< remaining targets list

 public:

  /**
   * @brief constructor
   * @param[in] m_max_depth maximum search depth in the graph (-1 to search
   *                        through all graph)
   */
  LoopFinderVisitor(EntryCostType m_max_depth = -1) : m_max_depth(m_max_depth)
  {
  }

  /**
   * @brief expand the graph through the source entries set by the method
   *        addSource (do not forget to clear them...)
   */
  void compute();

  /**
   * @brief add source variable in the graph from which the visitor will expand
   *        its search
   * @param[in] source graph variable
   */
  inline void addSource(srrg2_solver::VariableBase* source)
  {
    m_sources_.add(LoopFinderEntry(source));
  }  // inline void
     // LoopFinderVisitor::addSource(srrg2_solver::VariableBase*)

  /**
   * @brief add target variable in the graph to stop the visitor search
   *        expansion upon target found
   * @param[in] target graph variable
   */
  inline void addTarget(srrg2_solver::VariableBase* target)
  {
    m_targets_.add(LoopFinderEntry(target));
  }  // inline void
     // LoopFinderVisitor::addTarget(srrg2_solver::VariableBase*)

  /**
   * @brief clear the visitor entries container (require before executing
   *        addSource and compute)
   */
  inline void clear()
  {
    m_entries_.clear();
    m_sources_.clear();
    m_targets_.clear();
  }  // inline void LoopFinderVisitor::clear()

  /**
   * @brief print the path from target to source variables (latter should be set
   *        in the visitor using addSource) in the graph (if not found, message
   *        printed along the way...)
   *        (just for debug purposes!)
   * @param[in] source graph variable (previously set in visitor w/ addSource)
   * @param[in] target graph variable
   */
  void printPath(srrg2_solver::VariableBase* source,
                 srrg2_solver::VariableBase* target);

  /**
   * @brief print the path from target to closest source variables (latter
   *        previously set in the visitor using addSource) in the graph
   *        (just for debug purposes!)
   * @param[in] target graph variable
   */
  void printPath(srrg2_solver::VariableBase* target);

  /**
   * @brief print graph slices (just for debug purposes!)
   */
  void printGraphSlices();

 public:

  inline void setGraph(srrg2_solver::FactorGraphInterface* const graph)
  {
    m_graph_ = graph;
  }

  inline LoopFinderEntryContainer& getEntries() { return m_entries_; }
  inline LoopFinderEntryContainer& getSources() { return m_sources_; }
  inline LoopFinderEntryContainer& getTargets() { return m_targets_; }

  /**
   * @brief get the graph slices in the graph (partial set of nodes that do not
   *        have a path between them), considering the max_depth parameter
   * @param[in,out] slices graph slices container
   */
  inline void getGraphSlices(SlicesType& slices)
  {
    slices.clear();

    for (auto& e : m_entries_.getEntries())
    {
      slices[e.second.m_graph_slice_idx].emplace(&e.second);
    }
  }  // inline void LoopFinderVisitor::getGraphSlices(SlicesType&)

  /**
   * @brief get the graph slices in the graph (partial set of nodes that do not
   *        have a path between them), considering the max_depth parameter
   * @return const SlicesType& graph slices container
   */
  inline SlicesType getGraphSlices()
  {
    SlicesType slices;

    for (auto& e : m_entries_.getEntries())
    {
      slices[e.second.m_graph_slice_idx].emplace(&e.second);
    }

    return slices;
  }  // inline void LoopFinderVisitor::getGraphSlices(SlicesType&)

};  // class LoopFinderVisitor

}  // namespace slam

}  // namespace inesctec_mrdt_slam_distmap_2d