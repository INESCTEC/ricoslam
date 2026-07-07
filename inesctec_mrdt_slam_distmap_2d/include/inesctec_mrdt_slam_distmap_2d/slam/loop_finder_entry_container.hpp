#pragma once

#include <srrg2_solver/solver_core/variable.h>

#include <map>

#include "inesctec_mrdt_slam_distmap_2d/slam/loop_finder_entry.hpp"

namespace inesctec_mrdt_slam_distmap_2d
{

namespace slam
{

/**
 * @brief container of the graph visitor entries for the loop finder
 *        (holds only UNIQUE entries identified by their variable's graph ID)
 */
class LoopFinderEntryContainer
{
 protected:

  std::map<srrg2_solver::VariableBase::Id, LoopFinderEntry> m_entries_;

 public:

  /**
   * @brief find entry identified by its variable graph ID
   * @param[in] id entry's variable graph ID
   * @return LoopFinderEntry* entry or nullptr if not found
   */
  inline LoopFinderEntry* at(srrg2_solver::VariableBase::Id id)
  {
    auto it = m_entries_.find(id);

    if (it != m_entries_.end())
    {
      return &(it->second);
    }

    return nullptr;
  }

  /**
   * @brief check if entry identified by its variable graph ID is initialized in
   *        the container
   * @param[in] id entry's variable graph ID
   * @return true entry already exists in the container
   * @return false otherwise
   */
  inline bool isInit(srrg2_solver::VariableBase::Id id) const
  {
    return (m_entries_.find(id) != m_entries_.end());
  }

  /**
   * @brief check if entry is initialized in the container
   * @param[in] id entry struct
   * @return true entry already exists in the container
   * @return false otherwise
   */
  inline bool isInit(const LoopFinderEntry& entry) const
  {
    return (m_entries_.find(entry.m_variable->graphId()) != m_entries_.end());
  }

  /**
   * @brief add new entry to the container
   * @param[in] id entry struct
   * @return LoopFinderEntry*
   */
  inline LoopFinderEntry* add(const LoopFinderEntry& entry)
  {
    if (isInit(entry))
    {
      return this->at(entry.m_variable->graphId());
    }

    auto result =
        m_entries_.emplace(std::make_pair(entry.m_variable->graphId(), entry));

    return &(result.first->second);
  }

  /**
   * @brief clear all entries in the container
   */
  inline void clear() { m_entries_.clear(); }

  /**
   * @brief container current size
   */
  inline std::size_t size() const { return m_entries_.size(); }

  /**
   * @brief container empty
   */
  inline bool empty() const { return m_entries_.empty(); }

  /**
   * @brief get all entries hold by the container
   * @return std::map<srrg2_solver::VariableBase::Id, LoopFinderEntry>& entries
   */
  inline std::map<srrg2_solver::VariableBase::Id, LoopFinderEntry>& getEntries()
  {
    return m_entries_;
  }
};  // class LoopFinderEntryContainer

}  // namespace slam

}  // namespace inesctec_mrdt_slam_distmap_2d
