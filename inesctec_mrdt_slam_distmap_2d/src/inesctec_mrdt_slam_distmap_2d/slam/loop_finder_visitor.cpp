#include "inesctec_mrdt_slam_distmap_2d/slam/loop_finder_visitor.hpp"

namespace inesctec_mrdt_slam_distmap_2d
{

namespace slam
{

void LoopFinderVisitor::compute()
{
  assert(m_graph_ && "LoopFinderVisitor::compute | graph not set...");

  bool is_max_graph_depth_enabled = (m_max_depth > 0);

  SliceIdxType slice_idx = 0;

  m_queue_ = LoopFinderEntryQueue();

  for (auto& source : m_sources_.getEntries())
  {
    assert(source.second.m_variable &&
           "LoopFinderVisitor::compute | source variable member not set...");

    source.second.m_graph_slice_idx = slice_idx++;
    source.second.m_parent = source.second.m_variable;
    source.second.m_cost = 0;

    m_entries_.add(source.second);
    m_queue_.emplace(source.second);
  }

  m_target_queue_.clear();

  for (auto& target : m_targets_.getEntries())
  {
    m_target_queue_.emplace(target.first);
  }

  while (!m_queue_.empty())
  {
    LoopFinderEntry entry = m_queue_.top();

    m_queue_.pop();

    if (!m_targets_.empty() && m_target_queue_.empty())
    {
      break;
    }

    srrg2_solver::VariableBase* variable = entry.m_variable;
    LoopFinderEntry* stored_entry = m_entries_.at(variable->graphId());

    assert(stored_entry &&
           "LoopFinderVisitor::compute | "
           "entry should have been already set...");

    if (!m_target_queue_.empty())
    {
      if (m_target_queue_.find(variable->graphId()) != m_target_queue_.end())
      {
        m_target_queue_.erase(variable->graphId());
      }
    }

    if (stored_entry->m_cost < entry.m_cost)
    {
      continue;
    }

    EntryCostType new_entry_cost = 1;

    if (is_max_graph_depth_enabled)
    {
      if (new_entry_cost + entry.m_cost > m_max_depth)
      {
        continue;
      }
    }

    for (auto f : m_graph_->factors(variable))
    {
      if (!f->enabled())
      {
        continue;
      }

      for (int idx = 0; idx < f->numVariables(); ++idx)
      {
        srrg2_solver::VariableBase* new_variable = f->variable(idx);

        if (new_variable == variable)
        {
          continue;
        }

        LoopFinderEntry* stored_new_entry =
            m_entries_.at(new_variable->graphId());

        if (!stored_new_entry)
        {
          stored_new_entry = m_entries_.add(LoopFinderEntry(new_variable));

          stored_new_entry->m_graph_slice_idx = entry.m_graph_slice_idx;
          stored_new_entry->m_parent = variable;
          stored_new_entry->m_cost = new_entry_cost + entry.m_cost;

          m_queue_.emplace(*stored_new_entry);
        }
        else
        {
          if (stored_new_entry->m_graph_slice_idx != entry.m_graph_slice_idx)
          {
            for (auto& e : m_entries_.getEntries())
            {
              if (e.second.m_graph_slice_idx == entry.m_graph_slice_idx)
              {
                e.second.m_graph_slice_idx =
                    stored_new_entry->m_graph_slice_idx;
              }
            }
          }

          if (new_entry_cost + entry.m_cost < stored_new_entry->m_cost)
          {
            stored_new_entry->m_parent = entry.m_parent;
            stored_new_entry->m_cost = new_entry_cost + entry.m_cost;

            m_queue_.emplace(*stored_new_entry);
          }
        }
      }
    }
  }
}  // void LoopFinderVisitor::compute()

void LoopFinderVisitor::printPath(srrg2_solver::VariableBase* source,
                                  srrg2_solver::VariableBase* target)
{
  assert(source && "LoopFinderVisitor::printPath | source not set...");
  assert(target && "LoopFinderVisitor::printPath | target not set...");

  LoopFinderEntry* entry = m_entries_.at(source->graphId());

  if (!entry)
  {
    std::cerr << "Node " << source->graphId()
              << " not found in the entries container!" << std::endl;
    return;
  }
  if ((entry->m_cost != 0) || (entry->m_variable != entry->m_parent))
  {
    std::cerr << "Node " << source->graphId()
              << " not a source variable in the graph (required to set "
                 "addSource in the visitor)!"
              << std::endl;
    return;
  }

  entry = m_entries_.at(target->graphId());

  if (!entry)
  {
    std::cerr << "Node " << target->graphId()
              << " not found in the entries container!" << std::endl;
    return;
  }

  std::cout << "( path Node " << source->graphId() << " to Node "
            << target->graphId() << ": ";

  EntryCostType curr_cost = 0;
  EntryCostType target_cost = entry->m_cost;

  while ((entry->m_variable->graphId() != source->graphId()) &&
         (curr_cost <= target_cost))
  {
    std::cout << entry->m_variable->graphId() << " -> ";

    entry = m_entries_.at(entry->m_parent->graphId());

    curr_cost++;
  }

  if (curr_cost > target_cost)
  {
    std::cout << " ... PATH NOT FOUND / POSSIBLE )";
    return;
  }

  std::cout << entry->m_variable->graphId() << " )";
}  // void LoopFinderVisitor::printPath(srrg2_solver::VariableBase*,
   // srrg2_solver::VariableBase*)

void LoopFinderVisitor::printPath(srrg2_solver::VariableBase* target)
{
  assert(target && "LoopFinderVisitor::printPath | target not set...");

  LoopFinderEntry* entry = m_entries_.at(target->graphId());

  if (!entry)
  {
    std::cerr << "Node " << target->graphId()
              << " not found in the entries container!" << std::endl;
    return;
  }

  std::cout << "( path from Node " << target->graphId() << ": ";

  while (entry->m_variable != entry->m_parent)
  {
    std::cout << entry->m_variable->graphId() << " -> ";

    entry = m_entries_.at(entry->m_parent->graphId());
  }

  std::cout << entry->m_variable->graphId() << " )";
}  // void LoopFinderVisitor::printPath(srrg2_solver::VariableBase*)

void LoopFinderVisitor::printGraphSlices()
{
  SlicesType slices;

  getGraphSlices(slices);

  for (auto& slice : slices)
  {
    std::cout << "[ slice idx: " << slice.first << "   ;   nodes: ";

    for (auto& slice_node : slice.second)
    {
      std::cout << slice_node->m_variable->graphId()
                << " (c: " << slice_node->m_cost
                << ",p:" << slice_node->m_parent->graphId() << ") ";
    }

    std::cout << "]" << std::endl;
  }
}

}  // namespace slam

}  // namespace inesctec_mrdt_slam_distmap_2d
