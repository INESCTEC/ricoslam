#pragma once

#include <srrg2_solver/solver_core/factor_graph_interface.h>

#include <Eigen/Core>
#include <exception>
#include <limits>
#include <memory>
#include <set>

#include "inesctec_mrdt_slam_distmap_2d/slam/loop_finder_visitor.hpp"

namespace inesctec_mrdt_slam_distmap_2d
{

namespace slam
{

/**
 * @brief search possible loops in the graph considering a minimum graph length
 *        between current keyframe (source node) and possible loop hints
 *        (targets), considering the proximity of the graph nodes to the current
 *        SE2 robot estimation in the world
 * @tparam SLAMType_ SLAM implementation type
 */
template <typename SLAMType_>
class LoopFinder_
{
 public:

  using ThisType = LoopFinder_<SLAMType_>;
  using SLAMType = SLAMType_;

  using GraphLenType = typename LoopFinderVisitor::EntryCostType;

  using VariableType = typename SLAMType::VariableType;
  using EstimateType = typename VariableType::EstimateType;

  using FactorInformationMatrix = typename SLAMType::FactorInformationMatrix;

 public:

  /**
   * @brief loop finder parametrization
   */
  struct Param
  {
    GraphLenType m_loop_min_graph_len = 5;  //!< loop hint minimum graph length
    float m_loop_max_dist = 2.5f;           //!< initial guess SE2 max distance

    /**
     * @brief parametrization constructor
     */
    Param() : m_loop_min_graph_len(5), m_loop_max_dist(2.5f) {}

    /**
     * @brief parametrization constructor
     * @param[in] loop_min_graph_len loop hint minimum graph length
     * @param[in] loop_max_dist initial guess SE2 max distance
     */
    Param(GraphLenType loop_min_graph_len, float loop_max_dist)
        : m_loop_min_graph_len(loop_min_graph_len),
          m_loop_max_dist(loop_max_dist)
    {
    }
  };  // struct LoopFinder_<SLAMType_>::Param

  struct Hint
  {
    VariableType* m_local_map;           //!< graph node
    EstimateType m_initial_guess;        //!< SE2 relative pose
    FactorInformationMatrix m_info_mat;  //!< matching information matrix
    GraphLenType m_graph_len;            //!< loop hint graph length
    float m_chi_inliers;                 //!< matching error
    float m_inliers_ratio;               //!< matching inliers ratio

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief hint constructor
     * @param[in] candidate graph node
     * @param[in] initial_guess SE2 relative pose
     * @param[in] info_mat matching information matrix
     * @param[in] graph_len loop hint graph length
     * @param[in] chi_inliers matching error
     * @param[in] inliers_ratio matching inliers ratio
     */
    Hint(VariableType* candidate,
         EstimateType initial_guess = EstimateType::Identity(),
         FactorInformationMatrix info_mat = FactorInformationMatrix::Identity(),
         GraphLenType graph_len = GraphLenType(-1),
         float chi_inliers = std::numeric_limits<float>::max(),
         float inliers_ratio = std::numeric_limits<float>::max())
        : m_local_map(candidate),
          m_initial_guess(initial_guess),
          m_info_mat(info_mat),
          m_graph_len(graph_len),
          m_chi_inliers(chi_inliers),
          m_inliers_ratio(inliers_ratio)
    {
    }

   private:

    Hint() = delete;
  };  // struct LoopFinder_<SLAMType_>::Hint

  using HintPtr = std::shared_ptr<Hint>;

 public:

  /**
   * @brief comparator for the set of loop closure hints, to automatically order
   *        the loops in the set by the closest one, in terms of SE2 norm of the
   *        initial guess's relative pose
   */
  struct HintPtrComparatorClosest
  {
    inline bool operator()(const HintPtr& a, const HintPtr& b) const
    {
      if (a->m_initial_guess.translation().norm() <
          b->m_initial_guess.translation().norm())
      {
        return true;
      }

      return (a->m_local_map->graphId() < b->m_local_map->graphId());
    }
  };  // struct LoopFinder_<SLAMType_>::HintPtrComparatorClosest

  /**
   * @brief comparator for the set of loop closure hints, to automatically order
   *        the loops in the set by the longer graph length (number of edges
   *        between the loop hint and the current graph node through the shorter
   *        graph path), then by the closest pose
   */
  struct HintPtrComparatorLongerLength
  {
    inline bool operator()(const HintPtr& a, const HintPtr& b) const
    {
      if (a->m_graph_len > b->m_graph_len)
      {
        return true;
      }
      if (a->m_initial_guess.translation().norm() <
          b->m_initial_guess.translation().norm())
      {
        return true;
      }

      return (a->m_local_map->graphId() < b->m_local_map->graphId());
    }
  };  // struct LoopFinder_<SLAMType_>::HintPtrComparatorLongerLength

  /**
   * @brief comparator for the set of loop closure hints, to automatically order
   *        the loops in the set by the lower inliers chi error from the
   *        matching process
   */
  struct HintPtrComparatorLowerChiInliers
  {
    inline bool operator()(const HintPtr& a, const HintPtr& b) const
    {
      if (a->m_chi_inliers < b->m_chi_inliers)
      {
        return true;
      }

      return (a->m_local_map->graphId() < b->m_local_map->graphId());
    }
  };  // struct LoopFinder_<SLAMType_>::HintPtrComparatorLowerChiInliers

  /**
   * @brief comparator for the set of loop closure hints, to automatically order
   *        the loops in the set by the higher inliers ratio from the matching
   *        process
   */
  struct HintPtrComparatorHigherInliersRatio
  {
    inline bool operator()(const HintPtr& a, const HintPtr& b) const
    {
      if (a->m_inliers_ratio > b->m_inliers_ratio)
      {
        return true;
      }

      return (a->m_local_map->graphId() < b->m_local_map->graphId());
    }
  };  // struct LoopFinder_<SLAMType_>::HintPtrComparatorHigherInliersRatio

 public:

  template <typename HintPtrComparator_>
  using HintPtrSet_ = std::set<HintPtr, HintPtrComparator_>;

  template <typename HintPtrSetType_>
  using HintPtrSetPtr_ = std::shared_ptr<HintPtrSetType_>;

  using HintPtrSetClosest = HintPtrSet_<HintPtrComparatorClosest>;
  using HintPtrSetLongerLength = HintPtrSet_<HintPtrComparatorLongerLength>;
  using HintPtrSetLowerChiInliers =
      HintPtrSet_<HintPtrComparatorLowerChiInliers>;
  using HintPtrSetHigherInliersRatio =
      HintPtrSet_<HintPtrComparatorHigherInliersRatio>;

  using HintPtrSetClosestPtr = HintPtrSetPtr_<HintPtrSetClosest>;
  using HintPtrSetLongerLengthPtr = HintPtrSetPtr_<HintPtrSetLongerLength>;
  using HintPtrSetLowerChiInliersPtr =
      HintPtrSetPtr_<HintPtrSetLowerChiInliers>;
  using HintPtrSetHigherInliersRatioPtr =
      HintPtrSetPtr_<HintPtrSetHigherInliersRatio>;

 protected:

  Param m_param_;  //!< parametrization

  LoopFinderVisitor m_visitor_;  //!< graph visitor
  SLAMType* m_slam_ = nullptr;   //!< SLAM algorithm implementation

  HintPtrSetClosestPtr m_hints_;        //!< final loop hints
  HintPtrSetClosestPtr m_short_loops_;  //!< hints wo/ graph length constraint

 public:

  /**
   * @brief loop finder construct
   * @param[in] param parametrization
   */
  LoopFinder_(const Param& param = Param())
      : m_param_(param),
        m_hints_(std::make_shared<HintPtrSetClosest>()),
        m_short_loops_(std::make_shared<HintPtrSetClosest>())
  {
    assertInit(m_param_);
  }

  /**
   * @brief find loop hints based on the graph length and relative pose distance
   */
  void compute()
  {
    m_hints_->clear();
    m_short_loops_->clear();

    srrg2_solver::FactorGraphInterface* graph = m_slam_->getGraph();

    VariableType* source = m_slam_->getKeyframe();

    EstimateType robot_in_world = m_slam_->getRobotInMap();
    EstimateType robot_in_local = m_slam_->getTracker()->getRobotInLocalMap();

    m_visitor_.clear();
    m_visitor_.setGraph(graph);
    m_visitor_.addSource(source);

    for (auto v : graph->variables())
    {
      VariableType* candidate = static_cast<VariableType*>(v.second);

      if (candidate->graphId() == source->graphId())
      {
        continue;
      }

      float candidate_loop_pose_dist =
          (candidate->estimate().translation() - robot_in_world.translation())
              .norm();

      if (candidate_loop_pose_dist > m_param_.m_loop_max_dist)
      {
        continue;
      }

      m_short_loops_->emplace(std::make_shared<Hint>(
          candidate, candidate->estimate().inverse() * source->estimate() *
                         robot_in_local));
    }

    for (auto& candidate : *m_short_loops_)
    {
      m_visitor_.addTarget(candidate->m_local_map);
    }

    m_visitor_.m_max_depth = -1;
    m_visitor_.compute();

    for (auto& candidate : *m_short_loops_)
    {
      LoopFinderEntry* entry =
          m_visitor_.getEntries().at(candidate->m_local_map->graphId());

      if (entry)
      {
        candidate->m_graph_len = entry->m_cost;

        if (candidate->m_graph_len >= m_param_.m_loop_min_graph_len)
        {
          m_hints_->emplace(candidate);
        }
      }
    }
  }  // void LoopFinder_<SLAMType_>::compute()

  /**
   * @brief set new parametrization for the loop finder
   * @param[in] param new parametrization
   */
  void setParam(const Param& param)
  {
    assertInit(param);
    m_param_ = param;
  }

 public:

  inline const Param& getParam() const { return m_param_; }
  inline HintPtrSetClosest* getHints() const { return m_hints_.get(); }
  inline HintPtrSetClosest* getShortLoops() const
  {
    return m_short_loops_.get();
  }
  inline const HintPtrSetClosestPtr& getHintsPtr() const { return m_hints_; }
  inline const HintPtrSetClosestPtr& getShortLoopsPtr() const
  {
    return m_short_loops_;
  }

  inline void setSLAM(SLAMType* const slam) { m_slam_ = slam; }

 protected:

  static void assertInit(const Param& param)
  {
    if (param.m_loop_max_dist <= 0)
    {
      throw std::runtime_error(
          "LoopFinder_::assertInit | loop maximum distance must be greater "
          "than 0 (m)");
    }
    if (param.m_loop_min_graph_len < 1)
    {
      throw std::runtime_error(
          "LoopFinder_::assertInit | loop minimum graph length must be greater "
          "than 0 (#edges)");
    }
  }

};  // class LoopFinder_<SLAMType_>

}  // namespace slam

}  // namespace inesctec_mrdt_slam_distmap_2d
