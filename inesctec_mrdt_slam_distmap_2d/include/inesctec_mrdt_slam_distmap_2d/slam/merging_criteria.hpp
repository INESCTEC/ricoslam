#pragma once

#include <exception>

// Eigen
#include <Eigen/Core>

#include "inesctec_mrdt_slam_distmap_2d/geometry/geometry_2d.hpp"
#include "inesctec_mrdt_slam_distmap_2d/slam/msg/laser_scan.hpp"

namespace inesctec_mrdt_slam_distmap_2d
{

namespace slam
{

/**
 * @brief merging criteria template (virtualized compute method, generic API)
 *        to trigger the integration of a point cloud into the current local map
 * @tparam TrackerType_ tracker algorithm type
 */
template <typename TrackerType_>
class MergingCriteria_
{
 public:

  using ThisType = MergingCriteria_<TrackerType_>;
  using TrackerType = TrackerType_;
  using EstimateType = typename TrackerType::EstimateType;

 protected:

  TrackerType* m_tracker_ = nullptr;  //!< tracker algorithm (to get the robot
                                      //!< w.r.t. local map transformation)
  bool m_merge_ = false;              //!< merge boolean trigger
  bool m_force_merge_ = false;  //!< force merging trigger (ignoring compute!),
                                //!< must be implemented in criteria!
  EstimateType m_robot_in_map_init_ =
      EstimateType::Identity();  //!< initialization robot w.r.t. the local map
                                 //!< SE(2) transformation

 public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief virtual method to compute if the merge boolean should be triggered
   *        based on different metrics
   */
  virtual void compute() = 0;

  /**
   * @brief accessor method to merge boolean trigeer
   * @return true tracker should merge / trigger the integration of the point
   *              cloud into the current local map
   * @return false otherwise
   */
  inline bool isToMerge() const { return m_merge_; }

  /**
   * @brief force merging on next scan message, ignoring compute instructions
   *        (must be implemented in criteria compute function)
   */
  inline void forceMerge() { m_force_merge_ = true; }

  /**
   * @brief reset the merge boolean trigger (and, possibly, some internal state
   *        of the criteria supervision class, such as the m_robot_in_map_init_
   *        SE(2) initialization transformation)
   */
  virtual void reset()
  {
    m_merge_ = false;
    this->m_robot_in_map_init_ = this->m_tracker_->getRobotInLocalMap();
  }  // void MergingCriteria_::reset()

  /**
   * @brief set the tracker algorithm (to ask for the robot w.r.t. the local
   *        map SE(2) transformation, etc.)
   * @param[in] tracker tracker algorithm
   */
  inline void setTracker(TrackerType* const tracker) { m_tracker_ = tracker; }

  /**
   * @brief get the robot pose w.r.t. local map upon the internal initialization
   *        of the merging criteria
   * @return const EstimateType& initialization robot w.r.t. the local map SE(2)
   *                             transformation
   */
  inline const EstimateType& getRobotInMapInit() const
  {
    return m_robot_in_map_init_;
  }

 public:

  friend std::ostream& operator<<(std::ostream& os,
                                  const ThisType& merging_criteria)
  {
    merging_criteria.print(os);
    return os;
  }

 protected:

  virtual void print(std::ostream&) const {}

};  // class MergingCriteria_<TrackerType_>

/**
 * @brief empty merging criteria (only by forcing through external entities)
 * @tparam TrackerType_ tracker algorithm type
 */
template <typename TrackerType_>
class MergingCriteriaEmpty_ : public MergingCriteria_<TrackerType_>
{
 public:

  using ThisType = MergingCriteriaEmpty_<TrackerType_>;
  using BaseType = MergingCriteria_<TrackerType_>;

  using TrackerType = TrackerType_;

 public:

  virtual void compute() override
  {
    if (!this->m_tracker_)
    {
      throw std::runtime_error(
          "MergingCriteriaEmpty_::compute | no tracker algorithm set...");
    }

    if (this->m_force_merge_)
    {
      this->m_force_merge_ = false;
      this->m_merge_ = true;
      return;
    }
  }  // virtual void MergingCriteriaEmpty_::compute() override

 protected:

  virtual void print(std::ostream& os) const
  {
    os << "- merging criteria type: empty" << std::endl;
  }

};  // class MergingCriteriaEmpty_<TrackerType_>

/**
 * @brief merging criteria based on rotation (minimum travel heading of the
 *        current robot heading w.r.t. the local map)
 *
 * m_min_travel_heading defines the minimum threshold to trigger the merging
 * boolean variable on compute() (rad)
 *
 * @tparam TrackerType_ tracker algorithm type
 */
template <typename TrackerType_>
class MergingCriteriaRotation_ : public MergingCriteria_<TrackerType_>
{
 public:

  using ThisType = MergingCriteriaRotation_<TrackerType_>;
  using BaseType = MergingCriteria_<TrackerType_>;

  using TrackerType = TrackerType_;
  using EstimateType = typename TrackerType::EstimateType;

 public:

  float m_min_travel_heading =
      geometry::degToRad(5.0f);  //!< minimum travel heading threshold to merge
                                 //!< the pc into the local map (rad)

 public:

  virtual void compute() override
  {
    if (!this->m_tracker_)
    {
      throw std::runtime_error(
          "MergingCriteriaRotation_::compute | "
          "no tracker algorithm set...");
    }

    if (this->m_force_merge_)
    {
      this->m_force_merge_ = false;
      this->m_merge_ = true;
      return;
    }

    this->m_merge_ = false;

    const EstimateType& robot_in_map = this->m_tracker_->getRobotInLocalMap();
    const Eigen::Vector3f delta =
        geometry::t2v(this->m_robot_in_map_init_.inverse() * robot_in_map);

    float delta_heading = std::fabs(delta(2));

    if (delta_heading > m_min_travel_heading)
    {
      this->m_merge_ = true;
      return;
    }
  }  // virtual void MergingCriteriaRotation_::compute() override

 protected:

  virtual void print(std::ostream& os) const
  {
    os << "- merging criteria type       : rotation" << std::endl;
    os << "- minimum travel heading (deg): "
       << geometry::radToDeg(this->m_min_travel_heading) << std::endl;
  }

};  // class MergingCriteriaRotation_<TrackerType_>

/**
 * @brief merging criteria based on translation (minimum travel distance of
 *        the current robot pose w.r.t. the local map)
 *
 * m_min_travel_dist defines the minimum threshold to trigger the merging
 * boolean variable on compute() (m)
 *
 * @tparam TrackerType_ tracker algorithm type
 */
template <typename TrackerType_>
class MergingCriteriaTranslation_ : public MergingCriteria_<TrackerType_>
{
 public:

  using ThisType = MergingCriteriaTranslation_<TrackerType_>;
  using BaseType = MergingCriteria_<TrackerType_>;

  using TrackerType = TrackerType_;
  using EstimateType = typename TrackerType::EstimateType;

 public:

  float m_min_travel_dist = 0.10f;  //!< minimum travel distance threshold to
                                    //!< merge the pc into the local map (m)

 public:

  virtual void compute() override
  {
    if (!this->m_tracker_)
    {
      throw std::runtime_error(
          "MergingCriteriaTranslation_::compute | "
          "no tracker algorithm set...");
    }

    if (this->m_force_merge_)
    {
      this->m_force_merge_ = false;
      this->m_merge_ = true;
      return;
    }

    this->m_merge_ = false;

    const EstimateType& robot_in_map = this->m_tracker_->getRobotInLocalMap();
    const Eigen::Vector3f delta =
        geometry::t2v(this->m_robot_in_map_init_.inverse() * robot_in_map);

    float delta_dist = delta.head(2).norm();

    if (delta_dist > m_min_travel_dist)
    {
      this->m_merge_ = true;
      return;
    }
  }  // virtual void MergingCriteriaTranslation_::compute() override

 protected:

  virtual void print(std::ostream& os) const
  {
    os << "- merging criteria type       : translation" << std::endl;
    os << "- minimum travel distance  (m): " << this->m_min_travel_dist
       << std::endl;
  }

};  // class MergingCriteriaTranslation_<TrackerType_>

/**
 * @brief merging criteria based on viewpoint (minimum travel heading and/or
 *        distance of the current robot pose w.r.t. the local map)
 *
 * m_min_travel_dist (m), m_min_travel_heading (rad) define the thresholds to
 * trigger the merging boolean variable on compute()
 *
 * @tparam TrackerType_ tracker algorithm type
 */
template <typename TrackerType_>
class MergingCriteriaViewpoint_ : public MergingCriteria_<TrackerType_>
{
 public:

  using ThisType = MergingCriteriaViewpoint_<TrackerType_>;
  using BaseType = MergingCriteria_<TrackerType_>;

  using TrackerType = TrackerType_;
  using EstimateType = typename TrackerType::EstimateType;

 public:

  float m_min_travel_dist = 0.10f;  //!< minimum travel distance threshold to
                                    //!< merge the pc into the local map (m)
  float m_min_travel_heading =
      geometry::degToRad(5.0f);  //!< minimum travel heading threshold to merge
                                 //!< the pc into the local map (rad)

 public:

  virtual void compute() override
  {
    if (!this->m_tracker_)
    {
      throw std::runtime_error(
          "MergingCriteriaViewpoint_::compute | "
          "no tracker algorithm set...");
    }

    if (this->m_force_merge_)
    {
      this->m_force_merge_ = false;
      this->m_merge_ = true;
      return;
    }

    this->m_merge_ = false;

    const EstimateType& robot_in_map = this->m_tracker_->getRobotInLocalMap();
    const Eigen::Vector3f delta =
        geometry::t2v(this->m_robot_in_map_init_.inverse() * robot_in_map);

    float delta_dist = delta.head(2).norm();
    float delta_heading = std::fabs(delta(2));

    if (delta_dist > m_min_travel_dist)
    {
      this->m_merge_ = true;
      return;
    }

    if (delta_heading > m_min_travel_heading)
    {
      this->m_merge_ = true;
      return;
    }
  }  // virtual void MergingCriteriaViewpoint_::compute() override

 protected:

  virtual void print(std::ostream& os) const
  {
    os << "- merging criteria type       : viewpoint" << std::endl;
    os << "- minimum travel heading (deg): "
       << geometry::radToDeg(this->m_min_travel_heading) << std::endl;
    os << "- minimum travel distance  (m): " << this->m_min_travel_dist
       << std::endl;
  }

};  // class MergingCriteriaViewpoint_<TrackerType_>

/**
 * @brief merging criteria based on the inliers ratio from the tracker's
 *        alignment results
 *
 * m_max_ratio_inliers define the thresholds to trigger the merging boolean
 * variable on compute() (0..1)
 *
 * m_max_travel_dist can be enabled (if greater than 0.f) to define a maximum
 * travel distance to avoid sparsity on merging (m)
 *
 * m_max_travel_heading can be enabled (if greater than 0.f) to define a maximum
 * travel angular distance to avoid sparsity on merging (rad)
 *
 * m_min_travel_dist (m), m_min_travel_heading (rad) define minimum thresholds
 * to avoid merging without moving
 *
 * @tparam TrackerType_ tracker algorithm type
 */
template <typename TrackerType_>
class MergingCriteriaInliersRatio_ : public MergingCriteria_<TrackerType_>
{
 public:

  using ThisType = MergingCriteriaInliersRatio_<TrackerType_>;
  using BaseType = MergingCriteria_<TrackerType_>;

  using TrackerType = TrackerType_;
  using AlignerBaseTypeStats = typename TrackerType::AlignerBaseTypeStats;
  using EstimateType = typename TrackerType::EstimateType;

 public:

  float m_min_travel_dist = 0.10f;  //!< minimum travel distance required to
                                    //!< evaluate the inliers ratio (m)
  float m_min_travel_heading =
      geometry::degToRad(5.0f);  //!< minimum travel heading required to
                                 //!< evaluate the inliers ratio (rad)
  float m_max_ratio_inliers =
      0.80f;  //!< maximum ratio of inliers ratio to merge the local map (0..1)
  float m_max_travel_dist = -1.f;     //!< maximum travel distance threshold to
                                      //!< force merging the local map (m)
  float m_max_travel_heading = -1.f;  //!< maximum travel heading threshold to
                                      //!< force merging the local map (rad)

 public:

  virtual void compute() override
  {
    if (!this->m_tracker_)
    {
      throw std::runtime_error(
          "MergingCriteriaInliersRatio_::compute | "
          "no tracker algorithm set...");
    }

    if (this->m_force_merge_)
    {
      this->m_force_merge_ = false;
      this->m_merge_ = true;
      return;
    }

    this->m_merge_ = false;

    const AlignerBaseTypeStats& stats = this->m_tracker_->getAlignerStats();

    const EstimateType& robot_in_map = this->m_tracker_->getRobotInLocalMap();
    const Eigen::Vector3f delta =
        geometry::t2v(this->m_robot_in_map_init_.inverse() * robot_in_map);

    float delta_dist = delta.head(2).norm();
    float delta_heading = std::fabs(delta(2));

    if ((((delta_dist > this->m_min_travel_dist) ||
          (delta_heading > this->m_min_travel_heading)) &&
         (stats.m_inliers_ratio < this->m_max_ratio_inliers)) ||
        ((this->m_max_travel_dist < delta_dist) &&
         (this->m_max_travel_dist > 0.f)) ||
        ((this->m_max_travel_heading < delta_heading) &&
         (this->m_max_travel_heading > 0.f)))
    {
      this->m_merge_ = true;
    }
  }  // virtual void MergingCriteriaInliersRatio_::compute() override

 protected:

  virtual void print(std::ostream& os) const
  {
    os << "- merging criteria type       : inliers ratio" << std::endl;
    os << "- minimum travel heading (deg): "
       << geometry::radToDeg(this->m_min_travel_heading) << std::endl;
    os << "- minimum travel distance  (m): " << this->m_min_travel_dist
       << std::endl;
    if (this->m_max_ratio_inliers <= 0.f)
    {
      os << "- maximum inliers ratio (0..1): not enabled" << std::endl;
    }
    else
    {
      os << "- maximum inliers ratio (0..1): " << this->m_max_ratio_inliers
         << std::endl;
    }
    if (this->m_max_travel_dist <= 0.f)
    {
      os << "- maximum travel distance  (m): not enabled" << std::endl;
    }
    else
    {
      os << "- maximum travel distance  (m): " << this->m_max_travel_dist
         << std::endl;
    }
    if (this->m_max_travel_heading <= 0.f)
    {
      os << "- maximum travel heading (deg): not enabled" << std::endl;
    }
    else
    {
      os << "- maximum travel heading (deg): "
         << geometry::radToDeg(this->m_max_travel_heading) << std::endl;
    }
  }

};  // class MergingCriteriaInliersRatio_<TrackerType_>

/**
 * @brief merging criteria based on time (latest tracker's scan message vs
 *        previous merged scan)
 *
 * m_min_time_interval defines the minimum time interval in seconds to
 * trigger the merge boolean variable on compute (s)
 *
 * m_max_ratio_inliers define the thresholds to trigger the merging boolean
 * variable on compute() (0..1)
 *
 * @tparam TrackerType_ tracker algorithm type
 */
template <typename TrackerType_>
class MergingCriteriaTime_ : public MergingCriteria_<TrackerType_>
{
 public:

  using ThisType = MergingCriteriaTime_<TrackerType_>;
  using BaseType = MergingCriteria_<TrackerType_>;

  using TrackerType = TrackerType_;
  using AlignerBaseTypeStats = typename TrackerType::AlignerBaseTypeStats;
  using EstimateType = typename TrackerType::EstimateType;

 public:

  double m_min_time_interval =
      1.0;  //!< minimum time interval to merge the local map (s)
  float m_max_ratio_inliers =
      0.80f;  //!< maximum ratio of inliers ratio to merge the local map (0..1)

 protected:

  double m_t_prev_ = -1.0;  //!< previous scan timestamp (s)

 public:

  virtual void compute() override
  {
    if (!this->m_tracker_)
    {
      throw std::runtime_error(
          "MergingCriteriaTime_::compute | no tracker algorithm set...");
    }

    if (this->m_force_merge_)
    {
      this->m_force_merge_ = false;
      this->m_merge_ = true;
      return;
    }

    this->m_merge_ = false;

    const msg::LaserScanPtr& msg = this->m_tracker_->getScanPtr();

    if (!msg)
    {
      throw std::runtime_error(
          "MergingCriteriaTime_::compute | no scan message on the tracker "
          "algorithm set...");
    }

    const AlignerBaseTypeStats& stats = this->m_tracker_->getAlignerStats();

    if ((msg->m_header.m_t - m_t_prev_ > m_min_time_interval) ||
        (stats.m_inliers_ratio < this->m_max_ratio_inliers))
    {
      this->m_merge_ = true;
      return;
    }
  }  // virtual void MergingCriteriaTime_::compute() override

  /**
   * @brief reset internal state of the criteria supervision class, including
   *        the m_t_prev_ timestamp of the reference scan message
   */
  virtual void reset() override
  {
    const msg::LaserScanPtr& msg = this->m_tracker_->getScanPtr();

    if (!msg)
    {
      throw std::runtime_error(
          "MergingCriteriaTime_::reset | no scan message on the tracker "
          "algorithm set...");
    }

    m_t_prev_ = msg->m_header.m_t;

    BaseType::reset();
  }

 protected:

  virtual void print(std::ostream& os) const
  {
    os << "- merging criteria type       : time" << std::endl;
    os << "- minimum time interval (s)   : " << this->m_min_time_interval
       << std::endl;
    if (this->m_max_ratio_inliers <= 0.f)
    {
      os << "- maximum inliers ratio (0..1): not enabled" << std::endl;
    }
    else
    {
      os << "- maximum inliers ratio (0..1): " << this->m_max_ratio_inliers
         << std::endl;
    }
  }

};  // class MergingCriteriaTime_<TrackerType_>

}  // namespace slam

}  // namespace inesctec_mrdt_slam_distmap_2d
