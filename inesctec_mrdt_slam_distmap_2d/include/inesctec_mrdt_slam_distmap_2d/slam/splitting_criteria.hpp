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
 * @brief splitting criteria template (virtualized compute method, generic API)
 *        to trigger the split of the current local map onto a new node / map
 * @tparam TrackerType_ tracker algorithm type
 */
template <typename TrackerType_>
class SplittingCriteria_
{
 public:

  using ThisType = SplittingCriteria_<TrackerType_>;
  using TrackerType = TrackerType_;

 protected:

  TrackerType* m_tracker_ = nullptr;  //!< tracker algorithm (to get the robot
                                      //!< w.r.t. local map transformation)
  bool m_split_ = false;              //!< split boolean trigger
  bool m_force_split_ = false;  //!< force split trigger (ignoring compute!),
                                //!< must be implemented in criteria!

 public:

  /**
   * @brief virtual method to compute if the split boolean should be triggered
   *        based on different metrics
   */
  virtual void compute() = 0;

  /**
   * @brief accessor method to split boolean trigeer
   * @return true tracker should split / trigger the creation of a new local map
   * @return false otherwise
   */
  inline bool isToSplit() const { return m_split_; }

  /**
   * @brief force split on next scan message, ignoring compute instructions
   *        (must be implemented in criteria compute function)
   */
  inline void forceSplit() { m_force_split_ = true; }

  /**
   * @brief reset some internal state of the criteria supervision class, such as
   *        the m_t_prev_ timestamp of the reference scan message
   */
  virtual void reset() {}

  /**
   * @brief set the tracker algorithm (to ask for the robot w.r.t. the local
   *        map SE(2) transformation and, possibily, its latest stats of the
   *        alignment)
   * @param[in] tracker tracker algorithm
   */
  inline void setTracker(TrackerType* const tracker) { m_tracker_ = tracker; }

 public:

  friend std::ostream& operator<<(std::ostream& os,
                                  const ThisType& splitting_criteria)
  {
    splitting_criteria.print(os);
    return os;
  }

 protected:

  virtual void print(std::ostream&) const {}

};  // class SplittingCriteria_<TrackerType_>

/**
 * @brief empty splitting criteria (only by forcing through external entities)
 * @tparam TrackerType_ tracker algorithm type
 */
template <typename TrackerType_>
class SplittingCriteriaEmpty_ : public SplittingCriteria_<TrackerType_>
{
 public:

  using ThisType = SplittingCriteriaEmpty_<TrackerType_>;
  using BaseType = SplittingCriteria_<TrackerType_>;

  using TrackerType = TrackerType_;

 public:

  virtual void compute() override
  {
    if (!this->m_tracker_)
    {
      throw std::runtime_error(
          "SplittingCriteriaEmpty_::compute | no tracker algorithm set...");
    }

    if (this->m_force_split_)
    {
      this->m_force_split_ = false;
      this->m_split_ = true;
      return;
    }
  }  // virtual void SplittingCriteriaEmpty_::compute() override

 protected:

  virtual void print(std::ostream& os) const
  {
    os << "- splitting criteria type: empty (only forcing)" << std::endl;
  }

};  // class SplittingCriteriaEmpty_<TrackerType_>

/**
 * @brief splitting criteria based on rotation (minimum travel heading of the
 *        current robot heading w.r.t. the local map)
 *
 * m_min_travel_heading defines the minimum threshold to trigger the splitting
 * boolean variable on compute() (rad)
 *
 * @tparam TrackerType_ tracker algorithm type
 */
template <typename TrackerType_>
class SplittingCriteriaRotation_ : public SplittingCriteria_<TrackerType_>
{
 public:

  using ThisType = SplittingCriteriaRotation_<TrackerType_>;
  using BaseType = SplittingCriteria_<TrackerType_>;

  using TrackerType = TrackerType_;
  using EstimateType = typename TrackerType::EstimateType;

 public:

  float m_min_travel_heading = geometry::degToRad(
      30.0f);  //!< minimum travel heading to split the local map (rad)

 public:

  virtual void compute() override
  {
    if (!this->m_tracker_)
    {
      throw std::runtime_error(
          "SplittingCriteriaRotation_::compute | "
          "no tracker algorithm set...");
    }

    if (this->m_force_split_)
    {
      this->m_force_split_ = false;
      this->m_split_ = true;
      return;
    }

    this->m_split_ = false;

    const EstimateType& robot_in_map = this->m_tracker_->getRobotInLocalMap();
    const Eigen::Vector3f delta = geometry::t2v(robot_in_map);

    float delta_heading = std::fabs(delta(2));

    if (delta_heading > m_min_travel_heading)
    {
      this->m_split_ = true;
      return;
    }
  }  // virtual void SplittingCriteriaRotation_::compute() override

 protected:

  virtual void print(std::ostream& os) const
  {
    os << "- splitting criteria type     : rotation" << std::endl;
    os << "- minimum travel heading (deg): "
       << geometry::radToDeg(this->m_min_travel_heading) << std::endl;
  }

};  // class SplittingCriteriaRotation_<TrackerType_>

/**
 * @brief splitting criteria based on translation (minimum travel distance of
 *        the current robot pose w.r.t. the local map)
 *
 * m_min_travel_dist defines the minimum threshold to trigger the splitting
 * boolean variable on compute() (m)
 *
 * @tparam TrackerType_ tracker algorithm type
 */
template <typename TrackerType_>
class SplittingCriteriaTranslation_ : public SplittingCriteria_<TrackerType_>
{
 public:

  using ThisType = SplittingCriteriaTranslation_<TrackerType_>;
  using BaseType = SplittingCriteria_<TrackerType_>;

  using TrackerType = TrackerType_;
  using EstimateType = typename TrackerType::EstimateType;

 public:

  float m_min_travel_dist =
      0.50f;  //!< minimum travel distance to split the local map (m)

 public:

  virtual void compute() override
  {
    if (!this->m_tracker_)
    {
      throw std::runtime_error(
          "SplittingCriteriaTranslation_::compute | "
          "no tracker algorithm set...");
    }

    if (this->m_force_split_)
    {
      this->m_force_split_ = false;
      this->m_split_ = true;
      return;
    }

    this->m_split_ = false;

    const EstimateType& robot_in_map = this->m_tracker_->getRobotInLocalMap();
    const Eigen::Vector3f delta = geometry::t2v(robot_in_map);

    float delta_dist = delta.head(2).norm();

    if (delta_dist > m_min_travel_dist)
    {
      this->m_split_ = true;
      return;
    }
  }  // virtual void SplittingCriteriaTranslation_::compute() override

 protected:

  virtual void print(std::ostream& os) const
  {
    os << "- splitting criteria type     : translation" << std::endl;
    os << "- minimum travel distance  (m): " << this->m_min_travel_dist
       << std::endl;
  }

};  // class SplittingCriteriaTranslation_<TrackerType_>

/**
 * @brief splitting criteria based on viewpoint (minimum travel heading and/or
 *        distance of the current robot pose w.r.t. the local map)
 *
 * m_min_travel_dist (m), m_min_travel_heading (rad) define the thresholds to
 * trigger the splitting boolean variable on compute()
 *
 * @tparam TrackerType_ tracker algorithm type
 */
template <typename TrackerType_>
class SplittingCriteriaViewpoint_ : public SplittingCriteria_<TrackerType_>
{
 public:

  using ThisType = SplittingCriteriaViewpoint_<TrackerType_>;
  using BaseType = SplittingCriteria_<TrackerType_>;

  using TrackerType = TrackerType_;
  using EstimateType = typename TrackerType::EstimateType;

 public:

  float m_min_travel_dist =
      0.50f;  //!< minimum travel distance threshold to split the local map (m)
  float m_min_travel_heading =
      geometry::degToRad(30.0f);  //!< minimum travel heading threshold to split
                                  //!< the local map (rad)

 public:

  virtual void compute() override
  {
    if (!this->m_tracker_)
    {
      throw std::runtime_error(
          "SplittingCriteriaViewpoint_::compute | "
          "no tracker algorithm set...");
    }

    if (this->m_force_split_)
    {
      this->m_force_split_ = false;
      this->m_split_ = true;
      return;
    }

    this->m_split_ = false;

    const EstimateType& robot_in_map = this->m_tracker_->getRobotInLocalMap();
    const Eigen::Vector3f delta = geometry::t2v(robot_in_map);

    float delta_dist = delta.head(2).norm();
    float delta_heading = std::fabs(delta(2));

    if (delta_dist > m_min_travel_dist)
    {
      this->m_split_ = true;
      return;
    }

    if (delta_heading > m_min_travel_heading)
    {
      this->m_split_ = true;
      return;
    }
  }  // virtual void SplittingCriteriaViewpoint_::compute() override

 protected:

  virtual void print(std::ostream& os) const
  {
    os << "- splitting criteria type     : viewpoint" << std::endl;
    os << "- minimum travel heading (deg): "
       << geometry::radToDeg(this->m_min_travel_heading) << std::endl;
    os << "- minimum travel distance  (m): " << this->m_min_travel_dist
       << std::endl;
  }

};  // class SplittingCriteriaViewpoint_<TrackerType_>

/**
 * @brief splitting criteria based on the inliers ratio from the tracker's
 *        alignenment results
 *
 * m_max_ratio_inliers define the thresholds to trigger the splitting boolean
 * variable on compute() (0..1)
 *
 * m_max_travel_dist can be enabled (if greater than 0.f) to define a maximum
 * travel distance to avoid sparsity on splitting (m)
 *
 * m_max_travel_heading can be enabled (if greater than 0.f) to define a maximum
 * travel angular distance to avoid sparsity on splitting (rad)
 *
 * m_min_travel_dist (m), m_min_travel_heading (rad) define minimum thresholds
 * to avoid splitting without moving
 *
 * @tparam TrackerType_ tracker algorithm type
 */
template <typename TrackerType_>
class SplittingCriteriaInliersRatio_ : public SplittingCriteria_<TrackerType_>
{
 public:

  using ThisType = SplittingCriteriaInliersRatio_<TrackerType_>;
  using BaseType = SplittingCriteria_<TrackerType_>;

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
      0.60f;  //!< maximum ratio of inliers ratio to split the local map (0..1)
  float m_max_travel_dist = -1.f;     //!< maximum travel distance threshold to
                                      //!< force splitting the local map (m)
  float m_max_travel_heading = -1.f;  //!< maximum travel heading threshold to
                                      //!< force splitting the local map (rad)

 public:

  virtual void compute() override
  {
    if (!this->m_tracker_)
    {
      throw std::runtime_error(
          "SplittingCriteriaInliersRatio_::compute | "
          "no tracker algorithm set...");
    }

    if (this->m_force_split_)
    {
      this->m_force_split_ = false;
      this->m_split_ = true;
      return;
    }

    this->m_split_ = false;

    const AlignerBaseTypeStats& stats = this->m_tracker_->getAlignerStats();

    const EstimateType& robot_in_map = this->m_tracker_->getRobotInLocalMap();
    const Eigen::Vector3f delta = geometry::t2v(robot_in_map);

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
      this->m_split_ = true;
    }
  }  // virtual void SplittingCriteriaInliersRatio_::compute() override

 protected:

  virtual void print(std::ostream& os) const
  {
    os << "- splitting criteria type     : inliers ratio" << std::endl;
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

};  // class SplittingCriteriaInliersRatio_<TrackerType_>

/**
 * @brief splitting criteria based on time (latest tracker's scan message vs
 *        previous splitted scan)
 *
 * m_min_time_interval defines the minimum time interval in seconds to
 * trigger the splitting boolean variable on compute (s)
 *
 * m_max_ratio_inliers define the thresholds to trigger the splitting boolean
 * variable on compute() (0..1)
 *
 * @tparam TrackerType_ tracker algorithm type
 */
template <typename TrackerType_>
class SplittingCriteriaTime_ : public SplittingCriteria_<TrackerType_>
{
 public:

  using ThisType = SplittingCriteriaTime_<TrackerType_>;
  using BaseType = SplittingCriteria_<TrackerType_>;

  using TrackerType = TrackerType_;
  using AlignerBaseTypeStats = typename TrackerType::AlignerBaseTypeStats;
  using EstimateType = typename TrackerType::EstimateType;

 public:

  double m_min_time_interval =
      5.0;  //!< minimum time interval to split the local map (s)
  float m_max_ratio_inliers =
      0.60f;  //!< maximum ratio of inliers ratio to split the local map (0..1)

 protected:

  double m_t_prev_ = -1.0;  //!< previous scan timestamp (s)

 public:

  virtual void compute() override
  {
    if (!this->m_tracker_)
    {
      throw std::runtime_error(
          "SplittingCriteriaTime_::compute | no tracker algorithm set...");
    }

    if (this->m_force_split_)
    {
      this->m_force_split_ = false;
      this->m_split_ = true;
      return;
    }

    this->m_split_ = false;

    const msg::LaserScanPtr& msg = this->m_tracker_->getScanPtr();

    if (!msg)
    {
      throw std::runtime_error(
          "SplittingCriteriaTime_::compute | no scan message on the tracker "
          "algorithm set...");
    }

    const AlignerBaseTypeStats& stats = this->m_tracker_->getAlignerStats();

    if ((msg->m_header.m_t - m_t_prev_ > m_min_time_interval) ||
        (stats.m_inliers_ratio < this->m_max_ratio_inliers))
    {
      this->m_split_ = true;
      return;
    }
  }  // virtual void SplittingCriteriaTime_::compute() override

  /**
   * @brief reset some internal state of the criteria supervision class, such as
   *        the m_t_prev_ timestamp of the reference scan message
   */
  virtual void reset() override
  {
    const msg::LaserScanPtr& msg = this->m_tracker_->getScanPtr();

    if (!msg)
    {
      throw std::runtime_error(
          "SplittingCriteriaTime_::reset | no scan message on the tracker "
          "algorithm set...");
    }

    m_t_prev_ = msg->m_header.m_t;
  }

 protected:

  virtual void print(std::ostream& os) const
  {
    os << "- splitting criteria type     : time" << std::endl;
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

};  // class SplittingCriteriaTime_<TrackerType_>

}  // namespace slam

}  // namespace inesctec_mrdt_slam_distmap_2d
