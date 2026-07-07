#pragma once

#include <Eigen/Core>

#include "inesctec_mrdt_slam_distmap_2d/geometry/geometry_2d.hpp"
#include "inesctec_mrdt_slam_distmap_2d/slam/msg/laser_scan.hpp"

namespace inesctec_mrdt_slam_distmap_2d
{
namespace slam
{

template <typename EstimateType_>
class TriggeringCriteria_
{
 public:

  using ThisType = TriggeringCriteria_<EstimateType_>;
  using EstimateType = EstimateType_;
  using Scalar = typename EstimateType::Scalar;

 protected:

  bool m_trigger_ = false;        //!< boolean trigger
  bool m_force_trigger_ = false;  //!< force trigger (ignoring compute!),
                                  //!< must be implemented in criteria!

  double m_msg_t_in_reset_ = -1.;  //!< scan message timestamp (s) upon reset
  EstimateType m_robot_in_reset_ =
      EstimateType::Identity();  //!< initialization robot pose
                                 //!< (SE(2) transformation)
  EstimateType m_robot_in_reset_inv_ =
      EstimateType::Identity();  //!< inverse of the initialization robot pose
                                 //!< (SE(2) transformation)

 public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief virtual method to compute if trigger boolean should be triggered
   *        based on different metrics
   * @param[in] msg laser scan message
   * @param[in] pose current robot pose w.r.t. same frame as m_robot_in_reset_
   * @param[in] inliers_ratio current inliers ratio of the matcher
   */
  virtual void compute(const msg::LaserScan& msg, const EstimateType& pose,
                       const Scalar inliers_ratio) = 0;

  /**
   * @brief accessor method to trigger boolean
   * @return true trigger the associated action (implemented by external class)
   * @return false otherwise
   */
  inline bool isToTrigger() const { return m_trigger_; }

  /**
   * @brief virtual method to check if trigger boolean should be triggered
   *        based on different metrics without changing the internal state of
   *        the triggering class
   * @param[in] msg laser scan message
   * @param[in] pose current robot pose w.r.t. same frame as m_robot_in_reset_
   * @param[in] inliers_ratio current inliers ratio of the matcher
   * @return true would generate a trigger action
   * @return false otherwise
   */
  inline virtual bool isToTrigger(const msg::LaserScan& msg,
                                  const EstimateType& pose,
                                  const Scalar inliers_ratio) const = 0;

  /**
   * @brief force triggering on next scan message, ignoring compute instructions
   *        (must be implemented in criteria compute function)
   */
  inline void forceTrigger() { m_force_trigger_ = true; }

  /**
   * @brief reset the trigger boolean and the internal state of the criteria
   *        supervision class (m_msg_t_in_reset_, m_robot_in_reset_, and,
   *        consequently, m_robot_in_reset_inv_)
   * @param[in] msg laser scan message
   * @param[in] pose current robot pose w.r.t. same frame as m_robot_in_reset_
   */
  virtual void reset(const msg::LaserScan& msg, const EstimateType& pose)
  {
    m_trigger_ = false;
    m_msg_t_in_reset_ = msg.m_header.m_t;
    m_robot_in_reset_ = pose;
    m_robot_in_reset_inv_ = m_robot_in_reset_.inverse();
  }

 public:

  double getTimestampInReset() const { return m_msg_t_in_reset_; }
  const EstimateType& getRobotPoseInReset() const { return m_robot_in_reset_; }

 public:

  friend std::ostream& operator<<(std::ostream& os, const ThisType& criteria)
  {
    criteria.print(os);
    return os;
  }

 protected:

  virtual void print(std::ostream&) const {}
};

template <typename EstimateType_>
class TriggeringCriteriaEmpty_ : public TriggeringCriteria_<EstimateType_>
{
 public:

  using ThisType = TriggeringCriteriaEmpty_<EstimateType_>;
  using BaseType = TriggeringCriteria_<EstimateType_>;

  using BaseType::isToTrigger;
  using typename BaseType::EstimateType;
  using typename BaseType::Scalar;

 public:

  virtual void compute(const msg::LaserScan& msg, const EstimateType& pose,
                       const Scalar inliers_ratio) override
  {
    if (this->m_force_trigger_)
    {
      this->m_force_trigger_ = false;
      this->m_trigger_ = true;
      return;
    }

    this->m_trigger_ = false;
  }

  inline virtual bool isToTrigger(const msg::LaserScan& msg,
                                  const EstimateType& pose,
                                  const Scalar inliers_ratio) const override
  {
    return false;
  }

 protected:

  virtual void print(std::ostream& os) const override
  {
    os << "- triggering criteria type: empty" << std::endl;
  }
};

template <typename EstimateType_>
class TriggeringCriteriaRotation_ : public TriggeringCriteria_<EstimateType_>
{
 public:

  using ThisType = TriggeringCriteriaRotation_<EstimateType_>;
  using BaseType = TriggeringCriteria_<EstimateType_>;

  using BaseType::isToTrigger;
  using typename BaseType::EstimateType;
  using typename BaseType::Scalar;

 public:

  Scalar m_min_travel_heading =
      Scalar(-1.0);  //!< minimum travel heading threshold (rad)

 public:

  virtual void compute(const msg::LaserScan& msg, const EstimateType& pose,
                       const Scalar inliers_ratio) override
  {
    if (this->m_force_trigger_)
    {
      this->m_force_trigger_ = false;
      this->m_trigger_ = true;
      return;
    }

    this->m_trigger_ = false;

    const Eigen::Vector3f delta =
        geometry::t2v(this->m_robot_in_reset_inv_ * pose);

    Scalar delta_heading = std::fabs(delta(2));

    if (delta_heading > m_min_travel_heading)
    {
      this->m_trigger_ = true;
      return;
    }
  }

  inline virtual bool isToTrigger(const msg::LaserScan& msg,
                                  const EstimateType& pose,
                                  const Scalar inliers_ratio) const override
  {
    const Eigen::Vector3f delta =
        geometry::t2v(this->m_robot_in_reset_inv_ * pose);

    Scalar delta_heading = std::fabs(delta(2));

    if (delta_heading > m_min_travel_heading)
    {
      return true;
    }

    return false;
  }

 protected:

  virtual void print(std::ostream& os) const override
  {
    os << "- triggering criteria type    : rotation" << std::endl;
    os << "- minimum travel heading (deg): "
       << geometry::radToDeg(this->m_min_travel_heading) << std::endl;
  }
};

template <typename EstimateType_>
class TriggeringCriteriaTranslation_ : public TriggeringCriteria_<EstimateType_>
{
 public:

  using ThisType = TriggeringCriteriaRotation_<EstimateType_>;
  using BaseType = TriggeringCriteria_<EstimateType_>;

  using BaseType::isToTrigger;
  using typename BaseType::EstimateType;
  using typename BaseType::Scalar;

 public:

  Scalar m_min_travel_dist =
      Scalar(-1.);  //!< minimum travel distance threshold (m)

 public:

  virtual void compute(const msg::LaserScan& msg, const EstimateType& pose,
                       const Scalar inliers_ratio) override
  {
    if (this->m_force_trigger_)
    {
      this->m_force_trigger_ = false;
      this->m_trigger_ = true;
      return;
    }

    this->m_trigger_ = false;

    const Eigen::Vector3f delta =
        geometry::t2v(this->m_robot_in_reset_inv_ * pose);

    Scalar delta_dist = delta.head(2).norm();

    if (delta_dist > m_min_travel_dist)
    {
      this->m_trigger_ = true;
      return;
    }
  }

  inline virtual bool isToTrigger(const msg::LaserScan& msg,
                                  const EstimateType& pose,
                                  const Scalar inliers_ratio) const override
  {
    const Eigen::Vector3f delta =
        geometry::t2v(this->m_robot_in_reset_inv_ * pose);

    Scalar delta_dist = delta.head(2).norm();

    if (delta_dist > m_min_travel_dist)
    {
      return true;
    }

    return false;
  }

 protected:

  virtual void print(std::ostream& os) const override
  {
    os << "- triggering criteria type    : translation" << std::endl;
    os << "- minimum travel distance  (m): " << this->m_min_travel_dist
       << std::endl;
  }
};

template <typename EstimateType_>
class TriggeringCriteriaViewpoint_ : public TriggeringCriteria_<EstimateType_>
{
 public:

  using ThisType = TriggeringCriteriaViewpoint_<EstimateType_>;
  using BaseType = TriggeringCriteria_<EstimateType_>;

  using BaseType::isToTrigger;
  using typename BaseType::EstimateType;
  using typename BaseType::Scalar;

 public:

  Scalar m_min_travel_dist =
      Scalar(-1.);  //!< minimum travel distance threshold (m)
  Scalar m_min_travel_heading =
      Scalar(-1.);  //!< minimum travel heading threshold (rad)

 public:

  virtual void compute(const msg::LaserScan& msg, const EstimateType& pose,
                       const Scalar inliers_ratio) override
  {
    if (this->m_force_trigger_)
    {
      this->m_force_trigger_ = false;
      this->m_trigger_ = true;
      return;
    }

    this->m_trigger_ = false;

    const Eigen::Vector3f delta =
        geometry::t2v(this->m_robot_in_reset_inv_ * pose);

    Scalar delta_dist = delta.head(2).norm();
    Scalar delta_heading = std::fabs(delta(2));

    if (delta_dist > m_min_travel_dist)
    {
      this->m_trigger_ = true;
      return;
    }

    if (delta_heading > m_min_travel_heading)
    {
      this->m_trigger_ = true;
      return;
    }
  }

  inline virtual bool isToTrigger(const msg::LaserScan& msg,
                                  const EstimateType& pose,
                                  const Scalar inliers_ratio) const override
  {
    const Eigen::Vector3f delta =
        geometry::t2v(this->m_robot_in_reset_inv_ * pose);

    Scalar delta_dist = delta.head(2).norm();
    Scalar delta_heading = std::fabs(delta(2));

    if (delta_dist > m_min_travel_dist)
    {
      return true;
    }

    if (delta_heading > m_min_travel_heading)
    {
      return true;
    }

    return false;
  }

 protected:

  virtual void print(std::ostream& os) const override
  {
    os << "- triggering criteria type    : viewpoint" << std::endl;
    os << "- minimum travel heading (deg): "
       << geometry::radToDeg(this->m_min_travel_heading) << std::endl;
    os << "- minimum travel distance  (m): " << this->m_min_travel_dist
       << std::endl;
  }
};

template <typename EstimateType_>
class TriggeringCriteriaGridCell_ : public TriggeringCriteria_<EstimateType_>
{
 public:

  using ThisType = TriggeringCriteriaRotation_<EstimateType_>;
  using BaseType = TriggeringCriteria_<EstimateType_>;

  using BaseType::isToTrigger;
  using typename BaseType::EstimateType;
  using typename BaseType::Scalar;

 public:

  Scalar m_cell_size_2 = Scalar(-1.);  //!< 0.5 x grid cell size (m) (half!)

 public:

  virtual void compute(const msg::LaserScan& msg, const EstimateType& pose,
                       const Scalar inliers_ratio) override
  {
    if (this->m_force_trigger_)
    {
      this->m_force_trigger_ = false;
      this->m_trigger_ = true;
      return;
    }

    this->m_trigger_ = false;

    const Eigen::Vector3f delta =
        geometry::t2v(this->m_robot_in_reset_inv_ * pose);

    Scalar delta_x_abs = std::fabs(delta.x());
    Scalar delta_y_abs = std::fabs(delta.y());

    if (delta_x_abs > m_cell_size_2)
    {
      this->m_trigger_ = true;
      return;
    }

    if (delta_y_abs > m_cell_size_2)
    {
      this->m_trigger_ = true;
      return;
    }
  }

  inline virtual bool isToTrigger(const msg::LaserScan& msg,
                                  const EstimateType& pose,
                                  const Scalar inliers_ratio) const override
  {
    const Eigen::Vector3f delta =
        geometry::t2v(this->m_robot_in_reset_inv_ * pose);

    Scalar delta_x_abs = std::fabs(delta.x());
    Scalar delta_y_abs = std::fabs(delta.y());

    if (delta_x_abs > m_cell_size_2)
    {
      return true;
    }

    if (delta_y_abs > m_cell_size_2)
    {
      return true;
    }

    return false;
  }

 protected:

  virtual void print(std::ostream& os) const override
  {
    os << "- triggering criteria type: grid cell" << std::endl;
    os << "- grid cell size       (m): " << this->m_cell_size_2 * 2
       << std::endl;
  }
};

template <typename EstimateType_>
class TriggeringCriteriaInliersRatio_
    : public TriggeringCriteria_<EstimateType_>
{
 public:

  using ThisType = TriggeringCriteriaRotation_<EstimateType_>;
  using BaseType = TriggeringCriteria_<EstimateType_>;

  using BaseType::isToTrigger;
  using typename BaseType::EstimateType;
  using typename BaseType::Scalar;

 public:

  Scalar m_min_travel_dist = Scalar(-1.);     //!< minimum travel distance (m)
  Scalar m_min_travel_heading = Scalar(-1.);  //!< minimum travel heading (rad)
  Scalar m_max_ratio_inliers =
      Scalar(-1.);  //!< maximum ratio of inliers ratio (0..1)
  Scalar m_max_travel_dist =
      Scalar(-1.);  //!< maximum travel distance threshold (m)
  Scalar m_max_travel_heading =
      Scalar(-1.);  //!< maximum travel heading threshold (rad)

 public:

  virtual void compute(const msg::LaserScan& msg, const EstimateType& pose,
                       const Scalar inliers_ratio) override
  {
    if (this->m_force_trigger_)
    {
      this->m_force_trigger_ = false;
      this->m_trigger_ = true;
      return;
    }

    this->m_trigger_ = false;

    const Eigen::Vector3f delta =
        geometry::t2v(this->m_robot_in_reset_inv_ * pose);

    Scalar delta_dist = delta.head(2).norm();
    Scalar delta_heading = std::fabs(delta(2));

    if ((((delta_dist > this->m_min_travel_dist) ||
          (this->m_min_travel_dist <= Scalar(0.))) &&
         ((delta_heading > this->m_min_travel_heading) ||
          (this->m_min_travel_heading <= Scalar(0.))) &&
         (inliers_ratio < this->m_max_ratio_inliers)) ||
        ((this->m_max_travel_dist < delta_dist) &&
         (this->m_max_travel_dist > Scalar(0.))) ||
        ((this->m_max_travel_heading < delta_heading) &&
         (this->m_max_travel_heading > Scalar(0.))))
    {
      this->m_trigger_ = true;
      return;
    }
  }

  inline virtual bool isToTrigger(const msg::LaserScan& msg,
                                  const EstimateType& pose,
                                  const Scalar inliers_ratio) const override
  {
    const Eigen::Vector3f delta =
        geometry::t2v(this->m_robot_in_reset_inv_ * pose);

    Scalar delta_dist = delta.head(2).norm();
    Scalar delta_heading = std::fabs(delta(2));

    if ((((delta_dist > this->m_min_travel_dist) ||
          (this->m_min_travel_dist <= Scalar(0.))) &&
         ((delta_heading > this->m_min_travel_heading) ||
          (this->m_min_travel_heading <= Scalar(0.))) &&
         (inliers_ratio < this->m_max_ratio_inliers)) ||
        ((this->m_max_travel_dist < delta_dist) &&
         (this->m_max_travel_dist > Scalar(0.))) ||
        ((this->m_max_travel_heading < delta_heading) &&
         (this->m_max_travel_heading > Scalar(0.))))
    {
      return true;
    }

    return false;
  }

 protected:

  virtual void print(std::ostream& os) const override
  {
    os << "- triggering criteria type    : inliers ratio" << std::endl;
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
};

template <typename EstimateType_>
class TriggeringCriteriaTime_ : public TriggeringCriteria_<EstimateType_>
{
 public:

  using ThisType = TriggeringCriteriaViewpoint_<EstimateType_>;
  using BaseType = TriggeringCriteria_<EstimateType_>;

  using BaseType::isToTrigger;
  using typename BaseType::EstimateType;
  using typename BaseType::Scalar;

 public:

  double m_min_time_interval = 1.0;  //!< minimum time interval (s)
  float m_max_ratio_inliers =
      Scalar(-1.);  //!< maximum ratio of inliers ratio (0..1)

 public:

  virtual void compute(const msg::LaserScan& msg, const EstimateType& pose,
                       const Scalar inliers_ratio) override
  {
    if (this->m_force_trigger_)
    {
      this->m_force_trigger_ = false;
      this->m_trigger_ = true;
      return;
    }

    this->m_trigger_ = false;

    if ((msg.m_header.m_t - this->m_msg_t_in_reset_ > m_min_time_interval) ||
        (inliers_ratio < this->m_max_ratio_inliers))
    {
      this->m_trigger_ = true;
      return;
    }
  }

  inline virtual bool isToTrigger(const msg::LaserScan& msg,
                                  const EstimateType& pose,
                                  const Scalar inliers_ratio) const override
  {
    if ((msg.m_header.m_t - this->m_msg_t_in_reset_ > m_min_time_interval) ||
        (inliers_ratio < this->m_max_ratio_inliers))
    {
      return true;
    }

    return false;
  }

 protected:

  virtual void print(std::ostream& os) const override
  {
    os << "- triggering criteria type    : time" << std::endl;
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
};

}  // namespace slam
}  // namespace inesctec_mrdt_slam_distmap_2d
