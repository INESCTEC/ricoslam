#pragma once

// Eigen
#include <Eigen/Core>

#include "inesctec_mrdt_slam_distmap_2d/geometry/geometry_2d.hpp"

namespace inesctec_mrdt_slam_distmap_2d
{

namespace pcl
{

/**
 * @brief point field accumulator to compute in an iterative / accumulative way
 *        the mean and covariances (you can add or subtract to the accumulator)
 * @note analyse further the numerical stability when using the point
 *       accumulator with float types...
 * @tparam fieldIdx point field component index
 * @tparam PointType_ point type
 */
template <int fieldIdx, typename PointType_>
struct PointFieldAccumulator_
{
 public:

  using ThisType = PointFieldAccumulator_<fieldIdx, PointType_>;
  using PointType = PointType_;
  using VectorType = typename PointType::template ValueTypeAt<fieldIdx>;
  using Scalar = typename PointType::ComponentsType;

  using CovMatrixType = typename geometry::MatrixN_<Scalar, PointType::Dim>;

 protected:

  VectorType m_field_acc_;
  Scalar m_num_pts_;

  CovMatrixType m_field_xx_acc_;

 public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PointFieldAccumulator_() { reset(); }

  inline void reset()
  {
    m_field_acc_.setZero();
    m_field_xx_acc_.setZero();
    m_num_pts_ = Scalar(0);
  }

  inline void add(const PointType& point)
  {
    m_field_acc_ += point.template value<fieldIdx>();
    m_num_pts_ += Scalar(1);
    m_field_xx_acc_ += point.template value<fieldIdx>() *
                       point.template value<fieldIdx>().transpose();
  }

  inline void add(const ThisType& other)
  {  // assuming non-overlapping accumulators...
    m_field_acc_ += other.m_field_acc_;
    m_num_pts_ += other.m_num_pts_;
    m_field_xx_acc_ += other.m_field_xx_acc_;
  }

  inline void sub(const PointType& point)
  {
    if (m_num_pts_ > Scalar(0))
    {
      m_field_acc_ -= point.template value<fieldIdx>();
      m_num_pts_ -= Scalar(1);
      m_field_xx_acc_ -= point.template value<fieldIdx>() *
                         point.template value<fieldIdx>().transpose();
    }
    else
    {
      reset();
    }
  }

  inline void sub(const ThisType& other)
  {  // assuming *this is a superset of other, where other completly overlaps
     // this...
    if (m_num_pts_ > other.m_num_pts_)
    {
      m_field_acc_ -= other.m_field_acc_;
      m_num_pts_ -= other.m_num_pts_;
      m_field_xx_acc_ -= other.m_field_xx_acc_;
    }
    else
    {
      reset();
    }
  }

  inline const VectorType& getAcc() const { return m_field_acc_; }

  inline VectorType getMean() const
  {
    if (m_num_pts_ > Scalar(0))
    {
      return m_field_acc_ / m_num_pts_;
    }
    else
    {
      return VectorType::Zero();
    }
  }

  inline const CovMatrixType& getXXAcc() const { return m_field_xx_acc_; }

  inline CovMatrixType getCovariance() const
  {
    if (m_num_pts_ > Scalar(1))
    {
      VectorType mean = getMean();

      return m_field_xx_acc_ / m_num_pts_ - mean * mean.transpose();
    }
    else
    {
      return CovMatrixType::Zero();
    }
  }

  inline Scalar getNumPts() const { return m_num_pts_; }

};  // struct PointAccumulator_<PointType_>

}  // namespace pcl

}  // namespace inesctec_mrdt_slam_distmap_2d
