#pragma once

// SRRG Software
#include <srrg2_core/srrg_geometry/geometry2d.h>
#include <srrg2_core/srrg_geometry/geometry_defs.h>
#include <srrg2_core/srrg_viewer/viewer_canvas.h>
#include <srrg2_solver/solver_core/error_factor.h>
#include <srrg2_solver/solver_core/factor_base.h>
#include <srrg2_solver/solver_core/measurement_owner.h>
#include <srrg2_solver/solver_core/robustifier.h>

#include "inesctec_mrdt_slam_distmap_2d/srrg2_solver/variable_se2_distance_map.hpp"

namespace srrg2_solver
{

/**
 * @brief templified SE2 prior error factor based on
 *        srrg2_solver::SE2PriorErrorFactor
 * @tparam VariableSE2DistanceMapRightType_ distance map-based variable type
 *                                          (must be right-sided
 *                                          pertubation-based!)
 */
template <typename VariableSE2DistanceMapRightType_>
class SE2DistanceMapPriorErrorFactor_
    : public ErrorFactor_<VariableSE2DistanceMapRightType_::PerturbationDim,
                          VariableSE2DistanceMapRightType_>,
      public MeasurementOwnerEigen_<
          typename VariableSE2DistanceMapRightType_::EstimateType>
{
 public:

  using BaseFactorType =
      ErrorFactor_<VariableSE2DistanceMapRightType_::PerturbationDim,
                   VariableSE2DistanceMapRightType_>;
  using ThisType =
      SE2DistanceMapPriorErrorFactor_<VariableSE2DistanceMapRightType_>;

  using VariableType = VariableSE2DistanceMapRightType_;
  using EstimateType = typename VariableType::EstimateType;

  using MeasurementOwnerType = MeasurementOwnerEigen_<EstimateType>;
  using MeasurementType = typename MeasurementOwnerType::MeasurementType;

 protected:

  MeasurementType m_measurement_inverse_ = MeasurementType::Identity();

 public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  void setMeasurement(const MeasurementType& measurement) override
  {
    MeasurementOwnerType::setMeasurement(measurement);
    this->m_measurement_inverse_ = measurement.inverse();
  }

  void errorAndJacobian(bool error_only = false) override
  {
    const MeasurementType& X = this->_variables.template at<0>()->estimate();
    const MeasurementType error_SE2 = this->m_measurement_inverse_ * X;
    this->_e = geometry2d::t2v(error_SE2);

    if (error_only)
    {
      return;
    }

    this->_J.setZero();

    // tg derivative of the atan2 with respect to the involved terms in the
    // rotation matrix t2v[2] = atan2(R(1,0)/R(1,1))
    const Matrix2f& R_error = error_SE2.linear();
    const Vector2f row_error = R_error.row(1).transpose();

    // tg datan2/d[R10, R11] = 1/(R10^2 + R11^2) * [R11, -R10]
    Vector2f partial_atan2_R = geometry2d::skew(row_error);
    const float scale = row_error.squaredNorm();
    partial_atan2_R /= scale;

    Matrix2f skew_R;
    skew_R << 0, -1, 1, 0;

    this->_J.template block<2, 2>(0, 0) = R_error;

    // tg derivative R_error with respect to delta_theta (=0)
    Matrix2f R_aux_to = R_error * skew_R;
    Vector2f R_row = R_aux_to.row(1).transpose();

    this->_J(2, 2) = partial_atan2_R.dot(R_row);
  }  // void SE2DistanceMapPriorErrorFactor_::errorAndJacobian(bool = false)
     // override

  void _drawImpl(srrg2_core::ViewerCanvasPtr canvas) const override
  {
    if (!canvas)
    {
      throw std::runtime_error(
          "SE2DistanceMapPriorErrorFactor_::_drawImpl | invalid canvas");
    }

    Eigen::Vector3f segments[2];

    const VariableType* from_v =
        dynamic_cast<const VariableType*>(this->variable(0));

    if (!from_v)
    {
      return;
    }

    const Eigen::Isometry2f& from = from_v->estimate();

    segments[0] << from.translation().x(), from.translation().y(), 0.f;
    segments[1] << from.translation().x(), from.translation().y(), 1.f;

    canvas->putSegment(2, segments);
  }  // void
     // SE2DistanceMapPriorErrorFactor_::_drawImpl(srrg2_core::ViewerCanvasPtr)
     // const override

};  // class SE2DistanceMapPriorErrorFactor_<VariableSE2DistanceMapRightType_> :
    // public ErrorFactor_<VariableSE2DistanceMapRightType_::PerturbationDim,
    // VariableSE2DistanceMapRightType_> , public
    // MeasurementOwnerEigen_<VariableSE2DistanceMapRightType_::EstimateType>

using SE2DistanceMapStaticPoint2fVectorCloudRightPriorErrorFactor =
    SE2DistanceMapPriorErrorFactor_<
        VariableSE2DistanceMapStaticPoint2fVectorCloudRight>;
using SE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudRightPriorErrorFactor =
    SE2DistanceMapPriorErrorFactor_<
        VariableSE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudRight>;
using SE2DistanceMapSparseStaticPoint2fVectorCloudRightPriorErrorFactor =
    SE2DistanceMapPriorErrorFactor_<
        VariableSE2DistanceMapSparseStaticPoint2fVectorCloudRight>;
using SE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudRightPriorErrorFactor =
    SE2DistanceMapPriorErrorFactor_<
        VariableSE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudRight>;
using SE2DistanceMapNanoStaticPoint2fVectorCloudRightPriorErrorFactor =
    SE2DistanceMapPriorErrorFactor_<
        VariableSE2DistanceMapNanoStaticPoint2fVectorCloudRight>;
using SE2DistanceMapStaticPointNormal2fVectorCloudRightPriorErrorFactor =
    SE2DistanceMapPriorErrorFactor_<
        VariableSE2DistanceMapStaticPointNormal2fVectorCloudRight>;
using SE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRightPriorErrorFactor =
    SE2DistanceMapPriorErrorFactor_<
        VariableSE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRight>;
using SE2DistanceMapSparseStaticPointNormal2fVectorCloudRightPriorErrorFactor =
    SE2DistanceMapPriorErrorFactor_<
        VariableSE2DistanceMapSparseStaticPointNormal2fVectorCloudRight>;
using SE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRightPriorErrorFactor =
    SE2DistanceMapPriorErrorFactor_<
        VariableSE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRight>;
using SE2DistanceMapNanoStaticPointNormal2fVectorCloudRightPriorErrorFactor =
    SE2DistanceMapPriorErrorFactor_<
        VariableSE2DistanceMapNanoStaticPointNormal2fVectorCloudRight>;

/**
 * @note Prior error factor
 *
 * SE2DistanceMapStaticPoint2fVectorCloudRightPriorErrorFactor
 * SE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudRightPriorErrorFactor
 * SE2DistanceMapSparseStaticPoint2fVectorCloudRightPriorErrorFactor
 * SE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudRightPriorErrorFactor
 * SE2DistanceMapNanoStaticPoint2fVectorCloudRightPriorErrorFactor
 * SE2DistanceMapStaticPointNormal2fVectorCloudRightPriorErrorFactor
 * SE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRightPriorErrorFactor
 * SE2DistanceMapSparseStaticPointNormal2fVectorCloudRightPriorErrorFactor
 * SE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRightPriorErrorFactor
 * SE2DistanceMapNanoStaticPointNormal2fVectorCloudRightPriorErrorFactor
 */

}  // namespace srrg2_solver
