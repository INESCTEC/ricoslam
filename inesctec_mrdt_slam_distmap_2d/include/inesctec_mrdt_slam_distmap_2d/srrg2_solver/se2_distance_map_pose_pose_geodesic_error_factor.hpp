#pragma once

// SRRG Software
#include <srrg2_core/srrg_geometry/geometry2d.h>
#include <srrg2_core/srrg_geometry/geometry_defs.h>
#include <srrg2_core/srrg_viewer/viewer_canvas.h>
#include <srrg2_solver/solver_core/error_factor.h>
#include <srrg2_solver/solver_core/measurement_owner.h>

#include "inesctec_mrdt_slam_distmap_2d/srrg2_solver/variable_se2_distance_map.hpp"

namespace srrg2_solver
{

/**
 * @brief pose to pose SE(2) geodesic error factor between two distance
 *        map-based variables (holds an Isometry2f estimate type)
 * @tparam VariableSE2DistanceMapType_ distance map-based variable type
 *                                     (must be right-sided pertubation-based!)
 */
template <typename VariableSE2DistanceMapType_>
class SE2DistanceMapPosePoseGeodesicErrorFactor_
    : public ErrorFactor_<3, VariableSE2DistanceMapType_,
                          VariableSE2DistanceMapType_>,
      public MeasurementOwnerEigen_<srrg2_core::Isometry2f>
{
 public:

  using BaseFactorType =
      ErrorFactor_<3, VariableSE2DistanceMapType_, VariableSE2DistanceMapType_>;
  using ThisType =
      SE2DistanceMapPosePoseGeodesicErrorFactor_<VariableSE2DistanceMapType_>;

  using VariableType = VariableSE2DistanceMapType_;
  using EstimateType = typename VariableType::EstimateType;

  using MeasurementOwnerType = MeasurementOwnerEigen_<srrg2_core::Isometry2f>;
  using MeasurementType = typename MeasurementOwnerType::MeasurementType;

 protected:

  MeasurementType m_inverse_measured_relative_pose_;

 public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  inline void setMeasurement(const MeasurementType& measurement)
  {
    MeasurementOwnerType::setMeasurement(measurement);

    this->m_inverse_measured_relative_pose_ =
        MeasurementOwnerType::_measurement.inverse();
  }

  void errorAndJacobian(bool error_only = false) final
  {
    VariableType* v_from = this->_variables.template at<0>();
    VariableType* v_to = this->_variables.template at<1>();

    const srrg2_core::Isometry2f& from = v_from->estimate();
    const srrg2_core::Isometry2f& to = v_to->estimate();
    const srrg2_core::Matrix2f R_from_transpose = from.linear().transpose();
    const srrg2_core::Matrix2f& R_to = to.linear();
    const srrg2_core::Matrix2f& R_z_transpose =
        this->m_inverse_measured_relative_pose_.linear();
    const srrg2_core::Vector2f delta_translation =
        to.translation() - from.translation();
    const srrg2_core::Isometry2f prediction = from.inverse() * to;
    const srrg2_core::Isometry2f error_SE2 =
        this->m_inverse_measured_relative_pose_ * prediction;
    this->_e = srrg2_core::geometry2d::t2v(error_SE2);

    if (error_only)
    {
      return;
    }

    // tg derivative of the atan2 with respect to the involved terms in the
    // rotation matrix t2v[2] = atan2(R(1,0)/R(1,1))
    const srrg2_core::Matrix2f& R_error = error_SE2.linear();
    const srrg2_core::Vector2f row_error = R_error.row(1).transpose();
    // tg datan2/d[R10, R11] = 1/(R10^2 + R11^2) * [R11, -R10]
    srrg2_core::Vector2f partial_atan2_R =
        srrg2_core::geometry2d::skew(row_error);
    const float scale = row_error.squaredNorm();
    partial_atan2_R /= scale;

    // tg derivatives of R(delta_theta) and R(-delta_theta) with respect to
    // delta_theta(=0)
    srrg2_core::Matrix2f skew_R_transpose;
    skew_R_transpose << 0, 1, -1, 0;
    srrg2_core::Matrix2f skew_R;
    skew_R << 0, -1, 1, 0;

    Eigen::Matrix<float, 3, 3> J_from;

    J_from.setZero();
    // tg jacobian of the translation with respect to the full perturbation
    J_from.block<2, 2>(0, 0) = -R_z_transpose;
    J_from.block<2, 1>(0, 2) =
        R_z_transpose * skew_R_transpose * R_from_transpose * delta_translation;
    // tg derivative R_error with respect to delta_theta_from (=0)
    srrg2_core::Matrix2f R_aux_from =
        R_z_transpose * skew_R_transpose * R_from_transpose * R_to;
    srrg2_core::Vector2f R_row = R_aux_from.row(1).transpose();
    J_from(2, 2) = partial_atan2_R.dot(R_row);
    this->template jacobian<0>() = J_from;

    Eigen::Matrix<float, 3, 3> J_to;

    J_to.setZero();
    // tg jacobian of the translation with respect to the full perturbation
    J_to.block<2, 2>(0, 0) = R_error;
    // tg derivative R_error with respect to delta_theta_to (=0)
    srrg2_core::Matrix2f R_aux_to = R_error * skew_R;
    R_row = R_aux_to.row(1).transpose();
    J_to(2, 2) = partial_atan2_R.dot(R_row);
    this->template jacobian<1>() = J_to;
  }

  /**
   * @brief draw the pose to pose factor (connection between 2 variable nodes,
   *        in terms of visualization) in an OpenGL-based canvas implemented in
   *        SRRG software
   * @param[in,out] canvas_ OpenGL canvas
   */
  void _drawImpl(srrg2_core::ViewerCanvasPtr canvas) const override
  {
    if (!canvas)
    {
      return;
    }

    srrg2_core::Vector3f segments[2];

    const VariableType* from_v =
        dynamic_cast<const VariableType*>(this->variable(0));
    const VariableType* to_v =
        dynamic_cast<const VariableType*>(this->variable(1));

    if (!from_v || !to_v)
    {
      return;
    }

    const EstimateType& from = from_v->estimate();
    const EstimateType& to = to_v->estimate();

    segments[0] << from.translation().x(), from.translation().y(), 0.f;
    segments[1] << to.translation().x(), to.translation().y(), 0.f;

    canvas->putSegment(2, segments, 0);
  }  // void
     // SE2DistanceMapPosePoseGeodesicErrorFactor_::_drawImpl(srrg2_core::ViewerCanvasPtr)
     // const override

};  // class
    // SE2DistanceMapPosePoseGeodesicErrorFactor_<VariableSE2DistanceMapType_> :
    // public ErrorFactor_<3, VariableSE2DistanceMapType_,
    // VariableSE2DistanceMapType_>, public
    // MeasurementOwnerEigen_<srrg2_core::Isometry2f>

using SE2DistanceMapStaticPoint2fVectorCloudRightPosePoseGeodesicErrorFactor =
    SE2DistanceMapPosePoseGeodesicErrorFactor_<
        VariableSE2DistanceMapStaticPoint2fVectorCloudRight>;
using SE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudRightPosePoseGeodesicErrorFactor =
    SE2DistanceMapPosePoseGeodesicErrorFactor_<
        VariableSE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudRight>;
using SE2DistanceMapSparseStaticPoint2fVectorCloudRightPosePoseGeodesicErrorFactor =
    SE2DistanceMapPosePoseGeodesicErrorFactor_<
        VariableSE2DistanceMapSparseStaticPoint2fVectorCloudRight>;
using SE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudRightPosePoseGeodesicErrorFactor =
    SE2DistanceMapPosePoseGeodesicErrorFactor_<
        VariableSE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudRight>;
using SE2DistanceMapNanoStaticPoint2fVectorCloudRightPosePoseGeodesicErrorFactor =
    SE2DistanceMapPosePoseGeodesicErrorFactor_<
        VariableSE2DistanceMapNanoStaticPoint2fVectorCloudRight>;
using SE2DistanceMapStaticPointNormal2fVectorCloudRightPosePoseGeodesicErrorFactor =
    SE2DistanceMapPosePoseGeodesicErrorFactor_<
        VariableSE2DistanceMapStaticPointNormal2fVectorCloudRight>;
using SE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRightPosePoseGeodesicErrorFactor =
    SE2DistanceMapPosePoseGeodesicErrorFactor_<
        VariableSE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRight>;
using SE2DistanceMapSparseStaticPointNormal2fVectorCloudRightPosePoseGeodesicErrorFactor =
    SE2DistanceMapPosePoseGeodesicErrorFactor_<
        VariableSE2DistanceMapSparseStaticPointNormal2fVectorCloudRight>;
using SE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRightPosePoseGeodesicErrorFactor =
    SE2DistanceMapPosePoseGeodesicErrorFactor_<
        VariableSE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRight>;
using SE2DistanceMapNanoStaticPointNormal2fVectorCloudRightPosePoseGeodesicErrorFactor =
    SE2DistanceMapPosePoseGeodesicErrorFactor_<
        VariableSE2DistanceMapNanoStaticPointNormal2fVectorCloudRight>;

/**
 * @note Pose-pose geosedic error factor
 *
 * SE2DistanceMapStaticPoint2fVectorCloudRightPosePoseGeodesicErrorFactor
 * SE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudRightPosePoseGeodesicErrorFactor
 * SE2DistanceMapSparseStaticPoint2fVectorCloudRightPosePoseGeodesicErrorFactor
 * SE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudRightPosePoseGeodesicErrorFactor
 * SE2DistanceMapNanoStaticPoint2fVectorCloudRightPosePoseGeodesicErrorFactor
 * SE2DistanceMapStaticPointNormal2fVectorCloudRightPosePoseGeodesicErrorFactor
 * SE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRightPosePoseGeodesicErrorFactor
 * SE2DistanceMapSparseStaticPointNormal2fVectorCloudRightPosePoseGeodesicErrorFactor
 * SE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRightPosePoseGeodesicErrorFactor
 * SE2DistanceMapNanoStaticPointNormal2fVectorCloudRightPosePoseGeodesicErrorFactor
 */

}  // namespace srrg2_solver
