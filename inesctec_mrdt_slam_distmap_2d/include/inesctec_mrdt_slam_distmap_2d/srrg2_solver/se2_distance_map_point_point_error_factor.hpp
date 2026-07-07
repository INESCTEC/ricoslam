#pragma once

// SRRG Software
#include <srrg2_core/srrg_geometry/geometry2d.h>
#include <srrg2_core/srrg_geometry/geometry_defs.h>
#include <srrg2_core/srrg_viewer/viewer_canvas.h>
#include <srrg2_solver/solver_core/error_factor.h>
#include <srrg2_solver/solver_core/robustifier.h>

#include "inesctec_mrdt_slam_distmap_2d/pcl/point.hpp"
#include "inesctec_mrdt_slam_distmap_2d/srrg2_solver/factor_correspondence_free_driven.hpp"
#include "inesctec_mrdt_slam_distmap_2d/srrg2_solver/variable_se2_distance_map.hpp"

namespace srrg2_solver
{

/**
 * @brief SE(2) distance map-based point to point error factor
 *        (single point! >>> you need the correspondence free driven for point
 *        clouds)
 *
 * the main goal of this factor is to not required any correspondence update
 * during the matching process. the correspondence is indirectly, through the
 * projection of the moving point into the fixed reference frame, where the
 * distance error is the value of the factor's error!
 *
 * @note this factor is for right-sided perturbation update-based variables!
 *       (the error formulation is different in the manifold, depending on the
 *       perturbation update side!)
 * @tparam VariableSE2DistanceMapRightType_ distance map-based variable type
 *                                          (must be right-sided
 *                                          pertubation-based!)
 */
template <typename VariableSE2DistanceMapRightType_>
class SE2DistanceMapPointPointErrorFactor_
    : public ErrorFactor_<1, VariableSE2DistanceMapRightType_>
{
 public:

  using BaseFactorType = ErrorFactor_<1, VariableSE2DistanceMapRightType_>;
  using ThisType =
      SE2DistanceMapPointPointErrorFactor_<VariableSE2DistanceMapRightType_>;

  using VariableType = VariableSE2DistanceMapRightType_;
  using EstimateType = typename VariableType::EstimateType;

  using MovingType =
      typename VariableType::MeasurementOwnerType::PointCloudType::PointType;

 protected:

  const MovingType* m_moving_ = nullptr;
  int32_t m_correspondence_idx_fixed_ = -1;

 public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static inline const char* getTypeName()
  {
    return "SE2DistanceMapPointPointErrorFactor (distance map point-to-point)";
  }

  inline void setMoving(const MovingType& moving) { this->m_moving_ = &moving; }

  inline int32_t getCorrespondenceIdxFixed() const
  {
    return this->m_correspondence_idx_fixed_;
  }

  void errorAndJacobian(bool error_only = false) override
  {
    this->_is_valid = true;
    this->m_correspondence_idx_fixed_ = -1;

    const VariableType* X_var = this->_variables.template at<0>();
    const EstimateType& X = X_var->estimate();
    const Eigen::Matrix2f& R = X.linear();

    const Eigen::Vector2f moving_pred = X * (this->m_moving_->coordinates());

    typename VariableType::MeasurementOwnerType::DistanceMapType::CellTypeStd
        cell;

    if (!X_var->measurement()->getDistanceMap()->getValue(
            cell, moving_pred, *X_var->measurement()->getPointCloud()))
    /* if (!X_var->measurement()->getDistanceMap()->getValueSubPx(
            cell, moving_pred, *X_var->measurement()->getPointCloud())) */
    {
      this->_is_valid = false;
      return;
    }

    this->_e(0, 0) = cell.m_dist;
    this->m_correspondence_idx_fixed_ = cell.m_nn_idx;

    if (error_only)
    {
      return;
    }

    srrg2_core::Matrix2_3f J_icp;
    J_icp.block<2, 2>(0, 0) = R;
    J_icp.block<2, 1>(0, 2) =
        -R * geometry2d::skew(this->m_moving_->coordinates());

    this->_J = cell.m_dist_d * J_icp;
  }  // void SE2DistanceMapPointPointErrorFactor_::errorAndJacobian(bool =
     // false) override

};  // class
    // SE2DistanceMapPointPointErrorFactor_<VariableSE2DistanceMapRightType_> :
    // public ErrorFactor_<1, VariableSE2DistanceMapRightType_>

/**
 * @brief SE(2) distance map-based point to point error factor with sensor
 *        (single point! >>> you need the correspondence free driven for point
 *        clouds)
 *
 * the main goal of this factor is to not required any correspondence update
 * during the matching process. the correspondence is indirectly, through the
 * projection of the moving point into the fixed reference frame, where the
 * distance error is the value of the factor's error!
 *
 * main different to the other factor is the consideration that the sensor may
 * be displaced w.r.t. the robot coordinate frame. consequently, the error
 * formulation must also take that into account.
 *
 * @note this factor is for right-sided perturbation update-based variables!
 *       (the error formulation is different in the manifold, depending on the
 *       perturbation update side!)
 * @tparam VariableSE2DistanceMapRightType_ distance map-based variable type
 *                                          (must be right-sided
 *                                          pertubation-based!)
 */
template <typename VariableSE2DistanceMapRightType_>
class SE2DistanceMapPointPointWithSensorErrorFactor_
    : public SE2DistanceMapPointPointErrorFactor_<
          VariableSE2DistanceMapRightType_>
{
 public:

  using BaseFactorType =
      SE2DistanceMapPointPointErrorFactor_<VariableSE2DistanceMapRightType_>;
  using ThisType = SE2DistanceMapPointPointWithSensorErrorFactor_<
      VariableSE2DistanceMapRightType_>;

  using VariableType = VariableSE2DistanceMapRightType_;
  using EstimateType = typename VariableType::EstimateType;

  using MovingType =
      typename VariableType::MeasurementOwnerType::PointCloudType::PointType;

 protected:

  EstimateType m_sensor_in_robot_ =
      EstimateType::Identity();  //!< calibrated SE(2) pose of the sensor w.r.t.
                                 //!< the robot  coordinate frame
  EstimateType m_robot_in_sensor_ =
      EstimateType::Identity();  //!< calibrated SE(2) pose of the robot  w.r.t.
                                 //!< the sensor coordinate frame

 public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  inline void setSensorInRobot(const EstimateType& sensor_in_robot)
  {
    this->m_sensor_in_robot_ = sensor_in_robot;
    this->m_robot_in_sensor_ = sensor_in_robot.inverse();
  }

  void errorAndJacobian(bool error_only = false) final
  {
    this->_is_valid = true;
    this->m_correspondence_idx_fixed_ = -1;

    const VariableType* X_var = this->_variables.template at<0>();
    const EstimateType& X = X_var->estimate();

    const Matrix2f R_robot_in_sensor_R =
        this->m_robot_in_sensor_.linear() * X.linear();

    const Vector2f moving_in_robot =
        this->m_sensor_in_robot_ * this->m_moving_->coordinates();
    const Vector2f moving_pred_in_robot = X * moving_in_robot;
    const Vector2f moving_pred_in_sensor =
        this->m_robot_in_sensor_ * moving_pred_in_robot;

    typename VariableType::MeasurementOwnerType::DistanceMapType::CellTypeStd
        cell;

    if (!X_var->measurement()->getDistanceMap()->getValue(
            cell, moving_pred_in_sensor,
            *X_var->measurement()->getPointCloud()))
    /* if (!X_var->measurement()->getDistanceMap()->getValueSubPx(
            cell, moving_pred_in_sensor,
            *X_var->measurement()->getPointCloud())) */
    {
      this->_is_valid = false;
      return;
    }

    this->_e(0, 0) = cell.m_dist;
    this->m_correspondence_idx_fixed_ = cell.m_nn_idx;

    if (error_only)
    {
      return;
    }

    srrg2_core::Matrix2_3f J_icp;
    J_icp.block<2, 2>(0, 0) = R_robot_in_sensor_R;
    J_icp.block<2, 1>(0, 2) =
        -R_robot_in_sensor_R * geometry2d::skew(moving_in_robot);

    this->_J = cell.m_dist_d * J_icp;
  }  // void
     // SE2DistanceMapPointPointWithSensorErrorFactor_::errorAndJacobian(bool
     // error_only = false) final

};  // class
    // SE2DistanceMapPointPointWithSensorErrorFactor_<VariableSE2DistanceMapRightType_>
    // : public
    // SE2DistanceMapPointPointErrorFactor_<VariableSE2DistanceMapRightType_>

using SE2DistanceMapStaticPoint2fVectorCloudRightPointPointErrorFactor =
    SE2DistanceMapPointPointErrorFactor_<
        VariableSE2DistanceMapStaticPoint2fVectorCloudRight>;
using SE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudRightPointPointErrorFactor =
    SE2DistanceMapPointPointErrorFactor_<
        VariableSE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudRight>;
using SE2DistanceMapSparseStaticPoint2fVectorCloudRightPointPointErrorFactor =
    SE2DistanceMapPointPointErrorFactor_<
        VariableSE2DistanceMapSparseStaticPoint2fVectorCloudRight>;
using SE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudRightPointPointErrorFactor =
    SE2DistanceMapPointPointErrorFactor_<
        VariableSE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudRight>;
using SE2DistanceMapNanoStaticPoint2fVectorCloudRightPointPointErrorFactor =
    SE2DistanceMapPointPointErrorFactor_<
        VariableSE2DistanceMapNanoStaticPoint2fVectorCloudRight>;
using SE2DistanceMapStaticPointNormal2fVectorCloudRightPointPointErrorFactor =
    SE2DistanceMapPointPointErrorFactor_<
        VariableSE2DistanceMapStaticPointNormal2fVectorCloudRight>;
using SE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRightPointPointErrorFactor =
    SE2DistanceMapPointPointErrorFactor_<
        VariableSE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRight>;
using SE2DistanceMapSparseStaticPointNormal2fVectorCloudRightPointPointErrorFactor =
    SE2DistanceMapPointPointErrorFactor_<
        VariableSE2DistanceMapSparseStaticPointNormal2fVectorCloudRight>;
using SE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRightPointPointErrorFactor =
    SE2DistanceMapPointPointErrorFactor_<
        VariableSE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRight>;
using SE2DistanceMapNanoStaticPointNormal2fVectorCloudRightPointPointErrorFactor =
    SE2DistanceMapPointPointErrorFactor_<
        VariableSE2DistanceMapNanoStaticPointNormal2fVectorCloudRight>;

using SE2DistanceMapStaticPoint2fVectorCloudRightPointPointWithSensorErrorFactor =
    SE2DistanceMapPointPointWithSensorErrorFactor_<
        VariableSE2DistanceMapStaticPoint2fVectorCloudRight>;
using SE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudRightPointPointWithSensorErrorFactor =
    SE2DistanceMapPointPointWithSensorErrorFactor_<
        VariableSE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudRight>;
using SE2DistanceMapSparseStaticPoint2fVectorCloudRightPointPointWithSensorErrorFactor =
    SE2DistanceMapPointPointWithSensorErrorFactor_<
        VariableSE2DistanceMapSparseStaticPoint2fVectorCloudRight>;
using SE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudRightPointPointWithSensorErrorFactor =
    SE2DistanceMapPointPointWithSensorErrorFactor_<
        VariableSE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudRight>;
using SE2DistanceMapNanoStaticPoint2fVectorCloudRightPointPointWithSensorErrorFactor =
    SE2DistanceMapPointPointWithSensorErrorFactor_<
        VariableSE2DistanceMapNanoStaticPoint2fVectorCloudRight>;
using SE2DistanceMapStaticPointNormal2fVectorCloudRightPointPointWithSensorErrorFactor =
    SE2DistanceMapPointPointWithSensorErrorFactor_<
        VariableSE2DistanceMapStaticPointNormal2fVectorCloudRight>;
using SE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRightPointPointWithSensorErrorFactor =
    SE2DistanceMapPointPointWithSensorErrorFactor_<
        VariableSE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRight>;
using SE2DistanceMapSparseStaticPointNormal2fVectorCloudRightPointPointWithSensorErrorFactor =
    SE2DistanceMapPointPointWithSensorErrorFactor_<
        VariableSE2DistanceMapSparseStaticPointNormal2fVectorCloudRight>;
using SE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRightPointPointWithSensorErrorFactor =
    SE2DistanceMapPointPointWithSensorErrorFactor_<
        VariableSE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRight>;
using SE2DistanceMapNanoStaticPointNormal2fVectorCloudRightPointPointWithSensorErrorFactor =
    SE2DistanceMapPointPointWithSensorErrorFactor_<
        VariableSE2DistanceMapNanoStaticPointNormal2fVectorCloudRight>;

template <typename VariableSE2DistanceMapRightType_>
using SE2DistanceMapPointPointErrorFactorCorrespondenceFreeDriven_ =
    FactorCorrespondenceFreeDriven_<
        SE2DistanceMapPointPointErrorFactor_<VariableSE2DistanceMapRightType_>,
        typename VariableSE2DistanceMapRightType_::MeasurementOwnerType::
            PointCloudType>;

template <typename VariableSE2DistanceMapRightType_>
using SE2DistanceMapPointPointWithSensorErrorFactorCorrespondenceFreeDriven_ =
    FactorCorrespondenceFreeDriven_<
        SE2DistanceMapPointPointWithSensorErrorFactor_<
            VariableSE2DistanceMapRightType_>,
        typename VariableSE2DistanceMapRightType_::MeasurementOwnerType::
            PointCloudType>;

using SE2DistanceMapStaticPoint2fVectorCloudRightPointPointErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPointPointErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapStaticPoint2fVectorCloudRight>;
using SE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudRightPointPointErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPointPointErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudRight>;
using SE2DistanceMapSparseStaticPoint2fVectorCloudRightPointPointErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPointPointErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapSparseStaticPoint2fVectorCloudRight>;
using SE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudRightPointPointErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPointPointErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudRight>;
using SE2DistanceMapNanoStaticPoint2fVectorCloudRightPointPointErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPointPointErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapNanoStaticPoint2fVectorCloudRight>;
using SE2DistanceMapStaticPointNormal2fVectorCloudRightPointPointErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPointPointErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapStaticPointNormal2fVectorCloudRight>;
using SE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRightPointPointErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPointPointErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRight>;
using SE2DistanceMapSparseStaticPointNormal2fVectorCloudRightPointPointErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPointPointErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapSparseStaticPointNormal2fVectorCloudRight>;
using SE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRightPointPointErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPointPointErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRight>;
using SE2DistanceMapNanoStaticPointNormal2fVectorCloudRightPointPointErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPointPointErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapNanoStaticPointNormal2fVectorCloudRight>;

using SE2DistanceMapStaticPoint2fVectorCloudRightPointPointWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPointPointWithSensorErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapStaticPoint2fVectorCloudRight>;
using SE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudRightPointPointWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPointPointWithSensorErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudRight>;
using SE2DistanceMapSparseStaticPoint2fVectorCloudRightPointPointWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPointPointWithSensorErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapSparseStaticPoint2fVectorCloudRight>;
using SE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudRightPointPointWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPointPointWithSensorErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudRight>;
using SE2DistanceMapNanoStaticPoint2fVectorCloudRightPointPointWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPointPointWithSensorErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapNanoStaticPoint2fVectorCloudRight>;
using SE2DistanceMapStaticPointNormal2fVectorCloudRightPointPointWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPointPointWithSensorErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapStaticPointNormal2fVectorCloudRight>;
using SE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRightPointPointWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPointPointWithSensorErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRight>;
using SE2DistanceMapSparseStaticPointNormal2fVectorCloudRightPointPointWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPointPointWithSensorErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapSparseStaticPointNormal2fVectorCloudRight>;
using SE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRightPointPointWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPointPointWithSensorErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRight>;
using SE2DistanceMapNanoStaticPointNormal2fVectorCloudRightPointPointWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPointPointWithSensorErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapNanoStaticPointNormal2fVectorCloudRight>;

/**
 * @note Point-point error factor
 *
 * (simple factor)
 *
 * SE2DistanceMapStaticPoint2fVectorCloudRightPointPointErrorFactor
 * SE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudRightPointPointErrorFactor
 * SE2DistanceMapSparseStaticPoint2fVectorCloudRightPointPointErrorFactor
 * SE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudRightPointPointErrorFactor
 * SE2DistanceMapNanoStaticPoint2fVectorCloudRightPointPointErrorFactor
 * SE2DistanceMapStaticPointNormal2fVectorCloudRightPointPointErrorFactor
 * SE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRightPointPointErrorFactor
 * SE2DistanceMapSparseStaticPointNormal2fVectorCloudRightPointPointErrorFactor
 * SE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRightPointPointErrorFactor
 * SE2DistanceMapNanoStaticPointNormal2fVectorCloudRightPointPointErrorFactor
 *
 * SE2DistanceMapStaticPoint2fVectorCloudRightPointPointWithSensorErrorFactor
 * SE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudRightPointPointWithSensorErrorFactor
 * SE2DistanceMapSparseStaticPoint2fVectorCloudRightPointPointWithSensorErrorFactor
 * SE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudRightPointPointWithSensorErrorFactor
 * SE2DistanceMapNanoStaticPoint2fVectorCloudRightPointPointWithSensorErrorFactor
 * SE2DistanceMapStaticPointNormal2fVectorCloudRightPointPointWithSensorErrorFactor
 * SE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRightPointPointWithSensorErrorFactor
 * SE2DistanceMapSparseStaticPointNormal2fVectorCloudRightPointPointWithSensorErrorFactor
 * SE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRightPointPointWithSensorErrorFactor
 * SE2DistanceMapNanoStaticPointNormal2fVectorCloudRightPointPointWithSensorErrorFactor
 *
 * (factor correspondence driven)
 *
 * SE2DistanceMapStaticPoint2fVectorCloudRightPointPointErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudRightPointPointErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapSparseStaticPoint2fVectorCloudRightPointPointErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudRightPointPointErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapNanoStaticPoint2fVectorCloudRightPointPointErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapStaticPointNormal2fVectorCloudRightPointPointErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRightPointPointErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapSparseStaticPointNormal2fVectorCloudRightPointPointErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRightPointPointErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapNanoStaticPointNormal2fVectorCloudRightPointPointErrorFactorCorrespondenceFreeDriven
 *
 * SE2DistanceMapStaticPoint2fVectorCloudRightPointPointWithSensorErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudRightPointPointWithSensorErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapSparseStaticPoint2fVectorCloudRightPointPointWithSensorErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudRightPointPointWithSensorErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapNanoStaticPoint2fVectorCloudRightPointPointWithSensorErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapStaticPointNormal2fVectorCloudRightPointPointWithSensorErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRightPointPointWithSensorErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapSparseStaticPointNormal2fVectorCloudRightPointPointWithSensorErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRightPointPointWithSensorErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapNanoStaticPointNormal2fVectorCloudRightPointPointWithSensorErrorFactorCorrespondenceFreeDriven
 */

}  // namespace srrg2_solver
