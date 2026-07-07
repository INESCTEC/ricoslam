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
 * @note this factor is for left-sided perturbation update-based variables!
 *       (the error formulation is different in the manifold, depending on the
 *       perturbation update side!)
 * @tparam VariableSE2DistanceMapLeftType_ distance map-based variable type
 *                                         (must be left-sided
 *                                         pertubation-based!)
 */
template <typename VariableSE2DistanceMapLeftType_>
class SE2DistanceMapPointPointLeftErrorFactor_
    : public ErrorFactor_<1, VariableSE2DistanceMapLeftType_>
{
 public:

  using BaseFactorType = ErrorFactor_<1, VariableSE2DistanceMapLeftType_>;
  using ThisType =
      SE2DistanceMapPointPointLeftErrorFactor_<VariableSE2DistanceMapLeftType_>;

  using VariableType = VariableSE2DistanceMapLeftType_;
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
    return "SE2DistanceMapPointPointLeftErrorFactor (distance map "
           "point-to-point)";
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

    const Vector2f moving_pred = X * (this->m_moving_->coordinates());

    typename VariableType::MeasurementOwnerType::DistanceMapType::CellTypeStd
        cell;

    if (!X_var->measurement()->getDistanceMap()->getValue(
            cell, moving_pred, *X_var->measurement()->getPointCloud()))
    /* if (!X_var->measurement()->getDistanceMap()->getValueSubPx(
            cell, moving_pred,
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
    J_icp.block<2, 2>(0, 0) = Matrix2f::Identity();
    J_icp.block<2, 1>(0, 2) = -geometry2d::skew(moving_pred);

    this->_J = cell.m_dist_d * J_icp;
  }  // void SE2DistanceMapPointPointLeftErrorFactor_::errorAndJacobian(bool =
     // false) override

};  // class
    // SE2DistanceMapPointPointLeftErrorFactor_<VariableSE2DistanceMapLeftType_>
    // : public ErrorFactor_<1, VariableSE2DistanceMapLeftType_>

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
 * @note this factor is for left-sided perturbation update-based variables!
 *       (the error formulation is different in the manifold, depending on the
 *       perturbation update side!)
 * @tparam VariableSE2DistanceMapLeftType_ distance map-based variable type
 *                                         (must be left-sided
 *                                         pertubation-based!)
 */
template <typename VariableSE2DistanceMapLeftType_>
class SE2DistanceMapPointPointWithSensorLeftErrorFactor_
    : public SE2DistanceMapPointPointLeftErrorFactor_<
          VariableSE2DistanceMapLeftType_>
{
 public:

  using BaseFactorType =
      SE2DistanceMapPointPointLeftErrorFactor_<VariableSE2DistanceMapLeftType_>;
  using ThisType = SE2DistanceMapPointPointWithSensorLeftErrorFactor_<
      VariableSE2DistanceMapLeftType_>;

  using VariableType = VariableSE2DistanceMapLeftType_;
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

    const Matrix2f& R_robot_in_sensor = this->m_robot_in_sensor_.linear();

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
    J_icp.block<2, 2>(0, 0) = R_robot_in_sensor;
    J_icp.block<2, 1>(0, 2) =
        -R_robot_in_sensor * geometry2d::skew(moving_pred_in_robot);

    this->_J = cell.m_dist_d * J_icp;
  }  // void
     // SE2DistanceMapPointPointWithSensorLeftErrorFactor_::errorAndJacobian(bool
     // = false) final

};  // class
    // SE2DistanceMapPointPointWithSensorLeftErrorFactor_<VariableSE2DistanceMapLeftType_>
    // : public
    // SE2DistanceMapPointPointLeftErrorFactor_<VariableSE2DistanceMapLeftType_>

using SE2DistanceMapStaticPoint2fVectorCloudLeftPointPointErrorFactor =
    SE2DistanceMapPointPointLeftErrorFactor_<
        VariableSE2DistanceMapStaticPoint2fVectorCloudLeft>;
using SE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudLeftPointPointErrorFactor =
    SE2DistanceMapPointPointLeftErrorFactor_<
        VariableSE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudLeft>;
using SE2DistanceMapSparseStaticPoint2fVectorCloudLeftPointPointErrorFactor =
    SE2DistanceMapPointPointLeftErrorFactor_<
        VariableSE2DistanceMapSparseStaticPoint2fVectorCloudLeft>;
using SE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudLeftPointPointErrorFactor =
    SE2DistanceMapPointPointLeftErrorFactor_<
        VariableSE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudLeft>;
using SE2DistanceMapNanoStaticPoint2fVectorCloudLeftPointPointErrorFactor =
    SE2DistanceMapPointPointLeftErrorFactor_<
        VariableSE2DistanceMapNanoStaticPoint2fVectorCloudLeft>;
using SE2DistanceMapStaticPointNormal2fVectorCloudLeftPointPointErrorFactor =
    SE2DistanceMapPointPointLeftErrorFactor_<
        VariableSE2DistanceMapStaticPointNormal2fVectorCloudLeft>;
using SE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudLeftPointPointErrorFactor =
    SE2DistanceMapPointPointLeftErrorFactor_<
        VariableSE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudLeft>;
using SE2DistanceMapSparseStaticPointNormal2fVectorCloudLeftPointPointErrorFactor =
    SE2DistanceMapPointPointLeftErrorFactor_<
        VariableSE2DistanceMapSparseStaticPointNormal2fVectorCloudLeft>;
using SE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudLeftPointPointErrorFactor =
    SE2DistanceMapPointPointLeftErrorFactor_<
        VariableSE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudLeft>;
using SE2DistanceMapNanoStaticPointNormal2fVectorCloudLeftPointPointErrorFactor =
    SE2DistanceMapPointPointLeftErrorFactor_<
        VariableSE2DistanceMapNanoStaticPointNormal2fVectorCloudLeft>;

using SE2DistanceMapStaticPoint2fVectorCloudLeftPointPointWithSensorErrorFactor =
    SE2DistanceMapPointPointWithSensorLeftErrorFactor_<
        VariableSE2DistanceMapStaticPoint2fVectorCloudLeft>;
using SE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudLeftPointPointWithSensorErrorFactor =
    SE2DistanceMapPointPointWithSensorLeftErrorFactor_<
        VariableSE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudLeft>;
using SE2DistanceMapSparseStaticPoint2fVectorCloudLeftPointPointWithSensorErrorFactor =
    SE2DistanceMapPointPointWithSensorLeftErrorFactor_<
        VariableSE2DistanceMapSparseStaticPoint2fVectorCloudLeft>;
using SE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudLeftPointPointWithSensorErrorFactor =
    SE2DistanceMapPointPointWithSensorLeftErrorFactor_<
        VariableSE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudLeft>;
using SE2DistanceMapNanoStaticPoint2fVectorCloudLeftPointPointWithSensorErrorFactor =
    SE2DistanceMapPointPointWithSensorLeftErrorFactor_<
        VariableSE2DistanceMapNanoStaticPoint2fVectorCloudLeft>;
using SE2DistanceMapStaticPointNormal2fVectorCloudLeftPointPointWithSensorErrorFactor =
    SE2DistanceMapPointPointWithSensorLeftErrorFactor_<
        VariableSE2DistanceMapStaticPointNormal2fVectorCloudLeft>;
using SE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudLeftPointPointWithSensorErrorFactor =
    SE2DistanceMapPointPointWithSensorLeftErrorFactor_<
        VariableSE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudLeft>;
using SE2DistanceMapSparseStaticPointNormal2fVectorCloudLeftPointPointWithSensorErrorFactor =
    SE2DistanceMapPointPointWithSensorLeftErrorFactor_<
        VariableSE2DistanceMapSparseStaticPointNormal2fVectorCloudLeft>;
using SE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudLeftPointPointWithSensorErrorFactor =
    SE2DistanceMapPointPointWithSensorLeftErrorFactor_<
        VariableSE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudLeft>;
using SE2DistanceMapNanoStaticPointNormal2fVectorCloudLeftPointPointWithSensorErrorFactor =
    SE2DistanceMapPointPointWithSensorLeftErrorFactor_<
        VariableSE2DistanceMapNanoStaticPointNormal2fVectorCloudLeft>;

template <typename VariableSE2DistanceMapLeftType_>
using SE2DistanceMapPointPointLeftErrorFactorCorrespondenceFreeDriven_ =
    FactorCorrespondenceFreeDriven_<SE2DistanceMapPointPointLeftErrorFactor_<
                                        VariableSE2DistanceMapLeftType_>,
                                    typename VariableSE2DistanceMapLeftType_::
                                        MeasurementOwnerType::PointCloudType>;

template <typename VariableSE2DistanceMapLeftType_>
using SE2DistanceMapPointPointWithSensorLeftErrorFactorCorrespondenceFreeDriven_ =
    FactorCorrespondenceFreeDriven_<
        SE2DistanceMapPointPointWithSensorLeftErrorFactor_<
            VariableSE2DistanceMapLeftType_>,
        typename VariableSE2DistanceMapLeftType_::MeasurementOwnerType::
            PointCloudType>;

using SE2DistanceMapStaticPoint2fVectorCloudLeftPointPointErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPointPointLeftErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapStaticPoint2fVectorCloudLeft>;
using SE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudLeftPointPointErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPointPointLeftErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudLeft>;
using SE2DistanceMapSparseStaticPoint2fVectorCloudLeftPointPointErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPointPointLeftErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapSparseStaticPoint2fVectorCloudLeft>;
using SE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudLeftPointPointErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPointPointLeftErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudLeft>;
using SE2DistanceMapNanoStaticPoint2fVectorCloudLeftPointPointErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPointPointLeftErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapNanoStaticPoint2fVectorCloudLeft>;
using SE2DistanceMapStaticPointNormal2fVectorCloudLeftPointPointErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPointPointLeftErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapStaticPointNormal2fVectorCloudLeft>;
using SE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudLeftPointPointErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPointPointLeftErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudLeft>;
using SE2DistanceMapSparseStaticPointNormal2fVectorCloudLeftPointPointErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPointPointLeftErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapSparseStaticPointNormal2fVectorCloudLeft>;
using SE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudLeftPointPointErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPointPointLeftErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudLeft>;
using SE2DistanceMapNanoStaticPointNormal2fVectorCloudLeftPointPointErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPointPointLeftErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapNanoStaticPointNormal2fVectorCloudLeft>;

using SE2DistanceMapStaticPoint2fVectorCloudLeftPointPointWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPointPointWithSensorLeftErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapStaticPoint2fVectorCloudLeft>;
using SE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudLeftPointPointWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPointPointWithSensorLeftErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudLeft>;
using SE2DistanceMapSparseStaticPoint2fVectorCloudLeftPointPointWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPointPointWithSensorLeftErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapSparseStaticPoint2fVectorCloudLeft>;
using SE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudLeftPointPointWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPointPointWithSensorLeftErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudLeft>;
using SE2DistanceMapNanoStaticPoint2fVectorCloudLeftPointPointWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPointPointWithSensorLeftErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapNanoStaticPoint2fVectorCloudLeft>;
using SE2DistanceMapStaticPointNormal2fVectorCloudLeftPointPointWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPointPointWithSensorLeftErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapStaticPointNormal2fVectorCloudLeft>;
using SE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudLeftPointPointWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPointPointWithSensorLeftErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudLeft>;
using SE2DistanceMapSparseStaticPointNormal2fVectorCloudLeftPointPointWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPointPointWithSensorLeftErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapSparseStaticPointNormal2fVectorCloudLeft>;
using SE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudLeftPointPointWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPointPointWithSensorLeftErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudLeft>;
using SE2DistanceMapNanoStaticPointNormal2fVectorCloudLeftPointPointWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPointPointWithSensorLeftErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapNanoStaticPointNormal2fVectorCloudLeft>;

/**
 * @note Point-point error factor
 *
 * (simple factor)
 *
 * SE2DistanceMapStaticPoint2fVectorCloudLeftPointPointErrorFactor
 * SE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudLeftPointPointErrorFactor
 * SE2DistanceMapSparseStaticPoint2fVectorCloudLeftPointPointErrorFactor
 * SE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudLeftPointPointErrorFactor
 * SE2DistanceMapNanoStaticPoint2fVectorCloudLeftPointPointErrorFactor
 * SE2DistanceMapStaticPointNormal2fVectorCloudLeftPointPointErrorFactor
 * SE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudLeftPointPointErrorFactor
 * SE2DistanceMapSparseStaticPointNormal2fVectorCloudLeftPointPointErrorFactor
 * SE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudLeftPointPointErrorFactor
 * SE2DistanceMapNanoStaticPointNormal2fVectorCloudLeftPointPointErrorFactor
 *
 * SE2DistanceMapStaticPoint2fVectorCloudLeftPointPointWithSensorErrorFactor
 * SE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudLeftPointPointWithSensorErrorFactor
 * SE2DistanceMapSparseStaticPoint2fVectorCloudLeftPointPointWithSensorErrorFactor
 * SE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudLeftPointPointWithSensorErrorFactor
 * SE2DistanceMapNanoStaticPoint2fVectorCloudLeftPointPointWithSensorErrorFactor
 * SE2DistanceMapStaticPointNormal2fVectorCloudLeftPointPointWithSensorErrorFactor
 * SE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudLeftPointPointWithSensorErrorFactor
 * SE2DistanceMapSparseStaticPointNormal2fVectorCloudLeftPointPointWithSensorErrorFactor
 * SE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudLeftPointPointWithSensorErrorFactor
 * SE2DistanceMapNanoStaticPointNormal2fVectorCloudLeftPointPointWithSensorErrorFactor
 *
 * (factor correspondence driven)
 *
 * SE2DistanceMapStaticPoint2fVectorCloudLeftPointPointErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudLeftPointPointErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapSparseStaticPoint2fVectorCloudLeftPointPointErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudLeftPointPointErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapNanoStaticPoint2fVectorCloudLeftPointPointErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapStaticPointNormal2fVectorCloudLeftPointPointErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudLeftPointPointErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapSparseStaticPointNormal2fVectorCloudLeftPointPointErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudLeftPointPointErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapNanoStaticPointNormal2fVectorCloudLeftPointPointErrorFactorCorrespondenceFreeDriven
 *
 * SE2DistanceMapStaticPoint2fVectorCloudLeftPointPointWithSensorErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudLeftPointPointWithSensorErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapSparseStaticPoint2fVectorCloudLeftPointPointWithSensorErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudLeftPointPointWithSensorErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapNanoStaticPoint2fVectorCloudLeftPointPointWithSensorErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapStaticPointNormal2fVectorCloudLeftPointPointWithSensorErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudLeftPointPointWithSensorErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapSparseStaticPointNormal2fVectorCloudLeftPointPointWithSensorErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudLeftPointPointWithSensorErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapNanoStaticPointNormal2fVectorCloudLeftPointPointWithSensorErrorFactorCorrespondenceFreeDriven
 */

}  // namespace srrg2_solver
