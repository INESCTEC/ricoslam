#pragma once

// SRRG Software
#include <srrg2_core/srrg_geometry/geometry2d.h>
#include <srrg2_core/srrg_geometry/geometry_defs.h>
#include <srrg2_core/srrg_viewer/viewer_canvas.h>
#include <srrg2_solver/solver_core/error_factor.h>
#include <srrg2_solver/solver_core/robustifier.h>

#include "inesctec_mrdt_slam_distmap_2d/pcl/point_normal.hpp"
#include "inesctec_mrdt_slam_distmap_2d/srrg2_solver/factor_correspondence_free_driven.hpp"
#include "inesctec_mrdt_slam_distmap_2d/srrg2_solver/variable_se2_distance_map.hpp"

namespace srrg2_solver
{

/**
 * @brief SE(2) distance map-based point to plane error factor
 *        (single point! >>> you need the correspondence free driven for point
 *        clouds)
 *
 * this factor formulates the error factor by putting the normal vectors on the
 * moving instead of the fixed point cloud. this formulation requires the 2nd
 * derivative from the distance map / fixed point cloud (may be pre-computed or
 * computed for each point evaluation), depending on the distance map container.
 *
 * @note this factor is for left-sided perturbation update-based variables!
 *       (the error formulation is different in the manifold, depending on the
 *       perturbation update side!)
 * @tparam VariableSE2DistanceMapLeftType_ distance map-based variable type
 *                                         (must be left-sided
 *                                         pertubation-based!)
 */
template <typename VariableSE2DistanceMapLeftType_>
class SE2DistanceMapPointPlaneLeftErrorFactor_
    : public ErrorFactor_<1, VariableSE2DistanceMapLeftType_>
{
 public:

  using BaseFactorType = ErrorFactor_<1, VariableSE2DistanceMapLeftType_>;
  using ThisType =
      SE2DistanceMapPointPlaneLeftErrorFactor_<VariableSE2DistanceMapLeftType_>;

  using VariableType = VariableSE2DistanceMapLeftType_;
  using EstimateType = typename VariableType::EstimateType;

  using MovingType = inesctec_mrdt_slam_distmap_2d::pcl::PointNormal2_<
      typename VariableType::MeasurementOwnerType::DistanceMapType::Scalar>;

 protected:

  const MovingType* m_moving_ = nullptr;
  int32_t m_correspondence_idx_fixed_ = -1;

 public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static inline const char* getTypeName()
  {
    return "SE2DistanceMapPointPlaneLeftErrorFactor (distance map "
           "point-to-plane)";
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

    if (this->m_moving_->normal().isZero())
    {
      this->_is_valid = false;
      return;
    }

    const VariableType* X_var = this->_variables.template at<0>();
    const EstimateType& X = X_var->estimate();
    const Eigen::Matrix2f& R = X.linear();

    Eigen::Vector2f p_moving_pred = X * this->m_moving_->coordinates();

    typename VariableType::MeasurementOwnerType::DistanceMapType::
        CellTypeWith2ndDeriv cell;

    if (!X_var->measurement()->getDistanceMap()->getValueWith2ndDeriv(
            cell, p_moving_pred, *X_var->measurement()->getPointCloud()))
    /* if (!X_var->measurement()->getDistanceMap()->getValueSubPxWith2ndDeriv(
            cell, p_moving_pred, *X_var->measurement()->getPointCloud())) */
    {
      this->_is_valid = false;
      return;
    }

    using Matrix1_2f = Eigen::Matrix<float, 1, 2>;

    Eigen::Vector2f n_moving_pred = R * this->m_moving_->normal();
    Matrix1_2f n_moving_pred_trans = n_moving_pred.transpose();
    Eigen::Vector2f dist_d_v_normalized =
        cell.m_dist_d.normalized().transpose();

    this->_e(0, 0) = n_moving_pred_trans * dist_d_v_normalized;
    this->_e(0, 0) *= cell.m_dist;
    this->m_correspondence_idx_fixed_ = cell.m_nn_idx;

    if (error_only)
    {
      return;
    }

    using Matrix2_3f = Eigen::Matrix<float, 2, 3>;

    Matrix2_3f J_icp;
    J_icp.block<2, 2>(0, 0) = Eigen::Matrix2f::Identity();
    J_icp.block<2, 1>(0, 2) = -geometry2d::skew(p_moving_pred);

    Matrix2_3f J_aux;
    J_aux.block<2, 2>(0, 0) = Eigen::Matrix2f::Zero();
    J_aux.block<2, 1>(0, 2) = geometry2d::skew(dist_d_v_normalized);

    J_aux += cell.m_dist_d2 * J_icp;
    J_aux *= cell.m_dist;

    Eigen::Matrix2f dist_dd = dist_d_v_normalized * cell.m_dist_d;

    J_aux += dist_dd * J_icp;

    this->_J = n_moving_pred_trans * J_aux;
  }  // void SE2DistanceMapPointPlaneLeftErrorFactor_::errorAndJacobian(bool =
     // false) override

};  // class
    // SE2DistanceMapPointPlaneLeftErrorFactor_<VariableSE2DistanceMapLeftType_>
    // : public ErrorFactor_<1, VariableSE2DistanceMapLeftType_>

/**
 * @brief SE(2) distance map-based point to plane error factor with sensor
 *        (single point! >>> you need the correspondence free driven for point
 *        clouds)
 *
 * this factor formulates the error factor by putting the normal vectors on the
 * moving instead of the fixed point cloud. this formulation requires the 2nd
 * derivative from the distance map / fixed point cloud (may be pre-computed or
 * computed for each point evaluation), depending on the distance map container.
 *
 * @note this factor is for left-sided perturbation update-based variables!
 *       (the error formulation is different in the manifold, depending on the
 *       perturbation update side!)
 * @tparam VariableSE2DistanceMapLeftType_ distance map-based variable type
 *                                         (must be left-sided
 *                                         pertubation-based!)
 */
template <typename VariableSE2DistanceMapLeftType_>
class SE2DistanceMapPointPlaneWithSensorLeftErrorFactor_
    : public SE2DistanceMapPointPlaneLeftErrorFactor_<
          VariableSE2DistanceMapLeftType_>
{
 public:

  using BaseFactorType =
      SE2DistanceMapPointPlaneLeftErrorFactor_<VariableSE2DistanceMapLeftType_>;
  using ThisType = SE2DistanceMapPointPlaneWithSensorLeftErrorFactor_<
      VariableSE2DistanceMapLeftType_>;

  using VariableType = VariableSE2DistanceMapLeftType_;
  using EstimateType = typename VariableType::EstimateType;

  using MovingType = inesctec_mrdt_slam_distmap_2d::pcl::PointNormal2_<
      typename VariableType::MeasurementOwnerType::DistanceMapType::Scalar>;

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

    if (this->m_moving_->normal().isZero())
    {
      this->_is_valid = false;
      return;
    }

    const VariableType* X_var = this->_variables.template at<0>();
    const EstimateType& X = X_var->estimate();
    const Eigen::Matrix2f& R = X.linear();

    // const Eigen::Matrix2f& R_sensor_in_robot =
    //     this->m_sensor_in_robot_.linear();
    const Eigen::Matrix2f& R_robot_in_sensor =
        this->m_robot_in_sensor_.linear();

    const Eigen::Vector2f p_moving_in_robot =
        this->m_sensor_in_robot_ * this->m_moving_->coordinates();
    const Eigen::Vector2f p_moving_pred_in_robot = X * p_moving_in_robot;
    const Eigen::Vector2f p_moving_pred_in_sensor =
        this->m_robot_in_sensor_ * p_moving_pred_in_robot;

    typename VariableType::MeasurementOwnerType::DistanceMapType::
        CellTypeWith2ndDeriv cell;

    if (!X_var->measurement()->getDistanceMap()->getValueWith2ndDeriv(
            cell, p_moving_pred_in_sensor,
            *X_var->measurement()->getPointCloud()))
    /* if (!X_var->measurement()->getDistanceMap()->getValueSubPxWith2ndDeriv(
            cell, p_moving_pred_in_sensor,
            *X_var->measurement()->getPointCloud())) */
    {
      this->_is_valid = false;
      return;
    }

    using Matrix1_2f = Eigen::Matrix<float, 1, 2>;

    const Eigen::Vector2f n_moving_pred = R * this->m_moving_->normal();
    const Matrix1_2f n_moving_pred_trans = n_moving_pred.transpose();

    Eigen::Vector2f dist_d_v_normalized =
        cell.m_dist_d.normalized().transpose();

    this->_e(0, 0) = n_moving_pred_trans * dist_d_v_normalized;
    this->_e(0, 0) *= cell.m_dist;
    this->m_correspondence_idx_fixed_ = cell.m_nn_idx;

    if (error_only)
    {
      return;
    }

    using Matrix2_3f = Eigen::Matrix<float, 2, 3>;

    Matrix2_3f J_icp;
    J_icp.block<2, 2>(0, 0) = R_robot_in_sensor;
    J_icp.block<2, 1>(0, 2) =
        -R_robot_in_sensor * geometry2d::skew(p_moving_pred_in_robot);

    Eigen::Vector2f v_aux = dist_d_v_normalized * cell.m_dist;

    Matrix2_3f J_aux;
    J_aux.block<2, 2>(0, 0) = Eigen::Matrix2f::Zero();
    J_aux.block<2, 1>(0, 2) = geometry2d::skew(v_aux);

    Eigen::Matrix2f M_aux = cell.m_dist_d2 * cell.m_dist;
    M_aux += dist_d_v_normalized * cell.m_dist_d;

    J_aux += M_aux * J_icp;

    this->_J = n_moving_pred_trans * J_aux;
  }  // void
     // SE2DistanceMapPointPlaneWithSensorLeftErrorFactor_::errorAndJacobian(bool
     // = false) final

};  // class
    // SE2DistanceMapPointPlaneWithSensorLeftErrorFactor_<VariableSE2DistanceMapLeftType_>
    // : public
    // SE2DistanceMapPointPlaneLeftErrorFactor_<VariableSE2DistanceMapLeftType_>

using SE2DistanceMapStaticPointNormal2fVectorCloudLeftPointPlaneErrorFactor =
    SE2DistanceMapPointPlaneLeftErrorFactor_<
        VariableSE2DistanceMapStaticPointNormal2fVectorCloudLeft>;
using SE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudLeftPointPlaneErrorFactor =
    SE2DistanceMapPointPlaneLeftErrorFactor_<
        VariableSE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudLeft>;
using SE2DistanceMapSparseStaticPointNormal2fVectorCloudLeftPointPlaneErrorFactor =
    SE2DistanceMapPointPlaneLeftErrorFactor_<
        VariableSE2DistanceMapSparseStaticPointNormal2fVectorCloudLeft>;
using SE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudLeftPointPlaneErrorFactor =
    SE2DistanceMapPointPlaneLeftErrorFactor_<
        VariableSE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudLeft>;
using SE2DistanceMapNanoStaticPointNormal2fVectorCloudLeftPointPlaneErrorFactor =
    SE2DistanceMapPointPlaneLeftErrorFactor_<
        VariableSE2DistanceMapNanoStaticPointNormal2fVectorCloudLeft>;

using SE2DistanceMapStaticPointNormal2fVectorCloudLeftPointPlaneWithSensorErrorFactor =
    SE2DistanceMapPointPlaneWithSensorLeftErrorFactor_<
        VariableSE2DistanceMapStaticPointNormal2fVectorCloudLeft>;
using SE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudLeftPointPlaneWithSensorErrorFactor =
    SE2DistanceMapPointPlaneWithSensorLeftErrorFactor_<
        VariableSE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudLeft>;
using SE2DistanceMapSparseStaticPointNormal2fVectorCloudLeftPointPlaneWithSensorErrorFactor =
    SE2DistanceMapPointPlaneWithSensorLeftErrorFactor_<
        VariableSE2DistanceMapSparseStaticPointNormal2fVectorCloudLeft>;
using SE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudLeftPointPlaneWithSensorErrorFactor =
    SE2DistanceMapPointPlaneWithSensorLeftErrorFactor_<
        VariableSE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudLeft>;
using SE2DistanceMapNanoStaticPointNormal2fVectorCloudLeftPointPlaneWithSensorErrorFactor =
    SE2DistanceMapPointPlaneWithSensorLeftErrorFactor_<
        VariableSE2DistanceMapNanoStaticPointNormal2fVectorCloudLeft>;

template <typename VariableSE2DistanceMapLeftType_>
using SE2DistanceMapPointPlaneLeftErrorFactorCorrespondenceFreeDriven_ =
    FactorCorrespondenceFreeDriven_<
        SE2DistanceMapPointPlaneLeftErrorFactor_<
            VariableSE2DistanceMapLeftType_>,
        inesctec_mrdt_slam_distmap_2d::pcl::PointNormal2VectorCloud_<
            typename VariableSE2DistanceMapLeftType_::MeasurementOwnerType::
                DistanceMapType::Scalar>>;

template <typename VariableSE2DistanceMapLeftType_>
using SE2DistanceMapPointPlaneWithSensorLeftErrorFactorCorrespondenceFreeDriven_ =
    FactorCorrespondenceFreeDriven_<
        SE2DistanceMapPointPlaneWithSensorLeftErrorFactor_<
            VariableSE2DistanceMapLeftType_>,
        inesctec_mrdt_slam_distmap_2d::pcl::PointNormal2VectorCloud_<
            typename VariableSE2DistanceMapLeftType_::MeasurementOwnerType::
                DistanceMapType::Scalar>>;

using SE2DistanceMapStaticPointNormal2fVectorCloudLeftPointPlaneErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPointPlaneLeftErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapStaticPointNormal2fVectorCloudLeft>;
using SE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudLeftPointPlaneErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPointPlaneLeftErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudLeft>;
using SE2DistanceMapSparseStaticPointNormal2fVectorCloudLeftPointPlaneErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPointPlaneLeftErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapSparseStaticPointNormal2fVectorCloudLeft>;
using SE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudLeftPointPlaneErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPointPlaneLeftErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudLeft>;
using SE2DistanceMapNanoStaticPointNormal2fVectorCloudLeftPointPlaneErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPointPlaneLeftErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapNanoStaticPointNormal2fVectorCloudLeft>;

using SE2DistanceMapStaticPointNormal2fVectorCloudLeftPointPlaneWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPointPlaneWithSensorLeftErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapStaticPointNormal2fVectorCloudLeft>;
using SE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudLeftPointPlaneWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPointPlaneWithSensorLeftErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudLeft>;
using SE2DistanceMapSparseStaticPointNormal2fVectorCloudLeftPointPlaneWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPointPlaneWithSensorLeftErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapSparseStaticPointNormal2fVectorCloudLeft>;
using SE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudLeftPointPlaneWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPointPlaneWithSensorLeftErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudLeft>;
using SE2DistanceMapNanoStaticPointNormal2fVectorCloudLeftPointPlaneWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPointPlaneWithSensorLeftErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapNanoStaticPointNormal2fVectorCloudLeft>;

/**
 * @note Point-plane error factor
 *
 * (simple factor)
 *
 * SE2DistanceMapStaticPointNormal2fVectorCloudLeftPointPlaneErrorFactor
 * SE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudLeftPointPlaneErrorFactor
 * SE2DistanceMapSparseStaticPointNormal2fVectorCloudLeftPointPlaneErrorFactor
 * SE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudLeftPointPlaneErrorFactor
 * SE2DistanceMapNanoStaticPointNormal2fVectorCloudLeftPointPlaneErrorFactor
 *
 * SE2DistanceMapStaticPointNormal2fVectorCloudLeftPointPlaneWithSensorErrorFactor
 * SE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudLeftPointPlaneWithSensorErrorFactor
 * SE2DistanceMapSparseStaticPointNormal2fVectorCloudLeftPointPlaneWithSensorErrorFactor
 * SE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudLeftPointPlaneWithSensorErrorFactor
 * SE2DistanceMapNanoStaticPointNormal2fVectorCloudLeftPointPlaneWithSensorErrorFactor
 *
 * (factor correspondence driven)
 *
 * SE2DistanceMapStaticPointNormal2fVectorCloudLeftPointPlaneErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudLeftPointPlaneErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapSparseStaticPointNormal2fVectorCloudLeftPointPlaneErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudLeftPointPlaneErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapNanoStaticPointNormal2fVectorCloudLeftPointPlaneErrorFactorCorrespondenceFreeDriven
 *
 * SE2DistanceMapStaticPointNormal2fVectorCloudLeftPointPlaneWithSensorErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudLeftPointPlaneWithSensorErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapSparseStaticPointNormal2fVectorCloudLeftPointPlaneWithSensorErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudLeftPointPlaneWithSensorErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapNanoStaticPointNormal2fVectorCloudLeftPointPlaneWithSensorErrorFactorCorrespondenceFreeDriven
 */

}  // namespace srrg2_solver
