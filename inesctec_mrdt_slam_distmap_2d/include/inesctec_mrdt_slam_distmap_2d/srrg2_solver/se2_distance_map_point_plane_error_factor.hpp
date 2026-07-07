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
 * @note this factor is for right-sided perturbation update-based variables!
 *       (the error formulation is different in the manifold, depending on the
 *       perturbation update side!)
 * @tparam VariableSE2DistanceMapRightType_ distance map-based variable type
 *                                          (must be right-sided
 *                                          pertubation-based!)
 */
template <typename VariableSE2DistanceMapRightType_>
class SE2DistanceMapPointPlaneErrorFactor_
    : public ErrorFactor_<1, VariableSE2DistanceMapRightType_>
{
 public:

  using BaseFactorType = ErrorFactor_<1, VariableSE2DistanceMapRightType_>;
  using ThisType =
      SE2DistanceMapPointPlaneErrorFactor_<VariableSE2DistanceMapRightType_>;

  using VariableType = VariableSE2DistanceMapRightType_;
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
    return "SE2DistanceMapPointPlaneErrorFactor (distance map point-to-plane)";
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
    J_icp.block<2, 2>(0, 0) = R;
    J_icp.block<2, 1>(0, 2) =
        -R * geometry2d::skew(this->m_moving_->coordinates());

    Eigen::Vector2f dist_d_v_dir_rot = R.transpose() * dist_d_v_normalized;
    dist_d_v_dir_rot *= cell.m_dist;

    Matrix2_3f J_aux;
    J_aux.block<2, 2>(0, 0) = Matrix2f::Zero();
    J_aux.block<2, 1>(0, 2) = geometry2d::skew(dist_d_v_dir_rot);

    this->_J = this->m_moving_->normal().transpose() * J_aux;

    Eigen::Matrix2f M_aux = cell.m_dist_d2 * cell.m_dist;
    M_aux += dist_d_v_normalized * cell.m_dist_d;

    J_aux = M_aux * J_icp;

    this->_J += n_moving_pred_trans * J_aux;
  }  // void SE2DistanceMapPointPlaneErrorFactor_::errorAndJacobian(bool =
     // false) override

};  // class
    // SE2DistanceMapPointPlaneErrorFactor_<VariableSE2DistanceMapRightType_> :
    // public ErrorFactor_<1, VariableSE2DistanceMapRightType_>

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
 * @note this factor is for right-sided perturbation update-based variables!
 *       (the error formulation is different in the manifold, depending on the
 *       perturbation update side!)
 * @tparam VariableSE2DistanceMapRightType_ distance map-based variable type
 *                                          (must be right-sided
 *                                          pertubation-based!)
 */
template <typename VariableSE2DistanceMapRightType_>
class SE2DistanceMapPointPlaneWithSensorErrorFactor_
    : public SE2DistanceMapPointPlaneErrorFactor_<
          VariableSE2DistanceMapRightType_>
{
 public:

  using BaseFactorType =
      SE2DistanceMapPointPlaneErrorFactor_<VariableSE2DistanceMapRightType_>;
  using ThisType = SE2DistanceMapPointPlaneWithSensorErrorFactor_<
      VariableSE2DistanceMapRightType_>;

  using VariableType = VariableSE2DistanceMapRightType_;
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

    const Eigen::Matrix2f R_robot_in_sensor_R = R_robot_in_sensor * R;

    using Matrix2_3f = Eigen::Matrix<float, 2, 3>;

    Matrix2_3f J_icp;

    J_icp.block<2, 2>(0, 0) = R_robot_in_sensor_R;
    J_icp.block<2, 1>(0, 2) =
        -R_robot_in_sensor_R * geometry2d::skew(p_moving_in_robot);

    Eigen::Vector2f dist_d_v_dir_rot = R.transpose() * dist_d_v_normalized;
    dist_d_v_dir_rot *= cell.m_dist;

    Matrix2_3f J_aux;
    J_aux.block<2, 2>(0, 0) = Matrix2f::Zero();
    J_aux.block<2, 1>(0, 2) = geometry2d::skew(dist_d_v_dir_rot);

    this->_J = this->m_moving_->normal().transpose() * J_aux;

    Eigen::Matrix2f M_aux = cell.m_dist_d2 * cell.m_dist;
    M_aux += dist_d_v_normalized * cell.m_dist_d;

    J_aux = M_aux * J_icp;

    this->_J += n_moving_pred_trans * J_aux;
  }  // void
     // SE2DistanceMapPointPlaneWithSensorErrorFactor_::errorAndJacobian(bool =
     // false) final

};  // class
    // SE2DistanceMapPointPlaneWithSensorErrorFactor_<VariableSE2DistanceMapRightType_>
    // : public
    // SE2DistanceMapPointPlaneErrorFactor_<VariableSE2DistanceMapRightType_>

using SE2DistanceMapStaticPointNormal2fVectorCloudRightPointPlaneErrorFactor =
    SE2DistanceMapPointPlaneErrorFactor_<
        VariableSE2DistanceMapStaticPointNormal2fVectorCloudRight>;
using SE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRightPointPlaneErrorFactor =
    SE2DistanceMapPointPlaneErrorFactor_<
        VariableSE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRight>;
using SE2DistanceMapSparseStaticPointNormal2fVectorCloudRightPointPlaneErrorFactor =
    SE2DistanceMapPointPlaneErrorFactor_<
        VariableSE2DistanceMapSparseStaticPointNormal2fVectorCloudRight>;
using SE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRightPointPlaneErrorFactor =
    SE2DistanceMapPointPlaneErrorFactor_<
        VariableSE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRight>;
using SE2DistanceMapNanoStaticPointNormal2fVectorCloudRightPointPlaneErrorFactor =
    SE2DistanceMapPointPlaneErrorFactor_<
        VariableSE2DistanceMapNanoStaticPointNormal2fVectorCloudRight>;

using SE2DistanceMapStaticPointNormal2fVectorCloudRightPointPlaneWithSensorErrorFactor =
    SE2DistanceMapPointPlaneWithSensorErrorFactor_<
        VariableSE2DistanceMapStaticPointNormal2fVectorCloudRight>;
using SE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRightPointPlaneWithSensorErrorFactor =
    SE2DistanceMapPointPlaneWithSensorErrorFactor_<
        VariableSE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRight>;
using SE2DistanceMapSparseStaticPointNormal2fVectorCloudRightPointPlaneWithSensorErrorFactor =
    SE2DistanceMapPointPlaneWithSensorErrorFactor_<
        VariableSE2DistanceMapSparseStaticPointNormal2fVectorCloudRight>;
using SE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRightPointPlaneWithSensorErrorFactor =
    SE2DistanceMapPointPlaneWithSensorErrorFactor_<
        VariableSE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRight>;
using SE2DistanceMapNanoStaticPointNormal2fVectorCloudRightPointPlaneWithSensorErrorFactor =
    SE2DistanceMapPointPlaneWithSensorErrorFactor_<
        VariableSE2DistanceMapNanoStaticPointNormal2fVectorCloudRight>;

template <typename VariableSE2DistanceMapRightType_>
using SE2DistanceMapPointPlaneErrorFactorCorrespondenceFreeDriven_ =
    FactorCorrespondenceFreeDriven_<
        SE2DistanceMapPointPlaneErrorFactor_<VariableSE2DistanceMapRightType_>,
        inesctec_mrdt_slam_distmap_2d::pcl::PointNormal2VectorCloud_<
            typename VariableSE2DistanceMapRightType_::MeasurementOwnerType::
                DistanceMapType::Scalar>>;

template <typename VariableSE2DistanceMapRightType_>
using SE2DistanceMapPointPlaneWithSensorErrorFactorCorrespondenceFreeDriven_ =
    FactorCorrespondenceFreeDriven_<
        SE2DistanceMapPointPlaneWithSensorErrorFactor_<
            VariableSE2DistanceMapRightType_>,
        inesctec_mrdt_slam_distmap_2d::pcl::PointNormal2VectorCloud_<
            typename VariableSE2DistanceMapRightType_::MeasurementOwnerType::
                DistanceMapType::Scalar>>;

using SE2DistanceMapStaticPointNormal2fVectorCloudRightPointPlaneErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPointPlaneErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapStaticPointNormal2fVectorCloudRight>;
using SE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRightPointPlaneErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPointPlaneErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRight>;
using SE2DistanceMapSparseStaticPointNormal2fVectorCloudRightPointPlaneErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPointPlaneErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapSparseStaticPointNormal2fVectorCloudRight>;
using SE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRightPointPlaneErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPointPlaneErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRight>;
using SE2DistanceMapNanoStaticPointNormal2fVectorCloudRightPointPlaneErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPointPlaneErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapNanoStaticPointNormal2fVectorCloudRight>;

using SE2DistanceMapStaticPointNormal2fVectorCloudRightPointPlaneWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPointPlaneWithSensorErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapStaticPointNormal2fVectorCloudRight>;
using SE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRightPointPlaneWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPointPlaneWithSensorErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRight>;
using SE2DistanceMapSparseStaticPointNormal2fVectorCloudRightPointPlaneWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPointPlaneWithSensorErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapSparseStaticPointNormal2fVectorCloudRight>;
using SE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRightPointPlaneWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPointPlaneWithSensorErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRight>;
using SE2DistanceMapNanoStaticPointNormal2fVectorCloudRightPointPlaneWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPointPlaneWithSensorErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapNanoStaticPointNormal2fVectorCloudRight>;

/**
 * @note Point-plane error factor
 *
 * (simple factor)
 *
 * SE2DistanceMapStaticPointNormal2fVectorCloudRightPointPlaneErrorFactor
 * SE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRightPointPlaneErrorFactor
 * SE2DistanceMapSparseStaticPointNormal2fVectorCloudRightPointPlaneErrorFactor
 * SE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRightPointPlaneErrorFactor
 * SE2DistanceMapNanoStaticPointNormal2fVectorCloudRightPointPlaneErrorFactor
 *
 * SE2DistanceMapStaticPointNormal2fVectorCloudRightPointPlaneWithSensorErrorFactor
 * SE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRightPointPlaneWithSensorErrorFactor
 * SE2DistanceMapSparseStaticPointNormal2fVectorCloudRightPointPlaneWithSensorErrorFactor
 * SE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRightPointPlaneWithSensorErrorFactor
 * SE2DistanceMapNanoStaticPointNormal2fVectorCloudRightPointPlaneWithSensorErrorFactor
 *
 * (factor correspondence driven)
 *
 * SE2DistanceMapStaticPointNormal2fVectorCloudRightPointPlaneErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRightPointPlaneErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapSparseStaticPointNormal2fVectorCloudRightPointPlaneErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRightPointPlaneErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapNanoStaticPointNormal2fVectorCloudRightPointPlaneErrorFactorCorrespondenceFreeDriven
 *
 * SE2DistanceMapStaticPointNormal2fVectorCloudRightPointPlaneWithSensorErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRightPointPlaneWithSensorErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapSparseStaticPointNormal2fVectorCloudRightPointPlaneWithSensorErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRightPointPlaneWithSensorErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapNanoStaticPointNormal2fVectorCloudRightPointPlaneWithSensorErrorFactorCorrespondenceFreeDriven
 */

}  // namespace srrg2_solver
