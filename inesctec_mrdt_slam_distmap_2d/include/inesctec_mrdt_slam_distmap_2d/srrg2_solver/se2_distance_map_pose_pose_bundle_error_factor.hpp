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
 * @brief SE(2) distance map-based pose to pose global bundle adjustment based
 *        on a point to point error formulation
 *        (single point! >>> you need the correspondence free driven for point
 *        clouds)
 *
 * in this case, the matcher of point to point error factor is now "put into"
 * the pose to pose constraints in the graph. the goal is to take advantage of
 * the distance map being a lookup table, allowing a fast computation of the
 * errors and jacobians... (with the derivatives being pre-computed)
 *
 * @note this factor is for right-sided perturbation update-based variables!
 *       (the error formulation is different in the manifold, depending on the
 *       perturbation update side!)
 * @tparam VariableSE2DistanceMapRightType_ distance map-based variable type
 *                                          (must be right-sided
 *                                          pertubation-based!)
 */
template <typename VariableSE2DistanceMapRightType_>
class SE2DistanceMapPosePoseBundleErrorFactor_
    : public ErrorFactor_<1, VariableSE2DistanceMapRightType_,
                          VariableSE2DistanceMapRightType_>
{
 public:

  using BaseFactorType = ErrorFactor_<1, VariableSE2DistanceMapRightType_,
                                      VariableSE2DistanceMapRightType_>;
  using ThisType = SE2DistanceMapPosePoseBundleErrorFactor_<
      VariableSE2DistanceMapRightType_>;

  using VariableType = VariableSE2DistanceMapRightType_;
  using EstimateType = typename VariableType::EstimateType;

  using MovingType =
      typename VariableType::MeasurementOwnerType::PointCloudType::PointType;

 protected:

  const MovingType* m_moving_ = nullptr;

 public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  inline void setMoving(const MovingType& moving) { this->m_moving_ = &moving; }

  void errorAndJacobian(bool error_only = false) override
  {
    this->_is_valid = true;

    VariableType* v_from = this->_variables.template at<0>();
    VariableType* v_to = this->_variables.template at<1>();

    const EstimateType& from = v_from->estimate();
    const EstimateType& to = v_to->estimate();
    const EstimateType to_inv = to.inverse();

    const Eigen::Vector2f& moving = this->m_moving_->coordinates();
    const Eigen::Vector2f moving_in_w = from * moving;
    const Eigen::Vector2f moving_pred = to_inv * moving_in_w;

    typename VariableType::MeasurementOwnerType::DistanceMapType::CellTypeStd
        cell;

    if (!v_to->measurement()->getDistanceMap()->getValue(
            cell, moving_pred, *v_to->measurement()->getPointCloud()))
    {
      this->_is_valid = false;
      return;
    }

    this->_e(0, 0) = cell.m_dist;

    if (error_only)
    {
      return;
    }

    const Eigen::Matrix2f& R_from = from.linear();
    const Eigen::Matrix2f R_to_inv = to.linear().transpose();

    const Eigen::Matrix2f R_from_in_to = R_to_inv * R_from;

    using Matrix2_3f = Eigen::Matrix<float, 2, 3>;

    Matrix2_3f J_icp_from;
    J_icp_from.block<2, 2>(0, 0) = R_from_in_to;
    J_icp_from.block<2, 1>(0, 2) = -R_from_in_to * geometry2d::skew(moving);

    Matrix2_3f J_icp_to;
    J_icp_to.block<2, 2>(0, 0) = -Matrix2f::Identity();
    J_icp_to.block<2, 1>(0, 2) = geometry2d::skew(moving_pred);

    auto J_from = this->template jacobian<0>();
    auto J_to = this->template jacobian<1>();

    J_from = cell.m_dist_d * J_icp_from;
    J_to = cell.m_dist_d * J_icp_to;
  }  // void SE2DistanceMapPosePoseBundleErrorFactor_::errorAndJacobian(bool =
     // false) override

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

    Eigen::Vector3f segments[2];

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
     // SE2DistanceMapPosePoseBundleErrorFactor_::_drawImpl(srrg2_core::ViewerCanvasPtr)
     // const override

};  // class
    // SE2DistanceMapPosePoseBundleErrorFactor_<VariableSE2DistanceMapRightType_>
    // : public ErrorFactor_<1, VariableSE2DistanceMapRightType_,
    // VariableSE2DistanceMapRightType_>

/**
 * @brief SE(2) distance map-based pose to pose global bundle adjustment based
 *        on a point to point error formulation with sensor
 *        (single point! >>> you need the correspondence free driven for point
 *        clouds)
 *
 * in this case, the matcher of point to point error factor is now "put into"
 * the pose to pose constraints in the graph. the goal is to take advantage of
 * the distance map being a lookup table, allowing a fast computation of the
 * errors and jacobians... (with the derivatives being pre-computed)
 *
 * @note this factor is for right-sided perturbation update-based variables!
 *       (the error formulation is different in the manifold, depending on the
 *       perturbation update side!)
 * @tparam VariableSE2DistanceMapRightType_ distance map-based variable type
 *                                          (must be right-sided
 *                                          pertubation-based!)
 */
template <typename VariableSE2DistanceMapRightType_>
class SE2DistanceMapPosePoseBundleWithSensorErrorFactor_
    : public SE2DistanceMapPosePoseBundleErrorFactor_<
          VariableSE2DistanceMapRightType_>
{
 public:

  using BaseFactorType = SE2DistanceMapPosePoseBundleErrorFactor_<
      VariableSE2DistanceMapRightType_>;
  using ThisType = SE2DistanceMapPosePoseBundleWithSensorErrorFactor_<
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

    VariableType* v_from = this->_variables.template at<0>();
    VariableType* v_to = this->_variables.template at<1>();

    const EstimateType& from = v_from->estimate();
    const EstimateType& to = v_to->estimate();
    const EstimateType to_inv = to.inverse();

    const EstimateType& robot_in_sensor = this->m_robot_in_sensor_;
    const EstimateType& sensor_in_robot = this->m_sensor_in_robot_;

    const Eigen::Vector2f& moving = this->m_moving_->coordinates();
    const Eigen::Vector2f moving_in_from = sensor_in_robot * moving;
    const Eigen::Vector2f moving_in_w = from * moving_in_from;
    const Eigen::Vector2f moving_in_to = to_inv * moving_in_w;
    const Eigen::Vector2f moving_pred = robot_in_sensor * moving_in_to;

    typename VariableType::MeasurementOwnerType::DistanceMapType::CellTypeStd
        cell;

    if (!v_to->measurement()->getDistanceMap()->getValue(
            cell, moving_pred, *v_to->measurement()->getPointCloud()))
    {
      this->_is_valid = false;
      return;
    }

    this->_e(0, 0) = cell.m_dist;

    if (error_only)
    {
      return;
    }

    const Eigen::Matrix2f& R_from = from.linear();
    const Eigen::Matrix2f R_to_inv = to.linear().transpose();

    const Eigen::Matrix2f R_from_in_to = R_to_inv * R_from;

    const Eigen::Matrix2f& R_robot_in_sensor = robot_in_sensor.linear();
    /* const Eigen::Matrix2f& R_sensor_in_robot = sensor_in_robot.linear(); */

    const Eigen::Matrix2f R_from_in_to_sensor =
        R_robot_in_sensor * R_from_in_to;

    Matrix2_3f J_icp_from;
    J_icp_from.block<2, 2>(0, 0) = R_from_in_to_sensor;
    J_icp_from.block<2, 1>(0, 2) =
        -R_from_in_to_sensor * geometry2d::skew(moving_in_from);

    Matrix2_3f J_icp_to;
    J_icp_to.block<2, 2>(0, 0) = -R_robot_in_sensor;
    J_icp_to.block<2, 1>(0, 2) =
        R_robot_in_sensor * geometry2d::skew(moving_in_to);

    auto J_from = this->template jacobian<0>();
    auto J_to = this->template jacobian<1>();

    J_from = cell.m_dist_d * J_icp_from;
    J_to = cell.m_dist_d * J_icp_to;
  }  // void
     // SE2DistanceMapPosePoseBundleWithSensorErrorFactor_::errorAndJacobian(bool
     // = false) final

};  // class
    // SE2DistanceMapPosePoseBundleWithSensorErrorFactor_<VariableSE2DistanceMapRightType_>
    // : public
    // SE2DistanceMapPosePoseBundleErrorFactor_<VariableSE2DistanceMapRightType_>

using SE2DistanceMapStaticPoint2fVectorCloudRightPosePoseBundleErrorFactor =
    SE2DistanceMapPosePoseBundleErrorFactor_<
        VariableSE2DistanceMapStaticPoint2fVectorCloudRight>;
using SE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudRightPosePoseBundleErrorFactor =
    SE2DistanceMapPosePoseBundleErrorFactor_<
        VariableSE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudRight>;
using SE2DistanceMapSparseStaticPoint2fVectorCloudRightPosePoseBundleErrorFactor =
    SE2DistanceMapPosePoseBundleErrorFactor_<
        VariableSE2DistanceMapSparseStaticPoint2fVectorCloudRight>;
using SE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudRightPosePoseBundleErrorFactor =
    SE2DistanceMapPosePoseBundleErrorFactor_<
        VariableSE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudRight>;
using SE2DistanceMapNanoStaticPoint2fVectorCloudRightPosePoseBundleErrorFactor =
    SE2DistanceMapPosePoseBundleErrorFactor_<
        VariableSE2DistanceMapNanoStaticPoint2fVectorCloudRight>;
using SE2DistanceMapStaticPointNormal2fVectorCloudRightPosePoseBundleErrorFactor =
    SE2DistanceMapPosePoseBundleErrorFactor_<
        VariableSE2DistanceMapStaticPointNormal2fVectorCloudRight>;
using SE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRightPosePoseBundleErrorFactor =
    SE2DistanceMapPosePoseBundleErrorFactor_<
        VariableSE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRight>;
using SE2DistanceMapSparseStaticPointNormal2fVectorCloudRightPosePoseBundleErrorFactor =
    SE2DistanceMapPosePoseBundleErrorFactor_<
        VariableSE2DistanceMapSparseStaticPointNormal2fVectorCloudRight>;
using SE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRightPosePoseBundleErrorFactor =
    SE2DistanceMapPosePoseBundleErrorFactor_<
        VariableSE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRight>;
using SE2DistanceMapNanoStaticPointNormal2fVectorCloudRightPosePoseBundleErrorFactor =
    SE2DistanceMapPosePoseBundleErrorFactor_<
        VariableSE2DistanceMapNanoStaticPointNormal2fVectorCloudRight>;

using SE2DistanceMapStaticPoint2fVectorCloudRightPosePoseBundleWithSensorErrorFactor =
    SE2DistanceMapPosePoseBundleWithSensorErrorFactor_<
        VariableSE2DistanceMapStaticPoint2fVectorCloudRight>;
using SE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudRightPosePoseBundleWithSensorErrorFactor =
    SE2DistanceMapPosePoseBundleWithSensorErrorFactor_<
        VariableSE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudRight>;
using SE2DistanceMapSparseStaticPoint2fVectorCloudRightPosePoseBundleWithSensorErrorFactor =
    SE2DistanceMapPosePoseBundleWithSensorErrorFactor_<
        VariableSE2DistanceMapSparseStaticPoint2fVectorCloudRight>;
using SE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudRightPosePoseBundleWithSensorErrorFactor =
    SE2DistanceMapPosePoseBundleWithSensorErrorFactor_<
        VariableSE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudRight>;
using SE2DistanceMapNanoStaticPoint2fVectorCloudRightPosePoseBundleWithSensorErrorFactor =
    SE2DistanceMapPosePoseBundleWithSensorErrorFactor_<
        VariableSE2DistanceMapNanoStaticPoint2fVectorCloudRight>;
using SE2DistanceMapStaticPointNormal2fVectorCloudRightPosePoseBundleWithSensorErrorFactor =
    SE2DistanceMapPosePoseBundleWithSensorErrorFactor_<
        VariableSE2DistanceMapStaticPointNormal2fVectorCloudRight>;
using SE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRightPosePoseBundleWithSensorErrorFactor =
    SE2DistanceMapPosePoseBundleWithSensorErrorFactor_<
        VariableSE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRight>;
using SE2DistanceMapSparseStaticPointNormal2fVectorCloudRightPosePoseBundleWithSensorErrorFactor =
    SE2DistanceMapPosePoseBundleWithSensorErrorFactor_<
        VariableSE2DistanceMapSparseStaticPointNormal2fVectorCloudRight>;
using SE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRightPosePoseBundleWithSensorErrorFactor =
    SE2DistanceMapPosePoseBundleWithSensorErrorFactor_<
        VariableSE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRight>;
using SE2DistanceMapNanoStaticPointNormal2fVectorCloudRightPosePoseBundleWithSensorErrorFactor =
    SE2DistanceMapPosePoseBundleWithSensorErrorFactor_<
        VariableSE2DistanceMapNanoStaticPointNormal2fVectorCloudRight>;

template <typename VariableSE2DistanceMapRightType_>
using SE2DistanceMapPosePoseBundleErrorFactorCorrespondenceFreeDriven_ =
    FactorCorrespondenceFreeDriven_<SE2DistanceMapPosePoseBundleErrorFactor_<
                                        VariableSE2DistanceMapRightType_>,
                                    typename VariableSE2DistanceMapRightType_::
                                        MeasurementOwnerType::PointCloudType>;

template <typename VariableSE2DistanceMapRightType_>
using SE2DistanceMapPosePoseBundleWithSensorErrorFactorCorrespondenceFreeDriven_ =
    FactorCorrespondenceFreeDriven_<
        SE2DistanceMapPosePoseBundleWithSensorErrorFactor_<
            VariableSE2DistanceMapRightType_>,
        typename VariableSE2DistanceMapRightType_::MeasurementOwnerType::
            PointCloudType>;

using SE2DistanceMapStaticPoint2fVectorCloudRightPosePoseBundleErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPosePoseBundleErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapStaticPoint2fVectorCloudRight>;
using SE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudRightPosePoseBundleErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPosePoseBundleErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudRight>;
using SE2DistanceMapSparseStaticPoint2fVectorCloudRightPosePoseBundleErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPosePoseBundleErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapSparseStaticPoint2fVectorCloudRight>;
using SE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudRightPosePoseBundleErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPosePoseBundleErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudRight>;
using SE2DistanceMapNanoStaticPoint2fVectorCloudRightPosePoseBundleErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPosePoseBundleErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapNanoStaticPoint2fVectorCloudRight>;
using SE2DistanceMapStaticPointNormal2fVectorCloudRightPosePoseBundleErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPosePoseBundleErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapStaticPointNormal2fVectorCloudRight>;
using SE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRightPosePoseBundleErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPosePoseBundleErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRight>;
using SE2DistanceMapSparseStaticPointNormal2fVectorCloudRightPosePoseBundleErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPosePoseBundleErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapSparseStaticPointNormal2fVectorCloudRight>;
using SE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRightPosePoseBundleErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPosePoseBundleErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRight>;
using SE2DistanceMapNanoStaticPointNormal2fVectorCloudRightPosePoseBundleErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPosePoseBundleErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapNanoStaticPointNormal2fVectorCloudRight>;

using SE2DistanceMapStaticPoint2fVectorCloudRightPosePoseBundleWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPosePoseBundleWithSensorErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapStaticPoint2fVectorCloudRight>;
using SE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudRightPosePoseBundleWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPosePoseBundleWithSensorErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudRight>;
using SE2DistanceMapSparseStaticPoint2fVectorCloudRightPosePoseBundleWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPosePoseBundleWithSensorErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapSparseStaticPoint2fVectorCloudRight>;
using SE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudRightPosePoseBundleWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPosePoseBundleWithSensorErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudRight>;
using SE2DistanceMapNanoStaticPoint2fVectorCloudRightPosePoseBundleWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPosePoseBundleWithSensorErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapNanoStaticPoint2fVectorCloudRight>;
using SE2DistanceMapStaticPointNormal2fVectorCloudRightPosePoseBundleWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPosePoseBundleWithSensorErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapStaticPointNormal2fVectorCloudRight>;
using SE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRightPosePoseBundleWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPosePoseBundleWithSensorErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRight>;
using SE2DistanceMapSparseStaticPointNormal2fVectorCloudRightPosePoseBundleWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPosePoseBundleWithSensorErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapSparseStaticPointNormal2fVectorCloudRight>;
using SE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRightPosePoseBundleWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPosePoseBundleWithSensorErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRight>;
using SE2DistanceMapNanoStaticPointNormal2fVectorCloudRightPosePoseBundleWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPosePoseBundleWithSensorErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapNanoStaticPointNormal2fVectorCloudRight>;

/**
 * @note Pose-pose global bundle adjustment error factor (map global refinement)
 *
 * (simple factor)
 *
 * SE2DistanceMapStaticPoint2fVectorCloudRightPosePoseBundleErrorFactor
 * SE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudRightPosePoseBundleErrorFactor
 * SE2DistanceMapSparseStaticPoint2fVectorCloudRightPosePoseBundleErrorFactor
 * SE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudRightPosePoseBundleErrorFactor
 * SE2DistanceMapNanoStaticPoint2fVectorCloudRightPosePoseBundleErrorFactor
 * SE2DistanceMapStaticPointNormal2fVectorCloudRightPosePoseBundleErrorFactor
 * SE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRightPosePoseBundleErrorFactor
 * SE2DistanceMapSparseStaticPointNormal2fVectorCloudRightPosePoseBundleErrorFactor
 * SE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRightPosePoseBundleErrorFactor
 * SE2DistanceMapNanoStaticPointNormal2fVectorCloudRightPosePoseBundleErrorFactor
 *
 * SE2DistanceMapStaticPoint2fVectorCloudRightPosePoseBundleWithSensorErrorFactor
 * SE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudRightPosePoseBundleWithSensorErrorFactor
 * SE2DistanceMapSparseStaticPoint2fVectorCloudRightPosePoseBundleWithSensorErrorFactor
 * SE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudRightPosePoseBundleWithSensorErrorFactor
 * SE2DistanceMapNanoStaticPoint2fVectorCloudRightPosePoseBundleWithSensorErrorFactor
 * SE2DistanceMapStaticPointNormal2fVectorCloudRightPosePoseBundleWithSensorErrorFactor
 * SE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRightPosePoseBundleWithSensorErrorFactor
 * SE2DistanceMapSparseStaticPointNormal2fVectorCloudRightPosePoseBundleWithSensorErrorFactor
 * SE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRightPosePoseBundleWithSensorErrorFactor
 * SE2DistanceMapNanoStaticPointNormal2fVectorCloudRightPosePoseBundleWithSensorErrorFactor
 *
 * (factor correspondence driven)
 *
 * SE2DistanceMapStaticPoint2fVectorCloudRightPosePoseBundleErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudRightPosePoseBundleErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapSparseStaticPoint2fVectorCloudRightPosePoseBundleErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudRightPosePoseBundleErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapNanoStaticPoint2fVectorCloudRightPosePoseBundleErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapStaticPointNormal2fVectorCloudRightPosePoseBundleErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRightPosePoseBundleErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapSparseStaticPointNormal2fVectorCloudRightPosePoseBundleErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRightPosePoseBundleErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapNanoStaticPointNormal2fVectorCloudRightPosePoseBundleErrorFactorCorrespondenceFreeDriven
 *
 * SE2DistanceMapStaticPoint2fVectorCloudRightPosePoseBundleWithSensorErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudRightPosePoseBundleWithSensorErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapSparseStaticPoint2fVectorCloudRightPosePoseBundleWithSensorErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudRightPosePoseBundleWithSensorErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapNanoStaticPoint2fVectorCloudRightPosePoseBundleWithSensorErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapStaticPointNormal2fVectorCloudRightPosePoseBundleWithSensorErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRightPosePoseBundleWithSensorErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapSparseStaticPointNormal2fVectorCloudRightPosePoseBundleWithSensorErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRightPosePoseBundleWithSensorErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapNanoStaticPointNormal2fVectorCloudRightPosePoseBundleWithSensorErrorFactorCorrespondenceFreeDriven
 */

}  // namespace srrg2_solver
