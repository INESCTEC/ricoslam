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
 * @brief SE(2) distance map-based pose to pose global bundle adjustment based
 *        on a point to plane error formulation
 *        (single point! >>> you need the correspondence free driven for point
 *        clouds)
 *
 * in this case, the matcher of point to point error factor is now "put into"
 * the pose to pose constraints in the graph. the goal is to take advantage of
 * the distance map being a lookup table, allowing a fast computation of the
 * errors and jacobians... (with the derivatives being pre-computed)
 *
 * this factor differs from SE2DistanceMapPosePoseBundleErrorFactor_ by
 * considering the normals from the moving point cloud, while requiring the 2nd
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
class SE2DistanceMapPosePoseBundlePlaneErrorFactor_
    : public ErrorFactor_<1, VariableSE2DistanceMapRightType_,
                          VariableSE2DistanceMapRightType_>
{
 public:

  using BaseFactorType = ErrorFactor_<1, VariableSE2DistanceMapRightType_,
                                      VariableSE2DistanceMapRightType_>;
  using ThisType = SE2DistanceMapPosePoseBundlePlaneErrorFactor_<
      VariableSE2DistanceMapRightType_>;

  using VariableType = VariableSE2DistanceMapRightType_;
  using EstimateType = typename VariableType::EstimateType;

  using MovingType = inesctec_mrdt_slam_distmap_2d::pcl::PointNormal2_<
      typename VariableType::MeasurementOwnerType::DistanceMapType::Scalar>;

 protected:

  const MovingType* m_moving_ = nullptr;

 public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  inline void setMoving(const MovingType& moving) { this->m_moving_ = &moving; }

  void errorAndJacobian(bool error_only = false) override
  {
    this->_is_valid = true;

    if (this->m_moving_->normal().isZero())
    {
      this->_is_valid = false;
      return;
    }

    VariableType* v_from = this->_variables.template at<0>();
    VariableType* v_to = this->_variables.template at<1>();

    const EstimateType& from = v_from->estimate();
    const EstimateType& to = v_to->estimate();
    const EstimateType to_inv = to.inverse();

    const Eigen::Vector2f& moving = this->m_moving_->coordinates();
    const Eigen::Vector2f moving_in_w = from * moving;
    const Eigen::Vector2f moving_pred = to_inv * moving_in_w;

    typename VariableType::MeasurementOwnerType::DistanceMapType::
        CellTypeWith2ndDeriv cell;

    if (!v_to->measurement()->getDistanceMap()->getValueWith2ndDeriv(
            cell, moving_pred, *v_to->measurement()->getPointCloud()))
    {
      this->_is_valid = false;
      return;
    }

    const Eigen::Matrix2f& R_from = from.linear();
    const Eigen::Matrix2f R_to_inv = to.linear().transpose();

    const Eigen::Matrix2f R_from_in_to = R_to_inv * R_from;
    const Eigen::Matrix2f R_to_in_from = R_from_in_to.transpose();

    using Matrix1_2f = Eigen::Matrix<float, 1, 2>;

    const Eigen::Vector2f& n_moving = this->m_moving_->normal();
    const Eigen::Vector2f n_moving_in_w = R_from * n_moving;
    const Eigen::Vector2f n_moving_in_to = R_to_inv * n_moving_in_w;
    const Matrix1_2f n_moving_in_to_trans = n_moving_in_to.transpose();

    const Eigen::Vector2f dist_d_v_normalized =
        cell.m_dist_d.normalized().transpose();

    this->_e(0, 0) = n_moving_in_to_trans * dist_d_v_normalized;
    this->_e(0, 0) *= cell.m_dist;

    if (error_only)
    {
      return;
    }

    using Matrix2_3f = Eigen::Matrix<float, 2, 3>;

    Matrix2_3f J_icp_from;
    J_icp_from.block<2, 2>(0, 0) = R_from_in_to;
    J_icp_from.block<2, 1>(0, 2) = -R_from_in_to * geometry2d::skew(moving);

    Matrix2_3f J_icp_to;
    J_icp_to.block<2, 2>(0, 0) = -Matrix2f::Identity();
    J_icp_to.block<2, 1>(0, 2) = geometry2d::skew(moving_pred);

    auto J_from = this->template jacobian<0>();
    auto J_to = this->template jacobian<1>();

    Eigen::Vector2f dist_d_v_dir_rot_in_from =
        R_to_in_from * dist_d_v_normalized;
    dist_d_v_dir_rot_in_from *= cell.m_dist;

    /* from */

    Matrix2_3f J_aux;
    J_aux.block<2, 2>(0, 0) = Matrix2f::Zero();
    J_aux.block<2, 1>(0, 2) = geometry2d::skew(dist_d_v_dir_rot_in_from);

    J_from = n_moving.transpose() * J_aux;

    Eigen::Matrix2f M_aux = cell.m_dist_d2 * cell.m_dist;
    M_aux += dist_d_v_normalized * cell.m_dist_d;

    J_aux = M_aux * J_icp_from;

    J_from += n_moving_in_to_trans * J_aux;

    /* to */

    J_aux.block<2, 2>(0, 0) = Matrix2f::Zero();
    J_aux.block<2, 1>(0, 2) =
        -cell.m_dist * geometry2d::skew(dist_d_v_normalized);

    J_aux += M_aux * J_icp_to;

    J_to = n_moving_in_to_trans * J_aux;
  }  // void
     // SE2DistanceMapPosePoseBundlePlaneErrorFactor_::errorAndJacobian(bool =
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
     // SE2DistanceMapPosePoseBundlePlaneErrorFactor_::_drawImpl(srrg2_core::ViewerCanvasPtr)
     // const override

};  // class
    // SE2DistanceMapPosePoseBundlePlaneErrorFactor_<VariableSE2DistanceMapRightType_>
    // : public ErrorFactor_<1, VariableSE2DistanceMapRightType_,
    // VariableSE2DistanceMapRightType_>

/**
 * @brief SE(2) distance map-based pose to pose global bundle adjustment based
 *        on a point to plane error formulation with sensor
 *        (single point! >>> you need the correspondence free driven for point
 *        clouds)
 *
 * in this case, the matcher of point to point error factor is now "put into"
 * the pose to pose constraints in the graph. the goal is to take advantage of
 * the distance map being a lookup table, allowing a fast computation of the
 * errors and jacobians... (with the derivatives being pre-computed)
 *
 * this factor differs from SE2DistanceMapPosePoseBundleErrorFactor_ by
 * considering the normals from the moving point cloud, while requiring the 2nd
 * derivative from the distance map / fixed point cloud (may be pre-computed or
 * computed for each point evaluation), depending on the distance map container.
 *
 * @note this factor is for right-sided perturbation update-based variables!
 *       (the error formulation is different in the manifold, depending on the
 *       perturbation update side!)
 * @todo possible bug on formulation (sometimes solver returns bad coefficients)
 * @tparam VariableSE2DistanceMapRightType_ distance map-based variable type
 *                                          (must be right-sided
 *                                          pertubation-based!)
 */
template <typename VariableSE2DistanceMapRightType_>
class SE2DistanceMapPosePoseBundlePlaneWithSensorErrorFactor_
    : public SE2DistanceMapPosePoseBundlePlaneErrorFactor_<
          VariableSE2DistanceMapRightType_>
{
 public:

  using BaseFactorType = SE2DistanceMapPosePoseBundlePlaneErrorFactor_<
      VariableSE2DistanceMapRightType_>;
  using ThisType = SE2DistanceMapPosePoseBundlePlaneWithSensorErrorFactor_<
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

    if (this->m_moving_->normal().isZero())
    {
      this->_is_valid = false;
      return;
    }

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

    typename VariableType::MeasurementOwnerType::DistanceMapType::
        CellTypeWith2ndDeriv cell;

    if (!v_to->measurement()->getDistanceMap()->getValueWith2ndDeriv(
            cell, moving_pred, *v_to->measurement()->getPointCloud()))
    {
      this->_is_valid = false;
      return;
    }

    const Eigen::Matrix2f& R_from = from.linear();
    const Eigen::Matrix2f R_to_inv = to.linear().transpose();

    const Eigen::Matrix2f R_from_in_to = R_to_inv * R_from;
    const Eigen::Matrix2f R_to_in_from = R_from_in_to.transpose();

    const Eigen::Matrix2f& R_robot_in_sensor = robot_in_sensor.linear();
    /* const Eigen::Matrix2f& R_sensor_in_robot = sensor_in_robot.linear(); */

    using Matrix1_2f = Eigen::Matrix<float, 1, 2>;

    const Eigen::Vector2f& n_moving = this->m_moving_->normal();
    const Eigen::Vector2f n_moving_in_w = R_from * n_moving;
    const Eigen::Vector2f n_moving_in_to = R_to_inv * n_moving_in_w;
    const Matrix1_2f n_moving_in_to_trans = n_moving_in_to.transpose();

    const Eigen::Vector2f dist_d_v_normalized =
        cell.m_dist_d.normalized().transpose();

    this->_e(0, 0) = n_moving_in_to_trans * dist_d_v_normalized;
    this->_e(0, 0) *= cell.m_dist;

    if (error_only)
    {
      return;
    }

    using Matrix2_3f = Eigen::Matrix<float, 2, 3>;

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

    Eigen::Vector2f dist_d_v_dir_rot_in_from =
        R_to_in_from * dist_d_v_normalized;
    dist_d_v_dir_rot_in_from *= cell.m_dist;

    /* from */

    Matrix2_3f J_aux;
    J_aux.block<2, 2>(0, 0) = Matrix2f::Zero();
    J_aux.block<2, 1>(0, 2) = geometry2d::skew(dist_d_v_dir_rot_in_from);

    J_from = n_moving.transpose() * J_aux;

    Eigen::Matrix2f M_aux = cell.m_dist_d2 * cell.m_dist;
    M_aux += dist_d_v_normalized * cell.m_dist_d;

    J_aux = M_aux * J_icp_from;

    J_from += n_moving_in_to_trans * J_aux;

    /* to */

    J_aux.block<2, 2>(0, 0) = Matrix2f::Zero();
    J_aux.block<2, 1>(0, 2) =
        -cell.m_dist * geometry2d::skew(dist_d_v_normalized);

    J_aux += M_aux * J_icp_to;

    J_to = n_moving_in_to_trans * J_aux;
  }  // void
     // SE2DistanceMapPosePoseBundlePlaneWithSensorErrorFactor_::errorAndJacobian(bool
     // = false) final

};  // class
    // SE2DistanceMapPosePoseBundlePlaneWithSensorErrorFactor_<VariableSE2DistanceMapRightType_>
    // : public
    // SE2DistanceMapPosePoseBundlePlaneErrorFactor_<VariableSE2DistanceMapRightType_>

using SE2DistanceMapStaticPointNormal2fVectorCloudRightPosePoseBundlePlaneErrorFactor =
    SE2DistanceMapPosePoseBundlePlaneErrorFactor_<
        VariableSE2DistanceMapStaticPointNormal2fVectorCloudRight>;
using SE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRightPosePoseBundlePlaneErrorFactor =
    SE2DistanceMapPosePoseBundlePlaneErrorFactor_<
        VariableSE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRight>;
using SE2DistanceMapSparseStaticPointNormal2fVectorCloudRightPosePoseBundlePlaneErrorFactor =
    SE2DistanceMapPosePoseBundlePlaneErrorFactor_<
        VariableSE2DistanceMapSparseStaticPointNormal2fVectorCloudRight>;
using SE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRightPosePoseBundlePlaneErrorFactor =
    SE2DistanceMapPosePoseBundlePlaneErrorFactor_<
        VariableSE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRight>;
using SE2DistanceMapNanoStaticPointNormal2fVectorCloudRightPosePoseBundlePlaneErrorFactor =
    SE2DistanceMapPosePoseBundlePlaneErrorFactor_<
        VariableSE2DistanceMapNanoStaticPointNormal2fVectorCloudRight>;

using SE2DistanceMapStaticPointNormal2fVectorCloudRightPosePoseBundlePlaneWithSensorErrorFactor =
    SE2DistanceMapPosePoseBundlePlaneWithSensorErrorFactor_<
        VariableSE2DistanceMapStaticPointNormal2fVectorCloudRight>;
using SE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRightPosePoseBundlePlaneWithSensorErrorFactor =
    SE2DistanceMapPosePoseBundlePlaneWithSensorErrorFactor_<
        VariableSE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRight>;
using SE2DistanceMapSparseStaticPointNormal2fVectorCloudRightPosePoseBundlePlaneWithSensorErrorFactor =
    SE2DistanceMapPosePoseBundlePlaneWithSensorErrorFactor_<
        VariableSE2DistanceMapSparseStaticPointNormal2fVectorCloudRight>;
using SE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRightPosePoseBundlePlaneWithSensorErrorFactor =
    SE2DistanceMapPosePoseBundlePlaneWithSensorErrorFactor_<
        VariableSE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRight>;
using SE2DistanceMapNanoStaticPointNormal2fVectorCloudRightPosePoseBundlePlaneWithSensorErrorFactor =
    SE2DistanceMapPosePoseBundlePlaneWithSensorErrorFactor_<
        VariableSE2DistanceMapNanoStaticPointNormal2fVectorCloudRight>;

template <typename VariableSE2DistanceMapRightType_>
using SE2DistanceMapPosePoseBundlePlaneErrorFactorCorrespondenceFreeDriven_ =
    FactorCorrespondenceFreeDriven_<
        SE2DistanceMapPosePoseBundlePlaneErrorFactor_<
            VariableSE2DistanceMapRightType_>,
        inesctec_mrdt_slam_distmap_2d::pcl::PointNormal2VectorCloud_<
            typename VariableSE2DistanceMapRightType_::MeasurementOwnerType::
                DistanceMapType::Scalar>>;

template <typename VariableSE2DistanceMapRightType_>
using SE2DistanceMapPosePoseBundlePlaneWithSensorErrorFactorCorrespondenceFreeDriven_ =
    FactorCorrespondenceFreeDriven_<
        SE2DistanceMapPosePoseBundlePlaneWithSensorErrorFactor_<
            VariableSE2DistanceMapRightType_>,
        inesctec_mrdt_slam_distmap_2d::pcl::PointNormal2VectorCloud_<
            typename VariableSE2DistanceMapRightType_::MeasurementOwnerType::
                DistanceMapType::Scalar>>;

using SE2DistanceMapStaticPointNormal2fVectorCloudRightPosePoseBundlePlaneErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPosePoseBundlePlaneErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapStaticPointNormal2fVectorCloudRight>;
using SE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRightPosePoseBundlePlaneErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPosePoseBundlePlaneErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRight>;
using SE2DistanceMapSparseStaticPointNormal2fVectorCloudRightPosePoseBundlePlaneErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPosePoseBundlePlaneErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapSparseStaticPointNormal2fVectorCloudRight>;
using SE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRightPosePoseBundlePlaneErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPosePoseBundlePlaneErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRight>;
using SE2DistanceMapNanoStaticPointNormal2fVectorCloudRightPosePoseBundlePlaneErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPosePoseBundlePlaneErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapNanoStaticPointNormal2fVectorCloudRight>;

using SE2DistanceMapStaticPointNormal2fVectorCloudRightPosePoseBundlePlaneWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPosePoseBundlePlaneWithSensorErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapStaticPointNormal2fVectorCloudRight>;
using SE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRightPosePoseBundlePlaneWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPosePoseBundlePlaneWithSensorErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRight>;
using SE2DistanceMapSparseStaticPointNormal2fVectorCloudRightPosePoseBundlePlaneWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPosePoseBundlePlaneWithSensorErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapSparseStaticPointNormal2fVectorCloudRight>;
using SE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRightPosePoseBundlePlaneWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPosePoseBundlePlaneWithSensorErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRight>;
using SE2DistanceMapNanoStaticPointNormal2fVectorCloudRightPosePoseBundlePlaneWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapPosePoseBundlePlaneWithSensorErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapNanoStaticPointNormal2fVectorCloudRight>;

/**
 * @note Pose-pose global bundle adjustment error factor (map global refinement)
 *
 * (simple factor)
 *
 * SE2DistanceMapStaticPointNormal2fVectorCloudRightPosePoseBundlePlaneErrorFactor
 * SE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRightPosePoseBundlePlaneErrorFactor
 * SE2DistanceMapSparseStaticPointNormal2fVectorCloudRightPosePoseBundlePlaneErrorFactor
 * SE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRightPosePoseBundlePlaneErrorFactor
 * SE2DistanceMapNanoStaticPointNormal2fVectorCloudRightPosePoseBundlePlaneErrorFactor
 *
 * SE2DistanceMapStaticPointNormal2fVectorCloudRightPosePoseBundlePlaneWithSensorErrorFactor
 * SE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRightPosePoseBundlePlaneWithSensorErrorFactor
 * SE2DistanceMapSparseStaticPointNormal2fVectorCloudRightPosePoseBundlePlaneWithSensorErrorFactor
 * SE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRightPosePoseBundlePlaneWithSensorErrorFactor
 * SE2DistanceMapNanoStaticPointNormal2fVectorCloudRightPosePoseBundlePlaneWithSensorErrorFactor
 *
 * (factor correspondence driven)
 *
 * SE2DistanceMapStaticPointNormal2fVectorCloudRightPosePoseBundlePlaneErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRightPosePoseBundlePlaneErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapSparseStaticPointNormal2fVectorCloudRightPosePoseBundlePlaneErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRightPosePoseBundlePlaneErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapNanoStaticPointNormal2fVectorCloudRightPosePoseBundlePlaneErrorFactorCorrespondenceFreeDriven
 *
 * SE2DistanceMapStaticPointNormal2fVectorCloudRightPosePoseBundlePlaneWithSensorErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRightPosePoseBundlePlaneWithSensorErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapSparseStaticPointNormal2fVectorCloudRightPosePoseBundlePlaneWithSensorErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRightPosePoseBundlePlaneWithSensorErrorFactorCorrespondenceFreeDriven
 * SE2DistanceMapNanoStaticPointNormal2fVectorCloudRightPosePoseBundlePlaneWithSensorErrorFactorCorrespondenceFreeDriven
 */

}  // namespace srrg2_solver
