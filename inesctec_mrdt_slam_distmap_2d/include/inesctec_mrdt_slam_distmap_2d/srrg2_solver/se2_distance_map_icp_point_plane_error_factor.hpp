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
 * @brief SE(2) Iterative Closest Point (ICP) point to plane error factor
 *        using distance map strucutres for Nearest Neighbor (NN) search
 *        (single point! >>> you need the correspondence free driven for point
 *        clouds)
 *
 * the main goal of this factor is to not required any correspondence update
 * during the matching process. the correspondence is indirectly, through the
 * projection of the moving point into the fixed reference frame, where the NN
 * is found from the distance map computation, unlike kd-tree-based
 * implementations where you need to sesarch the tree for the correspondences.
 *
 * this factor formulates the point-to-plane error factor by putting the normals
 * ib the moving instead of the fixed point cloud (e.g., you could only compute
 * the normals of the moving / current point cloud and your map representation
 * based only on a point-based no-normals point cloud).
 *
 * @note this factor is for right-sided perturbation update-based variables!
 *       (the error formulation is different in the manifold, depending on the
 *       perturbation update side!)
 * @tparam VariableSE2DistanceMapRightType_ distance map-based variable type
 *                                          (must be right-sided
 *                                          pertubation-based!)
 */
template <typename VariableSE2DistanceMapRightType_>
class SE2DistanceMapICPPointPlaneErrorFactor_
    : public ErrorFactor_<1, VariableSE2DistanceMapRightType_>
{
 public:

  using BaseFactorType = ErrorFactor_<1, VariableSE2DistanceMapRightType_>;
  using ThisType =
      SE2DistanceMapICPPointPlaneErrorFactor_<VariableSE2DistanceMapRightType_>;

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
    return "SE2DistanceMapICPPointPlaneErrorFactor (ICP point-to-plane)";
  }

  inline void setMoving(const MovingType& moving) { this->m_moving_ = &moving; }

  inline int32_t getCorrespondenceIdxFixed() const
  {
    return this->m_correspondence_idx_fixed_;
  }

  void errorAndJacobian(bool error_only = false) override
  {
    using Matrix1_2f = Eigen::Matrix<float, 1, 2>;
    using Matrix2_3f = Eigen::Matrix<float, 2, 3>;

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

    const Eigen::Vector2f p_moving_pred = X * (this->m_moving_->coordinates());

    typename VariableType::MeasurementOwnerType::DistanceMapType::CellNNIdxType
        nn;

    if (!X_var->measurement()->getDistanceMap()->getNN(nn, p_moving_pred))
    {
      this->_is_valid = false;
      return;
    }

    const Eigen::Vector2f e_ICP =
        p_moving_pred -
        X_var->measurement()->getPointCloud()->at(nn).coordinates();
    const Eigen::Vector2f n_moving_pred = R * this->m_moving_->normal();
    const Matrix1_2f n_moving_pred_trans = n_moving_pred.transpose();

    this->_e(0, 0) = n_moving_pred_trans * e_ICP;
    this->m_correspondence_idx_fixed_ = nn;

    if (error_only)
    {
      return;
    }

    const Eigen::Vector2f e_ICP_rot = R.transpose() * e_ICP;

    Matrix2_3f J_aux;

    J_aux.block<2, 2>(0, 0) = R;  // J_ICP
    J_aux.block<2, 1>(0, 2) =
        -R * geometry2d::skew(this->m_moving_->coordinates());

    this->_J = n_moving_pred_trans * J_aux;

    J_aux.block<2, 2>(0, 0).setZero();
    J_aux.block<2, 1>(0, 2) = geometry2d::skew(e_ICP_rot);

    this->_J += this->m_moving_->normal().transpose() * J_aux;
  }  // void
     // SE2DistanceMapICPPointPlaneErrorFactor_<VariableSE2DistanceMapRightType_>::errorAndJacobian(bool
     // = false) override

};  // class
    // SE2DistanceMapICPPointPlaneErrorFactor_<VariableSE2DistanceMapRightType_>
    // : public ErrorFactor_<1, VariableSE2DistanceMapRightType_>

/**
 * @brief SE(2) Iterative Closest Point (ICP) point to plane error factor
 *        using distance map strucutres for Nearest Neighbor (NN) search
 *        (single point! >>> you need the correspondence free driven for point
 *        clouds)
 *
 * the main goal of this factor is to not required any correspondence update
 * during the matching process. the correspondence is indirectly, through the
 * projection of the moving point into the fixed reference frame, where the NN
 * is found from the distance map computation, unlike kd-tree-based
 * implementations where you need to sesarch the tree for the correspondences.
 *
 * this factor formulates the point-to-plane error factor by putting the normals
 * ib the moving instead of the fixed point cloud (e.g., you could only compute
 * the normals of the moving / current point cloud and your map representation
 * based only on a point-based no-normals point cloud).
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
class SE2DistanceMapICPPointPlaneWithSensorErrorFactor_
    : public SE2DistanceMapICPPointPlaneErrorFactor_<
          VariableSE2DistanceMapRightType_>
{
 public:

  using BaseFactorType =
      SE2DistanceMapICPPointPlaneErrorFactor_<VariableSE2DistanceMapRightType_>;
  using ThisType = SE2DistanceMapICPPointPlaneWithSensorErrorFactor_<
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
    using Matrix1_2f = Eigen::Matrix<float, 1, 2>;
    using Matrix2_3f = Eigen::Matrix<float, 2, 3>;

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

    const Matrix2f R_robot_in_sensor_R =
        this->m_robot_in_sensor_.linear() * X.linear();

    const Vector2f p_moving_in_robot =
        this->m_sensor_in_robot_ * this->m_moving_->coordinates();
    const Vector2f p_moving_pred_in_robot = X * p_moving_in_robot;
    const Vector2f p_moving_pred_in_sensor =
        this->m_robot_in_sensor_ * p_moving_pred_in_robot;

    typename VariableType::MeasurementOwnerType::DistanceMapType::CellNNIdxType
        nn;

    if (!X_var->measurement()->getDistanceMap()->getNN(nn,
                                                       p_moving_pred_in_sensor))
    {
      this->_is_valid = false;
      return;
    }

    const Eigen::Vector2f e_ICP =
        p_moving_pred_in_sensor -
        X_var->measurement()->getPointCloud()->at(nn).coordinates();
    const Eigen::Vector2f n_moving_pred = R * this->m_moving_->normal();
    const Matrix1_2f n_moving_pred_trans = n_moving_pred.transpose();

    this->_e(0, 0) = n_moving_pred_trans * e_ICP;
    this->m_correspondence_idx_fixed_ = nn;

    if (error_only)
    {
      return;
    }

    const Eigen::Vector2f e_ICP_rot = R.transpose() * e_ICP;

    Matrix2_3f J_aux;

    J_aux.block<2, 2>(0, 0) = R_robot_in_sensor_R;  // J_ICP
    J_aux.block<2, 1>(0, 2) =
        -R_robot_in_sensor_R * geometry2d::skew(p_moving_in_robot);

    this->_J = n_moving_pred_trans * J_aux;

    J_aux.block<2, 2>(0, 0).setZero();
    J_aux.block<2, 1>(0, 2) = geometry2d::skew(e_ICP_rot);

    this->_J += this->m_moving_->normal().transpose() * J_aux;
  }  // void
     // SE2DistanceMapICPPointPlaneWithSensorErrorFactor_<VariableSE2DistanceMapRightType_>::errorAndJacobian(bool
     // = false) final
};  // class
    // SE2DistanceMapICPPointPlaneWithSensorErrorFactor_<VariableSE2DistanceMapRightType_>
    // : public
    // SE2DistanceMapICPPointPlaneErrorFactor_<VariableSE2DistanceMapRightType_>

// VariableSE2DistanceMapStaticPoint2fVectorCloudRight
// VariableSE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudRight
// VariableSE2DistanceMapSparseStaticPoint2fVectorCloudRight
// VariableSE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudRight
// VariableSE2DistanceMapNanoStaticPoint2fVectorCloudRight
// VariableSE2DistanceMapStaticPointNormal2fVectorCloudRight
// VariableSE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRight
// VariableSE2DistanceMapSparseStaticPointNormal2fVectorCloudRight
// VariableSE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRight
// VariableSE2DistanceMapNanoStaticPointNormal2fVectorCloudRight

// StaticPoint2fVectorCloudRight
// StaticWith2ndDerivPoint2fVectorCloudRight
// SparseStaticPoint2fVectorCloudRight
// SparseStaticWith2ndDerivPoint2fVectorCloudRight
// NanoStaticPoint2fVectorCloudRight
// StaticPointNormal2fVectorCloudRight
// StaticWith2ndDerivPointNormal2fVectorCloudRight
// SparseStaticPointNormal2fVectorCloudRight
// SparseStaticWith2ndDerivPointNormal2fVectorCloudRight
// NanoStaticPointNormal2fVectorCloudRight

using SE2DistanceMapStaticPoint2fVectorCloudRightICPPointPlaneErrorFactor =
    SE2DistanceMapICPPointPlaneErrorFactor_<
        VariableSE2DistanceMapStaticPoint2fVectorCloudRight>;
using SE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudRightICPPointPlaneErrorFactor =
    SE2DistanceMapICPPointPlaneErrorFactor_<
        VariableSE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudRight>;
using SE2DistanceMapSparseStaticPoint2fVectorCloudRightICPPointPlaneErrorFactor =
    SE2DistanceMapICPPointPlaneErrorFactor_<
        VariableSE2DistanceMapSparseStaticPoint2fVectorCloudRight>;
using SE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudRightICPPointPlaneErrorFactor =
    SE2DistanceMapICPPointPlaneErrorFactor_<
        VariableSE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudRight>;
using SE2DistanceMapNanoStaticPoint2fVectorCloudRightICPPointPlaneErrorFactor =
    SE2DistanceMapICPPointPlaneErrorFactor_<
        VariableSE2DistanceMapNanoStaticPoint2fVectorCloudRight>;
using SE2DistanceMapStaticPointNormal2fVectorCloudRightICPPointPlaneErrorFactor =
    SE2DistanceMapICPPointPlaneErrorFactor_<
        VariableSE2DistanceMapStaticPointNormal2fVectorCloudRight>;
using SE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRightICPPointPlaneErrorFactor =
    SE2DistanceMapICPPointPlaneErrorFactor_<
        VariableSE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRight>;
using SE2DistanceMapSparseStaticPointNormal2fVectorCloudRightICPPointPlaneErrorFactor =
    SE2DistanceMapICPPointPlaneErrorFactor_<
        VariableSE2DistanceMapSparseStaticPointNormal2fVectorCloudRight>;
using SE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRightICPPointPlaneErrorFactor =
    SE2DistanceMapICPPointPlaneErrorFactor_<
        VariableSE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRight>;
using SE2DistanceMapNanoStaticPointNormal2fVectorCloudRightICPPointPlaneErrorFactor =
    SE2DistanceMapICPPointPlaneErrorFactor_<
        VariableSE2DistanceMapNanoStaticPointNormal2fVectorCloudRight>;

using SE2DistanceMapStaticPoint2fVectorCloudRightICPPointPlaneWithSensorErrorFactor =
    SE2DistanceMapICPPointPlaneWithSensorErrorFactor_<
        VariableSE2DistanceMapStaticPoint2fVectorCloudRight>;
using SE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudRightICPPointPlaneWithSensorErrorFactor =
    SE2DistanceMapICPPointPlaneWithSensorErrorFactor_<
        VariableSE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudRight>;
using SE2DistanceMapSparseStaticPoint2fVectorCloudRightICPPointPlaneWithSensorErrorFactor =
    SE2DistanceMapICPPointPlaneWithSensorErrorFactor_<
        VariableSE2DistanceMapSparseStaticPoint2fVectorCloudRight>;
using SE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudRightICPPointPlaneWithSensorErrorFactor =
    SE2DistanceMapICPPointPlaneWithSensorErrorFactor_<
        VariableSE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudRight>;
using SE2DistanceMapNanoStaticPoint2fVectorCloudRightICPPointPlaneWithSensorErrorFactor =
    SE2DistanceMapICPPointPlaneWithSensorErrorFactor_<
        VariableSE2DistanceMapNanoStaticPoint2fVectorCloudRight>;
using SE2DistanceMapStaticPointNormal2fVectorCloudRightICPPointPlaneWithSensorErrorFactor =
    SE2DistanceMapICPPointPlaneWithSensorErrorFactor_<
        VariableSE2DistanceMapStaticPointNormal2fVectorCloudRight>;
using SE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRightICPPointPlaneWithSensorErrorFactor =
    SE2DistanceMapICPPointPlaneWithSensorErrorFactor_<
        VariableSE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRight>;
using SE2DistanceMapSparseStaticPointNormal2fVectorCloudRightICPPointPlaneWithSensorErrorFactor =
    SE2DistanceMapICPPointPlaneWithSensorErrorFactor_<
        VariableSE2DistanceMapSparseStaticPointNormal2fVectorCloudRight>;
using SE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRightICPPointPlaneWithSensorErrorFactor =
    SE2DistanceMapICPPointPlaneWithSensorErrorFactor_<
        VariableSE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRight>;
using SE2DistanceMapNanoStaticPointNormal2fVectorCloudRightICPPointPlaneWithSensorErrorFactor =
    SE2DistanceMapICPPointPlaneWithSensorErrorFactor_<
        VariableSE2DistanceMapNanoStaticPointNormal2fVectorCloudRight>;

template <typename VariableSE2DistanceMapRightType_>
using SE2DistanceMapICPPointPlaneErrorFactorCorrespondenceFreeDriven_ =
    FactorCorrespondenceFreeDriven_<
        SE2DistanceMapICPPointPlaneErrorFactor_<
            VariableSE2DistanceMapRightType_>,
        inesctec_mrdt_slam_distmap_2d::pcl::PointNormal2VectorCloud_<
            typename VariableSE2DistanceMapRightType_::MeasurementOwnerType::
                DistanceMapType::Scalar>>;

template <typename VariableSE2DistanceMapRightType_>
using SE2DistanceMapICPPointPlaneWithSensorErrorFactorCorrespondenceFreeDriven_ =
    FactorCorrespondenceFreeDriven_<
        SE2DistanceMapICPPointPlaneWithSensorErrorFactor_<
            VariableSE2DistanceMapRightType_>,
        inesctec_mrdt_slam_distmap_2d::pcl::PointNormal2VectorCloud_<
            typename VariableSE2DistanceMapRightType_::MeasurementOwnerType::
                DistanceMapType::Scalar>>;

using SE2DistanceMapStaticPoint2fVectorCloudRightICPPointPlaneErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPlaneErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapStaticPoint2fVectorCloudRight>;
using SE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudRightICPPointPlaneErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPlaneErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudRight>;
using SE2DistanceMapSparseStaticPoint2fVectorCloudRightICPPointPlaneErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPlaneErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapSparseStaticPoint2fVectorCloudRight>;
using SE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudRightICPPointPlaneErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPlaneErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudRight>;
using SE2DistanceMapNanoStaticPoint2fVectorCloudRightICPPointPlaneErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPlaneErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapNanoStaticPoint2fVectorCloudRight>;
using SE2DistanceMapStaticPointNormal2fVectorCloudRightICPPointPlaneErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPlaneErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapStaticPointNormal2fVectorCloudRight>;
using SE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRightICPPointPlaneErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPlaneErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRight>;
using SE2DistanceMapSparseStaticPointNormal2fVectorCloudRightICPPointPlaneErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPlaneErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapSparseStaticPointNormal2fVectorCloudRight>;
using SE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRightICPPointPlaneErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPlaneErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRight>;
using SE2DistanceMapNanoStaticPointNormal2fVectorCloudRightICPPointPlaneErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPlaneErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapNanoStaticPointNormal2fVectorCloudRight>;

using SE2DistanceMapStaticPoint2fVectorCloudRightICPPointPlaneWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPlaneWithSensorErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapStaticPoint2fVectorCloudRight>;
using SE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudRightICPPointPlaneWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPlaneWithSensorErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudRight>;
using SE2DistanceMapSparseStaticPoint2fVectorCloudRightICPPointPlaneWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPlaneWithSensorErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapSparseStaticPoint2fVectorCloudRight>;
using SE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudRightICPPointPlaneWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPlaneWithSensorErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudRight>;
using SE2DistanceMapNanoStaticPoint2fVectorCloudRightICPPointPlaneWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPlaneWithSensorErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapNanoStaticPoint2fVectorCloudRight>;
using SE2DistanceMapStaticPointNormal2fVectorCloudRightICPPointPlaneWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPlaneWithSensorErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapStaticPointNormal2fVectorCloudRight>;
using SE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRightICPPointPlaneWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPlaneWithSensorErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRight>;
using SE2DistanceMapSparseStaticPointNormal2fVectorCloudRightICPPointPlaneWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPlaneWithSensorErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapSparseStaticPointNormal2fVectorCloudRight>;
using SE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRightICPPointPlaneWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPlaneWithSensorErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRight>;
using SE2DistanceMapNanoStaticPointNormal2fVectorCloudRightICPPointPlaneWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPlaneWithSensorErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapNanoStaticPointNormal2fVectorCloudRight>;

}  // namespace srrg2_solver
