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
 * @note this factor is for left-sided perturbation update-based variables!
 *       (the error formulation is different in the manifold, depending on the
 *       perturbation update side!)
 * @tparam VariableSE2DistanceMapLeftType_ distance map-based variable type
 *                                         (must be left-sided
 *                                         pertubation-based!)
 */
template <typename VariableSE2DistanceMapLeftType_>
class SE2DistanceMapICPPointPlaneLeftErrorFactor_
    : public ErrorFactor_<1, VariableSE2DistanceMapLeftType_>
{
 public:

  using BaseFactorType = ErrorFactor_<1, VariableSE2DistanceMapLeftType_>;
  using ThisType = SE2DistanceMapICPPointPlaneLeftErrorFactor_<
      VariableSE2DistanceMapLeftType_>;

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
    return "SE2DistanceMapICPPointPlaneLeftErrorFactor (ICP point-to-plane)";
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

    Matrix2_3f J_aux;

    J_aux.block<2, 2>(0, 0) = Matrix2f::Identity();
    J_aux.block<2, 1>(0, 2) = geometry2d::skew(e_ICP);
    J_aux.block<2, 1>(0, 2) -= geometry2d::skew(p_moving_pred);

    this->_J = n_moving_pred_trans * J_aux;
  }  // void
     // SE2DistanceMapICPPointPlaneLeftErrorFactor_<VariableSE2DistanceMapLeftType_>::errorAndJacobian(bool
     // = false) override

};  // class
    // SE2DistanceMapICPPointPlaneLeftErrorFactor_<VariableSE2DistanceMapLeftType_>
    // : public ErrorFactor_<1, VariableSE2DistanceMapLeftType_>

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
 * @note this factor is for left-sided perturbation update-based variables!
 *       (the error formulation is different in the manifold, depending on the
 *       perturbation update side!)
 * @tparam VariableSE2DistanceMapLeftType_ distance map-based variable type
 *                                         (must be left-sided
 *                                         pertubation-based!)
 */
template <typename VariableSE2DistanceMapLeftType_>
class SE2DistanceMapICPPointPlaneWithSensorLeftErrorFactor_
    : public SE2DistanceMapICPPointPlaneLeftErrorFactor_<
          VariableSE2DistanceMapLeftType_>
{
 public:

  using BaseFactorType = SE2DistanceMapICPPointPlaneLeftErrorFactor_<
      VariableSE2DistanceMapLeftType_>;
  using ThisType = SE2DistanceMapICPPointPlaneWithSensorLeftErrorFactor_<
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

    const Matrix2f& R_robot_in_sensor = this->m_robot_in_sensor_.linear();

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

    Matrix2_3f J_aux;

    J_aux.block<2, 2>(0, 0) = R_robot_in_sensor;
    J_aux.block<2, 1>(0, 2) = geometry2d::skew(e_ICP);
    J_aux.block<2, 1>(0, 2) -=
        R_robot_in_sensor * geometry2d::skew(p_moving_pred_in_robot);

    this->_J = n_moving_pred_trans * J_aux;
  }  // void
     // SE2DistanceMapICPPointPlaneWithSensorLeftErrorFactor_<VariableSE2DistanceMapLeftType_>::errorAndJacobian(bool
     // = false) final
};  // class
    // SE2DistanceMapICPPointPlaneWithSensorLeftErrorFactor_<VariableSE2DistanceMapLeftType_>
    // : public
    // SE2DistanceMapICPPointPlaneLeftErrorFactor_<VariableSE2DistanceMapLeftType_>

using SE2DistanceMapStaticPoint2fVectorCloudLeftICPPointPlaneErrorFactor =
    SE2DistanceMapICPPointPlaneLeftErrorFactor_<
        VariableSE2DistanceMapStaticPoint2fVectorCloudLeft>;
using SE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudLeftICPPointPlaneErrorFactor =
    SE2DistanceMapICPPointPlaneLeftErrorFactor_<
        VariableSE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudLeft>;
using SE2DistanceMapSparseStaticPoint2fVectorCloudLeftICPPointPlaneErrorFactor =
    SE2DistanceMapICPPointPlaneLeftErrorFactor_<
        VariableSE2DistanceMapSparseStaticPoint2fVectorCloudLeft>;
using SE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudLeftICPPointPlaneErrorFactor =
    SE2DistanceMapICPPointPlaneLeftErrorFactor_<
        VariableSE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudLeft>;
using SE2DistanceMapNanoStaticPoint2fVectorCloudLeftICPPointPlaneErrorFactor =
    SE2DistanceMapICPPointPlaneLeftErrorFactor_<
        VariableSE2DistanceMapNanoStaticPoint2fVectorCloudLeft>;
using SE2DistanceMapStaticPointNormal2fVectorCloudLeftICPPointPlaneErrorFactor =
    SE2DistanceMapICPPointPlaneLeftErrorFactor_<
        VariableSE2DistanceMapStaticPointNormal2fVectorCloudLeft>;
using SE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudLeftICPPointPlaneErrorFactor =
    SE2DistanceMapICPPointPlaneLeftErrorFactor_<
        VariableSE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudLeft>;
using SE2DistanceMapSparseStaticPointNormal2fVectorCloudLeftICPPointPlaneErrorFactor =
    SE2DistanceMapICPPointPlaneLeftErrorFactor_<
        VariableSE2DistanceMapSparseStaticPointNormal2fVectorCloudLeft>;
using SE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudLeftICPPointPlaneErrorFactor =
    SE2DistanceMapICPPointPlaneLeftErrorFactor_<
        VariableSE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudLeft>;
using SE2DistanceMapNanoStaticPointNormal2fVectorCloudLeftICPPointPlaneErrorFactor =
    SE2DistanceMapICPPointPlaneLeftErrorFactor_<
        VariableSE2DistanceMapNanoStaticPointNormal2fVectorCloudLeft>;

using SE2DistanceMapStaticPoint2fVectorCloudLeftICPPointPlaneWithSensorErrorFactor =
    SE2DistanceMapICPPointPlaneWithSensorLeftErrorFactor_<
        VariableSE2DistanceMapStaticPoint2fVectorCloudLeft>;
using SE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudLeftICPPointPlaneWithSensorErrorFactor =
    SE2DistanceMapICPPointPlaneWithSensorLeftErrorFactor_<
        VariableSE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudLeft>;
using SE2DistanceMapSparseStaticPoint2fVectorCloudLeftICPPointPlaneWithSensorErrorFactor =
    SE2DistanceMapICPPointPlaneWithSensorLeftErrorFactor_<
        VariableSE2DistanceMapSparseStaticPoint2fVectorCloudLeft>;
using SE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudLeftICPPointPlaneWithSensorErrorFactor =
    SE2DistanceMapICPPointPlaneWithSensorLeftErrorFactor_<
        VariableSE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudLeft>;
using SE2DistanceMapNanoStaticPoint2fVectorCloudLeftICPPointPlaneWithSensorErrorFactor =
    SE2DistanceMapICPPointPlaneWithSensorLeftErrorFactor_<
        VariableSE2DistanceMapNanoStaticPoint2fVectorCloudLeft>;
using SE2DistanceMapStaticPointNormal2fVectorCloudLeftICPPointPlaneWithSensorErrorFactor =
    SE2DistanceMapICPPointPlaneWithSensorLeftErrorFactor_<
        VariableSE2DistanceMapStaticPointNormal2fVectorCloudLeft>;
using SE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudLeftICPPointPlaneWithSensorErrorFactor =
    SE2DistanceMapICPPointPlaneWithSensorLeftErrorFactor_<
        VariableSE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudLeft>;
using SE2DistanceMapSparseStaticPointNormal2fVectorCloudLeftICPPointPlaneWithSensorErrorFactor =
    SE2DistanceMapICPPointPlaneWithSensorLeftErrorFactor_<
        VariableSE2DistanceMapSparseStaticPointNormal2fVectorCloudLeft>;
using SE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudLeftICPPointPlaneWithSensorErrorFactor =
    SE2DistanceMapICPPointPlaneWithSensorLeftErrorFactor_<
        VariableSE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudLeft>;
using SE2DistanceMapNanoStaticPointNormal2fVectorCloudLeftICPPointPlaneWithSensorErrorFactor =
    SE2DistanceMapICPPointPlaneWithSensorLeftErrorFactor_<
        VariableSE2DistanceMapNanoStaticPointNormal2fVectorCloudLeft>;

template <typename VariableSE2DistanceMapLeftType_>
using SE2DistanceMapICPPointPlaneLeftErrorFactorCorrespondenceFreeDriven_ =
    FactorCorrespondenceFreeDriven_<
        SE2DistanceMapICPPointPlaneLeftErrorFactor_<
            VariableSE2DistanceMapLeftType_>,
        inesctec_mrdt_slam_distmap_2d::pcl::PointNormal2VectorCloud_<
            typename VariableSE2DistanceMapLeftType_::MeasurementOwnerType::
                DistanceMapType::Scalar>>;

template <typename VariableSE2DistanceMapLeftType_>
using SE2DistanceMapICPPointPlaneWithSensorLeftErrorFactorCorrespondenceFreeDriven_ =
    FactorCorrespondenceFreeDriven_<
        SE2DistanceMapICPPointPlaneWithSensorLeftErrorFactor_<
            VariableSE2DistanceMapLeftType_>,
        inesctec_mrdt_slam_distmap_2d::pcl::PointNormal2VectorCloud_<
            typename VariableSE2DistanceMapLeftType_::MeasurementOwnerType::
                DistanceMapType::Scalar>>;

using SE2DistanceMapStaticPoint2fVectorCloudLeftICPPointPlaneErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPlaneLeftErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapStaticPoint2fVectorCloudLeft>;
using SE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudLeftICPPointPlaneErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPlaneLeftErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudLeft>;
using SE2DistanceMapSparseStaticPoint2fVectorCloudLeftICPPointPlaneErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPlaneLeftErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapSparseStaticPoint2fVectorCloudLeft>;
using SE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudLeftICPPointPlaneErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPlaneLeftErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudLeft>;
using SE2DistanceMapNanoStaticPoint2fVectorCloudLeftICPPointPlaneErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPlaneLeftErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapNanoStaticPoint2fVectorCloudLeft>;
using SE2DistanceMapStaticPointNormal2fVectorCloudLeftICPPointPlaneErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPlaneLeftErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapStaticPointNormal2fVectorCloudLeft>;
using SE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudLeftICPPointPlaneErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPlaneLeftErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudLeft>;
using SE2DistanceMapSparseStaticPointNormal2fVectorCloudLeftICPPointPlaneErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPlaneLeftErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapSparseStaticPointNormal2fVectorCloudLeft>;
using SE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudLeftICPPointPlaneErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPlaneLeftErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudLeft>;
using SE2DistanceMapNanoStaticPointNormal2fVectorCloudLeftICPPointPlaneErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPlaneLeftErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapNanoStaticPointNormal2fVectorCloudLeft>;

using SE2DistanceMapStaticPoint2fVectorCloudLeftICPPointPlaneWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPlaneWithSensorLeftErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapStaticPoint2fVectorCloudLeft>;
using SE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudLeftICPPointPlaneWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPlaneWithSensorLeftErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudLeft>;
using SE2DistanceMapSparseStaticPoint2fVectorCloudLeftICPPointPlaneWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPlaneWithSensorLeftErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapSparseStaticPoint2fVectorCloudLeft>;
using SE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudLeftICPPointPlaneWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPlaneWithSensorLeftErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudLeft>;
using SE2DistanceMapNanoStaticPoint2fVectorCloudLeftICPPointPlaneWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPlaneWithSensorLeftErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapNanoStaticPoint2fVectorCloudLeft>;
using SE2DistanceMapStaticPointNormal2fVectorCloudLeftICPPointPlaneWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPlaneWithSensorLeftErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapStaticPointNormal2fVectorCloudLeft>;
using SE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudLeftICPPointPlaneWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPlaneWithSensorLeftErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudLeft>;
using SE2DistanceMapSparseStaticPointNormal2fVectorCloudLeftICPPointPlaneWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPlaneWithSensorLeftErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapSparseStaticPointNormal2fVectorCloudLeft>;
using SE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudLeftICPPointPlaneWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPlaneWithSensorLeftErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudLeft>;
using SE2DistanceMapNanoStaticPointNormal2fVectorCloudLeftICPPointPlaneWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPlaneWithSensorLeftErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapNanoStaticPointNormal2fVectorCloudLeft>;

}  // namespace srrg2_solver
