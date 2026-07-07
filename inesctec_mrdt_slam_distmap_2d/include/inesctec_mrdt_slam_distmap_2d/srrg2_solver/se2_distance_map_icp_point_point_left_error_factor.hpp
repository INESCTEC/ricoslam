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
 * @brief SE(2) Iterative Closest Point (ICP) point to point error factor
 *        using distance map strucutres for Nearest Neighbor (NN) search
 *        (single point! >>> you need the correspondence free driven for point
 *        clouds)
 *
 * the main goal of this factor is to not required any correspondence update
 * during the matching process. the correspondence is indirectly, through the
 * projection of the moving point into the fixed reference frame, where the NN
 * is found from the distance map computation.
 *
 * @note this factor is for left-sided perturbation update-based variables!
 *       (the error formulation is different in the manifold, depending on the
 *       perturbation update side!)
 * @tparam VariableSE2DistanceMapLeftType_ distance map-based variable type
 *                                         (must be left-sided
 *                                         pertubation-based!)
 */
template <typename VariableSE2DistanceMapLeftType_>
class SE2DistanceMapICPPointPointLeftErrorFactor_
    : public ErrorFactor_<2, VariableSE2DistanceMapLeftType_>
{
 public:

  using BaseFactorType = ErrorFactor_<2, VariableSE2DistanceMapLeftType_>;
  using ThisType = SE2DistanceMapICPPointPointLeftErrorFactor_<
      VariableSE2DistanceMapLeftType_>;

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
    return "SE2DistanceMapICPPointPointLeftErrorFactor (ICP point-to-point)";
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

    typename VariableType::MeasurementOwnerType::DistanceMapType::CellNNIdxType
        nn;

    if (!X_var->measurement()->getDistanceMap()->getNN(nn, moving_pred))
    {
      this->_is_valid = false;
      return;
    }

    this->_e = moving_pred -
               X_var->measurement()->getPointCloud()->at(nn).coordinates();
    this->m_correspondence_idx_fixed_ = nn;

    if (error_only)
    {
      return;
    }

    this->_J.template block<2, 2>(0, 0) = Matrix2f::Identity();
    this->_J.template block<2, 1>(0, 2) = -geometry2d::skew(moving_pred);
  }  // void
     // SE2DistanceMapICPPointPointLeftErrorFactor_<VariableSE2DistanceMapLeftType_>::errorAndJacobian(bool
     // = false) override

};  // class
    // SE2DistanceMapICPPointPointLeftErrorFactor_<VariableSE2DistanceMapLeftType_>
    // : public ErrorFactor_<2, VariableSE2DistanceMapLeftType_>

/**
 * @brief SE(2) Iterative Closest Point (ICP) point to point error factor with
 *        sensor using distance map strucutres for Nearest Neighbor (NN) search
 *        (single point! >>> you need the correspondence free driven for point
 *        clouds)
 *
 * the main goal of this factor is to not required any correspondence update
 * during the matching process. the correspondence is indirectly, through the
 * projection of the moving point into the fixed reference frame, where the NN
 * is found from the distance map computation.
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
class SE2DistanceMapICPPointPointWithSensorLeftErrorFactor_
    : public SE2DistanceMapICPPointPointLeftErrorFactor_<
          VariableSE2DistanceMapLeftType_>
{
 public:

  using BaseFactorType = SE2DistanceMapICPPointPointLeftErrorFactor_<
      VariableSE2DistanceMapLeftType_>;
  using ThisType = SE2DistanceMapICPPointPointWithSensorLeftErrorFactor_<
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

    typename VariableType::MeasurementOwnerType::DistanceMapType::CellNNIdxType
        nn;

    if (!X_var->measurement()->getDistanceMap()->getNN(nn,
                                                       moving_pred_in_sensor))
    {
      this->_is_valid = false;
      return;
    }

    this->_e = moving_pred_in_sensor -
               X_var->measurement()->getPointCloud()->at(nn).coordinates();
    this->m_correspondence_idx_fixed_ = nn;

    if (error_only)
    {
      return;
    }

    this->_J.template block<2, 2>(0, 0) = R_robot_in_sensor;
    this->_J.template block<2, 1>(0, 2) =
        -R_robot_in_sensor * geometry2d::skew(moving_pred_in_robot);
  }  // void
     // SE2DistanceMapICPPointPointWithSensorLeftErrorFactor_<VariableSE2DistanceMapLeftType_>::errorAndJacobian(bool
     // = false) final

};  // class
    // SE2DistanceMapICPPointPointWithSensorLeftErrorFactor_<VariableSE2DistanceMapLeftType_>
    // : public
    // SE2DistanceMapICPPointPointLeftErrorFactor_<VariableSE2DistanceMapLeftType_>

using SE2DistanceMapStaticPoint2fVectorCloudLeftICPPointPointErrorFactor =
    SE2DistanceMapICPPointPointLeftErrorFactor_<
        VariableSE2DistanceMapStaticPoint2fVectorCloudLeft>;
using SE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudLeftICPPointPointErrorFactor =
    SE2DistanceMapICPPointPointLeftErrorFactor_<
        VariableSE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudLeft>;
using SE2DistanceMapSparseStaticPoint2fVectorCloudLeftICPPointPointErrorFactor =
    SE2DistanceMapICPPointPointLeftErrorFactor_<
        VariableSE2DistanceMapSparseStaticPoint2fVectorCloudLeft>;
using SE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudLeftICPPointPointErrorFactor =
    SE2DistanceMapICPPointPointLeftErrorFactor_<
        VariableSE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudLeft>;
using SE2DistanceMapNanoStaticPoint2fVectorCloudLeftICPPointPointErrorFactor =
    SE2DistanceMapICPPointPointLeftErrorFactor_<
        VariableSE2DistanceMapNanoStaticPoint2fVectorCloudLeft>;
using SE2DistanceMapStaticPointNormal2fVectorCloudLeftICPPointPointErrorFactor =
    SE2DistanceMapICPPointPointLeftErrorFactor_<
        VariableSE2DistanceMapStaticPointNormal2fVectorCloudLeft>;
using SE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudLeftICPPointPointErrorFactor =
    SE2DistanceMapICPPointPointLeftErrorFactor_<
        VariableSE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudLeft>;
using SE2DistanceMapSparseStaticPointNormal2fVectorCloudLeftICPPointPointErrorFactor =
    SE2DistanceMapICPPointPointLeftErrorFactor_<
        VariableSE2DistanceMapSparseStaticPointNormal2fVectorCloudLeft>;
using SE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudLeftICPPointPointErrorFactor =
    SE2DistanceMapICPPointPointLeftErrorFactor_<
        VariableSE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudLeft>;
using SE2DistanceMapNanoStaticPointNormal2fVectorCloudLeftICPPointPointErrorFactor =
    SE2DistanceMapICPPointPointLeftErrorFactor_<
        VariableSE2DistanceMapNanoStaticPointNormal2fVectorCloudLeft>;

using SE2DistanceMapStaticPoint2fVectorCloudLeftICPPointPointWithSensorErrorFactor =
    SE2DistanceMapICPPointPointWithSensorLeftErrorFactor_<
        VariableSE2DistanceMapStaticPoint2fVectorCloudLeft>;
using SE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudLeftICPPointPointWithSensorErrorFactor =
    SE2DistanceMapICPPointPointWithSensorLeftErrorFactor_<
        VariableSE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudLeft>;
using SE2DistanceMapSparseStaticPoint2fVectorCloudLeftICPPointPointWithSensorErrorFactor =
    SE2DistanceMapICPPointPointWithSensorLeftErrorFactor_<
        VariableSE2DistanceMapSparseStaticPoint2fVectorCloudLeft>;
using SE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudLeftICPPointPointWithSensorErrorFactor =
    SE2DistanceMapICPPointPointWithSensorLeftErrorFactor_<
        VariableSE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudLeft>;
using SE2DistanceMapNanoStaticPoint2fVectorCloudLeftICPPointPointWithSensorErrorFactor =
    SE2DistanceMapICPPointPointWithSensorLeftErrorFactor_<
        VariableSE2DistanceMapNanoStaticPoint2fVectorCloudLeft>;
using SE2DistanceMapStaticPointNormal2fVectorCloudLeftICPPointPointWithSensorErrorFactor =
    SE2DistanceMapICPPointPointWithSensorLeftErrorFactor_<
        VariableSE2DistanceMapStaticPointNormal2fVectorCloudLeft>;
using SE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudLeftICPPointPointWithSensorErrorFactor =
    SE2DistanceMapICPPointPointWithSensorLeftErrorFactor_<
        VariableSE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudLeft>;
using SE2DistanceMapSparseStaticPointNormal2fVectorCloudLeftICPPointPointWithSensorErrorFactor =
    SE2DistanceMapICPPointPointWithSensorLeftErrorFactor_<
        VariableSE2DistanceMapSparseStaticPointNormal2fVectorCloudLeft>;
using SE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudLeftICPPointPointWithSensorErrorFactor =
    SE2DistanceMapICPPointPointWithSensorLeftErrorFactor_<
        VariableSE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudLeft>;
using SE2DistanceMapNanoStaticPointNormal2fVectorCloudLeftICPPointPointWithSensorErrorFactor =
    SE2DistanceMapICPPointPointWithSensorLeftErrorFactor_<
        VariableSE2DistanceMapNanoStaticPointNormal2fVectorCloudLeft>;

template <typename VariableSE2DistanceMapLeftType_>
using SE2DistanceMapICPPointPointLeftErrorFactorCorrespondenceFreeDriven_ =
    FactorCorrespondenceFreeDriven_<SE2DistanceMapICPPointPointLeftErrorFactor_<
                                        VariableSE2DistanceMapLeftType_>,
                                    typename VariableSE2DistanceMapLeftType_::
                                        MeasurementOwnerType::PointCloudType>;

template <typename VariableSE2DistanceMapLeftType_>
using SE2DistanceMapICPPointPointWithSensorLeftErrorFactorCorrespondenceFreeDriven_ =
    FactorCorrespondenceFreeDriven_<
        SE2DistanceMapICPPointPointWithSensorLeftErrorFactor_<
            VariableSE2DistanceMapLeftType_>,
        typename VariableSE2DistanceMapLeftType_::MeasurementOwnerType::
            PointCloudType>;

using SE2DistanceMapStaticPoint2fVectorCloudLeftICPPointPointErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPointLeftErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapStaticPoint2fVectorCloudLeft>;
using SE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudLeftICPPointPointErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPointLeftErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudLeft>;
using SE2DistanceMapSparseStaticPoint2fVectorCloudLeftICPPointPointErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPointLeftErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapSparseStaticPoint2fVectorCloudLeft>;
using SE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudLeftICPPointPointErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPointLeftErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudLeft>;
using SE2DistanceMapNanoStaticPoint2fVectorCloudLeftICPPointPointErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPointLeftErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapNanoStaticPoint2fVectorCloudLeft>;
using SE2DistanceMapStaticPointNormal2fVectorCloudLeftICPPointPointErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPointLeftErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapStaticPointNormal2fVectorCloudLeft>;
using SE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudLeftICPPointPointErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPointLeftErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudLeft>;
using SE2DistanceMapSparseStaticPointNormal2fVectorCloudLeftICPPointPointErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPointLeftErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapSparseStaticPointNormal2fVectorCloudLeft>;
using SE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudLeftICPPointPointErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPointLeftErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudLeft>;
using SE2DistanceMapNanoStaticPointNormal2fVectorCloudLeftICPPointPointErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPointLeftErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapNanoStaticPointNormal2fVectorCloudLeft>;

using SE2DistanceMapStaticPoint2fVectorCloudLeftICPPointPointWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPointWithSensorLeftErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapStaticPoint2fVectorCloudLeft>;
using SE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudLeftICPPointPointWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPointWithSensorLeftErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudLeft>;
using SE2DistanceMapSparseStaticPoint2fVectorCloudLeftICPPointPointWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPointWithSensorLeftErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapSparseStaticPoint2fVectorCloudLeft>;
using SE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudLeftICPPointPointWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPointWithSensorLeftErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudLeft>;
using SE2DistanceMapNanoStaticPoint2fVectorCloudLeftICPPointPointWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPointWithSensorLeftErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapNanoStaticPoint2fVectorCloudLeft>;
using SE2DistanceMapStaticPointNormal2fVectorCloudLeftICPPointPointWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPointWithSensorLeftErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapStaticPointNormal2fVectorCloudLeft>;
using SE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudLeftICPPointPointWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPointWithSensorLeftErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudLeft>;
using SE2DistanceMapSparseStaticPointNormal2fVectorCloudLeftICPPointPointWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPointWithSensorLeftErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapSparseStaticPointNormal2fVectorCloudLeft>;
using SE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudLeftICPPointPointWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPointWithSensorLeftErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudLeft>;
using SE2DistanceMapNanoStaticPointNormal2fVectorCloudLeftICPPointPointWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPointWithSensorLeftErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapNanoStaticPointNormal2fVectorCloudLeft>;

}  // namespace srrg2_solver
