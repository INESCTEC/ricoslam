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
 * @tparam VariableSE2DistanceMapRightType_ distance map-based variable type
 *                                          (must be right-sided
 *                                          pertubation-based!)
 */
template <typename VariableSE2DistanceMapRightType_>
class SE2DistanceMapICPPointPointErrorFactor_
    : public ErrorFactor_<2, VariableSE2DistanceMapRightType_>
{
 public:

  using BaseFactorType = ErrorFactor_<2, VariableSE2DistanceMapRightType_>;
  using ThisType =
      SE2DistanceMapICPPointPointErrorFactor_<VariableSE2DistanceMapRightType_>;

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
    return "SE2DistanceMapICPPointPointErrorFactor (ICP point-to-point)";
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

    this->_J.template block<2, 2>(0, 0) = R;
    this->_J.template block<2, 1>(0, 2) =
        -R * geometry2d::skew(this->m_moving_->coordinates());
  }  // void
     // SE2DistanceMapICPPointPointErrorFactor_<VariableSE2DistanceMapRightType_>::errorAndJacobian(bool
     // = false) override

};  // class
    // SE2DistanceMapICPPointPointErrorFactor_<VariableSE2DistanceMapRightType_>
    // : public ErrorFactor_<2, VariableSE2DistanceMapRightType_>

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
 * @tparam VariableSE2DistanceMapRightType_ distance map-based variable type
 *                                          (must be right-sided
 *                                          pertubation-based!)
 */
template <typename VariableSE2DistanceMapRightType_>
class SE2DistanceMapICPPointPointWithSensorErrorFactor_
    : public SE2DistanceMapICPPointPointErrorFactor_<
          VariableSE2DistanceMapRightType_>
{
 public:

  using BaseFactorType =
      SE2DistanceMapICPPointPointErrorFactor_<VariableSE2DistanceMapRightType_>;
  using ThisType = SE2DistanceMapICPPointPointWithSensorErrorFactor_<
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

    this->_J.template block<2, 2>(0, 0) = R_robot_in_sensor_R;
    this->_J.template block<2, 1>(0, 2) =
        -R_robot_in_sensor_R * geometry2d::skew(moving_in_robot);
  }  // void
     // SE2DistanceMapICPPointPointWithSensorErrorFactor_<VariableSE2DistanceMapRightType_>::errorAndJacobian(bool
     // = false) final

};  // class
    // SE2DistanceMapICPPointPointWithSensorErrorFactor_<VariableSE2DistanceMapRightType_>
    // : public
    // SE2DistanceMapICPPointPointErrorFactor_<VariableSE2DistanceMapRightType_>

using SE2DistanceMapStaticPoint2fVectorCloudRightICPPointPointErrorFactor =
    SE2DistanceMapICPPointPointErrorFactor_<
        VariableSE2DistanceMapStaticPoint2fVectorCloudRight>;
using SE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudRightICPPointPointErrorFactor =
    SE2DistanceMapICPPointPointErrorFactor_<
        VariableSE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudRight>;
using SE2DistanceMapSparseStaticPoint2fVectorCloudRightICPPointPointErrorFactor =
    SE2DistanceMapICPPointPointErrorFactor_<
        VariableSE2DistanceMapSparseStaticPoint2fVectorCloudRight>;
using SE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudRightICPPointPointErrorFactor =
    SE2DistanceMapICPPointPointErrorFactor_<
        VariableSE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudRight>;
using SE2DistanceMapNanoStaticPoint2fVectorCloudRightICPPointPointErrorFactor =
    SE2DistanceMapICPPointPointErrorFactor_<
        VariableSE2DistanceMapNanoStaticPoint2fVectorCloudRight>;
using SE2DistanceMapStaticPointNormal2fVectorCloudRightICPPointPointErrorFactor =
    SE2DistanceMapICPPointPointErrorFactor_<
        VariableSE2DistanceMapStaticPointNormal2fVectorCloudRight>;
using SE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRightICPPointPointErrorFactor =
    SE2DistanceMapICPPointPointErrorFactor_<
        VariableSE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRight>;
using SE2DistanceMapSparseStaticPointNormal2fVectorCloudRightICPPointPointErrorFactor =
    SE2DistanceMapICPPointPointErrorFactor_<
        VariableSE2DistanceMapSparseStaticPointNormal2fVectorCloudRight>;
using SE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRightICPPointPointErrorFactor =
    SE2DistanceMapICPPointPointErrorFactor_<
        VariableSE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRight>;
using SE2DistanceMapNanoStaticPointNormal2fVectorCloudRightICPPointPointErrorFactor =
    SE2DistanceMapICPPointPointErrorFactor_<
        VariableSE2DistanceMapNanoStaticPointNormal2fVectorCloudRight>;

using SE2DistanceMapStaticPoint2fVectorCloudRightICPPointPointWithSensorErrorFactor =
    SE2DistanceMapICPPointPointWithSensorErrorFactor_<
        VariableSE2DistanceMapStaticPoint2fVectorCloudRight>;
using SE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudRightICPPointPointWithSensorErrorFactor =
    SE2DistanceMapICPPointPointWithSensorErrorFactor_<
        VariableSE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudRight>;
using SE2DistanceMapSparseStaticPoint2fVectorCloudRightICPPointPointWithSensorErrorFactor =
    SE2DistanceMapICPPointPointWithSensorErrorFactor_<
        VariableSE2DistanceMapSparseStaticPoint2fVectorCloudRight>;
using SE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudRightICPPointPointWithSensorErrorFactor =
    SE2DistanceMapICPPointPointWithSensorErrorFactor_<
        VariableSE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudRight>;
using SE2DistanceMapNanoStaticPoint2fVectorCloudRightICPPointPointWithSensorErrorFactor =
    SE2DistanceMapICPPointPointWithSensorErrorFactor_<
        VariableSE2DistanceMapNanoStaticPoint2fVectorCloudRight>;
using SE2DistanceMapStaticPointNormal2fVectorCloudRightICPPointPointWithSensorErrorFactor =
    SE2DistanceMapICPPointPointWithSensorErrorFactor_<
        VariableSE2DistanceMapStaticPointNormal2fVectorCloudRight>;
using SE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRightICPPointPointWithSensorErrorFactor =
    SE2DistanceMapICPPointPointWithSensorErrorFactor_<
        VariableSE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRight>;
using SE2DistanceMapSparseStaticPointNormal2fVectorCloudRightICPPointPointWithSensorErrorFactor =
    SE2DistanceMapICPPointPointWithSensorErrorFactor_<
        VariableSE2DistanceMapSparseStaticPointNormal2fVectorCloudRight>;
using SE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRightICPPointPointWithSensorErrorFactor =
    SE2DistanceMapICPPointPointWithSensorErrorFactor_<
        VariableSE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRight>;
using SE2DistanceMapNanoStaticPointNormal2fVectorCloudRightICPPointPointWithSensorErrorFactor =
    SE2DistanceMapICPPointPointWithSensorErrorFactor_<
        VariableSE2DistanceMapNanoStaticPointNormal2fVectorCloudRight>;

template <typename VariableSE2DistanceMapRightType_>
using SE2DistanceMapICPPointPointErrorFactorCorrespondenceFreeDriven_ =
    FactorCorrespondenceFreeDriven_<SE2DistanceMapICPPointPointErrorFactor_<
                                        VariableSE2DistanceMapRightType_>,
                                    typename VariableSE2DistanceMapRightType_::
                                        MeasurementOwnerType::PointCloudType>;

template <typename VariableSE2DistanceMapRightType_>
using SE2DistanceMapICPPointPointWithSensorErrorFactorCorrespondenceFreeDriven_ =
    FactorCorrespondenceFreeDriven_<
        SE2DistanceMapICPPointPointWithSensorErrorFactor_<
            VariableSE2DistanceMapRightType_>,
        typename VariableSE2DistanceMapRightType_::MeasurementOwnerType::
            PointCloudType>;

using SE2DistanceMapStaticPoint2fVectorCloudRightICPPointPointErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPointErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapStaticPoint2fVectorCloudRight>;
using SE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudRightICPPointPointErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPointErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudRight>;
using SE2DistanceMapSparseStaticPoint2fVectorCloudRightICPPointPointErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPointErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapSparseStaticPoint2fVectorCloudRight>;
using SE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudRightICPPointPointErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPointErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudRight>;
using SE2DistanceMapNanoStaticPoint2fVectorCloudRightICPPointPointErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPointErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapNanoStaticPoint2fVectorCloudRight>;
using SE2DistanceMapStaticPointNormal2fVectorCloudRightICPPointPointErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPointErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapStaticPointNormal2fVectorCloudRight>;
using SE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRightICPPointPointErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPointErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRight>;
using SE2DistanceMapSparseStaticPointNormal2fVectorCloudRightICPPointPointErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPointErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapSparseStaticPointNormal2fVectorCloudRight>;
using SE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRightICPPointPointErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPointErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRight>;
using SE2DistanceMapNanoStaticPointNormal2fVectorCloudRightICPPointPointErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPointErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapNanoStaticPointNormal2fVectorCloudRight>;

using SE2DistanceMapStaticPoint2fVectorCloudRightICPPointPointWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPointWithSensorErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapStaticPoint2fVectorCloudRight>;
using SE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudRightICPPointPointWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPointWithSensorErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudRight>;
using SE2DistanceMapSparseStaticPoint2fVectorCloudRightICPPointPointWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPointWithSensorErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapSparseStaticPoint2fVectorCloudRight>;
using SE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudRightICPPointPointWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPointWithSensorErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudRight>;
using SE2DistanceMapNanoStaticPoint2fVectorCloudRightICPPointPointWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPointWithSensorErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapNanoStaticPoint2fVectorCloudRight>;
using SE2DistanceMapStaticPointNormal2fVectorCloudRightICPPointPointWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPointWithSensorErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapStaticPointNormal2fVectorCloudRight>;
using SE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRightICPPointPointWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPointWithSensorErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRight>;
using SE2DistanceMapSparseStaticPointNormal2fVectorCloudRightICPPointPointWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPointWithSensorErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapSparseStaticPointNormal2fVectorCloudRight>;
using SE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRightICPPointPointWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPointWithSensorErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRight>;
using SE2DistanceMapNanoStaticPointNormal2fVectorCloudRightICPPointPointWithSensorErrorFactorCorrespondenceFreeDriven =
    SE2DistanceMapICPPointPointWithSensorErrorFactorCorrespondenceFreeDriven_<
        VariableSE2DistanceMapNanoStaticPointNormal2fVectorCloudRight>;

}  // namespace srrg2_solver
