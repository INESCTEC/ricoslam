#pragma once

#include <exception>

// Eigen
#include <Eigen/Core>

// SRRG Software
#include <srrg2_core/srrg_geometry/geometry3d.h>
#include <srrg2_core/srrg_pcl/point.h>
#include <srrg2_core/srrg_viewer/viewer_canvas.h>
#include <srrg2_solver/variables_and_factors/types_2d/variable_se2.h>

#include "inesctec_mrdt_slam_distmap_2d/pcl/point_types.hpp"
#include "inesctec_mrdt_slam_distmap_2d/srrg2_solver/measurement_owner_distance_map_point_cloud.hpp"

namespace srrg2_solver
{

/**
 * @brief SE(2) distance map-based variable (SRRG variable that has a
 *        measurement owner)
 * @tparam MeasurementOwnerType_ measurement owner type
 * @tparam PerturbationSide_ right / left perturbation update
 */
template <typename MeasurementOwnerType_,
          VariableSE2Base::PerturbationSide PerturbationSide_ =
              VariableSE2Base::PerturbationSide::Right>
class VariableSE2DistanceMap_ : public VariableSE2_<PerturbationSide_>,
                                public MeasurementOwnerType_
{
 public:

  using BaseVariableType = VariableSE2_<PerturbationSide_>;
  using ThisType =
      VariableSE2DistanceMap_<MeasurementOwnerType_, PerturbationSide_>;

  using VariableType =
      VariableSE2DistanceMap_<MeasurementOwnerType_, PerturbationSide_>;
  using EstimateType = typename BaseVariableType::EstimateType;

  using MeasurementOwnerType = MeasurementOwnerType_;
  using MeasurementType = typename MeasurementOwnerType::MeasurementType;

  using PointType = typename MeasurementOwnerType::PointCloudPointType;
  using ComponentsType = typename PointType::ComponentsType;

 protected:

  EstimateType m_sensor_in_robot_ =
      EstimateType::Identity();  //!< calibrated SE(2) pose of the sensor w.r.t.
                                 //!< the robot coordinate frame

 public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  inline void setSensorInRobot(const EstimateType& sensor_in_robot)
  {
    this->m_sensor_in_robot_ = sensor_in_robot;
  }

  inline const EstimateType& getSensorInRobot() const
  {
    return m_sensor_in_robot_;
  }

  /**
   * @brief draw the variable and the respective measurement (considering the
   *        SE(2) pose of the sensor w.r.t. the robot coordinate frame) in an
   *        OpenGL-based canvas implemented in SRRG software
   * @note if the point cloud is empty or measurement not set or map not set,
   *       no output is put in the canvas
   * @param[in,out] canvas_ OpenGL canvas
   */
  void _drawImpl(srrg2_core::ViewerCanvasPtr canvas_) const override
  {
    if (!canvas_)
    {
      throw std::runtime_error(
          "VariableSE2DistanceMap_::_drawImpl | invalid canvas");
    }

    canvas_->pushColor();
    canvas_->setColor(srrg2_core::ColorPalette::color4fDarkRed(0.5));
    canvas_->pushMatrix();
    canvas_->multMatrix(
        srrg2_core::geometry3d::get3dFrom2dPose(this->_estimate).matrix());
    canvas_->putSphere(0.05);
    canvas_->popMatrix();
    canvas_->popAttribute();

    if (!this->measurement())
    {
      return;
    }
    if (!this->measurement()->getPointCloudPtr())
    {
      return;
    }
    if (this->measurement()->getPointCloud()->empty())
    {
      return;
    }

    canvas_->pushColor();
    canvas_->setColor(srrg2_core::ColorPalette::color3fBlack());
    canvas_->pushMatrix();
    canvas_->multMatrix(
        srrg2_core::geometry3d::get3dFrom2dPose(this->_estimate).matrix());
    canvas_->pushMatrix();
    canvas_->multMatrix(
        srrg2_core::geometry3d::get3dFrom2dPose(m_sensor_in_robot_).matrix());

    _drawPtsIntlImpl(canvas_);

    canvas_->popMatrix();
    canvas_->popMatrix();
    canvas_->popAttribute();
  }  // void VariableSE2DistanceMap_::_drawImpl(srrg2_core::ViewerCanvasPtr)
     // const

 protected:

  template <typename T = PointType,
            std::enable_if_t<
                std::is_same<T, inesctec_mrdt_slam_distmap_2d::pcl::
                                    PointNormal2_<ComponentsType>>::value,
                bool> = 0>
  void _drawPtsIntlImpl(const srrg2_core::ViewerCanvasPtr& canvas_) const
  {
    srrg2_core::PointNormal3fVectorCloud pts;

    pts.resize(this->measurement()->getPointCloud()->size());

    int idx = 0;

    for (const auto& pt : *this->measurement()->getPointCloud())
    {
      if (pt.m_status !=
          inesctec_mrdt_slam_distmap_2d::pcl::PointStatus::kValid)
      {
        continue;
      }

      pts[idx].coordinates().x() = pt.coordinates().x();
      pts[idx].coordinates().y() = pt.coordinates().y();
      pts[idx].coordinates().z() = 0;

      pts[idx].normal().x() = pt.normal().x();
      pts[idx].normal().y() = pt.normal().y();
      pts[idx].normal().z() = 0;

      ++idx;
    }

    pts.resize(idx);

    canvas_->putPoints(pts);
  }

  template <typename T = PointType,
            std::enable_if_t<
                !std::is_same<T, inesctec_mrdt_slam_distmap_2d::pcl::
                                     PointNormal2_<ComponentsType>>::value,
                bool> = 0>
  void _drawPtsIntlImpl(const srrg2_core::ViewerCanvasPtr& canvas_) const
  {
    srrg2_core::Point3fVectorCloud pts;

    pts.resize(this->measurement()->getPointCloud()->size());

    int idx = 0;

    for (const auto& pt : *this->measurement()->getPointCloud())
    {
      if (pt.m_status !=
          inesctec_mrdt_slam_distmap_2d::pcl::PointStatus::kValid)
      {
        continue;
      }

      pts[idx].coordinates().x() = pt.coordinates().x();
      pts[idx].coordinates().y() = pt.coordinates().y();
      pts[idx].coordinates().z() = 0;

      ++idx;
    }

    pts.resize(idx);

    canvas_->putPoints(pts);
  }

};  // class VariableSE2DistanceMap_<MeasurementOwnerType_, PerturbationSide_> :
    // public VariableSE2_<PerturbationSide_>, public MeasurementOwnerType_

template <typename DistanceMapType_>
using VariableSE2DistanceMapPoint2VectorCloudRight_ = VariableSE2DistanceMap_<
    MeasurementOwnerDistanceMapPointCloudLaserMsgs_<
        std::shared_ptr<DistanceMapType_>,
        std::shared_ptr<inesctec_mrdt_slam_distmap_2d::pcl::Point2VectorCloud_<
            typename DistanceMapType_::Scalar>>,
        VariableSE2Base::EstimateType>,
    VariableSE2Base::PerturbationSide::Right>;

template <typename DistanceMapType_>
using VariableSE2DistanceMapPointNormal2VectorCloudRight_ =
    VariableSE2DistanceMap_<
        MeasurementOwnerDistanceMapPointCloudLaserMsgs_<
            std::shared_ptr<DistanceMapType_>,
            std::shared_ptr<
                inesctec_mrdt_slam_distmap_2d::pcl::PointNormal2VectorCloud_<
                    typename DistanceMapType_::Scalar>>,
            VariableSE2Base::EstimateType>,
        VariableSE2Base::PerturbationSide::Right>;

template <typename DistanceMapType_>
using VariableSE2DistanceMapPoint2VectorCloudLeft_ = VariableSE2DistanceMap_<
    MeasurementOwnerDistanceMapPointCloudLaserMsgs_<
        std::shared_ptr<DistanceMapType_>,
        std::shared_ptr<inesctec_mrdt_slam_distmap_2d::pcl::Point2VectorCloud_<
            typename DistanceMapType_::Scalar>>,
        VariableSE2Base::EstimateType>,
    VariableSE2Base::PerturbationSide::Left>;

template <typename DistanceMapType_>
using VariableSE2DistanceMapPointNormal2VectorCloudLeft_ =
    VariableSE2DistanceMap_<
        MeasurementOwnerDistanceMapPointCloudLaserMsgs_<
            std::shared_ptr<DistanceMapType_>,
            std::shared_ptr<
                inesctec_mrdt_slam_distmap_2d::pcl::PointNormal2VectorCloud_<
                    typename DistanceMapType_::Scalar>>,
            VariableSE2Base::EstimateType>,
        VariableSE2Base::PerturbationSide::Left>;

using VariableSE2DistanceMapStaticPoint2fVectorCloudRight =
    VariableSE2DistanceMapPoint2VectorCloudRight_<
        inesctec_mrdt_slam_distmap_2d::data_structures::DistanceMapStaticFloat>;
using VariableSE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudRight =
    VariableSE2DistanceMapPoint2VectorCloudRight_<
        inesctec_mrdt_slam_distmap_2d::data_structures::
            DistanceMapStaticWith2ndDerivFloat>;
using VariableSE2DistanceMapSparseStaticPoint2fVectorCloudRight =
    VariableSE2DistanceMapPoint2VectorCloudRight_<
        inesctec_mrdt_slam_distmap_2d::data_structures::
            DistanceMapSparseStaticFloat>;
using VariableSE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudRight =
    VariableSE2DistanceMapPoint2VectorCloudRight_<
        inesctec_mrdt_slam_distmap_2d::data_structures::
            DistanceMapSparseStaticWith2ndDerivFloat>;
using VariableSE2DistanceMapNanoStaticPoint2fVectorCloudRight =
    VariableSE2DistanceMapPoint2VectorCloudRight_<
        inesctec_mrdt_slam_distmap_2d::data_structures::
            DistanceMapNanoStaticFloat>;

using VariableSE2DistanceMapStaticPointNormal2fVectorCloudRight =
    VariableSE2DistanceMapPointNormal2VectorCloudRight_<
        inesctec_mrdt_slam_distmap_2d::data_structures::DistanceMapStaticFloat>;
using VariableSE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRight =
    VariableSE2DistanceMapPointNormal2VectorCloudRight_<
        inesctec_mrdt_slam_distmap_2d::data_structures::
            DistanceMapStaticWith2ndDerivFloat>;
using VariableSE2DistanceMapSparseStaticPointNormal2fVectorCloudRight =
    VariableSE2DistanceMapPointNormal2VectorCloudRight_<
        inesctec_mrdt_slam_distmap_2d::data_structures::
            DistanceMapSparseStaticFloat>;
using VariableSE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRight =
    VariableSE2DistanceMapPointNormal2VectorCloudRight_<
        inesctec_mrdt_slam_distmap_2d::data_structures::
            DistanceMapSparseStaticWith2ndDerivFloat>;
using VariableSE2DistanceMapNanoStaticPointNormal2fVectorCloudRight =
    VariableSE2DistanceMapPointNormal2VectorCloudRight_<
        inesctec_mrdt_slam_distmap_2d::data_structures::
            DistanceMapNanoStaticFloat>;

using VariableSE2DistanceMapStaticPoint2fVectorCloudLeft =
    VariableSE2DistanceMapPoint2VectorCloudLeft_<
        inesctec_mrdt_slam_distmap_2d::data_structures::DistanceMapStaticFloat>;
using VariableSE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudLeft =
    VariableSE2DistanceMapPoint2VectorCloudLeft_<
        inesctec_mrdt_slam_distmap_2d::data_structures::
            DistanceMapStaticWith2ndDerivFloat>;
using VariableSE2DistanceMapSparseStaticPoint2fVectorCloudLeft =
    VariableSE2DistanceMapPoint2VectorCloudLeft_<
        inesctec_mrdt_slam_distmap_2d::data_structures::
            DistanceMapSparseStaticFloat>;
using VariableSE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudLeft =
    VariableSE2DistanceMapPoint2VectorCloudLeft_<
        inesctec_mrdt_slam_distmap_2d::data_structures::
            DistanceMapSparseStaticWith2ndDerivFloat>;
using VariableSE2DistanceMapNanoStaticPoint2fVectorCloudLeft =
    VariableSE2DistanceMapPoint2VectorCloudLeft_<
        inesctec_mrdt_slam_distmap_2d::data_structures::
            DistanceMapNanoStaticFloat>;

using VariableSE2DistanceMapStaticPointNormal2fVectorCloudLeft =
    VariableSE2DistanceMapPointNormal2VectorCloudLeft_<
        inesctec_mrdt_slam_distmap_2d::data_structures::DistanceMapStaticFloat>;
using VariableSE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudLeft =
    VariableSE2DistanceMapPointNormal2VectorCloudLeft_<
        inesctec_mrdt_slam_distmap_2d::data_structures::
            DistanceMapStaticWith2ndDerivFloat>;
using VariableSE2DistanceMapSparseStaticPointNormal2fVectorCloudLeft =
    VariableSE2DistanceMapPointNormal2VectorCloudLeft_<
        inesctec_mrdt_slam_distmap_2d::data_structures::
            DistanceMapSparseStaticFloat>;
using VariableSE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudLeft =
    VariableSE2DistanceMapPointNormal2VectorCloudLeft_<
        inesctec_mrdt_slam_distmap_2d::data_structures::
            DistanceMapSparseStaticWith2ndDerivFloat>;
using VariableSE2DistanceMapNanoStaticPointNormal2fVectorCloudLeft =
    VariableSE2DistanceMapPointNormal2VectorCloudLeft_<
        inesctec_mrdt_slam_distmap_2d::data_structures::
            DistanceMapNanoStaticFloat>;

/**
 * @note Variables types
 *
 * (right pertubation update + point cloud 2D)
 *
 * VariableSE2DistanceMapStaticPoint2fVectorCloudRight
 * VariableSE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudRight
 * VariableSE2DistanceMapSparseStaticPoint2fVectorCloudRight
 * VariableSE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudRight
 * VariableSE2DistanceMapNanoStaticPoint2fVectorCloudRight
 *
 * (right pertubation update + point cloud 2D with normal)
 *
 * VariableSE2DistanceMapStaticPointNormal2fVectorCloudRight
 * VariableSE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRight
 * VariableSE2DistanceMapSparseStaticPointNormal2fVectorCloudRight
 * VariableSE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRight
 * VariableSE2DistanceMapNanoStaticPointNormal2fVectorCloudRight
 *
 * (left pertubation update + point cloud 2D)
 *
 * VariableSE2DistanceMapStaticPoint2fVectorCloudLeft
 * VariableSE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudLeft
 * VariableSE2DistanceMapSparseStaticPoint2fVectorCloudLeft
 * VariableSE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudLeft
 * VariableSE2DistanceMapNanoStaticPoint2fVectorCloudLeft
 *
 * (left pertubation update + point cloud 2D with normal)
 *
 * VariableSE2DistanceMapStaticPointNormal2fVectorCloudLeft
 * VariableSE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudLeft
 * VariableSE2DistanceMapSparseStaticPointNormal2fVectorCloudLeft
 * VariableSE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudLeft
 * VariableSE2DistanceMapNanoStaticPointNormal2fVectorCloudLeft
 */

}  // namespace srrg2_solver
