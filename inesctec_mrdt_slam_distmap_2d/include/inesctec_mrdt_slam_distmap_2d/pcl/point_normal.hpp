#pragma once

#include <memory>

// Eigen
#include <Eigen/StdVector>

#include "inesctec_mrdt_slam_distmap_2d/data_structures/matrix.hpp"
#include "inesctec_mrdt_slam_distmap_2d/geometry/defs.hpp"
#include "inesctec_mrdt_slam_distmap_2d/pcl/point.hpp"
#include "inesctec_mrdt_slam_distmap_2d/pcl/point_cloud.hpp"
#include "inesctec_mrdt_slam_distmap_2d/pcl/point_derived.hpp"
#include "inesctec_mrdt_slam_distmap_2d/pcl/point_field_coordinates.hpp"
#include "inesctec_mrdt_slam_distmap_2d/pcl/point_field_direction.hpp"

namespace inesctec_mrdt_slam_distmap_2d
{

namespace pcl
{

template <int Dim_, typename ComponentsType_>
struct PointNormal_
    : public PointDerived_<Point_<Dim_, ComponentsType_>,
                           PointFieldDirection_<ComponentsType_, Dim_>>
{
 public:

  using ThisType = PointNormal_<Dim_, ComponentsType_>;
  using BaseType = PointDerived_<Point_<Dim_, ComponentsType_>,
                                 PointFieldDirection_<ComponentsType_, Dim_>>;

  using ComponentsType = ComponentsType_;
  using PointFieldCoordinatesType =
      typename BaseType::PointBaseType::PointFieldCoordinatesType;
  using PointFieldNormalType = PointFieldDirection_<ComponentsType_, Dim_>;
  using VectorType = typename PointFieldCoordinatesType::FieldType;

 public:

  static constexpr int Dim = Dim_;

 public:

  PointNormal_() { this->template setZero<1>(); }
  PointNormal_(const BaseType& point) : BaseType(point) {}
  PointNormal_(const PointFieldCoordinatesType& coordinates,
               const PointFieldNormalType& normal)
      : BaseType(coordinates, normal)
  {
  }

  inline ThisType& operator=(const BaseType& point)
  {
    BaseType::operator=(point);
    return *this;
  }

  inline VectorType& normal() { return BaseType::template value<1>(); }

  inline const VectorType& normal() const
  {
    return BaseType::template value<1>();
  }

};  // struct PointNormal_<Dim_, ComponentsType_> : public
    // PointDerived_<Point_<...>, PointFieldDirection_<...>>

//! 2D and 3D points definition
template <typename ComponentsType_>
using PointNormal2_ = PointNormal_<2, ComponentsType_>;
template <typename ComponentsType_>
using PointNormal3_ = PointNormal_<3, ComponentsType_>;

using PointNormal2i = PointNormal2_<int>;
using PointNormal2f = PointNormal2_<float>;
using PointNormal2d = PointNormal2_<double>;

using PointNormal3i = PointNormal3_<int>;
using PointNormal3f = PointNormal3_<float>;
using PointNormal3d = PointNormal3_<double>;

//! vector point cloud
template <typename ComponentsType_>
using PointNormal2VectorCloud_ = PointCloud_<
    std::vector<PointNormal2_<ComponentsType_>,
                Eigen::aligned_allocator<PointNormal2_<ComponentsType_>>>>;

template <typename ComponentsType_>
using PointNormal3VectorCloud_ = PointCloud_<
    std::vector<PointNormal3_<ComponentsType_>,
                Eigen::aligned_allocator<PointNormal3_<ComponentsType_>>>>;

using PointNormal2iVectorCloud = PointNormal2VectorCloud_<int>;
using PointNormal2fVectorCloud = PointNormal2VectorCloud_<float>;
using PointNormal2dVectorCloud = PointNormal2VectorCloud_<double>;

using PointNormal3iVectorCloud = PointNormal3VectorCloud_<int>;
using PointNormal3fVectorCloud = PointNormal3VectorCloud_<float>;
using PointNormal3dVectorCloud = PointNormal3VectorCloud_<double>;

//! matrix point cloud
template <typename ComponentsType_>
using PointNormal2MatrixCloud_ =
    data_structures::Matrix_<PointNormal2_<ComponentsType_>>;

template <typename ComponentsType_>
using PointNormal3MatrixCloud_ =
    data_structures::Matrix_<PointNormal3_<ComponentsType_>>;

using PointNormal2iMatrixCloud = PointNormal2MatrixCloud_<int>;
using PointNormal2fMatrixCloud = PointNormal2MatrixCloud_<float>;
using PointNormal2dMatrixCloud = PointNormal2MatrixCloud_<double>;

using PointNormal3iMatrixCloud = PointNormal3MatrixCloud_<int>;
using PointNormal3fMatrixCloud = PointNormal3MatrixCloud_<float>;
using PointNormal3dMatrixCloud = PointNormal3MatrixCloud_<double>;

//! cloud smart pointers...
using PointNormal2iVectorCloudPtr = std::shared_ptr<PointNormal2iVectorCloud>;
using PointNormal2fVectorCloudPtr = std::shared_ptr<PointNormal2fVectorCloud>;
using PointNormal2dVectorCloudPtr = std::shared_ptr<PointNormal2dVectorCloud>;
using PointNormal3iVectorCloudPtr = std::shared_ptr<PointNormal3iVectorCloud>;
using PointNormal3fVectorCloudPtr = std::shared_ptr<PointNormal3fVectorCloud>;
using PointNormal3dVectorCloudPtr = std::shared_ptr<PointNormal3dVectorCloud>;
using PointNormal2iMatrixCloudPtr = std::shared_ptr<PointNormal2iMatrixCloud>;
using PointNormal2fMatrixCloudPtr = std::shared_ptr<PointNormal2fMatrixCloud>;
using PointNormal2dMatrixCloudPtr = std::shared_ptr<PointNormal2dMatrixCloud>;
using PointNormal3iMatrixCloudPtr = std::shared_ptr<PointNormal3iMatrixCloud>;
using PointNormal3fMatrixCloudPtr = std::shared_ptr<PointNormal3fMatrixCloud>;
using PointNormal3dMatrixCloudPtr = std::shared_ptr<PointNormal3dMatrixCloud>;

}  // namespace pcl

}  // namespace inesctec_mrdt_slam_distmap_2d
