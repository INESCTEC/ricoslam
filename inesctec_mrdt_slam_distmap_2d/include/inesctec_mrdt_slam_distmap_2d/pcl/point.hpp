#pragma once

#include <memory>

// Eigen
#include <Eigen/StdVector>

#include "inesctec_mrdt_slam_distmap_2d/data_structures/matrix.hpp"
#include "inesctec_mrdt_slam_distmap_2d/geometry/defs.hpp"
#include "inesctec_mrdt_slam_distmap_2d/pcl/point_base.hpp"
#include "inesctec_mrdt_slam_distmap_2d/pcl/point_cloud.hpp"
#include "inesctec_mrdt_slam_distmap_2d/pcl/point_field_coordinates.hpp"

namespace inesctec_mrdt_slam_distmap_2d
{

namespace pcl
{

template <int Dim_, typename ComponentsType_>
struct Point_
    : public PointBase_<Dim_, PointFieldCoordinates_<ComponentsType_, Dim_>>
{
 public:

  using ThisType = Point_<Dim_, ComponentsType_>;
  using BaseType =
      PointBase_<Dim_, PointFieldCoordinates_<ComponentsType_, Dim_>>;

  using ComponentsType = ComponentsType_;
  using PointFieldCoordinatesType =
      PointFieldCoordinates_<ComponentsType_, Dim_>;
  using VectorType = typename PointFieldCoordinatesType::FieldType;

 public:

  static constexpr int Dim = Dim_;

 public:

  Point_() = default;
  Point_(const BaseType& point) : BaseType(point) {}
  Point_(const PointFieldCoordinatesType& coordinates) : BaseType(coordinates)
  {
  }

  inline ThisType& operator=(const BaseType& point)
  {
    BaseType::operator=(point);
    return *this;
  }

  inline VectorType& coordinates() { return BaseType::template value<0>(); }

  inline const VectorType& coordinates() const
  {
    return BaseType::template value<0>();
  }

};  // struct Point_<Dim_, ComponentsType_> : PointBase_<Dim_,
    // PointFieldCoordinates_<ComponentsType_, Dim_>>

//! 2D and 3D points definition
template <typename ComponentsType_>
using Point2_ = Point_<2, ComponentsType_>;
template <typename ComponentsType_>
using Point3_ = Point_<3, ComponentsType_>;

using Point2i = Point2_<int>;
using Point2f = Point2_<float>;
using Point2d = Point2_<double>;

using Point3i = Point3_<int>;
using Point3f = Point3_<float>;
using Point3d = Point3_<double>;

//! vector point cloud
template <typename ComponentsType_>
using Point2VectorCloud_ = PointCloud_<
    std::vector<Point2_<ComponentsType_>,
                Eigen::aligned_allocator<Point2_<ComponentsType_>>>>;

template <typename ComponentsType_>
using Point3VectorCloud_ = PointCloud_<
    std::vector<Point3_<ComponentsType_>,
                Eigen::aligned_allocator<Point3_<ComponentsType_>>>>;

using Point2iVectorCloud = Point2VectorCloud_<int>;
using Point2fVectorCloud = Point2VectorCloud_<float>;
using Point2dVectorCloud = Point2VectorCloud_<double>;

using Point3iVectorCloud = Point3VectorCloud_<int>;
using Point3fVectorCloud = Point3VectorCloud_<float>;
using Point3dVectorCloud = Point3VectorCloud_<double>;

//! matrix point cloud
template <typename ComponentsType_>
using Point2MatrixCloud_ = data_structures::Matrix_<Point2_<ComponentsType_>>;

template <typename ComponentsType_>
using Point3MatrixCloud_ = data_structures::Matrix_<Point3_<ComponentsType_>>;

using Point2iMatrixCloud = Point2MatrixCloud_<int>;
using Point2fMatrixCloud = Point2MatrixCloud_<float>;
using Point2dMatrixCloud = Point2MatrixCloud_<double>;

using Point3iMatrixCloud = Point3MatrixCloud_<int>;
using Point3fMatrixCloud = Point3MatrixCloud_<float>;
using Point3dMatrixCloud = Point3MatrixCloud_<double>;

//! cloud smart pointers...
using Point2iVectorCloudPtr = std::shared_ptr<Point2iVectorCloud>;
using Point2fVectorCloudPtr = std::shared_ptr<Point2fVectorCloud>;
using Point2dVectorCloudPtr = std::shared_ptr<Point2dVectorCloud>;
using Point3iVectorCloudPtr = std::shared_ptr<Point3iVectorCloud>;
using Point3fVectorCloudPtr = std::shared_ptr<Point3fVectorCloud>;
using Point3dVectorCloudPtr = std::shared_ptr<Point3dVectorCloud>;
using Point2iMatrixCloudPtr = std::shared_ptr<Point2iMatrixCloud>;
using Point2fMatrixCloudPtr = std::shared_ptr<Point2fMatrixCloud>;
using Point2dMatrixCloudPtr = std::shared_ptr<Point2dMatrixCloud>;
using Point3iMatrixCloudPtr = std::shared_ptr<Point3iMatrixCloud>;
using Point3fMatrixCloudPtr = std::shared_ptr<Point3fMatrixCloud>;
using Point3dMatrixCloudPtr = std::shared_ptr<Point3dMatrixCloud>;

}  // namespace pcl

}  // namespace inesctec_mrdt_slam_distmap_2d
