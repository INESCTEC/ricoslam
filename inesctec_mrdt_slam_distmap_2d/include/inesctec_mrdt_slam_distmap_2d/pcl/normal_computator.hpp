#pragma once

#include <chrono>
#include <exception>
#include <thread>
#include <type_traits>

// Eigen
#include <Eigen/Eigenvalues>

// SRRG Software
#include <srrg2_core/srrg_pcl/instances.h>
#include <srrg2_core/srrg_pcl/point_types.h>
#include <srrg2_core/srrg_viewer/viewer_canvas.h>
#include <srrg2_core/srrg_viewer/viewer_core/color_palette.h>
#include <srrg2_qgl_viewport/viewer_core_shared_qgl.h>

// YAML
#include <yaml-cpp/yaml.h>

#include "inesctec_mrdt_slam_distmap_2d/geometry/defs.hpp"
#include "inesctec_mrdt_slam_distmap_2d/pcl/point_defs.hpp"
#include "inesctec_mrdt_slam_distmap_2d/pcl/point_types.hpp"
#include "inesctec_mrdt_slam_distmap_2d/pcl/point_utils.hpp"
#include "inesctec_mrdt_slam_distmap_2d/srrg2_qgl_viewport/viewer_core_shared_qgl_custom.hpp"

namespace inesctec_mrdt_slam_distmap_2d
{

namespace pcl
{

/**
 * @brief normal computator base class (does not do anything, provides also
 *        generic methods for OpenGL-based debug visualization of normal point
 *        clouds)
 * @note the normals visualization only work in SRRG by toggling it with the 'N'
 *       key
 * @tparam PointCloudType_ point cloud type
 */
template <typename PointCloudType_>
class NormalComputatorBase_
{
 public:

  using ThisType = NormalComputatorBase_<PointCloudType_>;
  using PointCloudType = PointCloudType_;
  using PointType = typename PointCloudType::PointType;
  using Scalar = typename PointType::ComponentsType;

  using OGLAppType = QApplication;
  using OGLAppTypePtr = std::shared_ptr<OGLAppType>;

  using OGLViewerType = srrg2_qgl_viewport::ViewerCoreSharedQGLCustom;
  using OGLViewerTypePtr = std::shared_ptr<OGLViewerType>;

  using OGLCanvasType = srrg2_core::ViewerCanvas;
  using OGLCanvasTypePtr = std::shared_ptr<OGLCanvasType>;

  using OGLPointCloudType = srrg2_core::PointNormal3fVectorCloud;
  using OGLPointCloudTypePtr = std::shared_ptr<OGLPointCloudType>;

 public:

  const Eigen::Vector4f OGLColorPts = srrg2_core::ColorPalette::color4fBlack();

 protected:

  OGLAppTypePtr m_dbg_ogl_qapp_;       //!< Qt application required for the SRRG
                                       //!< ViewerCoreSharedQGL viz
  OGLViewerTypePtr m_dbg_ogl_viewer_;  //!< ViewerCoreSharedQGL handler
  OGLCanvasTypePtr
      m_dbg_ogl_canvas_;  //!< viz canvas to show the normal computator results
  OGLPointCloudTypePtr
      m_dbg_ogl_pc_;  //!< normal point cloud in the SRRG format

 public:

  NormalComputatorBase_() = default;
  virtual ~NormalComputatorBase_() = default;

  /**
   * @brief interface API to compute the normals of a generic point cloud
   * @param[in] cloud point cloud
   */
  virtual void computeNormals(PointCloudType&) const {

  }  // virtual void NormalComputatorBase_::computeNormals(PointCloudType&)
     // const

  /**
   * @brief setup the debug OpenGL-based visualization
   * @param[in] argc number of input arguments of the executable in the main
   *                 thread
   * @param[in] argv array of strings corresponding to the input arguments of
   *                 the executable in the main thread
   * @param[in] qapp QApplication shared object
   * @param[in] viewer shared viewer master
   */
  void setupDbgOGL(int argc = 0, char* argv[] = nullptr,
                   const OGLAppTypePtr& qapp = nullptr,
                   const OGLViewerTypePtr& viewer = nullptr)
  {
    if (!qapp)
    {
      m_dbg_ogl_qapp_ = std::make_shared<OGLAppType>(argc, argv);
    }
    else
    {
      m_dbg_ogl_qapp_ = qapp;
    }

    if (!viewer)
    {
      m_dbg_ogl_viewer_ =
          std::make_shared<OGLViewerType>(argc, argv, m_dbg_ogl_qapp_.get(),
                                          BUFFER_SIZE_50MEGABYTE, 3, 25, true);
    }
    else
    {
      m_dbg_ogl_viewer_ = viewer;
    }

    m_dbg_ogl_canvas_ = m_dbg_ogl_viewer_->getCanvas("Normal Computator");
    m_dbg_ogl_canvas_->_setup();

    std::this_thread::sleep_for(std::chrono::seconds(1));
  }  // void NormalComputatorBase_::setupDbgOGL(int = 0, char*[] = nullptr,
     // const OGLAppTypePtr& = nullptr, const OGLViewerTypePtr& = nullptr)

  /**
   * @brief update the canvas for the debug OpenGL-based visualization
   * @param[in] cloud input point cloud of the normal computator (must have a
   *                  point normal field in the cloud definition!)
   */
  template <typename T = PointType,
            std::enable_if_t<std::is_same<T, pcl::PointNormal2_<Scalar>>::value,
                             bool> = 0>
  void updateDbgOPGLCanvas(const PointCloudType& cloud)
  {
    if ((!m_dbg_ogl_qapp_) || (!m_dbg_ogl_viewer_) || (!m_dbg_ogl_canvas_))
    {
      throw std::runtime_error(
          "NormalComputatorBase_::updateDbgOPGLCanvas | "
          "missing setup of the OpenGL-based debug visualization");
    }

    if (!m_dbg_ogl_pc_)
    {
      m_dbg_ogl_pc_ = std::make_shared<OGLPointCloudType>();
    }

    m_dbg_ogl_pc_->resize(cloud.size());

    for (size_t idx = 0; idx < cloud.size(); idx++)
    {
      m_dbg_ogl_pc_->operator[](idx).coordinates().x() =
          cloud[idx].coordinates().x();
      m_dbg_ogl_pc_->operator[](idx).coordinates().y() =
          cloud[idx].coordinates().y();
      m_dbg_ogl_pc_->operator[](idx).coordinates().z() = 0;

      m_dbg_ogl_pc_->operator[](idx).normal().x() = cloud[idx].normal().x();
      m_dbg_ogl_pc_->operator[](idx).normal().y() = cloud[idx].normal().y();
      m_dbg_ogl_pc_->operator[](idx).normal().z() = 0;
    }

    m_dbg_ogl_canvas_->pushColor();
    m_dbg_ogl_canvas_->setColor(OGLColorPts);
    m_dbg_ogl_canvas_->putPoints(*m_dbg_ogl_pc_);
    m_dbg_ogl_canvas_->popAttribute();

    m_dbg_ogl_canvas_->flush();
  }  // void NormalComputatorBase_::updateDbgOPGLCanvas(PointCloudType&)

  template <typename T = PointType,
            std::enable_if_t<
                !std::is_same<T, pcl::PointNormal2_<Scalar>>::value, bool> = 0>
  void updateDbgOPGLCanvas(const PointCloudType&)
  {
  }  // void NormalComputatorBase_::updateDbgOPGLCanvas(PointCloudType&)

 public:

  inline const OGLAppTypePtr& getDbgOGLAppPtr() const
  {
    return m_dbg_ogl_qapp_;
  }
  inline const OGLViewerTypePtr& getDbgOGLViewerPtr() const
  {
    return m_dbg_ogl_viewer_;
  }
  inline const OGLCanvasTypePtr& getDbgOGLCanvasPtr() const
  {
    return m_dbg_ogl_canvas_;
  }

};  // class NormalComputatorBase_<PointCloudType_>

/**
 * @brief compute the normals based on the nearest points (assuming a 2D
 *        organized point cloud)
 * @todo have the possibility of setting a maximum curvature, based on the
 *       covariance matrix's eigen values, to avoid "interpolation" /
 *       "suavization" upon corners...
 * @tparam PointCloudType_ point cloud type
 */
template <typename PointCloudType_>
class NormalComputatorNearestPoints1D_
    : public NormalComputatorBase_<PointCloudType_>
{
 public:

  using typename NormalComputatorBase_<PointCloudType_>::PointCloudType;
  using typename NormalComputatorBase_<PointCloudType_>::PointType;
  using typename NormalComputatorBase_<PointCloudType_>::Scalar;

 public:

  using ThisType = NormalComputatorNearestPoints1D_<PointCloudType_>;
  using BaseType = NormalComputatorBase_<PointCloudType_>;

  using VectorType = geometry::VectorN_<Scalar, PointType::Dim>;
  using MatrixType = geometry::MatrixN_<Scalar, PointType::Dim>;

 public:

  int m_min_pts = 5;  //!< minimum points of the nearest points neighborhood to
                      //!< compute the normal
  float m_radius = 0.1f;  //!< nearest neighborhood radius (m)

 public:

  NormalComputatorNearestPoints1D_() { checkPointType(); }

  virtual ~NormalComputatorNearestPoints1D_() = default;

  /**
   * @brief compute the normals of the point cloud accordingly to the nearest
   *        neighbors at a certain radius (inclusively)
   * @note wrapToRange utility method allows to wrap the range automatically
   *       in the case of the points still being inside the search radius
   * @param[in,out] cloud point cloud
   */
  void computeNormals(PointCloudType& cloud) const override
  {
    computeNormalsImpl(cloud);
  }  // void NormalComputatorNearestPoints1D_::computeNormals(PointCloudType&)
     // const override

 private:

  template <typename T = PointType,
            std::enable_if_t<std::is_same<T, pcl::PointNormal2_<Scalar>>::value,
                             bool> = 0>
  inline void checkPointType() const
  {
  }

  template <typename T = PointType,
            std::enable_if_t<
                !std::is_same<T, pcl::PointNormal2_<Scalar>>::value, bool> = 0>
  inline void checkPointType() const
  {
    throw std::runtime_error(
        "NormalComputatorNearestPoints1D_::checkPointType | "
        "point type is not pcl::PointNormal2_<Scalar>! "
        "The normal computator should only be called for "
        "pcl::PointNormal2_<Scalar> point types");
  }

  template <typename T = PointType,
            std::enable_if_t<std::is_same<T, pcl::PointNormal2_<Scalar>>::value,
                             bool> = 0>
  void computeNormalsImpl(PointCloudType& cloud) const
  {
    PointFieldAccumulator_<0, PointType> pt_acc;

    for (int idx = 0; idx < static_cast<int>(cloud.size()); idx++)
    {
      pt_acc.reset();

      for (int sub_idx = idx;
           (cloud[idx].coordinates() - cloud[sub_idx].coordinates()).norm() <=
               m_radius &&
           pt_acc.getNumPts() < static_cast<int>(cloud.size());
           sub_idx = geometry::wrapToRange(sub_idx - 1,
                                           static_cast<int>(cloud.size())))
      {
        pt_acc.add(cloud[sub_idx]);
      }

      for (int sub_idx =
               geometry::wrapToRange(idx + 1, static_cast<int>(cloud.size()));
           (cloud[idx].coordinates() - cloud[sub_idx].coordinates()).norm() <=
               m_radius &&
           pt_acc.getNumPts() < static_cast<int>(cloud.size());
           sub_idx = geometry::wrapToRange(sub_idx + 1,
                                           static_cast<int>(cloud.size())))
      {
        pt_acc.add(cloud[sub_idx]);
      }

      if (pt_acc.getNumPts() < m_min_pts)
      {
        cloud[idx].normal().setZero();
        continue;
      }

      Eigen::SelfAdjointEigenSolver<MatrixType> solver;

      solver.computeDirect(pt_acc.getCovariance());

      if (solver.info() != Eigen::ComputationInfo::Success)
      {
        cloud[idx].normal().setZero();
        continue;
      }

      int eigen_value_min_idx = 0;

      for (int eigen_idx = 1; eigen_idx < PointType::Dim; eigen_idx++)
      {
        if (solver.eigenvalues()(eigen_idx, 0) <
            solver.eigenvalues()(eigen_value_min_idx, 0))
        {
          eigen_value_min_idx = eigen_idx;
        }
      }

      cloud[idx].normal() =
          solver.eigenvectors().col(eigen_value_min_idx).real();

      if (cloud[idx].normal().dot(cloud[idx].coordinates()) > 0)
      {
        cloud[idx].normal() = -cloud[idx].normal();
      }
    }
  }

  template <typename T = PointType,
            std::enable_if_t<
                !std::is_same<T, pcl::PointNormal2_<Scalar>>::value, bool> = 0>
  void computeNormalsImpl(PointCloudType& cloud) const
  {
  }

};  // class NormalComputatorNearestPoints1D_<PointCloudType_> : public
    // NormalComputatorBase_<PointCloudType_>

/**
 * @brief compute the normals with a 1D sliding window on the point cloud
 *        indexation (assuming a 2D organized point cloud)
 * @todo have the possibility of setting a maximum curvature, based on the
 *       covariance matrix's eigen values, to avoid "interpolation" /
 *       "suavization" upon corners...
 * @tparam PointCloudType_ point cloud type
 */
template <typename PointCloudType_>
class NormalComputatorSlidingWindow1D_
    : public NormalComputatorBase_<PointCloudType_>
{
 public:

  using typename NormalComputatorBase_<PointCloudType_>::PointCloudType;
  using typename NormalComputatorBase_<PointCloudType_>::PointType;
  using typename NormalComputatorBase_<PointCloudType_>::Scalar;

 public:

  using ThisType = NormalComputatorSlidingWindow1D_<PointCloudType_>;
  using BaseType = NormalComputatorBase_<PointCloudType_>;

  using VectorType = geometry::VectorN_<Scalar, PointType::Dim>;
  using MatrixType = geometry::MatrixN_<Scalar, PointType::Dim>;

 public:

  bool m_wrap_range = false;
  int m_window_size = 2;  //!

 public:

  NormalComputatorSlidingWindow1D_() { checkPointType(); }

  virtual ~NormalComputatorSlidingWindow1D_() = default;

  /**
   * @brief compute the normals of the point cloud accordingly to a sliding
   *        window on the point cloud indexes
   * @note the computator's parameter wrap_range allows the user to enable it
   *       in the case of being a 360deg sensor
   * @param[in,out] cloud point cloud
   */
  void computeNormals(PointCloudType& cloud) const override
  {
    computeNormalsImpl(cloud);
  }  // void NormalComputatorSlidingWindow1D_::computeNormals(PointCloudType&)
     // const override

 private:

  template <typename T = PointType,
            std::enable_if_t<std::is_same<T, pcl::PointNormal2_<Scalar>>::value,
                             bool> = 0>
  inline void checkPointType() const
  {
  }

  template <typename T = PointType,
            std::enable_if_t<
                !std::is_same<T, pcl::PointNormal2_<Scalar>>::value, bool> = 0>
  inline void checkPointType() const
  {
    throw std::runtime_error(
        "NormalComputatorSlidingWindow1D_::checkPointType | "
        "point type is not pcl::PointNormal2_<Scalar>! "
        "The normal computator should only be called for "
        "pcl::PointNormal2_<Scalar> point types");
  }

  template <typename T = PointType,
            std::enable_if_t<std::is_same<T, pcl::PointNormal2_<Scalar>>::value,
                             bool> = 0>
  void computeNormalsImpl(PointCloudType& cloud) const
  {
    PointFieldAccumulator_<0, PointType> pt_acc;

    for (int idx = 0; idx < static_cast<int>(cloud.size()); idx++)
    {
      pt_acc.reset();
      pt_acc.add(cloud[idx]);

      for (int idx_offset = 1;
           idx_offset <= m_window_size &&
           pt_acc.getNumPts() < static_cast<int>(cloud.size());
           idx_offset++)
      {
        int sub_idx = idx - idx_offset;

        if ((sub_idx < 0) && (m_wrap_range))
        {
          sub_idx =
              geometry::wrapToRange(sub_idx, static_cast<int>(cloud.size()));
        }
        else if ((sub_idx < 0) && (!m_wrap_range))
        {
          break;
        }

        pt_acc.add(cloud[sub_idx]);
      }

      for (int idx_offset = 1;
           idx_offset <= m_window_size &&
           pt_acc.getNumPts() < static_cast<int>(cloud.size());
           idx_offset++)
      {
        int sub_idx = idx + idx_offset;

        if ((sub_idx >= static_cast<int>(cloud.size())) && (m_wrap_range))
        {
          sub_idx =
              geometry::wrapToRange(sub_idx, static_cast<int>(cloud.size()));
        }
        else if ((sub_idx >= static_cast<int>(cloud.size())) && (!m_wrap_range))
        {
          break;
        }

        pt_acc.add(cloud[sub_idx]);
      }

      if (pt_acc.getNumPts() < m_window_size + 1)
      {
        cloud[idx].normal().setZero();
        continue;
      }

      Eigen::SelfAdjointEigenSolver<MatrixType> solver;

      solver.computeDirect(pt_acc.getCovariance());

      if (solver.info() != Eigen::ComputationInfo::Success)
      {
        cloud[idx].normal().setZero();
        continue;
      }

      int eigen_value_min_idx = 0;

      for (int eigen_idx = 1; eigen_idx < PointType::Dim; eigen_idx++)
      {
        if (solver.eigenvalues()(eigen_idx, 0) <
            solver.eigenvalues()(eigen_value_min_idx, 0))
        {
          eigen_value_min_idx = eigen_idx;
        }
      }

      cloud[idx].normal() =
          solver.eigenvectors().col(eigen_value_min_idx).real();

      if (cloud[idx].normal().dot(cloud[idx].coordinates()) > 0)
      {
        cloud[idx].normal() = -cloud[idx].normal();
      }
    }
  }

  template <typename T = PointType,
            std::enable_if_t<
                !std::is_same<T, pcl::PointNormal2_<Scalar>>::value, bool> = 0>
  void computeNormalsImpl(PointCloudType& cloud) const
  {
  }

};  // class NormalComputatorSlidingWindow1D_<PointCloudType_, normalIdx> :
    // public NormalComputatorBase_<PointCloudType_, normalIdx>

}  // namespace pcl

}  // namespace inesctec_mrdt_slam_distmap_2d
