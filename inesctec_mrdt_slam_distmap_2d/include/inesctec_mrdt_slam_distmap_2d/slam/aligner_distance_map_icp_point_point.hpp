#pragma once

#include <functional>
#include <type_traits>

#include "inesctec_mrdt_slam_distmap_2d/slam/aligner.hpp"
#include "inesctec_mrdt_slam_distmap_2d/srrg2_solver/se2_distance_map_icp_point_point_error_factor.hpp"

namespace inesctec_mrdt_slam_distmap_2d
{

namespace slam
{

template <typename VariableSE2DistanceMapRightType_>
class AlignerDistanceMapICPPointPoint_
    : public Aligner_<VariableSE2DistanceMapRightType_>
{
 public:

  using typename Aligner_<VariableSE2DistanceMapRightType_>::Param;
  using typename Aligner_<VariableSE2DistanceMapRightType_>::DistMapParam;
  using typename Aligner_<VariableSE2DistanceMapRightType_>::VariableType;
  using typename Aligner_<VariableSE2DistanceMapRightType_>::EstimateType;
  using typename Aligner_<VariableSE2DistanceMapRightType_>::PointCloudTypePtr;
  using typename Aligner_<
      VariableSE2DistanceMapRightType_>::MeasurementTypeElement;
  using typename Aligner_<VariableSE2DistanceMapRightType_>::PointType;
  using typename Aligner_<VariableSE2DistanceMapRightType_>::PointVectorType;
  using typename Aligner_<VariableSE2DistanceMapRightType_>::Scalar;

  using ThisType =
      AlignerDistanceMapICPPointPoint_<VariableSE2DistanceMapRightType_>;
  using BaseType = Aligner_<VariableSE2DistanceMapRightType_>;

  using FactorType = srrg2_solver::
      SE2DistanceMapICPPointPointWithSensorErrorFactorCorrespondenceFreeDriven_<
          VariableSE2DistanceMapRightType_>;
  using FactorTypePtr = std::shared_ptr<FactorType>;
  using InformationMatrix = typename FactorType::InformationMatrixType;
  using InformationMatrixVectorType =
      typename FactorType::InformationMatrixVectorType;
  using MovingType = typename FactorType::MovingContainerType;
  using MovingTypePtr = std::shared_ptr<MovingType>;

  using OGLPointCloudSourceType = srrg2_core::Point3fVectorCloud;
  using OGLPointCloudSourceTypeWithNormals =
      srrg2_core::PointNormal3fVectorCloud;

 public:

  const Eigen::Vector4f OGLPointCloudSourceColorVar =
      srrg2_core::ColorPalette::color4fDarkRed(0.75f);
  const Eigen::Vector4f OGLPointCloudSourceColorPtsInliers =
      srrg2_core::ColorPalette::color4fRed();
  const Eigen::Vector4f OGLPointCloudSourceColorPtsOutliers =
      srrg2_core::ColorPalette::color4fBlue();
  const Eigen::Vector4f OGLPointCloudSourceColorPtsSurppressed =
      srrg2_core::ColorPalette::color4fLightGray();

 protected:

  FactorTypePtr m_factor_ =
      std::make_shared<FactorType>();  //!< SE(2) point-to-point distance
                                       //!< map-based matching error factor
  InformationMatrixVectorType
      m_factor_info_;  //!< matching error information matrix for factor
                       //!< correspondence driven abstraction
  MovingTypePtr m_moving_ =
      std::make_shared<MovingType>();  //!< source moving point cloud

  OGLPointCloudSourceType
      m_dbg_ogl_pts_factor_inliers_;  //!< OpenGL-based cached point cloud for
                                      //!< the source measurement (inlier pts
                                      //!< based on robustifier policy)
  OGLPointCloudSourceType
      m_dbg_ogl_pts_factor_outliers_;  //!< OpenGL-based cached point cloud for
                                       //!< the source measurement (outlier pts
                                       //!< based on robustifier policy)
  OGLPointCloudSourceType
      m_dbg_ogl_pts_factor_surppressed_;  //!< OpenGL-based cached point cloud
                                          //!< for the source measurement
                                          //!< (surpressed pts due to invalid
                                          //!< distance)

  OGLPointCloudSourceTypeWithNormals
      m_dbg_ogl_pts_factor_normals_;  //!< OpenGL-based cached point cloud
                                      //!< for the source measurement with
                                      //!< normals

 public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  AlignerDistanceMapICPPointPoint_(
      const Param& param = Param(),
      const DistMapParam& dist_map_param = DistMapParam())
      : Aligner_<VariableSE2DistanceMapRightType_>(param, dist_map_param)
  {
    init();
  }

  virtual void setSource(const PointCloudTypePtr& source,
                         const EstimateType& sensor_in_robot) final
  {
    assert(
        source &&
        "AlignerDistanceMapICPPointPoint_::setSource | source point cloud not "
        "set");

    m_moving_ = source;
    m_factor_->setSensorInRobot(sensor_in_robot);
    m_factor_->setMoving(*m_moving_);

    if (this->m_solver_.param_robustifier_policies.size() > 0)
    {
      m_factor_->setRobustifier(
          this->m_solver_.param_robustifier_policies.value(0)
              ->param_robustifier.value()
              .get());
    }
  }

  virtual void setSource(VariableType* const source) final
  {
    assert(
        source &&
        "AlignerDistanceMapICPPointPoint_::setSource | source SE2 variable not "
        "set");
    assert(
        source->measurement() &&
        "AlignerDistanceMapICPPointPoint_::setSource | source measurement not "
        "set");
    assert(source->measurement()->getPointCloudPtr() &&
           "AlignerDistanceMapICPPointPoint_::setSource | source point cloud "
           "shared ptr not set");

    m_moving_ = source->measurement()->getPointCloudPtr();
    m_factor_->setSensorInRobot(source->getSensorInRobot());
    m_factor_->setMoving(*m_moving_);

    if (this->m_solver_.param_robustifier_policies.size() > 0)
    {
      m_factor_->setRobustifier(
          this->m_solver_.param_robustifier_policies.value(0)
              ->param_robustifier.value()
              .get());
    }
  }

  virtual void setTarget(const PointCloudTypePtr& target,
                         const EstimateType& sensor_in_robot) final
  {
    assert(
        target &&
        "AlignerDistanceMapICPPointPoint_::setTarget | target point cloud not "
        "set");

    if (!this->m_variable_->measurement())
    {
      this->m_variable_->setMeasurement(
          std::make_shared<MeasurementTypeElement>());
    }

    this->m_variable_->measurement()->m_pc = target;
    this->m_variable_->measurement()->getDistanceMap()->setGoals(
        *this->m_variable_->measurement()->m_pc,
        this->m_dist_map_param_.m_dist_map_res,
        this->m_dist_map_param_.m_dist_map_size,
        this->m_dist_map_param_.m_dist_map_max_dist);
    this->m_variable_->setSensorInRobot(sensor_in_robot);
  }

  virtual void setTarget(VariableType* const target) final
  {
    assert(
        target &&
        "AlignerDistanceMapICPPointPoint_::setTarget | target SE2 variable not "
        "set");
    assert(
        target->measurement() &&
        "AlignerDistanceMapICPPointPoint_::setTarget | target measurement not "
        "set");
    assert(target->measurement()->getPointCloudPtr() &&
           "AlignerDistanceMapICPPointPoint_::setTarget | target point cloud "
           "shared ptr not set");

    if ((this->m_dist_map_param_.m_dist_map_res !=
         target->measurement()->getDistanceMap()->getRes()) ||
        (this->m_dist_map_param_.m_dist_map_size !=
         target->measurement()->getDistanceMap()->getMapSize()) ||
        (this->m_dist_map_param_.m_dist_map_max_dist !=
         target->measurement()->getDistanceMap()->getMaxDist()) ||
        (!target->measurement()->getDistanceMap()->getMap()))
    {
      this->m_variable_->setMeasurement(
          std::make_shared<MeasurementTypeElement>());
      this->m_variable_->measurement()->m_pc =
          target->measurement()->getPointCloudPtr();
      this->m_variable_->measurement()->getDistanceMap()->setGoals(
          *this->m_variable_->measurement()->m_pc,
          this->m_dist_map_param_.m_dist_map_res,
          this->m_dist_map_param_.m_dist_map_size,
          this->m_dist_map_param_.m_dist_map_max_dist);
    }
    else
    {
      this->m_variable_->setMeasurement(target->measurement());
    }

    this->m_variable_->setSensorInRobot(target->getSensorInRobot());
  }

  virtual void computeStatsOnly() final
  {
    this->m_stats_.m_num_inliers = 0;
    this->m_stats_.m_num_outliers = 0;
    this->m_stats_.m_num_kernelized = 0;
    this->m_stats_.m_num_moving_pts = 0;
    this->m_stats_.m_inliers_ratio = 0;
    this->m_stats_.m_outliers_ratio = 0;
    this->m_stats_.m_suppressed_ratio = 0;
    this->m_stats_.m_chi_inliers = std::numeric_limits<float>::max();
    this->m_stats_.m_chi_outliers = std::numeric_limits<float>::max();
    this->m_stats_.m_chi_kernelized = std::numeric_limits<float>::max();

    m_factor_->setBegin();

    while (!m_factor_->isEnd())
    {
      typename FactorType::BaseFactorType* factor =
          dynamic_cast<typename FactorType::BaseFactorType*>(m_factor_->get());

      factor->resetStats();
      factor->compute(true, true);

      srrg2_solver::FactorStats fstats = factor->stats();

      switch (factor->stats().status)
      {
        case srrg2_solver::FactorStats::Status::Inlier:
        {
          this->m_stats_.m_num_inliers++;
          this->m_stats_.m_chi_inliers += fstats.chi;
          this->m_stats_.m_chi_kernelized += fstats.kernel_chi;
          break;
        }
        case srrg2_solver::FactorStats::Status::Kernelized:
        {
          this->m_stats_.m_num_outliers++;
          this->m_stats_.m_chi_outliers += fstats.chi;
          this->m_stats_.m_chi_kernelized += fstats.kernel_chi;
          break;
        }
        case srrg2_solver::FactorStats::Status::Suppressed:
        {
          this->m_stats_.m_num_kernelized++;
          break;
        }
        default:
        {
          break;
        }
      }

      m_factor_->next();
      this->m_stats_.m_num_moving_pts++;
    }

    if (!this->m_stats_.m_num_inliers)
    {
      this->m_stats_.m_chi_inliers /= this->m_stats_.m_num_inliers;
    }
    if (!this->m_stats_.m_num_outliers)
    {
      this->m_stats_.m_chi_outliers /= this->m_stats_.m_num_outliers;
    }
    if (!this->m_stats_.m_num_kernelized)
    {
      this->m_stats_.m_chi_kernelized /= this->m_stats_.m_num_kernelized;
    }

    if (this->m_stats_.m_num_moving_pts > 0)
    {
      this->m_stats_.m_inliers_ratio =
          this->m_stats_.m_num_inliers / this->m_stats_.m_num_moving_pts;
      this->m_stats_.m_outliers_ratio =
          this->m_stats_.m_num_outliers / this->m_stats_.m_num_moving_pts;
      this->m_stats_.m_suppressed_ratio =
          this->m_stats_.m_num_kernelized / this->m_stats_.m_num_moving_pts;
    }

    if ((this->m_param_.m_debug) && (this->m_param_.m_dbg_ogl_interval >= 0))
    {
      this->updateDbgOPGLCanvas(true);
    }
  }

  virtual void updateDbgOPGLCanvas(bool update_canvas = true) final
  {
    Aligner_<VariableSE2DistanceMapRightType_>::updateDbgOPGLCanvas(false);

    if (!m_factor_->getMoving())
    {
      this->m_dbg_ogl_canvas_->flush();
      return;
    }
    if (!m_factor_->getMoving()->size())
    {
      this->m_dbg_ogl_canvas_->flush();
      return;
    }

    int idx = 0;
    int idx_inlier = 0;
    int idx_outlier = 0;
    int idx_surppressed = 0;

    const srrg2_core::Isometry3_<Scalar> robot_in_world_3d =
        srrg2_core::geometry3d::get3dFrom2dPose(this->m_variable_->estimate());
    const srrg2_core::Isometry3_<Scalar> sensor_in_robot_3d =
        srrg2_core::geometry3d::get3dFrom2dPose(
            this->m_variable_->getSensorInRobot());

    if (this->m_param_.m_dbg_ogl_show_normals)
    {
      m_dbg_ogl_pts_factor_normals_.resize(m_factor_->getMoving()->size());
    }
    else
    {
      m_dbg_ogl_pts_factor_inliers_.resize(m_factor_->getMoving()->size());
      m_dbg_ogl_pts_factor_outliers_.resize(m_factor_->getMoving()->size());
      m_dbg_ogl_pts_factor_surppressed_.resize(m_factor_->getMoving()->size());
    }

    m_factor_->setBegin();

    while (!m_factor_->isEnd())
    {
      typename FactorType::BaseFactorType* factor =
          dynamic_cast<typename FactorType::BaseFactorType*>(m_factor_->get());

      factor->compute(true, true);

      if (this->m_param_.m_dbg_ogl_show_normals)  // color fails w/ normals!!!
      {
        m_dbg_ogl_pts_factor_normals_[idx].coordinates().x() =
            m_factor_->getMoving()->operator[](idx).coordinates().x();
        m_dbg_ogl_pts_factor_normals_[idx].coordinates().y() =
            m_factor_->getMoving()->operator[](idx).coordinates().y();
        m_dbg_ogl_pts_factor_normals_[idx].coordinates().z() = 0;

        updateDbgOPGLCanvasShowNormals(idx);
      }
      else
      {
        switch (factor->stats().status)
        {
          case srrg2_solver::FactorStats::Status::Inlier:
          {
            m_dbg_ogl_pts_factor_inliers_[idx_inlier].coordinates().x() =
                m_factor_->getMoving()->operator[](idx).coordinates().x();
            m_dbg_ogl_pts_factor_inliers_[idx_inlier].coordinates().y() =
                m_factor_->getMoving()->operator[](idx).coordinates().y();
            m_dbg_ogl_pts_factor_inliers_[idx_inlier].coordinates().z() = 0;
            idx_inlier++;
            break;
          }
          case srrg2_solver::FactorStats::Status::Kernelized:
          {
            m_dbg_ogl_pts_factor_outliers_[idx_outlier].coordinates().x() =
                m_factor_->getMoving()->operator[](idx).coordinates().x();
            m_dbg_ogl_pts_factor_outliers_[idx_outlier].coordinates().y() =
                m_factor_->getMoving()->operator[](idx).coordinates().y();
            m_dbg_ogl_pts_factor_outliers_[idx_outlier].coordinates().z() = 0;
            idx_outlier++;
            break;
          }
          case srrg2_solver::FactorStats::Status::Suppressed:
          {
            m_dbg_ogl_pts_factor_surppressed_[idx_surppressed]
                .coordinates()
                .x() =
                m_factor_->getMoving()->operator[](idx).coordinates().x();
            m_dbg_ogl_pts_factor_surppressed_[idx_surppressed]
                .coordinates()
                .y() =
                m_factor_->getMoving()->operator[](idx).coordinates().y();
            m_dbg_ogl_pts_factor_surppressed_[idx_surppressed]
                .coordinates()
                .z() = 0;
            idx_surppressed++;
            break;
          }
          default:
          {
            break;
          }
        }
      }

      if (this->m_param_.m_dbg_ogl_show_correspondences)
      {
        if (m_factor_->getCorrespondenceIdxFixed() >= 0)
        {
          const PointVectorType& moving =
              m_factor_->getMoving()->operator[](idx).coordinates();
          const PointVectorType& fixed =
              this->m_variable_->measurement()
                  ->m_pc->at(m_factor_->getCorrespondenceIdxFixed())
                  .coordinates();

          PointVectorType fixed_in_robot =
              this->m_variable_->getSensorInRobot() * fixed;

          PointVectorType moving_in_robot =
              this->m_variable_->getSensorInRobot() * moving;

          moving_in_robot = this->m_variable_->estimate() * moving_in_robot;

          srrg2_core::Vector3f correspondence[2];
          correspondence[0].x() = fixed_in_robot.x();
          correspondence[0].y() = fixed_in_robot.y();
          correspondence[0].z() = 0;
          correspondence[1].x() = moving_in_robot.x();
          correspondence[1].y() = moving_in_robot.y();
          correspondence[1].z() = 0;

          this->m_dbg_ogl_canvas_->pushColor();
          this->m_dbg_ogl_canvas_->setColor(
              BaseType::OGLPointCloudMovingColorVar);
          this->m_dbg_ogl_canvas_->putLine(2, correspondence);
          this->m_dbg_ogl_canvas_->popAttribute();
        }
      }

      m_factor_->next();
      ++idx;
    }

    m_dbg_ogl_pts_factor_inliers_.resize(idx_inlier);
    m_dbg_ogl_pts_factor_outliers_.resize(idx_outlier);
    m_dbg_ogl_pts_factor_surppressed_.resize(idx_surppressed);

    this->m_dbg_ogl_canvas_->pushColor();
    this->m_dbg_ogl_canvas_->setColor(OGLPointCloudSourceColorPtsInliers);
    this->m_dbg_ogl_canvas_->pushMatrix();
    this->m_dbg_ogl_canvas_->multMatrix(robot_in_world_3d.matrix());
    this->m_dbg_ogl_canvas_->pushMatrix();
    this->m_dbg_ogl_canvas_->multMatrix(sensor_in_robot_3d.matrix());
    this->m_dbg_ogl_canvas_->putPoints(m_dbg_ogl_pts_factor_inliers_);
    this->m_dbg_ogl_canvas_->popMatrix();
    this->m_dbg_ogl_canvas_->popMatrix();
    this->m_dbg_ogl_canvas_->popAttribute();

    this->m_dbg_ogl_canvas_->pushColor();
    this->m_dbg_ogl_canvas_->setColor(OGLPointCloudSourceColorPtsOutliers);
    this->m_dbg_ogl_canvas_->pushMatrix();
    this->m_dbg_ogl_canvas_->multMatrix(robot_in_world_3d.matrix());
    this->m_dbg_ogl_canvas_->pushMatrix();
    this->m_dbg_ogl_canvas_->multMatrix(sensor_in_robot_3d.matrix());
    this->m_dbg_ogl_canvas_->putPoints(m_dbg_ogl_pts_factor_outliers_);
    this->m_dbg_ogl_canvas_->popMatrix();
    this->m_dbg_ogl_canvas_->popMatrix();
    this->m_dbg_ogl_canvas_->popAttribute();

    this->m_dbg_ogl_canvas_->pushColor();
    this->m_dbg_ogl_canvas_->setColor(OGLPointCloudSourceColorPtsSurppressed);
    this->m_dbg_ogl_canvas_->pushMatrix();
    this->m_dbg_ogl_canvas_->multMatrix(robot_in_world_3d.matrix());
    this->m_dbg_ogl_canvas_->pushMatrix();
    this->m_dbg_ogl_canvas_->multMatrix(sensor_in_robot_3d.matrix());
    this->m_dbg_ogl_canvas_->putPoints(m_dbg_ogl_pts_factor_surppressed_);
    this->m_dbg_ogl_canvas_->popMatrix();
    this->m_dbg_ogl_canvas_->popMatrix();
    this->m_dbg_ogl_canvas_->popAttribute();

    if (update_canvas)
    {
      this->m_dbg_ogl_canvas_->flush();
    }
  }

 protected:

  virtual void initCorrespondencesRegularizer() final
  {
    this->m_source_correspondences_count_.clear();
    this->m_moving_correspondences_.resize(m_factor_->getMoving()->size());

    m_factor_info_.assign(m_factor_->getMoving()->size(),
                          InformationMatrix::Identity());

    m_factor_->setInformationMatrixVector(m_factor_info_);
  }

  virtual void updateCorrespondencesRegularizer() final
  {
    for (auto& source_counter : this->m_source_correspondences_count_)
    {
      source_counter.second = 0;
    }

    int idx = 0;

    m_factor_->setBegin();

    while (!m_factor_->isEnd())
    {
      typename FactorType::BaseFactorType* factor =
          dynamic_cast<typename FactorType::BaseFactorType*>(m_factor_->get());

      factor->compute(true, true);

      this->m_moving_correspondences_[idx] =
          m_factor_->getCorrespondenceIdxFixed();

      if (m_factor_->getCorrespondenceIdxFixed() >= 0)
      {
        ++this->m_source_correspondences_count_
              [m_factor_->getCorrespondenceIdxFixed()];
      }

      m_factor_->next();
      ++idx;
    }

    assert(static_cast<int>(m_factor_info_.size()) == idx &&
           "::updateCorrespondencesRegularizer: different sizes");

    for (int idx = 0; idx < static_cast<int>(m_factor_->getMoving()->size());
         idx++)
    {
      const int& moving_correspondence = this->m_moving_correspondences_[idx];

      if (moving_correspondence >= 0)
      {
        float moving_weight = 1.f / this->m_source_correspondences_count_.at(
                                        moving_correspondence);

        m_factor_info_[idx].diagonal().setConstant(moving_weight);
      }
    }
  }

 private:

  void init()
  {
    m_factor_->setVariableId(0, 0);
    m_factor_->setInformationMatrix(InformationMatrix::Identity());

    if (this->m_solver_.param_robustifier_policies.size() > 0)
    {
      m_factor_->setRobustifier(
          this->m_solver_.param_robustifier_policies.value(0)
              ->param_robustifier.value()
              .get());
    }

    this->m_graph_.addFactor(m_factor_);

    if ((this->m_param_.m_debug) && (this->m_param_.m_dbg_ogl_interval >= 0))
    {
      if (this->m_param_.m_dbg_ogl_mode.compare("iterationend") == 0)
      {
        srrg2_solver::SolverGenericAtIterationEndActionPtr iterationend =
            std::make_shared<srrg2_solver::SolverGenericAtIterationEndAction>();

        iterationend->m_sleep_ms = this->m_param_.m_dbg_ogl_interval;
        iterationend->m_action =
            std::bind(&ThisType::updateDbgOPGLCanvas, this, true);

        this->m_solver_.param_actions.pushBack(iterationend);
      }
      else if (this->m_param_.m_dbg_ogl_mode.compare("computeend") == 0)
      {
        srrg2_solver::SolverGenericAtComputeEndActionPtr computeend =
            std::make_shared<srrg2_solver::SolverGenericAtComputeEndAction>();

        computeend->m_sleep_ms = this->m_param_.m_dbg_ogl_interval;
        computeend->m_action =
            std::bind(&ThisType::updateDbgOPGLCanvas, this, true);

        this->m_solver_.param_actions.pushBack(computeend);
      }
    }
  }

  template <typename T = PointType,
            std::enable_if_t<std::is_same<T, pcl::PointNormal2_<Scalar>>::value,
                             bool> = 0>
  inline void updateDbgOPGLCanvasShowNormals(int idx)
  {
    m_dbg_ogl_pts_factor_normals_[idx].normal().x() =
        m_factor_->getMoving()->operator[](idx).normal().x();
    m_dbg_ogl_pts_factor_normals_[idx].normal().y() =
        m_factor_->getMoving()->operator[](idx).normal().y();
    m_dbg_ogl_pts_factor_normals_[idx].normal().z() = 0;
  }

  template <typename T = PointType,
            std::enable_if_t<
                !std::is_same<T, pcl::PointNormal2_<Scalar>>::value, bool> = 0>
  inline void updateDbgOPGLCanvasShowNormals(int)
  {
  }

};  // class AlignerDistanceMapICPPointPoint_<VariableSE2DistanceMapRightType_>
    // : Aligner_<VariableSE2DistanceMapRightType_>

}  // namespace slam

}  // namespace inesctec_mrdt_slam_distmap_2d
