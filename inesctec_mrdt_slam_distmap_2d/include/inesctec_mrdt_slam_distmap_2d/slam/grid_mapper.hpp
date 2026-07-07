#pragma once

#include <srrg2_core/srrg_viewer/viewer_canvas.h>
#include <srrg2_solver/solver_core/factor_graph_interface.h>
#include <srrg2_solver/solver_core/variable.h>

#include <Eigen/Geometry>
#include <memory>
#include <utility>

#include "inesctec_mrdt_slam_distmap_2d/data_structures/grid_map.hpp"
#include "inesctec_mrdt_slam_distmap_2d/pcl/point.hpp"
#include "inesctec_mrdt_slam_distmap_2d/slam/msg/laser_scan.hpp"

namespace inesctec_mrdt_slam_distmap_2d
{

namespace slam
{

class GridMapper
{
 public:

  using OccupancyGridMapType = data_structures::OccupancyGridMap;
  using OccupancyImageType = data_structures::OccupancyImage;
  using FrequencyGridMapType = data_structures::FrequencyIntGridMap;
  using GridMap2DType =
      data_structures::GridMap2D_<OccupancyGridMapType, OccupancyImageType,
                                  FrequencyGridMapType>;
  using GridMap2DTypePtr = std::shared_ptr<GridMap2DType>;

  using EstimateType = GridMap2DType::EstimateType;

 public:

  /**
   * @brief grid mapper parametrization
   */
  struct Param
  {
   public:

    float m_res;               //!< map resolution (m/px)
    float m_usable_range_min;  //!< minimum usable range (m)
    float m_usable_range_max;  //!< maximum usable range (m)
    float m_range_max;         //!< maximum range (m) for free-space ray trace
    float m_thresh_free;       //!< free occupancy probability threshold (0..1)
    float m_thresh_occ;        //!< occupied probability threshold (0..1)

   public:

    /**
     * @brief default parametrization constructor
     */
    Param()
        : m_res(0.05f),
          m_usable_range_min(0.f),
          m_usable_range_max(10.f),
          m_range_max(12.f),
          m_thresh_free(0.1f),
          m_thresh_occ(0.85f)
    {
    }

    /**
     * @brief construct a new parametrization
     * @param[in] res map resolution (m/px)
     * @param[in] usable_range_min minimum usable range (m)
     * @param[in] usable_range_max maximum usable range (m)
     * @param[in] range_max maximum range (m) for free-space ray trace
     * @param[in] thresh_free free occupancy probability threshold (0..1)
     * @param[in] thresh_occ occupied probability threshold (0..1)
     */
    Param(float res, float usable_range_min, float usable_range_max,
          float range_max, float thresh_free, float thresh_occ)
        : m_res(res),
          m_usable_range_min(usable_range_min),
          m_usable_range_max(usable_range_max),
          m_range_max(range_max),
          m_thresh_free(thresh_free),
          m_thresh_occ(thresh_occ)
    {
    }

    friend std::ostream& operator<<(std::ostream& os, const Param& param)
    {
      os << "- resolution             (m/px): " << param.m_res << std::endl;
      os << "- usable range - minimum    (m): ";
      if (param.m_usable_range_min < 0)
      {
        os << "read from msg.range_min" << std::endl;
      }
      else
      {
        os << param.m_usable_range_min << std::endl;
      }
      os << "- usable range - maximum    (m): ";
      if (param.m_usable_range_max <= 0)
      {
        os << "read from msg.range_max" << std::endl;
      }
      else
      {
        os << param.m_usable_range_max << std::endl;
      }
      os << "- maximum range (ray trace) (m): ";
      if (param.m_range_max <= 0)
      {
        os << (param.m_range_max <= 0 ? "read from msg.range_max"
                                      : "same as usable maximum range")
           << std::endl;
      }
      else
      {
        os << param.m_range_max << std::endl;
      }
      if (param.m_thresh_free >= 0)
      {
        os << "- threshold free     cell probab (0..1): " << param.m_thresh_free
           << std::endl;
      }
      else
      {
        os << "- threshold free     cell probab (0..1): not enabled"
           << std::endl;
      }
      os << "- threshold occupied cell probab (0..1): " << param.m_thresh_occ
         << std::endl;

      return os;
    }
  };  // struct Param

  /**
   * @brief update frequency grid map action during line traversal on ray
   *        tracing
   */
  struct UpdateFrequencyAction
  {
   public:

    FrequencyGridMapType& m_freq;

   public:

    UpdateFrequencyAction(FrequencyGridMapType& freq) : m_freq(freq) {}

    inline bool operator()(int x, int y)
    {
      if (!m_freq.isInside(x, y))
      {
        return false;
      }
      m_freq(x, y)(1) += 1;
      return true;
    }
  };  // struct UpdateFrequencyAction

  /**
   * @brief laser scan message with SE2 coordinate frame transformations
   *        (scan msg + sensor w.r.t. robot frame + robot pose w.r.t. local map
   *        + local map id)
   * @note with the SE2 coordinate frame transformations and the local map id
   *       you can estimate the global pose of the scan!
   */
  struct LaserScanMsgWithTf
  {
   public:

    msg::LaserScanPtr m_scan = nullptr;  //!< 2D laser scan message
    EstimateType m_sensor_in_robot =
        EstimateType::Identity();  //!< sensor w.r.t. robot frame
    EstimateType m_pose_in_local_map =
        EstimateType::Identity();  //!< robot pose w.r.t. local map
    srrg2_solver::VariableBase::Id m_local_map_id = -1;  //!< local map graph id

   public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    LaserScanMsgWithTf() = default;
    LaserScanMsgWithTf(const msg::LaserScanPtr& scan,
                       const EstimateType& sensor_in_robot,
                       const EstimateType& pose_in_local,
                       srrg2_solver::VariableBase::Id local_map_id)
        : m_scan(scan),
          m_sensor_in_robot(sensor_in_robot),
          m_pose_in_local_map(pose_in_local),
          m_local_map_id(local_map_id)
    {
    }
    LaserScanMsgWithTf(msg::LaserScanPtr&& scan, EstimateType&& sensor_in_robot,
                       EstimateType&& pose_in_local,
                       srrg2_solver::VariableBase::Id local_map_id)
        : m_scan(std::move(scan)),
          m_sensor_in_robot(std::move(sensor_in_robot)),
          m_pose_in_local_map(std::move(pose_in_local)),
          m_local_map_id(local_map_id)
    {
    }
  };  // struct LaserScanMsgWithTf

  using LaserScanMsgWithTfPtr = std::shared_ptr<LaserScanMsgWithTf>;

 protected:

  Param m_param_;  //!< parametrization

  srrg2_solver::FactorGraphInterface* m_graph_ = nullptr;  //!< factor graph

  GridMap2DTypePtr m_grid_map_ = nullptr;  //!< 2D grid map

  std::vector<LaserScanMsgWithTfPtr> m_processed_scans_;

  std::vector<std::pair<size_t, size_t>>* m_map_to_scan_pt_ = nullptr;

 public:

  GridMapper(const Param& param = Param());

  inline void pushLaserScanMsg(const msg::LaserScanPtr& scan,
                               const EstimateType& sensor_in_robot,
                               const EstimateType& pose_in_local,
                               srrg2_solver::VariableBase::Id local_map_id)
  {
    m_processed_scans_.push_back(std::make_shared<LaserScanMsgWithTf>(
        scan, sensor_in_robot, pose_in_local, local_map_id));
  }

  void setMapToScanPt(std::vector<std::pair<size_t, size_t>>* map_to_scan_pt)
  {
    m_map_to_scan_pt_ = map_to_scan_pt;
  }

  void updateMap();
  inline void updateMap(float res)
  {
    if (res <= 0)
    {
      throw std::runtime_error(
          "GridMapper::updateMap | occupancy grid map resolution must be "
          "greater than 0 (m/px)");
    }
    m_param_.m_res = res;
    updateMap();
  }

  void updateDbgOPGLCanvas(const srrg2_core::ViewerCanvasPtr& canvas);

 public:

  inline void setGraph(srrg2_solver::FactorGraphInterface* const graph)
  {
    m_graph_ = graph;
  }

  inline const std::vector<LaserScanMsgWithTfPtr>& getProcessedScans() const
  {
    return m_processed_scans_;
  }

  inline GridMap2DType* getGridMapContainer() const
  {
    return m_grid_map_.get();
  }
  inline const GridMap2DTypePtr& getGridMapContainerPtr() const
  {
    return m_grid_map_;
  }

 protected:

  void integrateScan(OccupancyGridMapType& grid_map,
                     FrequencyGridMapType& freq_map,
                     const EstimateType& scan_in_map,
                     const msg::LaserScan& scan);
  void integrateMap(OccupancyGridMapType& grid_map,
                    FrequencyGridMapType& freq_map);

 private:

  void assertInit();
  void init();
};  // class GridMapper

}  // namespace slam

}  // namespace inesctec_mrdt_slam_distmap_2d
