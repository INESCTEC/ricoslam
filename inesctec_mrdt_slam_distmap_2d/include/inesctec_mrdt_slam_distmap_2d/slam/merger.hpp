#pragma once

#include <exception>
#include <memory>
#include <utility>
#include <vector>

#include "inesctec_mrdt_slam_distmap_2d/slam/msg/laser_scan.hpp"

namespace inesctec_mrdt_slam_distmap_2d
{

namespace slam
{

/**
 * @brief merger class (update local map based on the laser scanner's sensor
 *        model, i.e., ray tracing to evaluate see-through vs new point)
 * @tparam EstimateType_ pose estimate type (SE2 transformation)
 * @tparam DistanceMapType_ distance map type
 * @tparam PointCloudType_ point cloud (moving, map) type
 */
template <typename EstimateType_, typename DistanceMapType_,
          typename PointCloudType_>
class Merger_
{
 public:

  using ThisType = Merger_<EstimateType_, DistanceMapType_, PointCloudType_>;

  using EstimateType = EstimateType_;

  using PointCloudType = PointCloudType_;
  using PointType = typename PointCloudType::PointType;
  using PointFieldCoordinatesType =
      typename PointType::PointFieldCoordinatesType;
  using PointVectorType = typename PointType::VectorType;
  using Scalar = typename PointType::ComponentsType;

  using DistanceMapType = DistanceMapType_;

  using LaserScanMsgType = msg::LaserScan;

 public:

  /**
   * @brief merger parametrization
   *
   * IF add new enabled AND observed point closer than mapped one that satisfies
   *    distance condition
   *    EXECUTE add new point to map
   *    CONTINUE
   *
   * IF see through enabled AND see through distance condition
   *    EXECUTE replace current mapped point
   *    CONTINUE
   * ELSE IF add new enabled AND new point distance condition considering the
   *    see through point as a new point
   *    EXECUTE add new point to map
   *    CONTINUE
   *
   * IF average enabled AND both mapped and observed pts in same ray
   *    (considering the observation as the same point)
   *    EXECUTE mapped point averaged by the observed one considering weights...
   */
  struct Param
  {
    Scalar m_ray_min_dist_through;  //!< ray minimum distance to consider a see
                                    //!< through (m) (-1 not used)
    Scalar m_ray_min_dist_add_new;  //!< ray minimum distance to consider a new
                                    //!< point not mapped yet (m) (-1 not used)
    bool m_enable_avg;  //!< enable the average operation when there is a mapped
                        //!< and an observed points in the same ray
    Scalar m_weight_old;  //!< if average enabled, weight of the old mapped
                          //!< point (0..1)
    Scalar m_weight_new;  //!< if average enabled, weight of the new observed
                          //!< point (0..1)

    Param(Scalar ray_min_dist_through = 0.1, Scalar ray_min_dist_add_new = 0.1)
        : m_ray_min_dist_through(ray_min_dist_through),
          m_ray_min_dist_add_new(ray_min_dist_add_new),
          m_enable_avg(false),
          m_weight_old(-1),
          m_weight_new(-1)
    {
    }

    Param(Scalar ray_min_dist_through, Scalar ray_min_dist_add_new,
          bool enable_avg, Scalar weight_old, Scalar weight_new)
        : m_ray_min_dist_through(ray_min_dist_through),
          m_ray_min_dist_add_new(ray_min_dist_add_new),
          m_enable_avg(enable_avg),
          m_weight_old(weight_old),
          m_weight_new(weight_new)
    {
    }

    friend std::ostream& operator<<(std::ostream& os, const Param& param)
    {
      if (param.m_ray_min_dist_through <= 0)
      {
        os << "- ray minimum distance to consider see through (m): not enabled"
           << std::endl;
      }
      else
      {
        os << "- ray minimum distance to consider see through (m): "
           << param.m_ray_min_dist_through << std::endl;
      }
      if (param.m_ray_min_dist_add_new <= 0)
      {
        os << "- ray minimum distance to consider new point   (m): not enabled"
           << std::endl;
      }
      else
      {
        os << "- ray minimum distance to consider new point   (m): "
           << param.m_ray_min_dist_add_new << std::endl;
      }
      if (param.m_enable_avg)
      {
        os << "- average measurements: enabled" << std::endl;
        os << "  - weight old measurement: " << param.m_weight_old << std::endl;
        os << "  - weight new measurement: " << param.m_weight_new << std::endl;
      }
      else
      {
        os << "- average measurements: not enabled" << std::endl;
      }
      return os;
    }

  };  // struct Merger_<EstimateType_, DistanceMapType_, PointCloudType_>::Param

 protected:

  Param m_param_;  //!< merger parametrization

  std::vector<std::pair<int, PointType>>
      m_sensor_analysis_;  //!< per-ray sensor analysis (map closest pc idx, map
                           //!< point in current laser frame)

 public:

  /**
   * @brief default constructor
   * @param[in] param merger parametrization
   */
  Merger_(const Param& param = Param()) : m_param_(param)
  {
    assertInit(m_param_);
  }

  /**
   * @brief merge current moving point cloud into the local map, assuming that
   *        the input point cloud has beend already filtered with range
   *        thresholds and/or voxelization
   *        (online SLAM with local maps)
   * @param[in] moving_pc moving point cloud
   * @param[in] moving_scan 2D laser scanner message corresponding to the moving
   *                        point cloud
   * @param[in] sensor_in_robot SE(2) transformation of the sensor w.r.t. robot
   *                            coordinate frame
   * @param[in] robot_in_map SE(2) transformation of the robot w.r.t. map
   * @param[in,out] map point cloud representation of the map
   * @param[in,out] map_dist distance map created by the map point cloud
   */
  void merge(const PointCloudType& moving_pc,
             const LaserScanMsgType& moving_scan,
             const EstimateType& sensor_in_robot,
             const EstimateType& robot_in_map, PointCloudType* const map,
             DistanceMapType* const map_dist = nullptr)
  {
    bool map_changed = false;

    initMerge(moving_scan);

    EstimateType se2_sensor_in_map_to_sensor_in_moving =
        sensor_in_robot.inverse() * robot_in_map.inverse() * sensor_in_robot;
    EstimateType se2_sensor_in_moving_to_sensor_in_map =
        sensor_in_robot.inverse() * robot_in_map * sensor_in_robot;

    for (size_t idx = 0; idx < map->size(); idx++)
    {
      PointType pt_in_moving =
          map->operator[](idx).transform(se2_sensor_in_map_to_sensor_in_moving);

      int ray_idx = moving_scan.ptToRayIdx(pt_in_moving.coordinates());

      if ((ray_idx < 0) ||
          (ray_idx >= static_cast<int>(moving_scan.m_ranges.size())))
      {
        continue;
      }

      if (m_sensor_analysis_[ray_idx].first < 0)  // pt not initialized...
      {
        m_sensor_analysis_[ray_idx].first = static_cast<int>(idx);
        m_sensor_analysis_[ray_idx].second = pt_in_moving;
      }
      else  // map point already in sensor ray analysis...
      {
        Scalar dist_curr_pt = moving_scan.ptToDist(pt_in_moving.coordinates());
        Scalar dist_prev_pt = moving_scan.ptToDist(
            m_sensor_analysis_[ray_idx].second.coordinates());

        if (dist_curr_pt < dist_prev_pt)
        {
          m_sensor_analysis_[ray_idx].first = static_cast<int>(idx);
          m_sensor_analysis_[ray_idx].second = pt_in_moving;
        }
      }
    }

    for (size_t idx = 0; idx < moving_pc.size(); idx++)
    {
      int ray_idx_pc_pt = moving_scan.ptToRayIdx(moving_pc[idx].coordinates());
      Scalar dist_pc_pt = moving_scan.ptToDist(moving_pc[idx].coordinates());

      if ((ray_idx_pc_pt < 0) ||
          (ray_idx_pc_pt >= static_cast<int>(moving_scan.m_ranges.size())))
      {
        continue;
      }

      PointType pc_pt_in_map =
          moving_pc[idx].transform(se2_sensor_in_moving_to_sensor_in_map);

      const auto& analysis_pair = m_sensor_analysis_[ray_idx_pc_pt];

      Scalar dist_map_pt =
          moving_scan.ptToDist(analysis_pair.second.coordinates());
      Scalar dist_diff = dist_pc_pt - dist_map_pt;

      //! add new point to the map
      //! (not currently in the map OR in front of an existing point)
      if ((m_param_.m_ray_min_dist_add_new > 0) &&
          ((analysis_pair.first < 0) ||    // no map point on current ray...
           ((analysis_pair.first >= 0) &&  // OR distance threshold criteria
            (dist_diff < -m_param_.m_ray_min_dist_add_new))))
      {
        map_changed = true;
        map->emplace_back(pc_pt_in_map);
        continue;
      }

      //! can see through >>> replace current mapped point
      if ((m_param_.m_ray_min_dist_through > 0) &&
          (analysis_pair.first >= 0) &&  // distance threshold criteria
          (dist_diff > m_param_.m_ray_min_dist_through))
      {
        map_changed = true;
        map->at(analysis_pair.first) = pc_pt_in_map;
        continue;
      }
      //! OTHERWISE, with see through replacement disabled,
      //! add new point if add new enabled
      else if ((m_param_.m_ray_min_dist_add_new > 0) &&
               (analysis_pair.first >= 0) &&  // OR distance threshold criteria
               (dist_diff > m_param_.m_ray_min_dist_add_new))
      {
        map_changed = true;
        map->emplace_back(pc_pt_in_map);
        continue;
      }

      //! FINALLY, if map point in same ray considered as the observed one,
      //! average it!!! (if enabled...)
      if ((m_param_.m_enable_avg) && (analysis_pair.first >= 0))
      {
        map_changed = true;
        map->at(analysis_pair.first) *= m_param_.m_weight_old;
        map->at(analysis_pair.first) += pc_pt_in_map * m_param_.m_weight_new;
      }
    }

    if (map_changed && map_dist)
    {
      map_dist->setGoals(*map, map_dist->getRes(), map_dist->getMapSize(),
                         map_dist->getMaxDist());
    }

  }  // void Merger_::merge(const PointCloudType&, const LaserScanMsgType&,
     // const EstimateType&, const EstimateType&, PointCloudType* const,
     // DistanceMapType* const)

  /**
   * @brief merge scan range data into a map (local or global, does not matter),
   *        with range filtering
   *        (offline 2D occupancy grid map generation)
   * @param[in] scan input 2D laser scanner message
   * @param[in] scan_idx scan index from original set to construct the map
   *                     (allows map point to scan range index association)
   * @param[in] sensor_in_robot SE(2) transformation of the sensor w.r.t. robot
   *                            coordinate frame
   * @param[in] robot_in_map SE(2) transformation of the robot w.r.t. map
   * @param[in] range_min minimum range value (m) (ignores scan msg thresholds)
   * @param[in] range_max maximum range value (m) (ignores scan msg thresholds)
   * @param[in,out] map point cloud representation of the map
   * @param[in,out] map_pt_to_scan_range association of each map point (same
   *                                     index in the vector) with the original
   *                                     laser scan message and respective range
   *                                     index
   */
  void merge(const LaserScanMsgType& scan, size_t scan_idx,
             const EstimateType& sensor_in_robot,
             const EstimateType& robot_in_map, float range_min, float range_max,
             PointCloudType& map,
             std::vector<std::pair<size_t, size_t>>& map_pt_to_scan_range)
  {
    initMerge(scan);

    EstimateType se2_robot_in_map_to_sensor_in_moving =
        sensor_in_robot.inverse() * robot_in_map.inverse();
    EstimateType se2_sensor_in_map = robot_in_map * sensor_in_robot;

    for (size_t idx = 0; idx < map.size(); idx++)
    {
      PointType pt_in_moving =
          map.operator[](idx).transform(se2_robot_in_map_to_sensor_in_moving);

      int ray_idx = scan.ptToRayIdx(pt_in_moving.coordinates());

      if ((ray_idx < 0) || (ray_idx >= static_cast<int>(scan.m_ranges.size())))
      {
        continue;
      }

      if (m_sensor_analysis_[ray_idx].first < 0)  // pt not initialized...
      {
        m_sensor_analysis_[ray_idx].first = static_cast<int>(idx);
        m_sensor_analysis_[ray_idx].second = pt_in_moving;
      }
      else  // map point already in sensor ray analysis...
      {
        Scalar dist_curr_pt = scan.ptToDist(pt_in_moving.coordinates());
        Scalar dist_prev_pt =
            scan.ptToDist(m_sensor_analysis_[ray_idx].second.coordinates());

        if (dist_curr_pt < dist_prev_pt)
        {
          m_sensor_analysis_[ray_idx].first = static_cast<int>(idx);
          m_sensor_analysis_[ray_idx].second = pt_in_moving;
        }
      }
    }

    for (size_t idx = 0; idx < scan.m_ranges.size(); idx++)
    {
      if (scan.m_ranges[idx] > range_max || scan.m_ranges[idx] < range_min)
      {
        continue;
      }

      float angle =
          scan.m_angle_min + static_cast<float>(idx) * scan.m_angle_inc;

      PointType moving_pt;

      scan.angleDistToPt(moving_pt.coordinates(), angle, scan.m_ranges[idx]);

      PointType pc_pt_in_map = moving_pt.transform(se2_sensor_in_map);

      const auto& analysis_pair = m_sensor_analysis_[idx];

      Scalar dist_map_pt = scan.ptToDist(analysis_pair.second.coordinates());
      Scalar dist_diff = scan.m_ranges[idx] - dist_map_pt;

      //! add new point to the map
      //! (not currently in the map OR in front of an existing point)
      if ((m_param_.m_ray_min_dist_add_new > 0) &&
          ((analysis_pair.first < 0) ||    // no map point on current ray...
           ((analysis_pair.first >= 0) &&  // OR distance threshold criteria
            (dist_diff < -m_param_.m_ray_min_dist_add_new))))
      {
        map.emplace_back(pc_pt_in_map);
        map_pt_to_scan_range.emplace_back(std::make_pair(scan_idx, idx));
        continue;
      }

      //! can see through >>> replace current mapped point
      if ((m_param_.m_ray_min_dist_through > 0) &&
          (analysis_pair.first >= 0) &&  // distance threshold criteria
          (dist_diff > m_param_.m_ray_min_dist_through))
      {
        map.at(analysis_pair.first) = pc_pt_in_map;
        map_pt_to_scan_range.at(analysis_pair.first) =
            std::make_pair(scan_idx, idx);
        continue;
      }
      //! OTHERWISE, with see through replacement disabled,
      //! add new point if add new enabled
      else if ((m_param_.m_ray_min_dist_add_new > 0) &&
               (analysis_pair.first >= 0) &&  // OR distance threshold criteria
               (dist_diff > m_param_.m_ray_min_dist_add_new))
      {
        map.emplace_back(pc_pt_in_map);
        map_pt_to_scan_range.emplace_back(std::make_pair(scan_idx, idx));
        continue;
      }

      map.emplace_back(pc_pt_in_map);
      map_pt_to_scan_range.emplace_back(std::make_pair(scan_idx, idx));
    }

  }  // void Merger_::merge(const LaserScanMsgType& scan, size_t scan_idx, const
     // EstimateType&, const EstimateType&, float, float, PointCloudType&,
     // std::vector<std::pair<size_t, size_t>>&)

  void setParam(const Param& param)
  {
    assertInit(param);
    m_param_ = param;
  }

 public:

  inline const Param& getParam() const { return m_param_; }

 protected:

  inline void initMerge(const LaserScanMsgType& moving_scan)
  {
    PointType pt_0;
    pt_0.setZero();
    m_sensor_analysis_.assign(moving_scan.m_ranges.size(),
                              std::pair<int, PointType>(-1, pt_0));
  }  // void Merger_::initMerge(const LaserScanMsgType&)

 private:

  static void assertInit(const Param& param)
  {
    if ((param.m_enable_avg) && (param.m_weight_old <= 0))
    {
      throw std::invalid_argument(
          "Merger_::assertInit | weight old measurement upon averaging on "
          "ray-tracing update must be greater than 0");
    }
    if ((param.m_enable_avg) && (param.m_weight_new <= 0))
    {
      throw std::invalid_argument(
          "Merger_::assertInit | weight new measurement upon averaging on "
          "ray-tracing update must be greater than 0");
    }
    if ((param.m_enable_avg) && (param.m_weight_old + param.m_weight_new != 1))
    {
      throw std::invalid_argument(
          "Merger_::assertInit | sum of the new and old measurements' weights "
          "upon averaging on ray-tracing update must be exactly 1");
    }
  }  // void Merger_::assertInit()

};  // class Merger_<EstimateType_, DistanceMapType_, PointCloudType_>

}  // namespace slam

}  // namespace inesctec_mrdt_slam_distmap_2d
