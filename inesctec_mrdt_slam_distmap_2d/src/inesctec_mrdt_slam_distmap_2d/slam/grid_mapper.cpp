#include "inesctec_mrdt_slam_distmap_2d/slam/grid_mapper.hpp"

#include <srrg2_core/srrg_pcl/point.h>
#include <srrg2_core/srrg_pcl/point_cloud.h>
#include <srrg2_core/srrg_viewer/viewer_core/color_palette.h>
#include <srrg2_solver/variables_and_factors/types_2d/variable_se2.h>

#include <exception>
#include <limits>
#include <string>
#include <vector>

#include "inesctec_mrdt_slam_distmap_2d/data_structures/traverse_line.hpp"
#include "inesctec_mrdt_slam_distmap_2d/geometry/geometry_2d.hpp"

namespace inesctec_mrdt_slam_distmap_2d
{

namespace slam
{

GridMapper::GridMapper(const Param& param) : m_param_(param) { assertInit(); }

void GridMapper::updateMap()
{
  if (!m_graph_)
  {
    throw std::runtime_error("GridMapper::updateMap | graph not set");
  }

  if (!m_processed_scans_.size())
  {
    return;
  }

  m_grid_map_ = std::make_shared<GridMap2DType>();

  assert(m_grid_map_->getOccupancyMap() &&
         "GridMapper::updateMap | occupancy map not initialized");
  assert(m_grid_map_->getFrequencyMap() &&
         "GridMapper::updateMap | frequency map not initialized");
  assert(m_grid_map_->getOccupancyImg() &&
         "GridMapper::updateMap | occupancy map image not initialized");

  m_grid_map_->setOrigin(GridMap2DType::EstimateType::Identity());
  m_grid_map_->setRes(m_param_.m_res);

  Eigen::Vector2f lower_right(std::numeric_limits<float>::max(),
                              std::numeric_limits<float>::max());
  Eigen::Vector2f upper_left(-std::numeric_limits<float>::max(),
                             -std::numeric_limits<float>::max());

  for (const auto& msg : m_processed_scans_)
  {
    srrg2_solver::VariableBase* var = m_graph_->variable(msg->m_local_map_id);

    if (!var)
    {
      throw std::runtime_error("GridMapper::updateMap | graph variable " +
                               std::to_string(msg->m_local_map_id) +
                               " not found when preprocessing the scans");
    }

    srrg2_solver::VariableSE2Base* v =
        dynamic_cast<srrg2_solver::VariableSE2Base*>(var);

    if (!v)
    {
      throw std::runtime_error(
          "GridMapper::updateMap | invalid variable dynamic cast to "
          "VariableSE2Base when preprocessing the scans");
    }

    const EstimateType scan_in_global =
        v->estimate() * msg->m_pose_in_local_map * msg->m_sensor_in_robot;

    float range_min =
        m_param_.m_usable_range_min < 0
            ? msg->m_scan->m_range_min
            : std::max(m_param_.m_usable_range_min, msg->m_scan->m_range_min);
    float range_max =
        m_param_.m_range_max > 0
            ? std::min(m_param_.m_range_max, msg->m_scan->m_range_max)
            : msg->m_scan->m_range_max;

    for (size_t idx = 0; idx < msg->m_scan->m_ranges.size(); idx++)
    {
      if (msg->m_scan->m_ranges[idx] > range_max ||
          msg->m_scan->m_ranges[idx] < range_min)
      {
        continue;
      }

      float r = msg->m_scan->m_ranges[idx];

      if (r > range_max)
      {
        r = range_max;
      }

      float angle = msg->m_scan->m_angle_min +
                    static_cast<float>(idx) * msg->m_scan->m_angle_inc;

      geometry::Vector2f pt_in_sensor;

      msg->m_scan->angleDistToPt(pt_in_sensor, angle, r);

      geometry::Vector2f pt_in_global = scan_in_global * pt_in_sensor;

      lower_right.x() = std::min(lower_right.x(), pt_in_global.x());
      lower_right.y() = std::min(lower_right.y(), pt_in_global.y());
      upper_left.x() = std::max(upper_left.x(), pt_in_global.x());
      upper_left.y() = std::max(upper_left.y(), pt_in_global.y());
    }
  }

  upper_left.x() += m_grid_map_->getRes();
  upper_left.y() += m_grid_map_->getRes();
  lower_right.x() -= m_grid_map_->getRes();
  lower_right.y() -= m_grid_map_->getRes();

  Eigen::Vector2f metric_size = upper_left - lower_right;

  m_grid_map_->setSize(metric_size);
  m_grid_map_->setOrigin(
      geometry::v2t(Eigen::Vector3f(lower_right.x(), lower_right.y(), 0.f)));
  m_grid_map_->getFrequencyMap()->fill(Eigen::Vector2i::Zero());

  if (!m_map_to_scan_pt_)
  {
    for (const auto& msg : m_processed_scans_)
    {
      srrg2_solver::VariableBase* var = m_graph_->variable(msg->m_local_map_id);

      if (!var)
      {
        throw std::runtime_error("GridMapper::updateMap | graph variable " +
                                 std::to_string(msg->m_local_map_id) +
                                 " not found when integrating the scans");
      }

      srrg2_solver::VariableSE2Base* v =
          dynamic_cast<srrg2_solver::VariableSE2Base*>(var);

      if (!v)
      {
        throw std::runtime_error(
            "GridMapper::updateMap | invalid variable dynamic cast to "
            "VariableSE2Base when integrating the scans");
      }

      const EstimateType scan_in_global =
          v->estimate() * msg->m_pose_in_local_map * msg->m_sensor_in_robot;

      integrateScan(*m_grid_map_->getOccupancyMap(),
                    *m_grid_map_->getFrequencyMap(), scan_in_global,
                    *msg->m_scan);
    }
  }
  else
  {
    integrateMap(*m_grid_map_->getOccupancyMap(),
                 *m_grid_map_->getFrequencyMap());
  }

  for (int rr = 0; rr < m_grid_map_->getOccupancyMap()->rows(); ++rr)
  {
    for (int cc = 0; cc < m_grid_map_->getOccupancyMap()->cols(); ++cc)
    {
      if ((*m_grid_map_->getFrequencyMap())(rr, cc)(1) != 0)
      {
        float occ =
            (*m_grid_map_->getFrequencyMap())(rr, cc)(0) /
            static_cast<float>((*m_grid_map_->getFrequencyMap())(rr, cc)(1));

        (*m_grid_map_->getOccupancyMap())(rr, cc) = occ;

        if (m_param_.m_thresh_free >= 0.f)
        {
          if (occ >= m_param_.m_thresh_occ)
          {
            (*m_grid_map_->getOccupancyImg())(rr, cc) = 0;
          }
          else if (occ <= m_param_.m_thresh_free)
          {
            (*m_grid_map_->getOccupancyImg())(rr, cc) = 255;
          }
          else
          {
            (*m_grid_map_->getOccupancyImg())(rr, cc) = 128;
          }
        }
        else
        {
          if (occ >= m_param_.m_thresh_occ)
          {
            (*m_grid_map_->getOccupancyImg())(rr, cc) = 0;
          }
          else
          {
            (*m_grid_map_->getOccupancyImg())(rr, cc) = 255;
          }
        }
      }
      else
      {
        (*m_grid_map_->getOccupancyMap())(rr, cc) = 0.5;
        (*m_grid_map_->getOccupancyImg())(rr, cc) = 128;
      }
    }
  }
}

void GridMapper::integrateScan(OccupancyGridMapType& grid_map,
                               FrequencyGridMapType& freq_map,
                               const EstimateType& scan_in_map,
                               const msg::LaserScan& scan)
{
  assert(grid_map.rows() == freq_map.rows() &&
         "GridMapper::integratePointCloud | rows size in grid_map vs freq_map "
         "different");
  assert(grid_map.cols() == freq_map.cols() &&
         "GridMapper::integratePointCloud | cols size in grid_map vs freq_map "
         "different");

  Eigen::Vector2f beam_origin = scan_in_map.translation();
  Eigen::Vector2i beam_origin_idx = m_grid_map_->global2Idx(beam_origin);

  if (!grid_map.isInside(beam_origin_idx))
  {
    return;
  }

  // data_structures::UCharImage img;

  UpdateFrequencyAction freq_action(freq_map);

  float range_min =
      m_param_.m_usable_range_min < 0
          ? scan.m_range_min
          : std::max(m_param_.m_usable_range_min, scan.m_range_min);
  float range_max = m_param_.m_range_max > 0
                        ? std::min(m_param_.m_range_max, scan.m_range_max)
                        : scan.m_range_max;
  float usable_range_max =
      m_param_.m_usable_range_max > 0
          ? std::min(m_param_.m_usable_range_max, range_max)
          : range_max;

  for (size_t idx = 0; idx < scan.m_ranges.size(); idx++)
  {
    if (scan.m_ranges[idx] > range_max || scan.m_ranges[idx] < range_min)
    {
      continue;
    }

    bool draw_endpoint = true;
    float r = scan.m_ranges[idx];

    if (r > usable_range_max)
    {
      draw_endpoint = false;
      r = std::min(r, range_max);
    }

    float angle = scan.m_angle_min + static_cast<float>(idx) * scan.m_angle_inc;

    Eigen::Vector2f pt_in_sensor;

    scan.angleDistToPt(pt_in_sensor, angle, r);

    Eigen::Vector2f pt_in_global = scan_in_map * pt_in_sensor;
    Eigen::Vector2i pt_in_global_idx = m_grid_map_->global2Idx(pt_in_global);

    draw_endpoint &= data_structures::traverseLine(
        beam_origin_idx.x(), beam_origin_idx.y(), pt_in_global_idx.x(),
        pt_in_global_idx.y(), freq_action);

    if (draw_endpoint)
    {
      int rr = pt_in_global_idx.x();
      int cc = pt_in_global_idx.y();

      freq_map(rr, cc)(0) += 1;
      // freq_map(rr, cc)(1) += 1;

      /**
       * @note at least for MRDT impl must be commented. if you analise
       *       traverseLine impl, you see that the action is always executed
       *       before checking if the endpoint was reached...
       */
    }
  }

  // m_grid_map_->toImageFrequencyMap(img);
  // cv::imshow("occupancy grid map", img);
  // cv::waitKey(100);
}

void GridMapper::integrateMap(OccupancyGridMapType& grid_map,
                              FrequencyGridMapType& freq_map)
{
  UpdateFrequencyAction freq_action(freq_map);

  for (const auto& scan_pt_pair : *m_map_to_scan_pt_)
  {
    const auto& msg = m_processed_scans_.at(scan_pt_pair.first);

    srrg2_solver::VariableBase* var = m_graph_->variable(msg->m_local_map_id);

    if (!var)
    {
      throw std::runtime_error("GridMapper::integrateMap | graph variable " +
                               std::to_string(msg->m_local_map_id) +
                               " not found when integrating the map points");
    }

    srrg2_solver::VariableSE2Base* v =
        dynamic_cast<srrg2_solver::VariableSE2Base*>(var);

    if (!v)
    {
      throw std::runtime_error(
          "GridMapper::integrateMap | invalid variable dynamic cast to "
          "VariableSE2Base when integrating the map points");
    }

    const EstimateType scan_in_map =
        v->estimate() * msg->m_pose_in_local_map * msg->m_sensor_in_robot;

    const msg::LaserScan& scan = *msg->m_scan;

    Eigen::Vector2f beam_origin = scan_in_map.translation();
    Eigen::Vector2i beam_origin_idx = m_grid_map_->global2Idx(beam_origin);

    if (!grid_map.isInside(beam_origin_idx))
    {
      continue;
    }

    float range_min =
        m_param_.m_usable_range_min < 0
            ? scan.m_range_min
            : std::max(m_param_.m_usable_range_min, scan.m_range_min);
    float range_max = m_param_.m_range_max > 0
                          ? std::min(m_param_.m_range_max, scan.m_range_max)
                          : scan.m_range_max;
    float usable_range_max =
        m_param_.m_usable_range_max > 0
            ? std::min(m_param_.m_usable_range_max, range_max)
            : range_max;

    size_t idx = scan_pt_pair.second;

    if (scan.m_ranges[idx] > range_max || scan.m_ranges[idx] < range_min)
    {
      continue;
    }

    bool draw_endpoint = true;
    float r = scan.m_ranges[idx];

    if (r > usable_range_max)
    {
      draw_endpoint = false;
      r = std::min(r, range_max);
    }

    float angle = scan.m_angle_min + static_cast<float>(idx) * scan.m_angle_inc;

    Eigen::Vector2f pt_in_sensor;

    scan.angleDistToPt(pt_in_sensor, angle, r);

    Eigen::Vector2f pt_in_global = scan_in_map * pt_in_sensor;
    Eigen::Vector2i pt_in_global_idx = m_grid_map_->global2Idx(pt_in_global);

    draw_endpoint &= data_structures::traverseLine(
        beam_origin_idx.x(), beam_origin_idx.y(), pt_in_global_idx.x(),
        pt_in_global_idx.y(), freq_action);

    if (draw_endpoint)
    {
      int rr = pt_in_global_idx.x();
      int cc = pt_in_global_idx.y();

      freq_map(rr, cc)(0) += 1;
      // freq_map(rr, cc)(1) += 1;

      /**
       * @note at least for MRDT impl must be commented. if you analise
       *       traverseLine impl, you see that the action is always executed
       *       before checking if the endpoint was reached...
       */
    }
  }
}

void GridMapper::assertInit()
{
  if (m_param_.m_res <= 0)
  {
    throw std::runtime_error(
        "GridMapper::assertInit | occupancy grid map resolution must be "
        "greater than 0 (m/px)");
  }
  if ((m_param_.m_usable_range_min >= 0) && (m_param_.m_usable_range_max >= 0))
  {
    if (m_param_.m_usable_range_max <= m_param_.m_usable_range_min)
    {
      throw std::runtime_error(
          "GridMapper::assertInit | "
          "when usable_range_max and usable_range_min enabled, "
          "usable_range_max must be greater than usable_range_min");
    }
  }
  if ((m_param_.m_usable_range_min >= 0) && (m_param_.m_range_max >= 0))
  {
    if (m_param_.m_range_max <= m_param_.m_usable_range_min)
    {
      throw std::runtime_error(
          "GridMapper::assertInit | "
          "when range_max and usable_range_min enabled, "
          "range_max must be greater than usable_range_min");
    }
  }
  if ((m_param_.m_usable_range_max >= 0) && (m_param_.m_range_max >= 0))
  {
    if (m_param_.m_range_max < m_param_.m_usable_range_max)
    {
      throw std::runtime_error(
          "GridMapper::assertInit | "
          "when range_max and usable_range_max enabled, "
          "range_max must be greater or equal to than usable_range_max");
    }
  }
  if ((m_param_.m_thresh_occ < 0) || (m_param_.m_thresh_occ > 1.f))
  {
    throw std::runtime_error(
        "GridMapper::assertInit | thresh_occ represents a probability (0..1)");
  }
  if (m_param_.m_thresh_free >= 0)
  {
    if (m_param_.m_thresh_free > 1.f)
    {
      throw std::runtime_error(
          "GridMapper::assertInit | thresh_free represents a probability "
          "(0..1) when enabled");
    }
    if (m_param_.m_thresh_free >= m_param_.m_thresh_occ)
    {
      throw std::runtime_error(
          "GridMapper::assertInit | thresh_free must be lower than "
          "thresh_occ when free probability enabled");
    }
  }
}

void GridMapper::updateDbgOPGLCanvas(const srrg2_core::ViewerCanvasPtr& canvas)
{
  srrg2_core::Point3fVectorCloud pc;

  canvas->putReferenceSystem(0.10);

  for (const auto& msg : m_processed_scans_)
  {
    srrg2_solver::VariableBase* var = m_graph_->variable(msg->m_local_map_id);

    if (!var)
    {
      throw std::runtime_error(
          "GridMapper::updateDbgOPGLCanvas | graph variable " +
          std::to_string(msg->m_local_map_id) +
          " not found when preprocessing the scans");
    }

    srrg2_solver::VariableSE2Base* v =
        dynamic_cast<srrg2_solver::VariableSE2Base*>(var);

    if (!v)
    {
      throw std::runtime_error(
          "GridMapper::updateDbgOPGLCanvas | invalid variable dynamic cast to "
          "VariableSE2Base when preprocessing the scans");
    }

    canvas->pushColor();
    canvas->setColor(srrg2_core::ColorPalette::color4fDarkRed(0.5));
    canvas->pushMatrix();
    canvas->multMatrix(
        srrg2_core::geometry3d::get3dFrom2dPose(v->estimate()).matrix());
    canvas->putSphere(0.05);
    canvas->popMatrix();
    canvas->popAttribute();

    const EstimateType scan_in_global =
        v->estimate() * msg->m_pose_in_local_map * msg->m_sensor_in_robot;

    float range_min =
        m_param_.m_usable_range_min < 0
            ? msg->m_scan->m_range_min
            : std::max(m_param_.m_usable_range_min, msg->m_scan->m_range_min);
    float range_max =
        m_param_.m_usable_range_max > 0
            ? std::min(m_param_.m_usable_range_max, msg->m_scan->m_range_max)
            : msg->m_scan->m_range_max;

    pc.resize(msg->m_scan->m_ranges.size());

    int num_valid_ranges = 0;

    for (size_t idx = 0; idx < msg->m_scan->m_ranges.size(); idx++)
    {
      if (msg->m_scan->m_ranges[idx] > range_max ||
          msg->m_scan->m_ranges[idx] < range_min)
      {
        continue;
      }

      float angle = msg->m_scan->m_angle_min +
                    static_cast<float>(idx) * msg->m_scan->m_angle_inc;

      Eigen::Vector2f pt_in_sensor;

      msg->m_scan->angleDistToPt(pt_in_sensor, angle,
                                 msg->m_scan->m_ranges[idx]);

      Eigen::Vector2f pt_in_global = scan_in_global * pt_in_sensor;

      pc[num_valid_ranges].coordinates().x() = pt_in_global.x();
      pc[num_valid_ranges].coordinates().y() = pt_in_global.y();
      pc[num_valid_ranges].coordinates().z() = 0.f;

      num_valid_ranges++;
    }

    pc.resize(num_valid_ranges);

    canvas->pushColor();
    canvas->setColor(srrg2_core::ColorPalette::color4fBlack());
    canvas->putPoints(pc);
    canvas->popAttribute();
  }

  for (auto factor : m_graph_->factors())
  {
    const auto* f = factor.second;

    f->_drawImpl(canvas);
  }

  canvas->flush();
}

}  // namespace slam

}  // namespace inesctec_mrdt_slam_distmap_2d
