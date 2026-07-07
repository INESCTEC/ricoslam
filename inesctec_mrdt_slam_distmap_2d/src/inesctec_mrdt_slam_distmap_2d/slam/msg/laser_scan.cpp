#include "inesctec_mrdt_slam_distmap_2d/slam/msg/laser_scan.hpp"

#include <limits>

using namespace srrg2_core;

namespace inesctec_mrdt_slam_distmap_2d
{

namespace slam
{

namespace msg
{

void LaserScan::serialize(srrg2_core::ObjectData& data,
                          srrg2_core::IdContext& context)
{
  ObjectData* header_data = new ObjectData;

  m_header.serialize(*header_data, context);

  data.setField("header", header_data);
  data.setFloat("angle_min", m_angle_min);
  data.setFloat("angle_max", m_angle_max);
  data.setFloat("angle_inc", m_angle_inc);
  data.setFloat("time_inc", m_time_inc);
  data.setFloat("scan_time", m_scan_time);
  data.setFloat("range_min", m_range_min);
  data.setFloat("range_max", m_range_max);

  ArrayData* ranges_data = new ArrayData;

  for (const auto& range : m_ranges)
  {
    if (std::isnan(range))
    {
      ranges_data->add(-1.f);
    }
    else if (std::isinf(range))
    {
      ranges_data->add(std::numeric_limits<float>::max());
    }
    else
    {
      ranges_data->add(range);
    }
  }

  data.setField("ranges", ranges_data);

  ArrayData* intensities_data = new ArrayData;

  for (const auto& intensity : m_intensities)
  {
    if (std::isnan(intensity))
    {
      intensities_data->add(-1.f);
    }
    else if (std::isinf(intensity))
    {
      intensities_data->add(std::numeric_limits<float>::max());
    }
    else
    {
      intensities_data->add(intensity);
    }
  }

  data.setField("intensities", intensities_data);
}

void LaserScan::deserialize(srrg2_core::ObjectData& data,
                            srrg2_core::IdContext& context)
{
  ObjectData* header_data = dynamic_cast<ObjectData*>(data.getField("header"));

  m_header.deserialize(*header_data, context);

  m_angle_min = static_cast<float>(data.getFloat("angle_min"));
  m_angle_max = static_cast<float>(data.getFloat("angle_max"));
  m_angle_inc = static_cast<float>(data.getFloat("angle_inc"));
  m_time_inc = static_cast<float>(data.getFloat("time_inc"));
  m_scan_time = static_cast<float>(data.getFloat("scan_time"));
  m_range_min = static_cast<float>(data.getFloat("range_min"));
  m_range_max = static_cast<float>(data.getFloat("range_max"));

  ArrayData* ranges_data = dynamic_cast<ArrayData*>(data.getField("ranges"));

  m_ranges.resize(ranges_data->size());

  for (size_t idx = 0; idx < m_ranges.size(); idx++)
  {
    m_ranges[idx] = static_cast<float>((*ranges_data)[idx]);
  }

  ArrayData* intensities_data =
      dynamic_cast<ArrayData*>(data.getField("intensities"));

  m_intensities.resize(intensities_data->size());

  for (size_t idx = 0; idx < m_intensities.size(); idx++)
  {
    m_intensities[idx] = static_cast<float>((*intensities_data)[idx]);
  }
}

}  // namespace msg

}  // namespace slam

}  // namespace inesctec_mrdt_slam_distmap_2d
