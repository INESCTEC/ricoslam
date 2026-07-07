#include "inesctec_mrdt_slam_distmap_2d/slam/msg/header.hpp"

namespace inesctec_mrdt_slam_distmap_2d
{

namespace slam
{

namespace msg
{

void Header::serialize(srrg2_core::ObjectData& data, srrg2_core::IdContext&)
{
  data.setDouble("t", m_t);
  data.setString("frame_id", m_frame_id);
}

void Header::deserialize(srrg2_core::ObjectData& data, srrg2_core::IdContext&)
{
  m_t = static_cast<double>(data.getDouble("t"));
  m_frame_id = static_cast<std::string>(data.getString("frame_id"));
}

}  // namespace msg

}  // namespace slam

}  // namespace inesctec_mrdt_slam_distmap_2d
