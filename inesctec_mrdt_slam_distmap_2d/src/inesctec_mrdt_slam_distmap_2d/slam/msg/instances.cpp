#include "inesctec_mrdt_slam_distmap_2d/slam/msg/instances.hpp"

#include <srrg2_core/srrg_boss/serializable.h>

#include "inesctec_mrdt_slam_distmap_2d/slam/msg/header.hpp"
#include "inesctec_mrdt_slam_distmap_2d/slam/msg/laser_scan.hpp"

namespace inesctec_mrdt_slam_distmap_2d
{

namespace slam
{

namespace msg
{

void inesctec_mrdt_slam_distmap_2d_slam_msg_registerTypes()
{
  BOSS_REGISTER_CLASS(Header);
  BOSS_REGISTER_CLASS(LaserScan);
}

}  // namespace msg

}  // namespace slam

}  // namespace inesctec_mrdt_slam_distmap_2d
