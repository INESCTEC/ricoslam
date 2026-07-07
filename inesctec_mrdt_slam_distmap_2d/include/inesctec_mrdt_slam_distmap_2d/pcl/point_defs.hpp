#pragma once

namespace inesctec_mrdt_slam_distmap_2d
{

namespace pcl
{

/**
 * @brief point status
 */
enum PointStatus
{
  kValid,       //!< valid point
  kOutOfRange,  //!< point out of range of the sensor's model
  kInvalid      //!< invalid point
};  // enum PointStatus

}  // namespace pcl

}  // namespace inesctec_mrdt_slam_distmap_2d
