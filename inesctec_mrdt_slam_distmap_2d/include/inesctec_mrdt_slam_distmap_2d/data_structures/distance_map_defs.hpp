#pragma once

namespace inesctec_mrdt_slam_distmap_2d
{

namespace data_structures
{

/**
 * @brief type of the distance map implementation (in terms of the lookup tables
 *        that it has implemented...)
 */
enum DistanceMapImplType
{
  kStandard,      //!< standard distance map formulation (dist + 1st derivative)
  kWith2ndDeriv,  //!< distance map with dist + 1st derivative + 2nd derivative
                  //!< over normalized 1st derivative
  kNNPointParent  //!< distance map with only the pointer to the nearest point
                  //!< from the cloud
};  // enum DistanceMapImplType

}  // namespace data_structures

}  // namespace inesctec_mrdt_slam_distmap_2d
