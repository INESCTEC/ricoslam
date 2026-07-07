#include "inesctec_mrdt_slam_distmap_2d/data_structures/instances.hpp"

// SRRG Software
#include <srrg2_core/srrg_boss/serializable.h>
#include <srrg2_core/srrg_pcl/instances.h>
#include <srrg2_core/srrg_pcl/point_types_data.h>

#include "inesctec_mrdt_slam_distmap_2d/data_structures/all_types.hpp"

namespace srrg2_solver
{

using namespace srrg2_core;
using namespace inesctec_mrdt_slam_distmap_2d;
using namespace data_structures;

void inesctec_mrdt_slam_distmap_2d_data_structures_registerTypes()
{
  BOSS_REGISTER_CLASS(DistanceMapStaticFloat);
  BOSS_REGISTER_CLASS(DistanceMapStaticDouble);
  BOSS_REGISTER_CLASS(DistanceMapStaticWith2ndDerivFloat);
  BOSS_REGISTER_CLASS(DistanceMapStaticWith2ndDerivDouble);

  BOSS_REGISTER_CLASS(DistanceMapSparseStaticFloat);
  BOSS_REGISTER_CLASS(DistanceMapSparseStaticDouble);
  BOSS_REGISTER_CLASS(DistanceMapSparseStaticWith2ndDerivFloat);
  BOSS_REGISTER_CLASS(DistanceMapSparseStaticWith2ndDerivDouble);

  BOSS_REGISTER_CLASS(DistanceMapNanoStaticFloat);
  BOSS_REGISTER_CLASS(DistanceMapNanoStaticDouble);
}  // void inesctec_mrdt_slam_distmap_2d_data_structures_registerTypes()

}  // namespace srrg2_solver
