#pragma once

#include <Eigen/StdVector>
#include <inesctec_mrdt_slam_distmap_2d/geometry/geometry_2d.hpp>
#include <sstream>
#include <string>

namespace inesctec_mrdt_slam_distmap_2d
{
namespace data_structures
{

/**
 * @brief TUM pose struct (timestamp + 3D position + quaternion)
 */
struct TUMPose
{
 public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double m_t;  //!< timestamp (s)
  geometry::Vector3d m_pos =
      geometry::Vector3d::Zero();  //!< position (x,y,z) axis (m)
  geometry::Quaterniond m_q =
      geometry::Quaterniond::Identity();  //!< quaternion

 public:

  /**
   * @brief get TUM pose from a line string (e.g., retrieved from a .tum file)
   * @param[out] pose TUM pose
   * @param[in] line string with the format t px py pz qx qy qz qw
   * @return true line correctly parsed into a TUM pose
   * @return false otherwise
   */
  static inline bool getTUMPose(TUMPose& pose, const std::string& line)
  {
    std::istringstream iss(line);

    return static_cast<bool>(iss >> pose.m_t >> pose.m_pos.x() >>
                             pose.m_pos.y() >> pose.m_pos.z() >> pose.m_q.x() >>
                             pose.m_q.y() >> pose.m_q.z() >> pose.m_q.w());
  }
};

}  // namespace data_structures
}  // namespace inesctec_mrdt_slam_distmap_2d
