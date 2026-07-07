#pragma once

#include "inesctec_mrdt_slam_distmap_2d/geometry/defs.hpp"

namespace inesctec_mrdt_slam_distmap_2d
{

namespace geometry
{

/**
 * @brief compute 2D rotation matrix from an angle
 * @tparam Scalar_ derived type (float, double)
 * @param[in] a angle (rad)
 * @return Matrix2_<Scalar_> 2D rotation matrix
 */
template <typename Scalar_>
inline Matrix2_<Scalar_> a2r(const Scalar_& a)
{
  Scalar_ c = cos(a);
  Scalar_ s = sin(a);

  Matrix2_<Scalar_> R;
  R << c, -s, s, c;

  return R;
}

/**
 * @brief compute angle from a 2D rotation matrix
 * @tparam Scalar_ derived type (float, double)
 * @param[in] R 2D rotation matrix
 * @return Scalar_ angle (rad)
 */
template <typename Scalar_>
inline Scalar_ r2a(const Matrix2_<Scalar_>& R)
{
  return atan2(R(1, 0), R(0, 0));
}

/**
 * @brief compute SE(2) transformation matrix from a pose
 *        (euclidean representation)
 * @tparam Scalar_ derived type (float, double)
 * @param[in] v pose (euclidean representation: x, y, th)
 * @return Isometry2_<Scalar_> SE(2) transformation matrix
 */
template <typename Scalar_>
inline Isometry2_<Scalar_> v2t(const Vector3_<Scalar_>& v)
{
  Isometry2_<Scalar_> T;

  T.translation() = v.template head<2>();
  T.linear() = a2r(v(2));

  return T;
}

/**
 * @brief compute the euclidean representation (pose) from an SE(2)
 *        transformation matrix
 * @tparam Scalar_ derived type (float, double)
 * @param[in] T SE(2) transformation matrix
 * @return Vector3_<Scalar_> pose (euclidean representation: x, y, th)
 */
template <typename Scalar_>
inline Vector3_<Scalar_> t2v(const Isometry2_<Scalar_>& T)
{
  Vector3_<Scalar_> v;

  v.template head<2>() = T.translation();

  const Matrix2_<Scalar_>& R = T.linear();
  v(2) = r2a(R);

  return v;
}

/**
 * @brief compute the 3D quaternion representation from an angle (yaw)
 *
 * yaw = rotation around z-axis
 * q_yaw = cos( th_yaw / 2 ) + k * sin( th_yaw / 2 )
 *
 * Source:
 * https://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToQuaternion/index.htm
 *
 * @tparam Scalar_ derived type (float, double)
 * @param[in] a angle (rad)
 * @return Quaternion_<Scalar_> 3D quaternion
 */
template <typename Scalar_>
inline Quaternion_<Scalar_> a2q(const Scalar_& a)
{
  Scalar_ yaw_2 = Scalar_(a) * Scalar_(0.5);
  Scalar_ c = cos(yaw_2);
  Scalar_ s = sin(yaw_2);

  return Quaternion_<Scalar_>(c, Scalar_(0.0), Scalar_(0.0), s);
}

/**
 * @brief compute heading angle (yaw) from a 3D quaternion
 * @tparam Scalar_ derived type (float, double)
 * @param[in] q 3D quaternion
 * @return Scalar_ angle (rad)
 */
template <typename Scalar_>
inline Scalar_ q2a(const Quaternion_<Scalar_>& q)
{
  return std::atan2(
      Scalar_(2.0) * (q.w() * q.z() + q.x() * q.y()),
      Scalar_(1.0) - Scalar_(2.0) * (q.y() * q.y() + q.z() * q.z()));
}

/**
 * @brief compute heading angle (yaw) from a quaternion assuming a 2D rotation
 * @tparam Scalar_ derived type (float, double)
 * @param[in] q 3D quaternion (assumed to represent 2D rotation)
 * @return Scalar_ angle (rad)
 */
template <typename Scalar_>
inline Scalar_ q2D2a(const Quaternion_<Scalar_>& q)
{
  return Scalar_(2.0) * std::atan2(q.z(), q.w());
}

}  // namespace geometry

}  // namespace inesctec_mrdt_slam_distmap_2d
