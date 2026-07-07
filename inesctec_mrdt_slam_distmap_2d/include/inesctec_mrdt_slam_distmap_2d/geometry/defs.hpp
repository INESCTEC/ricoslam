#pragma once

#include <cmath>

// Eigen
#include <Eigen/Geometry>

namespace inesctec_mrdt_slam_distmap_2d
{

namespace geometry
{

#ifndef M_PIf32
#define M_PIf32 3.141592653589793238462643383279502884f
#endif

#ifndef M_PI_2f32
#define M_PI_2f32 1.570796326794896619231321691639751442f
#endif

#ifndef M_PIf64
#define M_PIf64 3.141592653589793238462643383279502884
#endif

#ifndef M_PI_2f64
#define M_PI_2f64 1.570796326794896619231321691639751442
#endif

//! dynamic vector template
template <typename Scalar_>
using VectorX_ = Eigen::Matrix<Scalar_, Eigen::Dynamic, 1>;

//! static vector template
template <typename Scalar_, int Dim_>
using VectorN_ = Eigen::Matrix<Scalar_, Dim_, 1>;

//! dynamic matrix template
template <typename Scalar_>
using MatrixX_ = Eigen::Matrix<Scalar_, Eigen::Dynamic, Eigen::Dynamic>;

//! static square matrix template
template <typename Scalar_, int Dim_>
using MatrixN_ = Eigen::Matrix<Scalar_, Dim_, Dim_>;

//! static vector templates definitions...
template <typename Scalar_>
using Vector0_ = VectorN_<Scalar_, 0>;
template <typename Scalar_>
using Vector1_ = VectorN_<Scalar_, 1>;
template <typename Scalar_>
using Vector2_ = VectorN_<Scalar_, 2>;
template <typename Scalar_>
using Vector3_ = VectorN_<Scalar_, 3>;
template <typename Scalar_>
using Vector4_ = VectorN_<Scalar_, 4>;
template <typename Scalar_>
using Vector5_ = VectorN_<Scalar_, 5>;
template <typename Scalar_>
using Vector6_ = VectorN_<Scalar_, 6>;
template <typename Scalar_>
using Vector7_ = VectorN_<Scalar_, 7>;
template <typename Scalar_>
using Vector8_ = VectorN_<Scalar_, 8>;
template <typename Scalar_>
using Vector9_ = VectorN_<Scalar_, 9>;
template <typename Scalar_>
using Vector10_ = VectorN_<Scalar_, 10>;
template <typename Scalar_>
using Vector11_ = VectorN_<Scalar_, 11>;
template <typename Scalar_>
using Vector12_ = VectorN_<Scalar_, 12>;

//! static square matrices templates definitions...
template <typename Scalar_>
using Matrix0_ = MatrixN_<Scalar_, 0>;
template <typename Scalar_>
using Matrix1_ = MatrixN_<Scalar_, 1>;
template <typename Scalar_>
using Matrix2_ = MatrixN_<Scalar_, 2>;
template <typename Scalar_>
using Matrix3_ = MatrixN_<Scalar_, 3>;
template <typename Scalar_>
using Matrix4_ = MatrixN_<Scalar_, 4>;
template <typename Scalar_>
using Matrix5_ = MatrixN_<Scalar_, 5>;
template <typename Scalar_>
using Matrix6_ = MatrixN_<Scalar_, 6>;
template <typename Scalar_>
using Matrix7_ = MatrixN_<Scalar_, 7>;
template <typename Scalar_>
using Matrix8_ = MatrixN_<Scalar_, 8>;
template <typename Scalar_>
using Matrix9_ = MatrixN_<Scalar_, 9>;
template <typename Scalar_>
using Matrix10_ = MatrixN_<Scalar_, 10>;
template <typename Scalar_>
using Matrix11_ = MatrixN_<Scalar_, 11>;
template <typename Scalar_>
using Matrix12_ = MatrixN_<Scalar_, 12>;

//! affine transformation template definition
template <typename Scalar_, int Dim_>
using AffineN_ = Eigen::Transform<Scalar_, Dim_, Eigen::Affine>;

//! isometry transformation (linear part is a rotation) template definition
template <typename Scalar_, int Dim_>
using IsometryN_ = Eigen::Transform<Scalar_, Dim_, Eigen::Isometry>;

template <typename Scalar_>
using Isometry2_ = IsometryN_<Scalar_, 2>;
template <typename Scalar_>
using Isometry3_ = IsometryN_<Scalar_, 3>;

//! 2D rotation matrix template definition
template <typename Scalar_>
using Rotation2_ = Eigen::RotationBase<Scalar_, 2>;

//! quaternion template definition
template <typename Scalar_>
using Quaternion_ = Eigen::Quaternion<Scalar_>;

//! angle axis representation template definition
template <typename Scalar_>
using AngleAxis_ = Eigen::AngleAxis<Scalar_>;

using VectorXf = VectorX_<float>;
using VectorXd = VectorX_<double>;
using VectorXi = VectorX_<int>;

using Vector1f = VectorN_<float, 1>;
using Vector2f = VectorN_<float, 2>;
using Vector3f = VectorN_<float, 3>;
using Vector4f = VectorN_<float, 4>;
using Vector5f = VectorN_<float, 5>;
using Vector6f = VectorN_<float, 6>;

using Vector1d = VectorN_<double, 1>;
using Vector2d = VectorN_<double, 2>;
using Vector3d = VectorN_<double, 3>;
using Vector4d = VectorN_<double, 4>;
using Vector5d = VectorN_<double, 5>;
using Vector6d = VectorN_<double, 6>;

using Vector1i = VectorN_<int, 1>;
using Vector2i = VectorN_<int, 2>;
using Vector3i = VectorN_<int, 3>;
using Vector4i = VectorN_<int, 4>;
using Vector5i = VectorN_<int, 5>;
using Vector6i = VectorN_<int, 6>;

using MatrixXf = MatrixX_<float>;
using MatrixXd = MatrixX_<double>;
using MatrixXi = MatrixX_<int>;

using Matrix2f = MatrixN_<float, 2>;
using Matrix3f = MatrixN_<float, 3>;
using Matrix4f = MatrixN_<float, 4>;
using Matrix6f = MatrixN_<float, 6>;

using Matrix2d = MatrixN_<double, 2>;
using Matrix3d = MatrixN_<double, 3>;
using Matrix4d = MatrixN_<double, 4>;
using Matrix6d = MatrixN_<double, 6>;

using Matrix2i = MatrixN_<int, 2>;
using Matrix3i = MatrixN_<int, 3>;
using Matrix4i = MatrixN_<int, 4>;
using Matrix6i = MatrixN_<int, 6>;

using Isometry2f = Isometry2_<float>;
using Isometry2d = Isometry2_<double>;

using Isometry3f = Isometry3_<float>;
using Isometry3d = Isometry3_<double>;

using Rotation2f = Rotation2_<float>;
using Rotation2d = Rotation2_<double>;

using Quaternionf = Quaternion_<float>;
using Quaterniond = Quaternion_<double>;

using AngleAxisf = AngleAxis_<float>;
using AngleAxisd = AngleAxis_<double>;

/**
 * @brief convert degrees to radians (float)
 * @param[in] x angle (deg)
 * @return float angle (rad)
 */
inline float degToRad(float x) noexcept { return x * M_PIf32 / 180.0f; }

/**
 * @brief convert degrees to radians (double)
 * @param[in] x angle (deg)
 * @return double angle (rad)
 */
inline double degToRad(double x) noexcept { return x * M_PIf64 / 180.0; }

/**
 * @brief convert radians to degrees (float)
 * @param[in] x angle (rad)
 * @return float angle (deg)
 */
inline float radToDeg(float x) noexcept { return x * 180.0f / M_PIf32; }

/**
 * @brief convert radians to degrees (double)
 * @param[in] x angle (rad)
 * @return double angle (deg)
 */
inline double radToDeg(double x) noexcept { return x * 180.0 / M_PIf64; }

/**
 * @brief wrap an index to [0,range[
 * @param[in] idx index
 * @param[in] range range of the index (0..range-1)
 * @return int index inside the interval [0,range[
 */
inline int wrapToRange(int idx, int range)
{
  idx = idx % range;

  if (idx < 0)
  {
    idx += range;
  }

  return idx;
}

/**
 * @brief wrap an index to [range_inf,range_sup[
 * @param[in] idx index
 * @param[in] range_inf inferior limit of the range (inclusively)
 * @param[in] range_sup superior / upper limit of the range (exclusively)
 * @return int index inside the interval [range_inf,range_sup[
 */
inline int wrapToRange(int idx, int range_inf, int range_sup)
{
  idx = (idx - range_inf) % (range_sup - range_inf);

  if (idx < 0)
  {
    idx += range_sup - range_inf;
  }

  return idx + range_inf;
}

/**
 * @brief wrap an angle to [-pi,pi[
 * @param[in] x angle (rad)
 * @return float angle inside the interval [-pi,pi[
 */
inline float wrapToPi(float x) noexcept
{
  x = fmod(x + M_PIf32, 2.0f * M_PIf32);

  if (x < 0.0f)
  {
    x += 2.0f * M_PIf32;
  }

  return x - M_PIf32;
}

/**
 * @brief wrap an angle to [-pi,pi[
 * @param[in] x angle (rad)
 * @return double angle inside the interval [-pi,pi[
 */
inline double wrapToPi(double x) noexcept
{
  x = fmod(x + M_PIf64, 2.0 * M_PIf64);

  if (x < 0.0)
  {
    x += 2.0 * M_PIf64;
  }

  return x - M_PIf64;
}

/**
 * @brief wrap an angle to [0,2*pi[
 * @param[in] x angle (rad)
 * @return float angle inside the interval [0,2*pi[
 */
inline float wrapToTwoPi(float x) noexcept
{
  x = fmod(x, 2.0f * M_PIf32);

  if (x < 0.0f)
  {
    x += 2.0f * M_PIf32;
  }

  return x;
}

/**
 * @brief wrap an angle to [0,2*pi[
 * @param[in] x angle (rad)
 * @return double angle inside the interval [0,2*pi[
 */
inline double wrapToTwoPi(double x) noexcept
{
  x = fmod(x, 2.0 * M_PIf64);

  if (x < 0.0)
  {
    x += 2.0 * M_PIf64;
  }

  return x;
}

/**
 * @brief wrap an angle to [-180º,180º[
 * @param[in] x angle (º)
 * @return float angle inside the interval [-180º,180º[
 */
inline float wrapTo180(float x) noexcept
{
  x = fmod(x + 180.0f, 360.0f);

  if (x < 0.0f)
  {
    x += 360.0f;
  }

  return x - 180.0f;
}

/**
 * @brief wrap an angle to [-180º,180º[
 * @param[in] x angle (º)
 * @return double angle inside the interval [-180º,180º[
 */
inline double wrapTo180(double x) noexcept
{
  x = fmod(x + 180.0, 360.0);

  if (x < 0.0)
  {
    x += 360.0;
  }

  return x - 180.0;
}

/**
 * @brief wrap an angle to [0,360º[
 * @param[in] x angle (º)
 * @return float angle inside the interval [0,360º[
 */
inline float wrapTo360(float x) noexcept
{
  x = fmod(x, 360.0f);

  if (x < 0.0f)
  {
    x += 360.0f;
  }

  return x;
}

/**
 * @brief wrap an angle to [0,360º[
 * @param[in] x angle (º)
 * @return double angle inside the interval [0,360º[
 */
inline double wrapTo360(double x) noexcept
{
  x = fmod(x, 360.0);

  if (x < 0.0)
  {
    x += 360.0;
  }

  return x;
}

}  // namespace geometry

}  // namespace inesctec_mrdt_slam_distmap_2d
