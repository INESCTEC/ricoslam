#pragma once

#include <cmath>





namespace inesctec_mrdt_slam_2d_datasets
{



#ifndef M_PIf32
#define M_PIf32 3.141592653589793238462643383279502884f
#endif

#ifndef M_PIf64
#define M_PIf64 3.141592653589793238462643383279502884
#endif

/**
 * @brief convert degrees to radians (float)
 * @param[in] x angle (deg)
 * @return float angle (rad)
 */
inline float degToRad(float x) noexcept
{
  return x * M_PIf32 / 180.0f;
}

/**
 * @brief convert degrees to radians (double)
 * @param[in] x angle (deg)
 * @return double angle (rad)
 */
inline double degToRad(double x) noexcept
{
  return x * M_PIf64 / 180.0;
}

/**
 * @brief convert radians to degrees (float)
 * @param[in] x angle (rad)
 * @return float angle (deg)
 */
inline float radToDeg(float x) noexcept
{
  return x * 180.0f / M_PIf32;
}

/**
 * @brief convert radians to degrees (double)
 * @param[in] x angle (rad)
 * @return double angle (deg)
 */
inline double radToDeg(double x) noexcept
{
  return x * 180.0 / M_PIf64;
}

/**
 * @brief wrap an angle to [-pi,pi[
 * @param[in] x angle (rad)
 * @return float angle inside the interval [-pi,pi[
 */
inline float wrapToPi(float x) noexcept
{
  x = fmod(x +  M_PIf32, 2.0f *  M_PIf32);

  if (x < 0.0f)
  {
    x += 2.0f *  M_PIf32;
  }

  return x -  M_PIf32;
}

/**
 * @brief wrap an angle to [-pi,pi[
 * @param[in] x angle (rad)
 * @return double angle inside the interval [-pi,pi[
 */
inline double wrapToPi(double x) noexcept
{
  x = fmod(x +  M_PIf64, 2.0 *  M_PIf64);

  if (x < 0.0)
  {
    x += 2.0 *  M_PIf64;
  }

  return x -  M_PIf64;
}



} // namespace inesctec_mrdt_slam_2d_datasets
