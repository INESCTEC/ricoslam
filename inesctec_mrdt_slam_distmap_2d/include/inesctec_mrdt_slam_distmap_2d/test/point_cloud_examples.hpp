#pragma once

#include <unistd.h>

#include <random>

#include "inesctec_mrdt_slam_distmap_2d/pcl/point_cloud.hpp"
#include "inesctec_mrdt_slam_distmap_2d/pcl/point_types.hpp"

namespace inesctec_mrdt_slam_distmap_2d
{

namespace test
{

/**
 * @brief get a 2D point cloud with random points in the radial direction,
 *        depending on the parameters of a typic laser scanner
 *        (angle resolution, FOV, maximum range)
 * @tparam PointCloudType point cloud type
 * @tparam Scalar scalar type
 * @param[out] pc point cloud (m)
 * @param[in] angle_res angle resolution of the laser scanner (rad)
 * @param[in] fov Field-of-View (FOV) of the laser scanner (rad)
 * @param[in] max_range maximum range of the laser scanner (m)
 * @param[in,out] rng random number generator (assuming initialized seed)
 */
template <typename PointCloudType,
          typename Scalar = typename PointCloudType::PointType::ComponentsType>
void getLaser2DPointCloudRandom(PointCloudType& pc, Scalar angle_res,
                                Scalar fov, Scalar max_range, std::mt19937& rng)
{
  using PointType = typename PointCloudType::PointType;

  std::uniform_real_distribution<float> dist_urd(0, max_range);

  size_t num_rays;
  Scalar new_angle_res;

  if (fov >= static_cast<Scalar>(2.0 * M_PIf64))
  {
    num_rays = static_cast<size_t>(std::round(fov / angle_res));
    new_angle_res = fov / num_rays;
  }
  else
  {
    num_rays = static_cast<size_t>(std::round(fov / angle_res)) + 1;
    new_angle_res = fov / (num_rays - 1);
  }

  pc.clear();

  for (size_t idx = 0; idx < num_rays; idx++)
  {
    Scalar ray = -fov * Scalar(0.5) + idx * new_angle_res;

    Scalar dist = dist_urd(rng);

    if (dist <= max_range)
    {
      PointType pt;

      pt.coordinates().x() = dist * cos(ray);
      pt.coordinates().y() = dist * sin(ray);

      pc.emplace_back(pt);
    }
  }
}  // void getLaser2DPointCloudRandom<PointCloudType, Scalar>(PointCloudType&,
   // Scalar, Scalar, Scalar, std::mt19937&)

/**
 * @brief get a 2D circular point cloud depending on the parameters of a typic
 *        laser scanner (angle resolution, FOV, maximum range)
 * @tparam PointCloudType point cloud type
 * @tparam Scalar scalar type
 * @param[out] pc point cloud (m)
 * @param[in] radius circle radius (m)
 * @param[in] angle_res angle resolution of the laser scanner (rad)
 * @param[in] fov Field-of-View (FOV) of the laser scanner (rad)
 * @param[in] max_range maximum range of the laser scanner (m)
 * @param[in,out] rng random number generator (assuming initialized seed)
 * @param[in] noise_std standard deviation of the noise on the point cloud (m)
 */
template <typename PointCloudType,
          typename Scalar = typename PointCloudType::PointType::ComponentsType>
void getLaser2DPointCloudCircle(PointCloudType& pc, Scalar radius,
                                Scalar angle_res, Scalar fov, Scalar max_range,
                                std::mt19937& rng,
                                Scalar noise_std = Scalar(0.0))
{
  using PointType = typename PointCloudType::PointType;

  std::normal_distribution<Scalar> noise(Scalar(0.0), noise_std);

  size_t num_rays;
  Scalar new_angle_res;

  if (fov >= static_cast<Scalar>(2.0 * M_PIf64))
  {
    num_rays = static_cast<size_t>(std::round(fov / angle_res));
    new_angle_res = fov / num_rays;
  }
  else
  {
    num_rays = static_cast<size_t>(std::round(fov / angle_res)) + 1;
    new_angle_res = fov / (num_rays - 1);
  }

  pc.clear();

  for (size_t idx = 0; idx < num_rays; idx++)
  {
    Scalar ray = -fov * Scalar(0.5) + idx * new_angle_res;

    if (radius <= max_range)
    {
      PointType pt;

      pt.coordinates().x() = (radius + noise(rng)) * cos(ray);
      pt.coordinates().y() = (radius + noise(rng)) * sin(ray);

      pc.emplace_back(pt);
    }
  }
}  // void getLaser2DPointCloudCircle<PointCloudType, Scalar>(PointCloudType&,
   // Scalar, Scalar, Scalar, Scalar, std::mt19937&, Scalar = Scalar(0.0))

/**
 * @brief get a 2D point cloud in the shape of a square, depending on the side
 *        length and points density per side
 * @tparam PointCloudType point cloud type
 * @tparam Scalar scalar type
 * @param[out] pc point cloud (m)
 * @param[in] side square side length (m)
 * @param[in] num_pts_side number of points of the point cloud per side
 * @param[in,out] rng random number generator (assuming initialized seed)
 * @param[in] noise_std standard deviation of the noise on the point cloud (m)
 */
template <typename PointCloudType,
          typename Scalar = typename PointCloudType::PointType::ComponentsType>
void getPointCloudSquare(PointCloudType& pc, Scalar side, int num_pts_side,
                         std::mt19937& rng, Scalar noise_std = Scalar(0.0))
{
  using PointType = typename PointCloudType::PointType;

  std::normal_distribution<Scalar> noise(Scalar(0.0), noise_std);

  Scalar res = side / num_pts_side;

  PointType pt;
  PointType pt_tmp;

  pc.clear();

  pt.coordinates().x() = side * Scalar(0.5);

  for (pt.coordinates().y() = -side * Scalar(0.5);
       pt.coordinates().y() < (side - res) * Scalar(0.5);
       pt.coordinates().y() += res)
  {
    pt_tmp.coordinates().x() = pt.coordinates().x() + noise(rng);
    pt_tmp.coordinates().y() = pt.coordinates().y();

    pc.emplace_back(pt_tmp);
  }

  pt.coordinates().y() = side * Scalar(0.5);

  for (pt.coordinates().x() = side * Scalar(0.5);
       pt.coordinates().x() > (-side + res) * Scalar(0.5);
       pt.coordinates().x() -= res)
  {
    pt_tmp.coordinates().x() = pt.coordinates().x();
    pt_tmp.coordinates().y() = pt.coordinates().y() + noise(rng);

    pc.emplace_back(pt_tmp);
  }

  pt.coordinates().x() = -side * Scalar(0.5);

  for (pt.coordinates().y() = side * Scalar(0.5);
       pt.coordinates().y() > (-side + res) * Scalar(0.5);
       pt.coordinates().y() -= res)
  {
    pt_tmp.coordinates().x() = pt.coordinates().x() + noise(rng);
    pt_tmp.coordinates().y() = pt.coordinates().y();

    pc.emplace_back(pt_tmp);
  }

  pt.coordinates().y() = -side * Scalar(0.5);

  for (pt.coordinates().x() = -side * Scalar(0.5);
       pt.coordinates().x() < (side - res) * Scalar(0.5);
       pt.coordinates().x() += res)
  {
    pt_tmp.coordinates().x() = pt.coordinates().x();
    pt_tmp.coordinates().y() = pt.coordinates().y() + noise(rng);

    pc.emplace_back(pt_tmp);
  }
}  // void getPointCloudSquare<PointCloudType, Scalar>(PointCloudType&, Scalar,
   // int, std::mt19937&, Scalar = Scalar(0.0))

/**
 * @brief get a 2D corridor-like point cloud, depending on the corridor width
 *        and length, direction (along x or y axes), and points density per side
 * @tparam PointCloudType point cloud type
 * @tparam Scalar scalar type
 * @param[out] pc point cloud (m)
 * @param[in] width corridor width (m)
 * @param[in] length corridor length (m)
 * @param[in] num_pts_side number of points of the point cloud per corridor side
 * @param[in,out] rng random number generator (assuming initialized seed)
 * @param[in] rotate enable rotation (default direction is and x-axis corridor)
 * @param[in] noise_std standard deviation of the noise on the point cloud (m)
 */
template <typename PointCloudType,
          typename Scalar = typename PointCloudType::PointType::ComponentsType>
void getPointCloudCorridor(PointCloudType& pc, Scalar width, Scalar length,
                           int num_pts_side, std::mt19937& rng,
                           bool rotate = false, Scalar noise_std = Scalar(0.0))
{
  using PointType = typename PointCloudType::PointType;

  std::normal_distribution<Scalar> noise(Scalar(0.0), noise_std);

  Scalar res = length / num_pts_side;

  PointType pt;
  PointType pt_tmp;

  pc.clear();

  if (!rotate)
  {
    pt.coordinates().y() = width * Scalar(0.5);

    for (pt.coordinates().x() = length * Scalar(0.5);
         pt.coordinates().x() >= (-length - res) * Scalar(0.5);
         pt.coordinates().x() -= res)
    {
      pt_tmp.coordinates().x() = pt.coordinates().x();
      pt_tmp.coordinates().y() = pt.coordinates().y() + noise(rng);

      pc.emplace_back(pt_tmp);
    }

    pt.coordinates().y() = -width * Scalar(0.5);

    for (pt.coordinates().x() = -length * Scalar(0.5);
         pt.coordinates().x() <= (length + res) * Scalar(0.5);
         pt.coordinates().x() += res)
    {
      pt_tmp.coordinates().x() = pt.coordinates().x();
      pt_tmp.coordinates().y() = pt.coordinates().y() + noise(rng);

      pc.emplace_back(pt_tmp);
    }
  }
  else
  {
    pt.coordinates().x() = width * Scalar(0.5);

    for (pt.coordinates().y() = -length * Scalar(0.5);
         pt.coordinates().y() < (length + res) * Scalar(0.5);
         pt.coordinates().y() += res)
    {
      pt_tmp.coordinates().x() = pt.coordinates().x() + noise(rng);
      pt_tmp.coordinates().y() = pt.coordinates().y();

      pc.emplace_back(pt_tmp);
    }

    pt.coordinates().x() = -width * Scalar(0.5);

    for (pt.coordinates().y() = length * Scalar(0.5);
         pt.coordinates().y() > (-length - res) * Scalar(0.5);
         pt.coordinates().y() -= res)
    {
      pt_tmp.coordinates().x() = pt.coordinates().x() + noise(rng);
      pt_tmp.coordinates().y() = pt.coordinates().y();

      pc.emplace_back(pt_tmp);
    }
  }
}  // void getPointCloudCorridor<PointCloudType, Scalar>(PointCloudType&,
   // Scalar, Scalar, int, std::mt19937&, bool = false, Scalar = Scalar(0.0))

}  // namespace test

}  // namespace inesctec_mrdt_slam_distmap_2d
