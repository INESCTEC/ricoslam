#pragma once

#include <random>

#include "inesctec_mrdt_slam_distmap_2d/slam/msg/laser_scan.hpp"

namespace inesctec_mrdt_slam_distmap_2d
{

namespace test
{

/**
 * @brief get a 2D point cloud with random points in the radial direction,
 *        depending on the parameters of a typic laser scanner
 *        (angle resolution, FOV, maximum range)
 * @param[out] msg laser scan message
 * @param[in] angle_res angle resolution of the laser scanner (rad)
 * @param[in] fov Field-of-View (FOV) of the laser scanner (rad)
 * @param[in] max_range maximum range of the laser scanner (m)
 * @param[in,out] rng random number generator (assuming initialized seed)
 */
void getLaser2DLaserScanMsgRandom(slam::msg::LaserScan& msg, float angle_res,
                                  float fov, float max_range,
                                  std::mt19937& rng);

/**
 * @brief get a 2D circular laser scan message depending on the parameters of a
 *        typic laser scanner (angle resolution, FOV, maximum range) and local
 *        SE2 pose transformation
 * @note just implements a simple random noise on the distance, not on the angle
 * @param[out] msg laser scan message
 * @param[in] pose SE2 laser scan pose w.r.t. local coordinate frame
 * @param[in] radius circle radius (m)
 * @param[in] angle_res angle resolution of the laser scanner (rad)
 * @param[in] fov Field-of-View (FOV) of the laser scanner (rad)
 * @param[in] max_range maximum range of the laser scanner (m)
 * @param[in,out] rng random number generator (assuming initialized seed)
 * @param[in] noise_std standard deviation of the noise on the point cloud (m)
 */
void getLaser2DLaserScanMsgCircle(slam::msg::LaserScan& msg,
                                  const Eigen::Isometry2f& pose, float radius,
                                  float angle_res, float fov, float max_range,
                                  std::mt19937& rng,
                                  float noise_std = float(0.0));

/**
 * @brief get a 2D laser scan message in the shape of a square, depending on the
 *        local SE2 pose transformation, laser scanner model, side length and
 *        points density per side
 * @note just implements a simple random noise on the distance, not on the angle
 * @param[out] msg laser scan message
 * @param[in] pose SE2 laser scan pose w.r.t. local coordinate frame
 * @param[in] angle_res angle resolution of the laser scanner (rad)
 * @param[in] fov Field-of-View (FOV) of the laser scanner (rad)
 * @param[in] max_range maximum range of the laser scanner (m)
 * @param[in] side square side length (m)
 * @param[in,out] rng random number generator (assuming initialized seed)
 * @param[in] noise_std standard deviation of the noise on the point cloud (m)
 */
void getLaserScanMsgSquare(slam::msg::LaserScan& msg,
                           const Eigen::Isometry2f& pose, float angle_res,
                           float fov, float max_range, float side,
                           std::mt19937& rng, float noise_std = float(0.0));

/**
 * @brief get a 2D corridor-like laser scan message, depending on the local SE2
 *        pose transformation, laser scanner model, corridor width and length,
 *        direction (along x or y axes), and points density per side
 * @note just implements a simple random noise on the distance, not on the angle
 * @param[out] msg laser scan message
 * @param[in] pose SE2 laser scan pose w.r.t. local coordinate frame
 * @param[in] angle_res angle resolution of the laser scanner (rad)
 * @param[in] fov Field-of-View (FOV) of the laser scanner (rad)
 * @param[in] max_range maximum range of the laser scanner (m)
 * @param[in] width corridor width (m)
 * @param[in] length corridor length (m)
 * @param[in,out] rng random number generator (assuming initialized seed)
 * @param[in] rotate enable rotation (default direction is and x-axis corridor)
 * @param[in] noise_std standard deviation of the noise on the point cloud (m)
 */
void getLaserScanMsgCorridor(slam::msg::LaserScan& msg,
                             const Eigen::Isometry2f& pose, float angle_res,
                             float fov, float max_range, float width,
                             float length, std::mt19937& rng,
                             bool rotate = false, float noise_std = float(0.0));

}  // namespace test

}  // namespace inesctec_mrdt_slam_distmap_2d
