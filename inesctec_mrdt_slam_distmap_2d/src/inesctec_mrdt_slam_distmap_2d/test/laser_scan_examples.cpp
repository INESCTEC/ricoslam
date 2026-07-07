#include "inesctec_mrdt_slam_distmap_2d/test/laser_scan_examples.hpp"

#include <unistd.h>

#include <Eigen/StdVector>
#include <exception>
#include <iomanip>
#include <iostream>
#include <random>
#include <vector>

#include "inesctec_mrdt_slam_distmap_2d/geometry/geometry_2d.hpp"

namespace inesctec_mrdt_slam_distmap_2d
{

using namespace geometry;

namespace test
{

void init2DLaserScanMsgModel(slam::msg::LaserScan& msg, float angle_res,
                             float fov, float max_range)
{
  size_t num_rays;

  if (fov >= 2.0f * M_PIf32 - angle_res * 0.25f)
  {
    num_rays = static_cast<size_t>(std::round(2.0f * M_PIf32 / angle_res));
    angle_res = 2.0f * M_PIf32 / static_cast<float>(num_rays);
  }
  else
  {
    num_rays = static_cast<size_t>(std::round(fov / angle_res)) + 1;
    angle_res = fov / static_cast<float>(num_rays - 1);
  }

  msg.m_angle_min = -fov * 0.5f;
  msg.m_angle_inc = angle_res;
  msg.m_angle_max =
      msg.m_angle_min + static_cast<float>(num_rays - 1) * msg.m_angle_inc;

  msg.m_range_min = 0.f;
  msg.m_range_max = max_range;

  msg.m_ranges.resize(num_rays);
}

void getLaser2DLaserScanMsgRandom(slam::msg::LaserScan& msg, float angle_res,
                                  float fov, float max_range, std::mt19937& rng)
{
  init2DLaserScanMsgModel(msg, angle_res, fov, max_range);

  std::uniform_real_distribution<float> dist_urd(0, msg.m_range_max);

  for (size_t idx = 0; idx < msg.m_ranges.size(); idx++)
  {
    msg.m_ranges[idx] = dist_urd(rng);
  }
}

void getLaser2DLaserScanMsgCircle(slam::msg::LaserScan& msg,
                                  const Eigen::Isometry2f& pose, float radius,
                                  float angle_res, float fov, float max_range,
                                  std::mt19937& rng, float noise_std)
{
  init2DLaserScanMsgModel(msg, angle_res, fov, max_range);

  std::normal_distribution<float> noise(0.f, noise_std);

  auto process_pts_intersection =
      [&rng, &noise](slam::msg::LaserScan& msg, const Eigen::Vector3f& pose_v,
                     const Eigen::Vector2f& pt1, const Eigen::Vector2f& pt2,
                     int idx) -> void
  {
    Eigen::Vector2f ray_pt1, ray_pt2;

    ray_pt1 = pt1 - pose_v.head<2>();
    ray_pt2 = pt2 - pose_v.head<2>();

    float angle_ray_pt1 =
        wrapToPi(atan2(ray_pt1.y(), ray_pt1.x()) - pose_v.z());
    float angle_ray_pt2 =
        wrapToPi(atan2(ray_pt2.y(), ray_pt2.x()) - pose_v.z());

    int idx_ray_pt1 = msg.angleToRayIdx(angle_ray_pt1);
    int idx_ray_pt2 = msg.angleToRayIdx(angle_ray_pt2);

    if ((idx_ray_pt1 == idx) && (idx_ray_pt2 == idx))
    {
      msg.m_ranges[idx] = std::min<float>(
          std::min<float>(
              sqrt(ray_pt1.x() * ray_pt1.x() + ray_pt1.y() * ray_pt1.y()) +
                  noise(rng),
              sqrt(ray_pt2.x() * ray_pt2.x() + ray_pt2.y() * ray_pt2.y()) +
                  noise(rng)),
          msg.m_range_max);
      return;
    }

    if ((idx_ray_pt1 == idx) ||
        (msg.is360() && idx_ray_pt1 == static_cast<int>(msg.m_ranges.size()) &&
         idx == 0))
    {
      msg.m_ranges[idx] = std::min<float>(
          sqrt(ray_pt1.x() * ray_pt1.x() + ray_pt1.y() * ray_pt1.y()) +
              noise(rng),
          msg.m_range_max);
      return;
    }

    if ((idx_ray_pt2 == idx) ||
        (msg.is360() && idx_ray_pt2 == static_cast<int>(msg.m_ranges.size()) &&
         idx == 0))
    {
      msg.m_ranges[idx] = std::min<float>(
          sqrt(ray_pt2.x() * ray_pt2.x() + ray_pt2.y() * ray_pt2.y()) +
              noise(rng),
          msg.m_range_max);
      return;
    }

    if ((idx_ray_pt1 != idx) && (idx_ray_pt2 != idx))
    {
      msg.m_ranges[idx] = msg.m_range_max;
      return;
    }
  };

  auto process_pt_tangent =
      [&rng, &noise](slam::msg::LaserScan& msg, const Eigen::Vector3f& pose_v,
                     const Eigen::Vector2f& pt, int idx) -> void
  {
    Eigen::Vector2f ray_pt;

    ray_pt = pt - pose_v.head<2>();

    float angle_ray_pt = wrapToPi(atan2(ray_pt.y(), ray_pt.x()) - pose_v.z());

    int idx_ray_pt = msg.angleToRayIdx(angle_ray_pt);

    if ((idx_ray_pt == idx) ||
        (msg.is360() && idx_ray_pt == static_cast<int>(msg.m_ranges.size()) &&
         idx == 0))
    {
      msg.m_ranges[idx] = std::min<float>(
          sqrt(ray_pt.x() * ray_pt.x() + ray_pt.y() * ray_pt.y()) + noise(rng),
          msg.m_range_max);

      return;
    }
    else
    {
      msg.m_ranges[idx] = msg.m_range_max;

      return;
    }
  };

  float ray;
  float angle;
  float c_angle;
  float s_angle;
  float line_m;
  float line_b;
  float intersect_a;
  float intersect_b;
  float intersect_c;
  float intersect_discrim;

  Eigen::Vector3f pose_v = t2v(pose);

  for (int idx = 0; idx < static_cast<int>(msg.m_ranges.size()); idx++)
  {
    ray = msg.m_angle_min + idx * msg.m_angle_inc;
    angle = pose_v.z() + ray;

    c_angle = cos(angle);
    s_angle = sin(angle);

    if (fabs(c_angle) < fabs(s_angle))  // line model: x = m * y + b
    {
      line_m = c_angle / s_angle;
      line_b = pose_v.x() - line_m * pose_v.y();

      intersect_a = 1 + line_m * line_m;
      intersect_b = 2 * line_b * line_m;
      intersect_c = line_b * line_b - radius * radius;

      intersect_discrim =
          intersect_b * intersect_b - 4 * intersect_a * intersect_c;

      if (intersect_discrim > 0)  // 2 intersection pts
      {
        Eigen::Vector2f pt1, pt2;

        pt1.y() = -intersect_b + sqrt(intersect_discrim);
        pt2.y() = -intersect_b - sqrt(intersect_discrim);

        pt1.y() = 0.5f * pt1.y() / intersect_a;
        pt2.y() = 0.5f * pt2.y() / intersect_a;

        pt1.x() = line_m * pt1.y() + line_b;
        pt2.x() = line_m * pt2.y() + line_b;

        process_pts_intersection(msg, pose_v, pt1, pt2, idx);
      }
      else if (intersect_discrim == 0)  // tangent to circle
      {
        Eigen::Vector2f pt;

        pt.y() = -0.5f * intersect_b / intersect_a;
        pt.x() = line_m * pt.y() + line_b;

        process_pt_tangent(msg, pose_v, pt, idx);
      }
      else  // no intersection
      {
        msg.m_ranges[idx] = msg.m_range_max;
      }
    }
    else  // line model: y = m * x + b
    {
      line_m = s_angle / c_angle;
      line_b = pose_v.y() - line_m * pose_v.x();

      intersect_a = 1 + line_m * line_m;
      intersect_b = 2 * line_b * line_m;
      intersect_c = line_b * line_b - radius * radius;

      intersect_discrim =
          intersect_b * intersect_b - 4 * intersect_a * intersect_c;

      if (intersect_discrim > 0)  // 2 intersection pts
      {
        Eigen::Vector2f pt1, pt2;

        pt1.x() = -intersect_b + sqrt(intersect_discrim);
        pt2.x() = -intersect_b - sqrt(intersect_discrim);

        pt1.x() = 0.5f * pt1.x() / intersect_a;
        pt2.x() = 0.5f * pt2.x() / intersect_a;

        pt1.y() = line_m * pt1.x() + line_b;
        pt2.y() = line_m * pt2.x() + line_b;

        process_pts_intersection(msg, pose_v, pt1, pt2, idx);
      }
      else if (intersect_discrim == 0)  // tangent to circle
      {
        Eigen::Vector2f pt;

        pt.x() = -0.5f * intersect_b / intersect_a;
        pt.y() = line_m * pt.x() + line_b;

        process_pt_tangent(msg, pose_v, pt, idx);
      }
      else  // no intersection
      {
        msg.m_ranges[idx] = msg.m_range_max;
      }
    }
  }
}

void getLaserScanMsgSquare(slam::msg::LaserScan& msg,
                           const Eigen::Isometry2f& pose, float angle_res,
                           float fov, float max_range, float side,
                           std::mt19937& rng, float noise_std)
{
  using Point2DVector =
      std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>;

  constexpr float kCmpError = 1e-5f;

  init2DLaserScanMsgModel(msg, angle_res, fov, max_range);

  std::normal_distribution<float> noise(0.f, noise_std);

  auto test_pt_in_square = [side](const Eigen::Vector2f& pt,
                                  float cpm_error = 0.f) -> bool
  {
    float pt_x_abs = fabs(pt.x());
    float pt_y_abs = fabs(pt.y());

    if ((fabs(pt_x_abs - side) < cpm_error) ||
        (fabs(pt_y_abs - side) < cpm_error))
    {
      if ((fabs(pt_x_abs) < side + cpm_error) &&
          (fabs(pt_y_abs) < side + cpm_error))
      {
        return true;
      }
    }

    return false;
  };

  auto process_pts_solutions = [&rng, &noise, &test_pt_in_square](
                                   slam::msg::LaserScan& msg,
                                   const Eigen::Vector3f& pose_v,
                                   const Point2DVector& pts, int idx) -> void
  {
    msg.m_ranges[idx] = msg.m_range_max;

    for (const auto& pt : pts)
    {
      if (!test_pt_in_square(pt, kCmpError))
      {
        continue;
      }

      Eigen::Vector2f ray_pt;

      ray_pt = pt - pose_v.head<2>();

      float angle_ray_pt = wrapToPi(atan2(ray_pt.y(), ray_pt.x()) - pose_v.z());

      int idx_ray_pt = msg.angleToRayIdx(angle_ray_pt);

      if ((idx_ray_pt != idx) &&
          !(msg.is360() && idx == 0 &&
            idx_ray_pt == static_cast<int>(msg.m_ranges.size())))
      {
        continue;
      }

      msg.m_ranges[idx] = std::min<float>(
          std::min<float>(
              sqrt(ray_pt.x() * ray_pt.x() + ray_pt.y() * ray_pt.y()) +
                  noise(rng),
              msg.m_ranges[idx]),
          msg.m_range_max);
    }
  };

  float ray;
  float angle;
  float c_angle;
  float s_angle;
  float line_m;
  float line_b;

  Eigen::Vector3f pose_v = t2v(pose);

  Point2DVector pt_solutions;

  for (int idx = 0; idx < static_cast<int>(msg.m_ranges.size()); idx++)
  {
    ray = msg.m_angle_min + idx * msg.m_angle_inc;
    angle = pose_v.z() + ray;

    c_angle = cos(angle);
    s_angle = sin(angle);

    pt_solutions.clear();

    if (fabs(c_angle) < fabs(s_angle))  // line model: x = m * y + b
    {
      line_m = c_angle / s_angle;
      line_b = pose_v.x() - line_m * pose_v.y();

      pt_solutions.emplace_back(Eigen::Vector2f(line_m * side + line_b, side));
      pt_solutions.emplace_back(
          Eigen::Vector2f(-line_m * side + line_b, -side));

      if (fabs(line_m) > kCmpError)
      {
        pt_solutions.emplace_back(
            Eigen::Vector2f(side, (side - line_b) / line_m));
        pt_solutions.emplace_back(
            Eigen::Vector2f(-side, (-side - line_b) / line_m));
      }

      process_pts_solutions(msg, pose_v, pt_solutions, idx);
    }
    else  // line model: y = m * x + b
    {
      line_m = s_angle / c_angle;
      line_b = pose_v.y() - line_m * pose_v.x();

      pt_solutions.emplace_back(Eigen::Vector2f(side, line_m * side + line_b));
      pt_solutions.emplace_back(
          Eigen::Vector2f(-side, -line_m * side + line_b));

      if (fabs(line_m) > kCmpError)
      {
        pt_solutions.emplace_back(
            Eigen::Vector2f((side - line_b) / line_m, side));
        pt_solutions.emplace_back(
            Eigen::Vector2f((-side - line_b) / line_m, -side));
      }

      process_pts_solutions(msg, pose_v, pt_solutions, idx);
    }
  }
}

void getLaserScanMsgCorridor(slam::msg::LaserScan& msg,
                             const Eigen::Isometry2f& pose, float angle_res,
                             float fov, float max_range, float width,
                             float length, std::mt19937& rng, bool rotate,
                             float noise_std)
{
  using Point2DVector =
      std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>;

  constexpr float kCmpError = 1e-5f;

  init2DLaserScanMsgModel(msg, angle_res, fov, max_range);

  std::normal_distribution<float> noise(0.f, noise_std);

  auto test_pt_in_corridor = [width, length, rotate](
                                 const Eigen::Vector2f& pt,
                                 float cpm_error = 0.f) -> bool
  {
    float pt_x_abs = fabs(pt.x());
    float pt_y_abs = fabs(pt.y());

    if ((!rotate && (fabs(pt_y_abs - width) < cpm_error) &&
         (fabs(pt_x_abs) < length + cpm_error)) ||
        (rotate && (fabs(pt_x_abs - width) < cpm_error) &&
         (fabs(pt_y_abs) < length + cpm_error)))
    {
      return true;
    }

    return false;
  };

  auto process_pts_solutions = [&rng, &noise, &test_pt_in_corridor](
                                   slam::msg::LaserScan& msg,
                                   const Eigen::Vector3f& pose_v,
                                   const Point2DVector& pts, int idx) -> void
  {
    msg.m_ranges[idx] = msg.m_range_max;

    for (const auto& pt : pts)
    {
      if (!test_pt_in_corridor(pt, kCmpError))
      {
        continue;
      }

      Eigen::Vector2f ray_pt;

      ray_pt = pt - pose_v.head<2>();

      float angle_ray_pt = wrapToPi(atan2(ray_pt.y(), ray_pt.x()) - pose_v.z());

      int idx_ray_pt = msg.angleToRayIdx(angle_ray_pt);

      if (idx_ray_pt != idx)
      {
        continue;
      }

      msg.m_ranges[idx] = std::min<float>(
          std::min<float>(
              sqrt(ray_pt.x() * ray_pt.x() + ray_pt.y() * ray_pt.y()) +
                  noise(rng),
              msg.m_ranges[idx]),
          msg.m_range_max);
    }
  };

  float ray;
  float angle;
  float c_angle;
  float s_angle;
  float line_m;
  float line_b;

  Eigen::Vector3f pose_v = t2v(pose);

  Point2DVector pt_solutions;

  for (int idx = 0; idx < static_cast<int>(msg.m_ranges.size()); idx++)
  {
    ray = msg.m_angle_min + idx * msg.m_angle_inc;
    angle = pose_v.z() + ray;

    c_angle = cos(angle);
    s_angle = sin(angle);

    pt_solutions.clear();

    if (fabs(c_angle) < fabs(s_angle))  // line model: x = m * y + b
    {
      line_m = c_angle / s_angle;
      line_b = pose_v.x() - line_m * pose_v.y();

      if (!rotate)  // corridor: -length <= x <= length , y = +/- width
      {
        pt_solutions.emplace_back(
            Eigen::Vector2f(line_m * width + line_b, width));
        pt_solutions.emplace_back(
            Eigen::Vector2f(-line_m * width + line_b, -width));
      }
      else  // corridor: x = +/- width , -length <= y <= length
      {
        if (fabs(line_m) > kCmpError)
        {
          pt_solutions.emplace_back(
              Eigen::Vector2f(width, (width - line_b) / line_m));
          pt_solutions.emplace_back(
              Eigen::Vector2f(-width, (-width - line_b) / line_m));
        }
      }

      process_pts_solutions(msg, pose_v, pt_solutions, idx);
    }
    else  // line model: y = m * x + b
    {
      line_m = s_angle / c_angle;
      line_b = pose_v.y() - line_m * pose_v.x();

      if (!rotate)  // corridor: -length <= x <= length , y = +/- width
      {
        if (fabs(line_m) > kCmpError)
        {
          pt_solutions.emplace_back(
              Eigen::Vector2f((width - line_b) / line_m, width));
          pt_solutions.emplace_back(
              Eigen::Vector2f((-width - line_b) / line_m, -width));
        }
      }
      else  // corridor: x = +/- width , -length <= y <= length
      {
        pt_solutions.emplace_back(
            Eigen::Vector2f(width, line_m * width + line_b));
        pt_solutions.emplace_back(
            Eigen::Vector2f(-width, -line_m * width + line_b));
      }

      process_pts_solutions(msg, pose_v, pt_solutions, idx);
    }
  }
}

}  // namespace test

}  // namespace inesctec_mrdt_slam_distmap_2d
