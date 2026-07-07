#include <gtest/gtest.h>

#include <inesctec_mrdt_slam_distmap_2d/geometry/defs.hpp>
#include <inesctec_mrdt_slam_distmap_2d/geometry/geometry_2d.hpp>
#include <inesctec_mrdt_slam_distmap_2d/test/laser_scan_examples.hpp>

namespace testing
{

using namespace inesctec_mrdt_slam_distmap_2d;
using namespace slam;
using namespace test;

#define VALIDATE_2DLASERSCANMSGMETA(msg, angle_min, angle_inc, angle_max, \
                                    range_min, range_max, num_rays)       \
  EXPECT_FLOAT_EQ(msg.m_angle_min, angle_min);                            \
  EXPECT_FLOAT_EQ(msg.m_angle_inc, angle_inc);                            \
  EXPECT_FLOAT_EQ(msg.m_angle_max, angle_max);                            \
  EXPECT_FLOAT_EQ(msg.m_range_min, range_min);                            \
  EXPECT_FLOAT_EQ(msg.m_range_max, range_max);                            \
  EXPECT_EQ(msg.m_ranges.size(), num_rays)

TEST(test_laser_scan_examples, init2DLaserScanMsgModel)
{
  using EstimateType = geometry::Isometry2f;

  std::random_device rd;
  std::mt19937 rng(rd());

  msg::LaserScan msg;

  float angle_res = geometry::degToRad(1.f);
  float fov = geometry::degToRad(270.f);
  float max_range = 10.f;

  getLaser2DLaserScanMsgRandom(msg, angle_res, fov, max_range, rng);
  VALIDATE_2DLASERSCANMSGMETA(msg, -fov * 0.5f, angle_res, fov * 0.5f, 0.f,
                              max_range, 271);

  getLaser2DLaserScanMsgCircle(msg, EstimateType::Identity(), 5.f, angle_res,
                               fov, max_range, rng);
  VALIDATE_2DLASERSCANMSGMETA(msg, -fov * 0.5f, angle_res, fov * 0.5f, 0.f,
                              max_range, 271);

  getLaserScanMsgSquare(msg, EstimateType::Identity(), angle_res, fov,
                        max_range, 5.f, rng);
  VALIDATE_2DLASERSCANMSGMETA(msg, -fov * 0.5f, angle_res, fov * 0.5f, 0.f,
                              max_range, 271);

  angle_res = geometry::degToRad(0.5f);
  fov = geometry::degToRad(360.f);

  getLaser2DLaserScanMsgRandom(msg, angle_res, fov, max_range, rng);
  VALIDATE_2DLASERSCANMSGMETA(msg, -fov * 0.5f, angle_res,
                              fov * 0.5f - angle_res, 0.f, max_range, 720);
  EXPECT_TRUE(msg.is360());

  getLaser2DLaserScanMsgCircle(msg, EstimateType::Identity(), 5.f, angle_res,
                               fov, max_range, rng);
  VALIDATE_2DLASERSCANMSGMETA(msg, -fov * 0.5f, angle_res,
                              fov * 0.5f - angle_res, 0.f, max_range, 720);
  EXPECT_TRUE(msg.is360());

  getLaserScanMsgSquare(msg, EstimateType::Identity(), angle_res, fov,
                        max_range, 5.f, rng);
  VALIDATE_2DLASERSCANMSGMETA(msg, -fov * 0.5f, angle_res,
                              fov * 0.5f - angle_res, 0.f, max_range, 720);
  EXPECT_TRUE(msg.is360());
}  // TEST(test_laser_scan_examples, init2DLaserScanMsgModel)

TEST(test_laser_scan_examples, getLaser2DLaserScanMsgRandom)
{
  // using EstimateType = geometry::Isometry2f;

  std::random_device rd;
  std::mt19937 rng(rd());

  msg::LaserScan msg;

  float angle_res = geometry::degToRad(1.f);
  float fov = geometry::degToRad(270.f);
  float max_range = 10.f;

  getLaser2DLaserScanMsgRandom(msg, angle_res, fov, max_range, rng);
  VALIDATE_2DLASERSCANMSGMETA(msg, -fov * 0.5f, angle_res, fov * 0.5f, 0.f,
                              max_range, 271);

  for (size_t idx = 0; idx < msg.m_ranges.size(); idx++)
  {
    EXPECT_TRUE(msg.m_ranges[idx] < max_range);
  }
}  // TEST(test_laser_scan_examples, getLaser2DLaserScanMsgRandom)

TEST(test_laser_scan_examples, getLaser2DLaserScanMsgCircle)
{
  // using EstimateType = geometry::Isometry2f;

  std::random_device rd;
  std::mt19937 rng(rd());

  msg::LaserScan msg;

  const float angle_res = geometry::degToRad(1.f);
  const float fov = geometry::degToRad(360.f);
  constexpr float max_range = 10.f;
  constexpr float radius = 5.f;

  for (int itest = 0; itest < 1000; itest++)
  {
    geometry::Vector3f pose_v = geometry::Vector3f::Random();

    pose_v.x() *= radius;
    pose_v.y() *= radius;
    pose_v.z() *= M_PIf32;

    geometry::Isometry2f pose = geometry::v2t(pose_v);

    getLaser2DLaserScanMsgCircle(msg, pose, radius, angle_res, fov, max_range,
                                 rng);

    ASSERT_TRUE(msg.is360());

    for (size_t idx = 0; idx < msg.m_ranges.size(); idx++)
    {
      geometry::Vector2f pt;

      float angle = msg.m_angle_min + static_cast<float>(idx) * msg.m_angle_inc;

      msg.angleDistToPt(pt, angle, msg.m_ranges[idx]);

      geometry::Vector2f pt_in_global = pose * pt;

      if ((msg.m_ranges[idx] < msg.m_range_min) ||
          (msg.m_ranges[idx] >= msg.m_range_max))
      {
        continue;
      }
      else
      {
        EXPECT_NEAR(sqrt(pt_in_global.x() * pt_in_global.x() +
                         pt_in_global.y() * pt_in_global.y()),
                    radius, 1e-5);
      }
    }

    /* getLaser2DLaserScanMsgCircle(msg, pose, radius, angle_res, fov,
    max_range, 0.01f);

    for (size_t idx = 0; idx < msg.m_ranges.size(); idx++)
    {
      geometry::Vector2f pt;

      float angle = msg.m_angle_min + static_cast<float>(idx) * msg.m_angle_inc;

      msg.angleDistToPt(pt, angle, msg.m_ranges[idx]);

      geometry::Vector2f pt_in_global = pose * pt;

      if ((msg.m_ranges[idx] < msg.m_range_min) ||
          (msg.m_ranges[idx] >= msg.m_range_max))
      {
        continue;
      }
      else
      {
        EXPECT_NEAR(sqrt(pt_in_global.x() * pt_in_global.x() +
                         pt_in_global.y() * pt_in_global.y()),
                    radius, 0.1);
      }
    } */
  }

  getLaser2DLaserScanMsgCircle(msg, geometry::Isometry2f::Identity(), radius,
                               angle_res, fov, max_range, rng);

  ASSERT_TRUE(msg.is360());

  for (size_t idx = 0; idx < msg.m_ranges.size(); idx++)
  {
    geometry::Vector2f pt;

    float angle = msg.m_angle_min + static_cast<float>(idx) * msg.m_angle_inc;

    msg.angleDistToPt(pt, angle, msg.m_ranges[idx]);

    geometry::Vector2f pt_in_global = geometry::Isometry2f::Identity() * pt;

    EXPECT_NEAR(sqrt(pt_in_global.x() * pt_in_global.x() +
                     pt_in_global.y() * pt_in_global.y()),
                radius, 1e-5);

    if (std::fabs(sqrt(pt_in_global.x() * pt_in_global.x() +
                       pt_in_global.y() * pt_in_global.y()) -
                  radius) > 1e-5)
    {
      std::cout << "index: " << idx << std::endl;
    }
  }
}  // TEST(test_laser_scan_examples, getLaser2DLaserScanMsgCircle)

TEST(test_laser_scan_examples, getLaserScanMsgSquare)
{
  // using EstimateType = geometry::Isometry2f;

  std::random_device rd;
  std::mt19937 rng(rd());

  msg::LaserScan msg;

  const float angle_res = geometry::degToRad(1.f);
  const float fov = geometry::degToRad(360.f);
  constexpr float max_range = 10.f;
  constexpr float side = 5.f;

  for (int itest = 0; itest < 1000; itest++)
  {
    geometry::Vector3f pose_v = geometry::Vector3f::Random();

    pose_v.x() *= side * 2.f;
    pose_v.y() *= side * 2.f;
    pose_v.z() *= M_PIf32;

    geometry::Isometry2f pose = geometry::v2t(pose_v);

    getLaserScanMsgSquare(msg, pose, angle_res, fov, max_range, side, rng);

    ASSERT_TRUE(msg.is360());

    for (size_t idx = 0; idx < msg.m_ranges.size(); idx++)
    {
      geometry::Vector2f pt;

      float angle = msg.m_angle_min + static_cast<float>(idx) * msg.m_angle_inc;

      msg.angleDistToPt(pt, angle, msg.m_ranges[idx]);

      geometry::Vector2f pt_in_global = pose * pt;

      if ((msg.m_ranges[idx] < msg.m_range_min) ||
          (msg.m_ranges[idx] >= msg.m_range_max))
      {
        continue;
      }
      else
      {
        float pt_x_abs = fabs(pt_in_global.x());
        float pt_y_abs = fabs(pt_in_global.y());

        EXPECT_TRUE((fabs(pt_x_abs - side) < 1e-5f) ||
                    (fabs(pt_y_abs - side) < 1e-5f));

        EXPECT_TRUE((fabs(pt_x_abs) < side + 1e-5f) &&
                    (fabs(pt_y_abs) < side + 1e-5f));
      }
    }
  }

  getLaserScanMsgSquare(msg, geometry::Isometry2f::Identity(), angle_res, fov,
                        max_range, side, rng);

  ASSERT_TRUE(msg.is360());

  for (size_t idx = 0; idx < msg.m_ranges.size(); idx++)
  {
    geometry::Vector2f pt;

    float angle = msg.m_angle_min + static_cast<float>(idx) * msg.m_angle_inc;

    msg.angleDistToPt(pt, angle, msg.m_ranges[idx]);

    geometry::Vector2f pt_in_global = geometry::Isometry2f::Identity() * pt;

    float pt_x_abs = fabs(pt_in_global.x());
    float pt_y_abs = fabs(pt_in_global.y());

    EXPECT_TRUE((fabs(pt_x_abs - side) < 1e-5f) ||
                (fabs(pt_y_abs - side) < 1e-5f));

    EXPECT_TRUE((fabs(pt_x_abs) < side + 1e-5f) &&
                (fabs(pt_y_abs) < side + 1e-5f));
  }
}  // TEST(test_laser_scan_examples, getLaserScanMsgSquare)

TEST(test_laser_scan_examples, getLaserScanMsgCorridor)
{
  // using EstimateType = geometry::Isometry2f;

  std::random_device rd;
  std::mt19937 rng(rd());

  msg::LaserScan msg;

  const float angle_res = geometry::degToRad(1.f);
  const float fov = geometry::degToRad(360.f);
  constexpr float max_range = 10.f;
  constexpr float width = 2.5f;
  constexpr float length = 5.f;

  for (int itest = 0; itest < 1000; itest++)
  {
    geometry::Vector3f pose_v = geometry::Vector3f::Random();

    pose_v.x() *= length;
    pose_v.y() *= length;
    pose_v.z() *= M_PIf32;

    geometry::Isometry2f pose = geometry::v2t(pose_v);

    getLaserScanMsgCorridor(msg, pose, angle_res, fov, max_range, width, length,
                            rng, false);

    ASSERT_TRUE(msg.is360());

    for (size_t idx = 0; idx < msg.m_ranges.size(); idx++)
    {
      geometry::Vector2f pt;

      float angle = msg.m_angle_min + static_cast<float>(idx) * msg.m_angle_inc;

      msg.angleDistToPt(pt, angle, msg.m_ranges[idx]);

      geometry::Vector2f pt_in_global = pose * pt;

      if ((msg.m_ranges[idx] < msg.m_range_min) ||
          (msg.m_ranges[idx] >= msg.m_range_max))
      {
        continue;
      }
      else
      {
        float pt_x_abs = fabs(pt_in_global.x());
        float pt_y_abs = fabs(pt_in_global.y());

        EXPECT_TRUE((fabs(pt_y_abs - width) < 1e-5f) &&
                    (fabs(pt_x_abs) < length + 1e-5f));
      }
    }
  }

  for (int itest = 0; itest < 1000; itest++)
  {
    geometry::Vector3f pose_v = geometry::Vector3f::Random();

    pose_v.x() *= length;
    pose_v.y() *= length;
    pose_v.z() *= M_PIf32;

    geometry::Isometry2f pose = geometry::v2t(pose_v);

    getLaserScanMsgCorridor(msg, pose, angle_res, fov, max_range, width, length,
                            rng, true);

    ASSERT_TRUE(msg.is360());

    for (size_t idx = 0; idx < msg.m_ranges.size(); idx++)
    {
      geometry::Vector2f pt;

      float angle = msg.m_angle_min + static_cast<float>(idx) * msg.m_angle_inc;

      msg.angleDistToPt(pt, angle, msg.m_ranges[idx]);

      geometry::Vector2f pt_in_global = pose * pt;

      if ((msg.m_ranges[idx] < msg.m_range_min) ||
          (msg.m_ranges[idx] >= msg.m_range_max))
      {
        continue;
      }
      else
      {
        float pt_x_abs = fabs(pt_in_global.x());
        float pt_y_abs = fabs(pt_in_global.y());

        EXPECT_TRUE((fabs(pt_x_abs - width) < 1e-5f) &&
                    (fabs(pt_y_abs) < length + 1e-5f));
      }
    }
  }
}  // TEST(test_laser_scan_examples, getLaserScanMsgCorridor)

}  // namespace testing