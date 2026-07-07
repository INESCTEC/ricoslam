#include <gtest/gtest.h>
#include <srrg2_core/srrg_boss/deserializer.h>
#include <srrg2_core/srrg_boss/serializer.h>

#include <cmath>
#include <inesctec_mrdt_slam_distmap_2d/slam/msg/instances.hpp>
#include <inesctec_mrdt_slam_distmap_2d/slam/msg/laser_scan.hpp>

#define TEST_LASER_SCAN_MSG(msg, gt)                                  \
  EXPECT_DOUBLE_EQ(msg.m_header.m_t, gt.m_header.m_t);                \
  EXPECT_STREQ(msg.m_header.m_frame_id.c_str(),                       \
               gt.m_header.m_frame_id.c_str());                       \
  EXPECT_FLOAT_EQ(msg.m_angle_min, gt.m_angle_min);                   \
  EXPECT_FLOAT_EQ(msg.m_angle_max, gt.m_angle_max);                   \
  EXPECT_FLOAT_EQ(msg.m_angle_inc, gt.m_angle_inc);                   \
  EXPECT_FLOAT_EQ(msg.m_time_inc, gt.m_time_inc);                     \
  EXPECT_FLOAT_EQ(msg.m_scan_time, gt.m_scan_time);                   \
  EXPECT_FLOAT_EQ(msg.m_range_min, gt.m_range_min);                   \
  EXPECT_FLOAT_EQ(msg.m_range_max, gt.m_range_max);                   \
  EXPECT_EQ(msg.m_ranges.size(), gt.m_ranges.size());                 \
  if (msg.m_ranges.size() == gt.m_ranges.size())                      \
  {                                                                   \
    for (size_t idx = 0; idx < msg.m_ranges.size(); idx++)            \
    {                                                                 \
      EXPECT_FLOAT_EQ(msg.m_ranges[idx], gt.m_ranges[idx]);           \
    }                                                                 \
  }                                                                   \
  EXPECT_EQ(msg.m_intensities.size(), gt.m_intensities.size());       \
  if (msg.m_intensities.size() == gt.m_intensities.size())            \
  {                                                                   \
    for (size_t idx = 0; idx < msg.m_intensities.size(); idx++)       \
    {                                                                 \
      EXPECT_FLOAT_EQ(msg.m_intensities[idx], gt.m_intensities[idx]); \
    }                                                                 \
  }

namespace testing
{

using namespace inesctec_mrdt_slam_distmap_2d;
using namespace slam;
using namespace msg;

void callbackLaserScan(LaserScanPtr& out, const LaserScan& in);
void callbackLaserScanPtr(LaserScanPtr& out, const LaserScanPtr& in);

TEST(slam_msg_laser_scan, constructor_type)
{
  constexpr double t_expr = 1234.56789;
  constexpr char frame_expr[] = "map";
  constexpr float angle_min_expr = -M_PI;
  constexpr float angle_max_expr = M_PI;
  constexpr float angle_inc_expr = 1.f * M_PI / 180.f;
  constexpr float time_inc_expr = 1.f / 40.f / 360.f;
  constexpr float scan_time_expr = 1.f / 40.f;
  constexpr float range_min_expr = 0.06f;
  constexpr float range_max_expr = 12.f;
  constexpr size_t num_rays = 360;

  double t = t_expr;
  std::string frame{frame_expr};
  float angle_min = angle_min_expr;
  float angle_max = angle_max_expr;
  float angle_inc = angle_inc_expr;
  float time_inc = time_inc_expr;
  float scan_time = scan_time_expr;
  float range_min = range_min_expr;
  float range_max = range_max_expr;

  Header header(t, frame);

  std::vector<float> ranges(num_rays);

  for (auto& r : ranges)
  {
    r = 10.f;
  }

  std::vector<float> intensities(num_rays);

  for (auto& i : intensities)
  {
    i = 0.5f;
  }

  // default constructor
  EXPECT_NO_FATAL_FAILURE(LaserScan msg);

  // standard parametrized constructor of laser scan message
  EXPECT_NO_FATAL_FAILURE(LaserScan msg(header, angle_min, angle_max, angle_inc,
                                        time_inc, scan_time, range_min,
                                        range_max, ranges, intensities));

  // construct a laser scan message using move semantics on the parameters
  EXPECT_NO_FATAL_FAILURE(
      LaserScan msg(Header{t_expr, frame_expr}, angle_min, angle_max, angle_inc,
                    time_inc, scan_time, range_min, range_max,
                    {0.1f, 0.2f, 0.3f}, {0.01f, 0.02f, 0.03f}));

  // move constructor
  EXPECT_NO_FATAL_FAILURE(LaserScan msg(
      LaserScan(Header{t_expr, frame_expr}, angle_min, angle_max, angle_inc,
                time_inc, scan_time, range_min, range_max, {0.1f, 0.2f, 0.3f},
                {0.01f, 0.02f, 0.03f})));

  // move assignment operator
  LaserScan msg_t;

  EXPECT_NO_FATAL_FAILURE(msg_t =
                              std::move(LaserScan{Header{t_expr, frame_expr},
                                                  angle_min,
                                                  angle_max,
                                                  angle_inc,
                                                  time_inc,
                                                  scan_time,
                                                  range_min,
                                                  range_max,
                                                  {0.1f, 0.2f, 0.3f},
                                                  {0.01f, 0.02f, 0.03f}}));
  EXPECT_NO_FATAL_FAILURE(
      msg_t = LaserScan(Header{t_expr, frame_expr}, angle_min, angle_max,
                        angle_inc, time_inc, scan_time, range_min, range_max,
                        {0.1f, 0.2f, 0.3f}, {0.01f, 0.02f, 0.03f}));

  // copy constructor
  LaserScan msg_cp;
  msg_cp.m_header = header;
  msg_cp.m_angle_min = angle_min;
  msg_cp.m_angle_max = angle_max;
  msg_cp.m_angle_inc = angle_inc;
  msg_cp.m_time_inc = time_inc;
  msg_cp.m_scan_time = scan_time;
  msg_cp.m_range_min = range_min;
  msg_cp.m_range_max = range_max;
  msg_cp.m_ranges = ranges;
  msg_cp.m_intensities = intensities;

  EXPECT_NO_FATAL_FAILURE(LaserScan msg(msg_cp));

  // copy assignment operator
  EXPECT_NO_FATAL_FAILURE(msg_t = msg_cp);

  // move assignment operator (the rvalue may have an unknown state after mv!)
  EXPECT_NO_FATAL_FAILURE(msg_t = std::move(msg_cp));

  // construct a laser scan message using move semantics on the parameters
  EXPECT_NO_FATAL_FAILURE(
      LaserScan msg(Header{header.m_t, header.m_frame_id}, angle_min, angle_max,
                    angle_inc, time_inc, scan_time, range_min, range_max,
                    std::move(ranges), std::move(intensities)));

}  // TEST(slam_msg_laser_scan, constructor_type)

TEST(slam_msg_laser_scan, create)
{
  constexpr double t_expr = 1234.56789;
  constexpr char frame_expr[] = "map";
  constexpr float angle_min_expr = -M_PI;
  constexpr float angle_max_expr = M_PI;
  constexpr float angle_inc_expr = 1.f * M_PI / 180.f;
  constexpr float time_inc_expr = 1.f / 40.f / 360.f;
  constexpr float scan_time_expr = 1.f / 40.f;
  constexpr float range_min_expr = 0.06f;
  constexpr float range_max_expr = 12.f;
  constexpr size_t num_rays = 360;

  std::vector<float> ranges(num_rays);
  ASSERT_EQ(ranges.size(), num_rays);
  for (auto& r : ranges)
  {
    r = 10.f;
  }

  std::vector<float> intensities(num_rays);
  ASSERT_EQ(intensities.size(), num_rays);
  for (auto& i : intensities)
  {
    i = 0.5f;
  }

  Header header(t_expr, frame_expr);

  LaserScan gt, gt_default;
  gt.m_header.m_t = t_expr;
  gt.m_header.m_frame_id = frame_expr;
  gt.m_angle_min = angle_min_expr;
  gt.m_angle_max = angle_max_expr;
  gt.m_angle_inc = angle_inc_expr;
  gt.m_time_inc = time_inc_expr;
  gt.m_scan_time = scan_time_expr;
  gt.m_range_min = range_min_expr;
  gt.m_range_max = range_max_expr;
  gt.m_ranges = ranges;
  gt.m_intensities = intensities;

  gt_default.m_header.m_t = -1.0;
  gt_default.m_header.m_frame_id.clear();
  gt_default.m_angle_min = 0.f;
  gt_default.m_angle_max = 0.f;
  gt_default.m_angle_inc = -1.f;
  gt_default.m_time_inc = -1.f;
  gt_default.m_scan_time = -1.f;
  gt_default.m_range_min = -1.f;
  gt_default.m_range_max = -1.f;
  gt_default.m_ranges.clear();
  gt_default.m_intensities.clear();

  // default constructor
  LaserScan msg_t_1;
  TEST_LASER_SCAN_MSG(msg_t_1, gt_default);
  EXPECT_EQ(msg_t_1.m_ranges.size(), 0);
  EXPECT_EQ(msg_t_1.m_intensities.size(), 0);

  // standard parametrized constructor of laser scan message
  LaserScan msg_t_2(header, angle_min_expr, angle_max_expr, angle_inc_expr,
                    time_inc_expr, scan_time_expr, range_min_expr,
                    range_max_expr, ranges, intensities);
  TEST_LASER_SCAN_MSG(msg_t_2, gt);
  EXPECT_EQ(msg_t_2.m_ranges.size(), num_rays);
  EXPECT_EQ(msg_t_2.m_intensities.size(), num_rays);

  // copy constructor
  LaserScan msg_t_3(gt);
  TEST_LASER_SCAN_MSG(msg_t_3, gt);
  EXPECT_EQ(msg_t_3.m_ranges.size(), num_rays);
  EXPECT_EQ(msg_t_3.m_intensities.size(), num_rays);

  // copy assignment operator
  LaserScan msg_t_4;
  msg_t_4 = gt;
  TEST_LASER_SCAN_MSG(msg_t_4, gt);
  EXPECT_EQ(msg_t_4.m_ranges.size(), num_rays);
  EXPECT_EQ(msg_t_4.m_intensities.size(), num_rays);

  // construct a laser scan message using move semantics on the parameters
  LaserScan msg_t_5(Header{t_expr, frame_expr}, angle_min_expr, angle_max_expr,
                    angle_inc_expr, time_inc_expr, scan_time_expr,
                    range_min_expr, range_max_expr, std::move(ranges),
                    std::move(intensities));
  TEST_LASER_SCAN_MSG(msg_t_5, gt);
  EXPECT_EQ(msg_t_5.m_ranges.size(), num_rays);
  EXPECT_EQ(msg_t_5.m_intensities.size(), num_rays);

  LaserScan mv, mv_op;
  mv.m_header.m_t = gt.m_header.m_t;
  mv.m_header.m_frame_id = gt.m_header.m_frame_id;
  mv.m_angle_min = gt.m_angle_min;
  mv.m_angle_max = gt.m_angle_max;
  mv.m_angle_inc = gt.m_angle_inc;
  mv.m_time_inc = gt.m_time_inc;
  mv.m_scan_time = gt.m_scan_time;
  mv.m_range_min = gt.m_range_min;
  mv.m_range_max = gt.m_range_max;
  mv.m_ranges = gt.m_ranges;
  mv.m_intensities = gt.m_intensities;
  mv_op.m_header.m_t = gt.m_header.m_t;
  mv_op.m_header.m_frame_id = gt.m_header.m_frame_id;
  mv_op.m_angle_min = gt.m_angle_min;
  mv_op.m_angle_max = gt.m_angle_max;
  mv_op.m_angle_inc = gt.m_angle_inc;
  mv_op.m_time_inc = gt.m_time_inc;
  mv_op.m_scan_time = gt.m_scan_time;
  mv_op.m_range_min = gt.m_range_min;
  mv_op.m_range_max = gt.m_range_max;
  mv_op.m_ranges = gt.m_ranges;
  mv_op.m_intensities = gt.m_intensities;

  // move constructor
  LaserScan msg_t_6(std::move(mv));
  TEST_LASER_SCAN_MSG(msg_t_6, gt);
  EXPECT_EQ(msg_t_6.m_ranges.size(), num_rays);
  EXPECT_EQ(msg_t_6.m_intensities.size(), num_rays);

  // move assignment operator
  LaserScan msg_t_7;
  msg_t_7 = std::move(mv_op);
  TEST_LASER_SCAN_MSG(msg_t_7, gt);
  EXPECT_EQ(msg_t_7.m_ranges.size(), num_rays);
  EXPECT_EQ(msg_t_7.m_intensities.size(), num_rays);

  // simulate ROS callback with dummy LaserScan msg
  ASSERT_DOUBLE_EQ(gt.m_header.m_t, t_expr);
  ASSERT_STREQ(gt.m_header.m_frame_id.c_str(), frame_expr);
  ASSERT_FLOAT_EQ(gt.m_angle_min, angle_min_expr);
  ASSERT_FLOAT_EQ(gt.m_angle_max, angle_max_expr);
  ASSERT_FLOAT_EQ(gt.m_angle_inc, angle_inc_expr);
  ASSERT_FLOAT_EQ(gt.m_time_inc, time_inc_expr);
  ASSERT_FLOAT_EQ(gt.m_scan_time, scan_time_expr);
  ASSERT_FLOAT_EQ(gt.m_range_min, range_min_expr);
  ASSERT_FLOAT_EQ(gt.m_range_max, range_max_expr);
  ASSERT_EQ(gt.m_ranges.size(), num_rays);
  ASSERT_EQ(gt.m_intensities.size(), num_rays);

  LaserScan ros_msg;
  ros_msg.m_header.m_t = gt.m_header.m_t;
  ros_msg.m_header.m_frame_id = gt.m_header.m_frame_id;
  ros_msg.m_angle_min = gt.m_angle_min;
  ros_msg.m_angle_max = gt.m_angle_max;
  ros_msg.m_angle_inc = gt.m_angle_inc;
  ros_msg.m_time_inc = gt.m_time_inc;
  ros_msg.m_scan_time = gt.m_scan_time;
  ros_msg.m_range_min = gt.m_range_min;
  ros_msg.m_range_max = gt.m_range_max;
  ros_msg.m_ranges = gt.m_ranges;
  ros_msg.m_intensities = gt.m_intensities;

  LaserScanPtr msg;

  callbackLaserScan(msg,
                    ros_msg);  // construct a laser scan message (even using
                               // std::move, possibly due to const LaserScan&
                               // and std::move only tries to move it)

  const LaserScan& msg_ref_1 = *msg;

  TEST_LASER_SCAN_MSG(msg_ref_1, gt);

  LaserScanPtr ros_msg_ptr = std::make_shared<LaserScan>();
  ros_msg_ptr->m_header.m_t = gt.m_header.m_t;
  ros_msg_ptr->m_header.m_frame_id = gt.m_header.m_frame_id;
  ros_msg_ptr->m_angle_min = gt.m_angle_min;
  ros_msg_ptr->m_angle_max = gt.m_angle_max;
  ros_msg_ptr->m_angle_inc = gt.m_angle_inc;
  ros_msg_ptr->m_time_inc = gt.m_time_inc;
  ros_msg_ptr->m_scan_time = gt.m_scan_time;
  ros_msg_ptr->m_range_min = gt.m_range_min;
  ros_msg_ptr->m_range_max = gt.m_range_max;
  ros_msg_ptr->m_ranges = gt.m_ranges;
  ros_msg_ptr->m_intensities = gt.m_intensities;

  callbackLaserScanPtr(
      msg,
      ros_msg_ptr);  // construct a laser scan message using move semantics
                     // on the parameters (because now the parameter is a
                     // smart point, no const constraint on the variable;
                     // posibly if it was LaserScanConstPtr it would not work!)

  const LaserScan& msg_ref_2 = *msg;

  TEST_LASER_SCAN_MSG(msg_ref_2, gt);
}  // TEST(slam_msg_laser_scan, create)

void callbackLaserScan(LaserScanPtr& out, const LaserScan& in)
{
  out = std::make_shared<LaserScan>(
      Header{in.m_header.m_t, std::move(in.m_header.m_frame_id)},
      in.m_angle_min, in.m_angle_max, in.m_angle_inc, in.m_time_inc,
      in.m_scan_time, in.m_range_min, in.m_range_max, std::move(in.m_ranges),
      std::move(in.m_intensities));
}

void callbackLaserScanPtr(LaserScanPtr& out, const LaserScanPtr& in)
{
  out = std::make_shared<LaserScan>(
      Header{in->m_header.m_t, std::move(in->m_header.m_frame_id)},
      in->m_angle_min, in->m_angle_max, in->m_angle_inc, in->m_time_inc,
      in->m_scan_time, in->m_range_min, in->m_range_max,
      std::move(in->m_ranges), std::move(in->m_intensities));
}

TEST(slam_msg_laser_scan, conversions)
{
  using Scalar = float;
  using PointCoordinatesType = geometry::Vector2_<float>;

  msg::LaserScan scan({}, geometry::degToRad(-135.f), geometry::degToRad(135.f),
                      geometry::degToRad(270.f) / 1080, 0.025f * 0.25f / 360.f,
                      0.025f, 0.06f, 10.f, std::vector<float>(1081), {});

  ASSERT_EQ(scan.m_ranges.size(), 1081);
  ASSERT_EQ(scan.m_intensities.size(), 0);

  EXPECT_NEAR(geometry::wrapTo180(geometry::radToDeg(
                  scan.ptToAngle(PointCoordinatesType(1, 0)))),
              0.f, 1e-5);
  EXPECT_NEAR(geometry::wrapTo180(geometry::radToDeg(
                  scan.ptToAngle(PointCoordinatesType(0, 1)))),
              90.f, 1e-5);
  EXPECT_NEAR(geometry::wrapTo180(geometry::radToDeg(
                  scan.ptToAngle(PointCoordinatesType(-1, 0)))),
              -180.f, 1e-5);
  EXPECT_NEAR(geometry::wrapTo180(geometry::radToDeg(
                  scan.ptToAngle(PointCoordinatesType(0, -1)))),
              -90.f, 1e-5);

  EXPECT_NEAR(scan.ptToDist(PointCoordinatesType(1, 0)), 1.f, 1e-5);
  EXPECT_NEAR(scan.ptToDist(PointCoordinatesType(0, 1)), 1.f, 1e-5);
  EXPECT_NEAR(scan.ptToDist(PointCoordinatesType(-1, 0)), 1.f, 1e-5);
  EXPECT_NEAR(scan.ptToDist(PointCoordinatesType(0, -1)), 1.f, 1e-5);
  EXPECT_NEAR(scan.ptToDist(PointCoordinatesType(2, 0)), 2.f, 1e-5);
  EXPECT_NEAR(scan.ptToDist(PointCoordinatesType(0, 2)), 2.f, 1e-5);
  EXPECT_NEAR(scan.ptToDist(PointCoordinatesType(-2, 0)), 2.f, 1e-5);
  EXPECT_NEAR(scan.ptToDist(PointCoordinatesType(0, -2)), 2.f, 1e-5);
  EXPECT_NEAR(scan.ptToDist(PointCoordinatesType(2, 2)), 2 * sqrt(2.f), 1e-5);
  EXPECT_NEAR(scan.ptToDist(PointCoordinatesType(-2, 2)), 2 * sqrt(2.f), 1e-5);

  Scalar angle, distance;

  scan.ptToAngleDist(angle, distance, PointCoordinatesType(1, 0));
  EXPECT_NEAR(geometry::wrapTo180(geometry::radToDeg(angle)), 0.f, 1e-5);
  EXPECT_NEAR(distance, 1.f, 1e-5);
  scan.ptToAngleDist(angle, distance, PointCoordinatesType(0, 1));
  EXPECT_NEAR(geometry::wrapTo180(geometry::radToDeg(angle)), 90.f, 1e-5);
  EXPECT_NEAR(distance, 1.f, 1e-5);
  scan.ptToAngleDist(angle, distance, PointCoordinatesType(-1, 0));
  EXPECT_NEAR(geometry::wrapTo180(geometry::radToDeg(angle)), -180.f, 1e-5);
  EXPECT_NEAR(distance, 1.f, 1e-5);
  scan.ptToAngleDist(angle, distance, PointCoordinatesType(0, -1));
  EXPECT_NEAR(geometry::wrapTo180(geometry::radToDeg(angle)), -90.f, 1e-5);
  EXPECT_NEAR(distance, 1.f, 1e-5);

  PointCoordinatesType pt;

  scan.angleDistToPt(pt, geometry::degToRad(0.f), 1.f);
  EXPECT_NEAR(pt.x(), 1.f, 1e-5);
  EXPECT_NEAR(pt.y(), 0.f, 1e-5);
  scan.angleDistToPt(pt, geometry::degToRad(90.f), 1.f);
  EXPECT_NEAR(pt.x(), 0.f, 1e-5);
  EXPECT_NEAR(pt.y(), 1.f, 1e-5);
  scan.angleDistToPt(pt, geometry::degToRad(-180.f), 1.f);
  EXPECT_NEAR(pt.x(), -1.f, 1e-5);
  EXPECT_NEAR(pt.y(), 0.f, 1e-5);
  scan.angleDistToPt(pt, geometry::degToRad(-90.f), 1.f);
  EXPECT_NEAR(pt.x(), 0.f, 1e-5);
  EXPECT_NEAR(pt.y(), -1.f, 1e-5);

  EXPECT_NEAR(scan.angleToRayIdx(geometry::degToRad(-135.f)), 0, 1e-5);
  EXPECT_NEAR(scan.angleToRayIdx(geometry::degToRad(0.f)), 540, 1e-5);
  EXPECT_NEAR(scan.angleToRayIdx(geometry::degToRad(135.f)), 1080, 1e-5);

  EXPECT_NEAR(scan.ptToRayIdx(PointCoordinatesType(-1, -1)), 0, 1e-5);
  EXPECT_NEAR(scan.ptToRayIdx(PointCoordinatesType(1, 0)), 540, 1e-5);
  EXPECT_NEAR(scan.ptToRayIdx(PointCoordinatesType(-1, 1)), 1080, 1e-5);
}  // TEST(slam_msg_laser_scan, conversions)

TEST(slam_msg_laser_scan, serializer)
{
  constexpr double t1 = 1757603912.123456789;
  constexpr double t2 = 1857603912.123456789;
  constexpr double t3 = 1957603912.123456789;
  constexpr char frame_id[] = "unnamed_robot/laser";
  constexpr float angle_min = -M_PI;
  constexpr float angle_max = M_PI;
  constexpr float angle_inc = 1.f * M_PI / 180.f;
  constexpr float time_inc = 1.f / 40.f / 360.f;
  constexpr float scan_time = 1.f / 40.f;
  constexpr float range_min = 0.06f;
  constexpr float range_max = 12.f;
  constexpr size_t num_rays = 360;

  std::vector<float> ranges(num_rays);
  ASSERT_EQ(ranges.size(), num_rays);
  for (auto& r : ranges)
  {
    r = 10.f;
  }

  std::vector<float> intensities(num_rays);
  ASSERT_EQ(intensities.size(), num_rays);
  for (auto& i : intensities)
  {
    i = 0.5f;
  }

  LaserScanPtr msg_1 = std::make_shared<LaserScan>(
      Header(t1, frame_id), angle_min, angle_max, angle_inc, time_inc,
      scan_time, range_min, range_max, std::vector<float>{},
      std::vector<float>{});
  LaserScanPtr msg_2 = std::make_shared<LaserScan>(
      Header(t2, frame_id), angle_min, angle_max, angle_inc, time_inc,
      scan_time, range_min, range_max, ranges, std::vector<float>{});
  LaserScanPtr msg_3 = std::make_shared<LaserScan>(
      Header(t3, frame_id), angle_min, angle_max, angle_inc, time_inc,
      scan_time, range_min, range_max, ranges, intensities);

  std::string filename = std::string(__FILE__) + ".json";

  srrg2_core::Serializer serializer;

  serializer.setFilePath(filename);
  serializer.setBinaryPath(filename +
                           ".d/<classname>.<nameAttribute>.<id>.<ext>");

  serializer.writeObject(*msg_1);
  serializer.writeObject(*msg_2);
  serializer.writeObject(*msg_3);

  srrg2_core::Deserializer deserializer;
  srrg2_core::SerializablePtr obj;

  deserializer.setFilePath(filename);

  LaserScanPtr msg_1_read = nullptr;
  LaserScanPtr msg_2_read = nullptr;
  LaserScanPtr msg_3_read = nullptr;

  while ((obj = deserializer.readObjectShared()))
  {
    LaserScanPtr obj_ptr = std::dynamic_pointer_cast<LaserScan>(obj);

    if (obj_ptr)
    {
      if (obj_ptr->m_header.m_t == t1)
      {
        msg_1_read = obj_ptr;
      }
      else if (obj_ptr->m_header.m_t == t2)
      {
        msg_2_read = obj_ptr;
      }
      else if (obj_ptr->m_header.m_t == t3)
      {
        msg_3_read = obj_ptr;
      }
    }
  }

  EXPECT_TRUE(msg_1_read);
  EXPECT_TRUE(msg_2_read);
  EXPECT_TRUE(msg_3_read);

  if (msg_1_read)
  {
    TEST_LASER_SCAN_MSG((*msg_1_read), (*msg_1));
  }

  if (msg_2_read)
  {
    TEST_LASER_SCAN_MSG((*msg_2_read), (*msg_2));
  }

  if (msg_3_read)
  {
    TEST_LASER_SCAN_MSG((*msg_3_read), (*msg_3));
  }

  /* std::cout << "msg_1" << std::endl << *msg_1_read << std::endl;
  std::cout << "msg_2" << std::endl << *msg_2_read << std::endl;
  std::cout << "msg_3" << std::endl << *msg_3_read << std::endl; */
}  // TEST(slam_msg_laser_scan, serializer)

}  // namespace testing
