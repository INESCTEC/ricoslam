#include <gtest/gtest.h>
#include <srrg2_core/srrg_boss/deserializer.h>
#include <srrg2_core/srrg_boss/serializer.h>
#include <srrg2_solver/variables_and_factors/types_2d/all_types.h>
#include <srrg2_solver/variables_and_factors/types_2d/instances.h>

#include <inesctec_mrdt_slam_distmap_2d/srrg2_solver/measurement_owner_distance_map_point_cloud.hpp>
#include <inesctec_mrdt_slam_distmap_2d/test/point_cloud_examples.hpp>

#define TEST_LASER_SCAN_MSG_BOSS(msg, gt)                             \
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
using namespace data_structures;
using namespace slam;
using namespace msg;
using namespace srrg2_solver;

template <typename DistanceMapType_, typename PointScalarType_>
class VariableSE2WithDistanceMapAndPoint2VectorCloud_
    : public VariableSE2_<VariableSE2Base::PerturbationSide::Right>,
      public MeasurementOwnerDistanceMapPointCloudLaserMsgs_<
          std::shared_ptr<DistanceMapType_>,
          std::shared_ptr<pcl::Point2VectorCloud_<PointScalarType_>>,
          typename VariableSE2_<
              VariableSE2Base::PerturbationSide::Right>::EstimateType>
{
 public:

  using BaseVariableType =
      VariableSE2_<VariableSE2Base::PerturbationSide::Right>;
  using ThisType =
      VariableSE2WithDistanceMapAndPoint2VectorCloud_<DistanceMapType_,
                                                      PointScalarType_>;

  using VariableType = ThisType;
  using EstimateType = typename BaseVariableType::EstimateType;

  using MeasurementOwnerType = MeasurementOwnerDistanceMapPointCloudLaserMsgs_<
      std::shared_ptr<DistanceMapType_>,
      std::shared_ptr<pcl::Point2VectorCloud_<PointScalarType_>>, EstimateType>;
  using MeasurementType = typename MeasurementOwnerType::MeasurementType;

 public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template <typename DistanceMapType_, typename PointScalarType_>
class VariableSE2WithDistanceMapAndPointNormal2VectorCloud_
    : public VariableSE2_<VariableSE2Base::PerturbationSide::Right>,
      public MeasurementOwnerDistanceMapPointCloudLaserMsgs_<
          std::shared_ptr<DistanceMapType_>,
          std::shared_ptr<pcl::PointNormal2VectorCloud_<PointScalarType_>>,
          typename VariableSE2_<
              VariableSE2Base::PerturbationSide::Right>::EstimateType>
{
 public:

  using BaseVariableType =
      VariableSE2_<VariableSE2Base::PerturbationSide::Right>;
  using ThisType =
      VariableSE2WithDistanceMapAndPointNormal2VectorCloud_<DistanceMapType_,
                                                            PointScalarType_>;

  using VariableType = ThisType;
  using EstimateType = typename BaseVariableType::EstimateType;

  using MeasurementOwnerType = MeasurementOwnerDistanceMapPointCloudLaserMsgs_<
      std::shared_ptr<DistanceMapType_>,
      std::shared_ptr<pcl::PointNormal2VectorCloud_<PointScalarType_>>,
      EstimateType>;
  using MeasurementType = typename MeasurementOwnerType::MeasurementType;

 public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template <typename PointScalarType_>
using VariableSE2WithDistanceMapStaticAndPoint2VectorCloud_ =
    VariableSE2WithDistanceMapAndPoint2VectorCloud_<
        DistanceMapStatic_<DistanceMapCell_<PointScalarType_>>,
        PointScalarType_>;
template <typename PointScalarType_>
using VariableSE2WithDistanceMapSparseStaticAndPoint2VectorCloud_ =
    VariableSE2WithDistanceMapAndPoint2VectorCloud_<
        DistanceMapSparseStatic_<DistanceMapCell_<PointScalarType_>>,
        PointScalarType_>;
template <typename PointScalarType_>
using VariableSE2WithDistanceMapNanoStaticAndPoint2VectorCloud_ =
    VariableSE2WithDistanceMapAndPoint2VectorCloud_<
        DistanceMapNanoStatic_<int32_t, PointScalarType_>, PointScalarType_>;

template <typename PointScalarType_>
using VariableSE2WithDistanceMapStaticAndPointNormal2VectorCloud_ =
    VariableSE2WithDistanceMapAndPointNormal2VectorCloud_<
        DistanceMapStatic_<DistanceMapCell_<PointScalarType_>>,
        PointScalarType_>;
template <typename PointScalarType_>
using VariableSE2WithDistanceMapSparseStaticAndPointNormal2VectorCloud_ =
    VariableSE2WithDistanceMapAndPointNormal2VectorCloud_<
        DistanceMapSparseStatic_<DistanceMapCell_<PointScalarType_>>,
        PointScalarType_>;
template <typename PointScalarType_>
using VariableSE2WithDistanceMapNanoStaticAndPointNormal2VectorCloud_ =
    VariableSE2WithDistanceMapAndPointNormal2VectorCloud_<
        DistanceMapNanoStatic_<int32_t, PointScalarType_>, PointScalarType_>;

using VariableSE2WithDistanceMapStaticAndPoint2fVectorCloud =
    VariableSE2WithDistanceMapStaticAndPoint2VectorCloud_<float>;
using VariableSE2WithDistanceMapSparseStaticAndPoint2fVectorCloud =
    VariableSE2WithDistanceMapSparseStaticAndPoint2VectorCloud_<float>;
using VariableSE2WithDistanceMapNanoStaticAndPoint2fVectorCloud =
    VariableSE2WithDistanceMapNanoStaticAndPoint2VectorCloud_<float>;
using VariableSE2WithDistanceMapStaticAndPointNormal2fVectorCloud =
    VariableSE2WithDistanceMapStaticAndPointNormal2VectorCloud_<float>;
using VariableSE2WithDistanceMapSparseStaticAndPointNormal2fVectorCloud =
    VariableSE2WithDistanceMapSparseStaticAndPointNormal2VectorCloud_<float>;
using VariableSE2WithDistanceMapNanoStaticAndPointNormal2fVectorCloud =
    VariableSE2WithDistanceMapNanoStaticAndPointNormal2VectorCloud_<float>;

void inesctec_mrdt_slam_distmap_2d_testing_srrg2_solver_measurement_owner_distance_map_point_cloud_test_registerTypes()
{
  BOSS_REGISTER_CLASS(VariableSE2WithDistanceMapStaticAndPoint2fVectorCloud);
  BOSS_REGISTER_CLASS(
      VariableSE2WithDistanceMapSparseStaticAndPoint2fVectorCloud);
  BOSS_REGISTER_CLASS(
      VariableSE2WithDistanceMapNanoStaticAndPoint2fVectorCloud);
  BOSS_REGISTER_CLASS(
      VariableSE2WithDistanceMapStaticAndPointNormal2fVectorCloud);
  BOSS_REGISTER_CLASS(
      VariableSE2WithDistanceMapSparseStaticAndPointNormal2fVectorCloud);
  BOSS_REGISTER_CLASS(
      VariableSE2WithDistanceMapNanoStaticAndPointNormal2fVectorCloud);
}

class NameGenerator
{
 public:

  template <typename T>
  static std::string GetName(int)
  {
    if constexpr (std::is_same_v<
                      T, VariableSE2WithDistanceMapStaticAndPoint2fVectorCloud>)
      return "DistanceMapStaticAndPoint2fVectorCloud";
    if constexpr (
        std::is_same_v<
            T, VariableSE2WithDistanceMapSparseStaticAndPoint2fVectorCloud>)
      return "DistanceMapSparseStaticAndPoint2fVectorCloud";
    if constexpr (
        std::is_same_v<
            T, VariableSE2WithDistanceMapNanoStaticAndPoint2fVectorCloud>)
      return "DistanceMapNanoStaticAndPoint2fVectorCloud";
    if constexpr (
        std::is_same_v<
            T, VariableSE2WithDistanceMapStaticAndPointNormal2fVectorCloud>)
      return "DistanceMapStaticAndPointNormal2fVectorCloud";
    if constexpr (
        std::is_same_v<
            T,
            VariableSE2WithDistanceMapSparseStaticAndPointNormal2fVectorCloud>)
      return "DistanceMapSparseStaticAndPointNormal2fVectorCloud";
    if constexpr (
        std::is_same_v<
            T, VariableSE2WithDistanceMapNanoStaticAndPointNormal2fVectorCloud>)
      return "DistanceMapNanoStaticAndPointNormal2fVectorCloud";
  }
};

template <typename T>
class MyInterfaceTest : public ::testing::Test
{
 public:

  using VariableType = T;
  using VariableTypePtr = std::shared_ptr<T>;
  using EstimateType = typename VariableType::EstimateType;
  using MeasurementType = typename VariableType::MeasurementType;
  using MeasurementObjType = typename MeasurementType::element_type;

  using DistanceMapType = typename MeasurementObjType::DistanceMapType;
  using PointCloudType = typename MeasurementObjType::PointCloudType;

  using Scalar = typename DistanceMapType::Scalar;

  using List = std::list<T>;

  std::random_device rd;
  std::mt19937 rng;

 public:

  MyInterfaceTest() : ::testing::Test::Test(), rng(rd())
  {
    srrg2_core::point_cloud_registerTypes();
    inesctec_mrdt_slam_distmap_2d_testing_srrg2_solver_measurement_owner_distance_map_point_cloud_test_registerTypes();
  }

  void testSerializeBoss()
  {
    constexpr Scalar res = 0.025f;
    constexpr Scalar max_dist = 2.5f;
    constexpr Scalar map_size = 10.f;

    constexpr Scalar square_side = 10.0;
    constexpr int square_side_pts = 250;

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

    Scalar noise_std = 0;

    MeasurementType measurement = std::make_shared<MeasurementObjType>();

    measurement->m_sensor_in_robot = EstimateType::Identity();
    measurement->m_pc = std::make_shared<PointCloudType>();

    ASSERT_NO_FATAL_FAILURE(test::getPointCloudSquare(
        *measurement->m_pc, square_side, square_side_pts, rng, noise_std));

    size_t pc_size = measurement->m_pc->size();

    measurement->m_dist_map = std::make_shared<DistanceMapType>();

    EXPECT_NO_THROW(measurement->m_dist_map->setGoals(*measurement->m_pc, res,
                                                      map_size, max_dist));

    measurement->m_msgs->push_back(
        std::make_pair(msg_1, EstimateType::Identity()));
    measurement->m_msgs->push_back(
        std::make_pair(msg_2, EstimateType::Identity()));
    measurement->m_msgs->push_back(
        std::make_pair(msg_3, EstimateType::Identity()));

    VariableTypePtr v = std::make_shared<VariableType>();

    v->setMeasurement(measurement);

    std::string filename = std::string(__FILE__) + "serializerBoss" +
                           NameGenerator::GetName<T>(0) + ".json";

    srrg2_core::Serializer serializer;

    serializer.setFilePath(filename);
    serializer.setBinaryPath(filename +
                             ".d/<classname>.<nameAttribute>.<id>.<ext>");
    serializer.setCommentsEnabled(true);
    serializer.writeObject(*v);

    v.reset();

    ASSERT_FALSE(v);

    srrg2_core::Deserializer deserializer;
    srrg2_core::SerializablePtr obj;

    deserializer.setFilePath(filename);

    while ((obj = deserializer.readObjectShared()))
    {
      VariableTypePtr obj_ptr = std::dynamic_pointer_cast<VariableType>(obj);

      if (obj_ptr)
      {
        v = obj_ptr;

        ASSERT_TRUE(v->measurement()->getDistanceMapPtr());
        ASSERT_TRUE(v->measurement()->getPointCloudPtr());
        ASSERT_TRUE(v->measurement()->getLaserScanMsgsPtr());

        EXPECT_FLOAT_EQ(v->measurement()->getDistanceMap()->getRes(), res);
        EXPECT_FLOAT_EQ(v->measurement()->getDistanceMap()->getMapSize(),
                        map_size);
        EXPECT_FLOAT_EQ(v->measurement()->getDistanceMap()->getMaxDist(),
                        max_dist);

        EXPECT_EQ(v->measurement()->getPointCloud()->size(), pc_size);

        EXPECT_EQ(v->measurement()->getLaserScanMsgs()->size(), 3);

        EXPECT_TRUE(v->measurement()->getSensorInRobot().isApprox(
            EstimateType::Identity()));

        break;
      }
    }

    ASSERT_EQ(v->measurement()->getLaserScanMsgs()->size(), 3);

    EXPECT_TRUE(v->measurement()->getLaserScanMsgs()->operator[](0).first);
    EXPECT_TRUE(v->measurement()->getLaserScanMsgs()->operator[](1).first);
    EXPECT_TRUE(v->measurement()->getLaserScanMsgs()->operator[](2).first);

    if (v->measurement()->getLaserScanMsgs()->operator[](0).first)
    {
      TEST_LASER_SCAN_MSG_BOSS(
          (*v->measurement()->getLaserScanMsgs()->operator[](0).first),
          (*msg_1));
    }

    if (v->measurement()->getLaserScanMsgs()->operator[](1).first)
    {
      TEST_LASER_SCAN_MSG_BOSS(
          (*v->measurement()->getLaserScanMsgs()->operator[](1).first),
          (*msg_2));
    }

    if (v->measurement()->getLaserScanMsgs()->operator[](2).first)
    {
      TEST_LASER_SCAN_MSG_BOSS(
          (*v->measurement()->getLaserScanMsgs()->operator[](2).first),
          (*msg_3));
    }
  }
};

using MyTypes = ::testing::Types<
    VariableSE2WithDistanceMapStaticAndPoint2fVectorCloud,
    VariableSE2WithDistanceMapSparseStaticAndPoint2fVectorCloud,
    VariableSE2WithDistanceMapNanoStaticAndPoint2fVectorCloud,
    VariableSE2WithDistanceMapStaticAndPointNormal2fVectorCloud,
    VariableSE2WithDistanceMapSparseStaticAndPointNormal2fVectorCloud,
    VariableSE2WithDistanceMapNanoStaticAndPointNormal2fVectorCloud>;

TYPED_TEST_SUITE(MyInterfaceTest, MyTypes, NameGenerator);

TYPED_TEST(MyInterfaceTest, serializeBoss) { this->testSerializeBoss(); }

TEST(srrg2_solver_measurement_owner_distance_map_point_cloud, normals)
{
  using VariableType =
      VariableSE2WithDistanceMapStaticAndPointNormal2fVectorCloud;
  using VariableTypePtr = std::shared_ptr<VariableType>;
  using EstimateType = typename VariableType::EstimateType;
  using MeasurementType = typename VariableType::MeasurementType;
  using MeasurementObjType = typename MeasurementType::element_type;

  using DistanceMapType = typename MeasurementObjType::DistanceMapType;
  using PointCloudType = typename MeasurementObjType::PointCloudType;
  using PointVectorType = typename PointCloudType::PointType::VectorType;

  using Scalar = typename DistanceMapType::Scalar;

  constexpr float res = 0.025f;
  constexpr float max_dist = 2.5f;
  constexpr float map_size = 10.f;

  constexpr float square_side = 10.0;
  constexpr int square_side_pts = 250;

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

  std::random_device rd;
  std::mt19937 rng(rd());

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

  Scalar noise_std = 0;

  MeasurementType measurement = std::make_shared<MeasurementObjType>();

  measurement->m_sensor_in_robot = EstimateType::Identity();
  measurement->m_pc = std::make_shared<PointCloudType>();

  ASSERT_NO_FATAL_FAILURE(test::getPointCloudSquare(
      *measurement->m_pc, square_side, square_side_pts, rng, noise_std));

  for (auto& pt : *measurement->m_pc)
  {
    pt.normal() = PointVectorType::Ones();
  }

  size_t pc_size = measurement->m_pc->size();

  measurement->m_dist_map = std::make_shared<DistanceMapType>();

  EXPECT_NO_THROW(measurement->m_dist_map->setGoals(*measurement->m_pc, res,
                                                    map_size, max_dist));

  measurement->m_msgs->push_back(
      std::make_pair(msg_1, EstimateType::Identity()));
  measurement->m_msgs->push_back(
      std::make_pair(msg_2, EstimateType::Identity()));
  measurement->m_msgs->push_back(
      std::make_pair(msg_3, EstimateType::Identity()));

  VariableTypePtr v1 = std::make_shared<VariableType>();

  v1->setMeasurement(measurement);

  std::string filename =
      std::string(__FILE__) + "serializerBoss_test_normals.json";

  srrg2_core::Serializer serializer;

  serializer.setFilePath(filename);
  serializer.setBinaryPath(filename +
                           ".d/<classname>.<nameAttribute>.<id>.<ext>");
  serializer.setCommentsEnabled(true);
  serializer.writeObject(*v1);

  VariableTypePtr v2 = nullptr;

  srrg2_core::Deserializer deserializer;
  srrg2_core::SerializablePtr obj;

  deserializer.setFilePath(filename);

  while ((obj = deserializer.readObjectShared()))
  {
    VariableTypePtr obj_ptr = std::dynamic_pointer_cast<VariableType>(obj);

    if (obj_ptr)
    {
      v2 = obj_ptr;

      ASSERT_TRUE(v2->measurement()->getDistanceMapPtr());
      ASSERT_TRUE(v2->measurement()->getPointCloudPtr());
      ASSERT_TRUE(v2->measurement()->getLaserScanMsgsPtr());

      EXPECT_FLOAT_EQ(v2->measurement()->getDistanceMap()->getRes(), res);
      EXPECT_FLOAT_EQ(v2->measurement()->getDistanceMap()->getMapSize(),
                      map_size);
      EXPECT_FLOAT_EQ(v2->measurement()->getDistanceMap()->getMaxDist(),
                      max_dist);

      EXPECT_EQ(v2->measurement()->getPointCloud()->size(), pc_size);

      EXPECT_EQ(v2->measurement()->getLaserScanMsgs()->size(), 3);

      EXPECT_TRUE(v2->measurement()->getSensorInRobot().isApprox(
          EstimateType::Identity()));

      break;
    }
  }

  ASSERT_EQ(v1->measurement()->getPointCloud()->size(),
            v2->measurement()->getPointCloud()->size());

  for (size_t idx = 0; idx < v2->measurement()->getPointCloud()->size(); idx++)
  {
    EXPECT_TRUE(v1->measurement()
                    ->getPointCloud()
                    ->operator[](idx)
                    .coordinates()
                    .isApprox(v2->measurement()
                                  ->getPointCloud()
                                  ->operator[](idx)
                                  .coordinates()));
    EXPECT_TRUE(
        v1->measurement()->getPointCloud()->operator[](idx).normal().isApprox(
            v2->measurement()->getPointCloud()->operator[](idx).normal()));
  }

  ASSERT_EQ(v2->measurement()->getLaserScanMsgs()->size(), 3);

  EXPECT_TRUE(v2->measurement()->getLaserScanMsgs()->operator[](0).first);
  EXPECT_TRUE(v2->measurement()->getLaserScanMsgs()->operator[](1).first);
  EXPECT_TRUE(v2->measurement()->getLaserScanMsgs()->operator[](2).first);

  if (v2->measurement()->getLaserScanMsgs()->operator[](0).first)
  {
    TEST_LASER_SCAN_MSG_BOSS(
        (*v2->measurement()->getLaserScanMsgs()->operator[](0).first),
        (*msg_1));
  }

  if (v2->measurement()->getLaserScanMsgs()->operator[](1).first)
  {
    TEST_LASER_SCAN_MSG_BOSS(
        (*v2->measurement()->getLaserScanMsgs()->operator[](1).first),
        (*msg_2));
  }

  if (v2->measurement()->getLaserScanMsgs()->operator[](2).first)
  {
    TEST_LASER_SCAN_MSG_BOSS(
        (*v2->measurement()->getLaserScanMsgs()->operator[](2).first),
        (*msg_3));
  }
}  // TEST(srrg2_solver_measurement_owner_distance_map_point_cloud, normals)

}  // namespace testing
