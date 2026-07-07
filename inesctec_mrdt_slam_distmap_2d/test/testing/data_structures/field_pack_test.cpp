#include <gtest/gtest.h>

#include <Eigen/Core>
#include <array>
#include <inesctec_mrdt_slam_distmap_2d/data_structures/field_pack.hpp>
#include <type_traits>
#include <typeinfo>

namespace testing
{

using namespace inesctec_mrdt_slam_distmap_2d;
using namespace data_structures;

TEST(data_structures_field_pack, construct)
{
  using FieldPackType = FieldPack_<int, float, double>;

  FieldPackType pack_int_float_double_1;
  FieldPackType pack_int_float_double_2(4, 5.f, 8.33);

  EXPECT_EQ(FieldPackType::kNumFields, 3);
  EXPECT_EQ(pack_int_float_double_1.kNumFields, 3);
  EXPECT_EQ(pack_int_float_double_2.kNumFields, 3);
}  // TEST(data_structures_field_pack, construct)

TEST(data_structures_field_pack, typeAt)
{
  using FieldPackType1 = FieldPack_<int, float, double>;

  EXPECT_TRUE(static_cast<bool>(
      std::is_same<FieldPackItemTypeAt_<FieldPackType1, 0>::FieldPackItemType,
                   int>::value));
  EXPECT_TRUE(static_cast<bool>(
      std::is_same<FieldPackItemTypeAt_<FieldPackType1, 1>::FieldPackItemType,
                   float>::value));
  EXPECT_TRUE(static_cast<bool>(
      std::is_same<FieldPackItemTypeAt_<FieldPackType1, 2>::FieldPackItemType,
                   double>::value));

  using FieldPackType2 =
      FieldPack_<Eigen::Vector2f, Eigen::Vector3f, double, int[5]>;

  EXPECT_TRUE(static_cast<bool>(
      std::is_same<FieldPackItemTypeAt_<FieldPackType2, 0>::FieldPackItemType,
                   Eigen::Vector2f>::value));
  EXPECT_TRUE(static_cast<bool>(
      std::is_same<FieldPackItemTypeAt_<FieldPackType2, 1>::FieldPackItemType,
                   Eigen::Vector3f>::value));
  EXPECT_TRUE(static_cast<bool>(
      std::is_same<FieldPackItemTypeAt_<FieldPackType2, 2>::FieldPackItemType,
                   double>::value));
  EXPECT_TRUE(static_cast<bool>(
      std::is_same<FieldPackItemTypeAt_<FieldPackType2, 3>::FieldPackItemType,
                   int[5]>::value));
}  // TEST(data_structures_field_pack, typeAt)

TEST(data_structures_field_pack, fieldAt)
{
  using FieldPackType1 = FieldPack_<int, float, double>;

  using FieldPackType2 =
      FieldPack_<Eigen::Vector2f, Eigen::Vector3f, double, std::array<int, 5>>;

  constexpr int kFieldPack1_int = 3;
  constexpr float kFieldPack1_float = 4.2f;
  constexpr double kFieldPack1_double = 9.32421;

  FieldPackType1 field_pack_1(kFieldPack1_int, kFieldPack1_float,
                              kFieldPack1_double);

  const Eigen::Vector2f kFieldPack2_EigenVector2f = Eigen::Vector2f(2, 4);
  const Eigen::Vector3f kFieldPack2_EigenVector3f = Eigen::Vector3f(-1, 3, 8);
  constexpr double kFieldPack2_double = -6.3242;
  const std::array<int, 5> kFieldPack2_int({-11, 5, 3, 6, 1});

  FieldPackType2 field_pack_2(kFieldPack2_EigenVector2f,
                              kFieldPack2_EigenVector3f, kFieldPack2_double,
                              kFieldPack2_int);

  EXPECT_EQ(static_cast<int>(
                FieldPackItemFieldAt_<FieldPackType1, 0>::field(field_pack_1)),
            kFieldPack1_int);
  EXPECT_FLOAT_EQ(
      static_cast<float>(
          FieldPackItemFieldAt_<FieldPackType1, 1>::field(field_pack_1)),
      kFieldPack1_float);
  EXPECT_FLOAT_EQ(
      static_cast<double>(
          FieldPackItemFieldAt_<FieldPackType1, 2>::field(field_pack_1)),
      kFieldPack1_double);

  EXPECT_FLOAT_EQ(
      static_cast<float>(
          FieldPackItemFieldAt_<FieldPackType2, 0>::field(field_pack_2).x()),
      kFieldPack2_EigenVector2f.x());
  EXPECT_FLOAT_EQ(
      static_cast<float>(
          FieldPackItemFieldAt_<FieldPackType2, 0>::field(field_pack_2).y()),
      kFieldPack2_EigenVector2f.y());
  EXPECT_FLOAT_EQ(
      static_cast<double>(
          FieldPackItemFieldAt_<FieldPackType2, 1>::field(field_pack_2).x()),
      kFieldPack2_EigenVector3f.x());
  EXPECT_FLOAT_EQ(
      static_cast<double>(
          FieldPackItemFieldAt_<FieldPackType2, 1>::field(field_pack_2).y()),
      kFieldPack2_EigenVector3f.y());
  EXPECT_FLOAT_EQ(
      static_cast<double>(
          FieldPackItemFieldAt_<FieldPackType2, 1>::field(field_pack_2).z()),
      kFieldPack2_EigenVector3f.z());
  EXPECT_FLOAT_EQ(
      static_cast<double>(
          FieldPackItemFieldAt_<FieldPackType2, 2>::field(field_pack_2)),
      kFieldPack2_double);
  EXPECT_EQ(
      static_cast<int>(
          FieldPackItemFieldAt_<FieldPackType2, 3>::field(field_pack_2).size()),
      5);
  EXPECT_EQ(
      static_cast<int>(
          FieldPackItemFieldAt_<FieldPackType2, 3>::field(field_pack_2).at(0)),
      -11);
  EXPECT_EQ(
      static_cast<int>(
          FieldPackItemFieldAt_<FieldPackType2, 3>::field(field_pack_2).at(1)),
      5);
  EXPECT_EQ(
      static_cast<int>(
          FieldPackItemFieldAt_<FieldPackType2, 3>::field(field_pack_2).at(2)),
      3);
  EXPECT_EQ(
      static_cast<int>(
          FieldPackItemFieldAt_<FieldPackType2, 3>::field(field_pack_2).at(3)),
      6);
  EXPECT_EQ(
      static_cast<int>(
          FieldPackItemFieldAt_<FieldPackType2, 3>::field(field_pack_2).at(4)),
      1);

  EXPECT_EQ(field_pack_1.field<0>(), kFieldPack1_int);
  EXPECT_FLOAT_EQ(field_pack_1.field<1>(), kFieldPack1_float);
  EXPECT_FLOAT_EQ(field_pack_1.field<2>(), kFieldPack1_double);

  EXPECT_FLOAT_EQ(field_pack_2.field<0>().x(), kFieldPack2_EigenVector2f.x());
  EXPECT_FLOAT_EQ(field_pack_2.field<0>().y(), kFieldPack2_EigenVector2f.y());
  EXPECT_FLOAT_EQ(field_pack_2.field<1>().x(), kFieldPack2_EigenVector3f.x());
  EXPECT_FLOAT_EQ(field_pack_2.field<1>().y(), kFieldPack2_EigenVector3f.y());
  EXPECT_FLOAT_EQ(field_pack_2.field<1>().z(), kFieldPack2_EigenVector3f.z());
  EXPECT_FLOAT_EQ(field_pack_2.field<2>(), kFieldPack2_double);
  EXPECT_EQ(field_pack_2.field<3>().size(), 5);
  EXPECT_EQ(field_pack_2.field<3>().at(0), -11);
  EXPECT_EQ(field_pack_2.field<3>().at(1), 5);
  EXPECT_EQ(field_pack_2.field<3>().at(2), 3);
  EXPECT_EQ(field_pack_2.field<3>().at(3), 6);
  EXPECT_EQ(field_pack_2.field<3>().at(4), 1);

  field_pack_2.field<0>().x() += 1;
  field_pack_2.field<0>().y() += 1;
  field_pack_2.field<1>().x() += 1;
  field_pack_2.field<1>().y() += 1;
  field_pack_2.field<1>().z() += 1;

  EXPECT_FLOAT_EQ(field_pack_2.field<0>().x(),
                  kFieldPack2_EigenVector2f.x() + 1);
  EXPECT_FLOAT_EQ(field_pack_2.field<0>().y(),
                  kFieldPack2_EigenVector2f.y() + 1);
  EXPECT_FLOAT_EQ(field_pack_2.field<1>().x(),
                  kFieldPack2_EigenVector3f.x() + 1);
  EXPECT_FLOAT_EQ(field_pack_2.field<1>().y(),
                  kFieldPack2_EigenVector3f.y() + 1);
  EXPECT_FLOAT_EQ(field_pack_2.field<1>().z(),
                  kFieldPack2_EigenVector3f.z() + 1);
}  // TEST(data_structures_field_pack, fieldAt)

TEST(data_structures_field_pack, unrolledIdx)
{
  using FieldPackType = FieldPack_<int, float, double>;

  EXPECT_EQ(FieldPackType::unrolledIdx<0>(), 0);
  EXPECT_EQ(FieldPackType::unrolledIdx<1>(), 1);
  EXPECT_EQ(FieldPackType::unrolledIdx<2>(), 2);
}  // TEST(data_structures_field_pack, unrolledIdx)

}  // namespace testing
