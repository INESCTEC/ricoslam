#include <gtest/gtest.h>

#include <Eigen/Core>
#include <array>
#include <inesctec_mrdt_slam_distmap_2d/data_structures/field_pack_derived.hpp>
#include <type_traits>
#include <typeinfo>

namespace testing
{

using namespace inesctec_mrdt_slam_distmap_2d;
using namespace data_structures;

TEST(data_structures_field_pack_derived, constructor)
{
  using FieldPackType = FieldPack_<int, float, double>;
  using FieldPackDerivedType = FieldPackDerived_<FieldPackType, char, uint16_t>;

  constexpr int field_pack_int = 3;
  constexpr float field_pack_float = -2.34f;
  constexpr double field_pack_double = 6.573456;

  constexpr char field_pack_derived_char = 'z';
  constexpr uint16_t field_pack_derived_uint16 = 256;

  FieldPackDerivedType field(
      FieldPackType(field_pack_int, field_pack_float, field_pack_double),
      field_pack_derived_char, field_pack_derived_uint16);

  EXPECT_EQ(FieldPackType::kNumFields, 3);
  EXPECT_EQ(FieldPackDerivedType::kNumFields, 5);
  EXPECT_EQ(FieldPackDerivedType::kNumOuterFields, 2);
  EXPECT_EQ(field.kNumFields, 5);
  EXPECT_EQ(field.kNumOuterFields, 2);
}  // TEST(data_structures_field_pack_derived, constructor)

TEST(data_structures_field_pack_derived, unrolledIdx)
{
  using FieldPackType = FieldPack_<int, float, double>;
  using FieldPackDerivedType = FieldPackDerived_<FieldPackType, char, uint16_t>;

  EXPECT_EQ(FieldPackType::unrolledIdx<0>(), 0);
  EXPECT_EQ(FieldPackType::unrolledIdx<1>(), 1);
  EXPECT_EQ(FieldPackType::unrolledIdx<2>(), 2);

  EXPECT_EQ(FieldPackDerivedType::unrolledIdx<0>(), 2);
  EXPECT_EQ(FieldPackDerivedType::unrolledIdx<1>(), 3);
  EXPECT_EQ(FieldPackDerivedType::unrolledIdx<2>(), 4);
  EXPECT_EQ(FieldPackDerivedType::unrolledIdx<3>(), 0);
  EXPECT_EQ(FieldPackDerivedType::unrolledIdx<4>(), 1);

  /**
   * @note due to the variadic definition of field pack and field pack derived
   *       types, the fields char, uint16_t are defined in the internal indexes
   *       0 and 1, respectively. Thus, the need to have an unrolledIdx method
   */
}  // TEST(data_structures_field_pack_derived, unrolledIdx)

TEST(data_structures_field_pack_derived, typeAt)
{
  using FieldPackType = FieldPack_<int, float, double>;
  using FieldPackDerivedType1 =
      FieldPackDerived_<FieldPackType, char, uint16_t>;

  EXPECT_TRUE((
      std::is_same<FieldPackDerivedType1::FieldPackItemTypeAt<0>, int>::value));
  EXPECT_TRUE((std::is_same<FieldPackDerivedType1::FieldPackItemTypeAt<1>,
                            float>::value));
  EXPECT_TRUE((std::is_same<FieldPackDerivedType1::FieldPackItemTypeAt<2>,
                            double>::value));
  EXPECT_TRUE((std::is_same<FieldPackDerivedType1::FieldPackItemTypeAt<3>,
                            char>::value));
  EXPECT_TRUE((std::is_same<FieldPackDerivedType1::FieldPackItemTypeAt<4>,
                            uint16_t>::value));

  using FieldPackDerivedType2 =
      FieldPackDerived_<FieldPackType, Eigen::Vector2f, Eigen::Vector3f, double,
                        int[5]>;

  EXPECT_TRUE((
      std::is_same<FieldPackDerivedType2::FieldPackItemTypeAt<0>, int>::value));
  EXPECT_TRUE((std::is_same<FieldPackDerivedType2::FieldPackItemTypeAt<1>,
                            float>::value));
  EXPECT_TRUE((std::is_same<FieldPackDerivedType2::FieldPackItemTypeAt<2>,
                            double>::value));
  EXPECT_TRUE((std::is_same<FieldPackDerivedType2::FieldPackItemTypeAt<3>,
                            Eigen::Vector2f>::value));
  EXPECT_TRUE((std::is_same<FieldPackDerivedType2::FieldPackItemTypeAt<4>,
                            Eigen::Vector3f>::value));
  EXPECT_TRUE((std::is_same<FieldPackDerivedType2::FieldPackItemTypeAt<5>,
                            double>::value));
  EXPECT_TRUE((std::is_same<FieldPackDerivedType2::FieldPackItemTypeAt<6>,
                            int[5]>::value));
}  // TEST(data_structures_field_pack_derived, typeAt)

TEST(data_structures_field_pack_derived, fieldAt)
{
  using FieldPackType = FieldPack_<int, float, double>;
  using FieldPackDerivedType = FieldPackDerived_<FieldPackType, char, uint16_t>;

  constexpr int field_pack_int = 3;
  constexpr float field_pack_float = -2.34f;
  constexpr double field_pack_double = 6.573456;

  constexpr char field_pack_derived_char = 'z';
  constexpr uint16_t field_pack_derived_uint16 = 256;

  FieldPackDerivedType field(
      FieldPackType(field_pack_int, field_pack_float, field_pack_double),
      field_pack_derived_char, field_pack_derived_uint16);

  EXPECT_EQ(field.field<0>(), field_pack_int);
  EXPECT_FLOAT_EQ(field.field<1>(), field_pack_float);
  EXPECT_FLOAT_EQ(field.field<2>(), field_pack_double);
  EXPECT_EQ(field.field<3>(), field_pack_derived_char);
  EXPECT_EQ(field.field<4>(), field_pack_derived_uint16);

  field.field<0>() += 1;
  field.field<1>() += 1;
  field.field<2>() += 1;
  field.field<3>() += 1;
  field.field<4>() += 1;

  EXPECT_EQ(field.field<0>(), field_pack_int + 1);
  EXPECT_FLOAT_EQ(field.field<1>(), field_pack_float + 1);
  EXPECT_FLOAT_EQ(field.field<2>(), field_pack_double + 1);
  EXPECT_EQ(field.field<3>(), field_pack_derived_char + 1);
  EXPECT_EQ(field.field<4>(), field_pack_derived_uint16 + 1);
}  // TEST(data_structures_field_pack_derived, fieldAt)

}  // namespace testing
