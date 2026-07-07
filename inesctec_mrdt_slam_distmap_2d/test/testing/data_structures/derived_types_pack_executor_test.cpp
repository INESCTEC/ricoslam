#include <gtest/gtest.h>

#include <inesctec_mrdt_slam_distmap_2d/data_structures/derived_types_pack_executor.hpp>

using namespace inesctec_mrdt_slam_distmap_2d;
using namespace data_structures;

/**
 * @note this test shows the usage of the derived types pack executor with a
 *       virtual method existent in the parent class, but that may not be
 *       necessarily the case
 */

namespace testing
{

class TestParent
{
 public:

  static constexpr const char* name() { return "TestParent"; }
  virtual const char* msg() const { return "Hello, I am the parent!"; }
};

class TestChildA : public TestParent
{
 public:

  static constexpr const char* name() { return "TestChildA"; }
  virtual const char* msg() const { return "Hello, I am the shy son!"; }
  const char* test() const
  {
    return "the smarter one, always the best @ school!";
  }
};

class TestChildB : public TestParent
{
 public:

  static constexpr const char* name() { return "TestChildB"; }
  virtual const char* msg() const { return "Hello, I am the rebel one!"; }
  const char* test() const { return "just trying to gather your attention!"; }
};

class TestChildC : public TestParent
{
 public:

  static constexpr const char* name() { return "TestChildC"; }
  virtual const char* msg() const { return "Hello, just here existing!"; }
  const char* test() const
  {
    return "appreciating arts, but parents forced me into engineering ;)";
  }
};

class TestChildD : public TestParent
{
 public:

  static constexpr const char* name() { return "TestChildD"; }
  virtual const char* msg() const { return "Hello, not sure, may be adopted!"; }
  const char* test() const { return "boringggggg!!!"; }
};

using MyTypes = DerivedTypesPackExecutor_<TestChildA, TestChildB, TestChildC>;

TEST(data_structures_derived_types_pack_executor, findMatchingType)
{
  TestChildA obj_a;
  TestChildB obj_b;
  TestChildC obj_c;
  TestChildD obj_d;

  TestParent* const ptr_obj_a = &obj_a;
  TestParent* const ptr_obj_b = &obj_b;
  TestParent* const ptr_obj_c = &obj_c;
  TestParent* const ptr_obj_d = &obj_d;

  EXPECT_EQ(MyTypes::findMatchingType(ptr_obj_a), 2);
  EXPECT_EQ(MyTypes::findMatchingType(ptr_obj_b), 1);
  EXPECT_EQ(MyTypes::findMatchingType(ptr_obj_c), 0);
  EXPECT_EQ(MyTypes::findMatchingType(ptr_obj_d), -1);
}  // TEST(data_structures_derived_types_pack_executor, findMatchingType)

TEST(data_structures_derived_types_pack_executor, executeWithMatchingType)
{
  TestChildA obj_a;
  TestChildB obj_b;
  TestChildC obj_c;
  TestChildD obj_d;

  TestParent* const ptr_obj_a = &obj_a;
  TestParent* const ptr_obj_b = &obj_b;
  TestParent* const ptr_obj_c = &obj_c;
  TestParent* const ptr_obj_d = &obj_d;

  MyTypes::executeWithMatchingType(
      ptr_obj_a, [](auto* ptr) { std::cout << ptr->msg() << std::endl; });
  MyTypes::executeWithMatchingType(
      ptr_obj_b, [](auto* ptr) { std::cout << ptr->msg() << std::endl; });
  MyTypes::executeWithMatchingType(
      ptr_obj_c, [](auto* ptr) { std::cout << ptr->msg() << std::endl; });
  MyTypes::executeWithMatchingType(
      ptr_obj_d, [](auto* ptr) { std::cout << ptr->msg() << std::endl; });

  std::string test_obj_a;
  std::string test_obj_b;
  std::string test_obj_c;
  std::string test_obj_d;

  EXPECT_TRUE(MyTypes::executeWithMatchingType(ptr_obj_a,
                                               [&test_obj_a](auto* ptr)
                                               {
                                                 std::cout << ptr->test()
                                                           << std::endl;
                                                 test_obj_a = ptr->test();
                                               }));
  EXPECT_TRUE(MyTypes::executeWithMatchingType(ptr_obj_b,
                                               [&test_obj_b](auto* ptr)
                                               {
                                                 std::cout << ptr->test()
                                                           << std::endl;
                                                 test_obj_b = ptr->test();
                                               }));
  EXPECT_TRUE(MyTypes::executeWithMatchingType(ptr_obj_c,
                                               [&test_obj_c](auto* ptr)
                                               {
                                                 std::cout << ptr->test()
                                                           << std::endl;
                                                 test_obj_c = ptr->test();
                                               }));
  EXPECT_FALSE(MyTypes::executeWithMatchingType(ptr_obj_d,
                                                [&test_obj_d](auto* ptr)
                                                {
                                                  std::cout << ptr->test()
                                                            << std::endl;
                                                  test_obj_d = ptr->test();
                                                }));

  EXPECT_STREQ(test_obj_a.c_str(), obj_a.test());
  EXPECT_STREQ(test_obj_b.c_str(), obj_b.test());
  EXPECT_STREQ(test_obj_c.c_str(), obj_c.test());
  EXPECT_TRUE(test_obj_d.empty());
}  // TEST(data_structures_derived_types_pack_executor, executeWithMatchingType)

}  // namespace testing
