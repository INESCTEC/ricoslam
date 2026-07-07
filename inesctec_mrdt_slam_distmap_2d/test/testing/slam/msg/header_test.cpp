#include <gtest/gtest.h>
#include <srrg2_core/srrg_boss/deserializer.h>
#include <srrg2_core/srrg_boss/serializer.h>

#include <inesctec_mrdt_slam_distmap_2d/slam/msg/header.hpp>
#include <inesctec_mrdt_slam_distmap_2d/slam/msg/instances.hpp>

#define TEST_HEADER_MSG(msg, t, frame) \
  EXPECT_DOUBLE_EQ(msg.m_t, t);        \
  EXPECT_STREQ(msg.m_frame_id.c_str(), frame)

namespace testing
{

using namespace inesctec_mrdt_slam_distmap_2d;
using namespace slam;
using namespace msg;

TEST(slam_msg_header, constructor_type)
{
  constexpr char frame_expr[]{"odom"};

  double t = 9.8;
  std::string frame = "map";

  // default constructor
  ASSERT_NO_FATAL_FAILURE(Header msg);  // default constructor

  // standard parametrized constructor of header message
  EXPECT_NO_FATAL_FAILURE(Header msg(t, frame));
  EXPECT_NO_FATAL_FAILURE(Header msg(11.1, frame));

  // construct a header message using move semantics on the parameters
  EXPECT_NO_FATAL_FAILURE(Header msg(t, "base_link"));
  EXPECT_NO_FATAL_FAILURE(Header msg(99.9, frame_expr));

  // move constructor
  EXPECT_NO_FATAL_FAILURE(Header msg(Header(t, frame)));
  EXPECT_NO_FATAL_FAILURE(Header msg(Header(11.1, frame)));
  EXPECT_NO_FATAL_FAILURE(Header msg(Header(t, "base_link")));
  EXPECT_NO_FATAL_FAILURE(Header msg(Header(99.9, frame_expr)));
  EXPECT_NO_FATAL_FAILURE(Header msg(Header{t, frame}));
  EXPECT_NO_FATAL_FAILURE(Header msg(Header{11.1, frame}));
  EXPECT_NO_FATAL_FAILURE(Header msg(Header{t, "base_link"}));
  EXPECT_NO_FATAL_FAILURE(Header msg(Header{99.9, frame_expr}));

  // move assignment operator

  Header msg_t_1;
  Header msg_t_2;
  Header msg_t_3;
  Header msg_t_4;
  Header msg_t_5;
  Header msg_t_6;
  Header msg_t_7;
  Header msg_t_8;

  EXPECT_NO_FATAL_FAILURE(msg_t_1 = std::move(Header{t, frame}));
  EXPECT_NO_FATAL_FAILURE(msg_t_2 = std::move(Header{11.1, frame}));
  EXPECT_NO_FATAL_FAILURE(msg_t_3 = std::move(Header{t, "base_link"}));
  EXPECT_NO_FATAL_FAILURE(msg_t_4 = std::move(Header{99.9, frame_expr}));
  EXPECT_NO_FATAL_FAILURE(msg_t_5 = Header(t, frame));
  EXPECT_NO_FATAL_FAILURE(msg_t_6 = Header(11.1, frame));
  EXPECT_NO_FATAL_FAILURE(msg_t_7 = Header(t, "base_link"));
  EXPECT_NO_FATAL_FAILURE(msg_t_8 = Header(99.9, frame_expr));

  // copy constructor

  Header msg_1;
  msg_1.m_t = t;
  msg_1.m_frame_id = frame;

  EXPECT_NO_FATAL_FAILURE(Header msg_2(msg_1));
  EXPECT_NO_FATAL_FAILURE(Header msg_2 = msg_1);

  // copy assignment operator
  EXPECT_NO_FATAL_FAILURE(msg_t_1 = msg_1);
  EXPECT_NO_FATAL_FAILURE(msg_t_1 = msg_t_1);
}  // TEST(slam_msg_header, constructor)

TEST(slam_msg_header, create)
{
  constexpr double t_expr = 987.6;
  constexpr char frame_expr[]{"map"};

  double t = t_expr;
  std::string frame{frame_expr};

  ASSERT_DOUBLE_EQ(t, t_expr);
  ASSERT_STREQ(frame.c_str(), frame_expr);

  Header msg_t;
  msg_t.m_t = t_expr;
  msg_t.m_frame_id = frame_expr;

  ASSERT_DOUBLE_EQ(msg_t.m_t, t_expr);
  ASSERT_STREQ(msg_t.m_frame_id.c_str(), frame_expr);

  Header msg_t_1;  // default ctt
  EXPECT_DOUBLE_EQ(msg_t_1.m_t, -1.0);
  EXPECT_TRUE(msg_t_1.m_frame_id.empty());

  Header msg_t_2(t, frame);            // ctt w/ param
  Header msg_t_3(t_expr, frame_expr);  // ctt w/ mv param
  Header msg_t_4(Header(t, frame));    // mv ctt
  Header msg_t_5, msg_t_6;             // mv asgn oprt

  msg_t_5 = std::move(Header{t, frame});
  msg_t_6 = Header(t, frame);

  Header msg_t_7(msg_t);  // cp ctt

  Header msg_t_8;
  msg_t_8 = msg_t;  // cp asgn oprt

  TEST_HEADER_MSG(msg_t_2, t_expr, frame_expr);
  TEST_HEADER_MSG(msg_t_3, t_expr, frame_expr);
  TEST_HEADER_MSG(msg_t_4, t_expr, frame_expr);
  TEST_HEADER_MSG(msg_t_5, t_expr, frame_expr);
  TEST_HEADER_MSG(msg_t_6, t_expr, frame_expr);
  TEST_HEADER_MSG(msg_t_7, t_expr, frame_expr);
  TEST_HEADER_MSG(msg_t_8, t_expr, frame_expr);

}  // TEST(slam_msg_header, create)

TEST(slam_msg_header, serializer)
{
  constexpr double t = 1757603912.123456789;
  constexpr char frame_id[] = "unnamed_robot/laser";

  inesctec_mrdt_slam_distmap_2d_slam_msg_registerTypes();

  HeaderPtr msg_ptr = std::make_shared<Header>(t, frame_id);

  ASSERT_DOUBLE_EQ(msg_ptr->m_t, t);
  ASSERT_STREQ(msg_ptr->m_frame_id.c_str(), frame_id);

  std::string filename = std::string(__FILE__) + ".json";

  srrg2_core::Serializer serializer;

  serializer.setFilePath(filename);
  serializer.setBinaryPath(filename +
                           ".d/<classname>.<nameAttribute>.<id>.<ext>");

  serializer.writeObject(*msg_ptr);

  msg_ptr.reset();

  ASSERT_FALSE(msg_ptr);

  srrg2_core::Deserializer deserializer;
  srrg2_core::SerializablePtr obj;

  deserializer.setFilePath(filename);

  while ((obj = deserializer.readObjectShared()))
  {
    HeaderPtr obj_ptr = std::dynamic_pointer_cast<Header>(obj);

    if (obj_ptr)
    {
      msg_ptr = obj_ptr;
      break;
    }
  }

  EXPECT_TRUE(msg_ptr);

  EXPECT_DOUBLE_EQ(msg_ptr->m_t, t);
  EXPECT_STREQ(msg_ptr->m_frame_id.c_str(), frame_id);
}  // TEST(slam_msg_header, serializer)

}  // namespace testing
