#include <gtest/gtest.h>

#include <inesctec_mrdt_slam_distmap_2d/geometry/geometry_2d.hpp>

namespace testing
{

using namespace inesctec_mrdt_slam_distmap_2d;
using namespace geometry;

TEST(geometry_2d, a2r)
{
  EXPECT_TRUE(a2r(static_cast<float>(0.0f)).isApprox(Matrix2f::Identity()));
  EXPECT_TRUE(a2r(static_cast<double>(0.0)).isApprox(Matrix2d::Identity()));

  Matrix2f mat2f;
  Matrix2d mat2d;

  mat2f << 0.f, -1.f, 1.f, 0.f;
  mat2d << 0.0, -1.0, 1.0, 0.0;

  EXPECT_TRUE(a2r<float>(M_PIf32 * 0.5f).isApprox(mat2f));
  EXPECT_TRUE(a2r<double>(M_PIf64 * 0.50).isApprox(mat2d));
}  // TEST(geometry_2d, a2r)

TEST(geometry_2d, r2a)
{
  EXPECT_FLOAT_EQ(r2a<float>(Matrix2f::Identity()), 0.f);
  EXPECT_DOUBLE_EQ(r2a<double>(Matrix2d::Identity()), 0.0);

  Matrix2f mat2f;
  Matrix2d mat2d;

  mat2f << 0.f, -1.f, 1.f, 0.f;
  mat2d << 0.0, -1.0, 1.0, 0.0;
  EXPECT_FLOAT_EQ(r2a(mat2f), M_PIf32 * 0.5f);
  EXPECT_DOUBLE_EQ(r2a(mat2d), M_PIf64 * 0.50);
}  // TEST(geometry_2d, r2a)

TEST(geometry_2d, a2r_r2a)
{
  EXPECT_NEAR(r2a(a2r(static_cast<float>(0.0f))), 0.0f, 1e-6);
  EXPECT_NEAR(r2a(a2r(static_cast<float>(M_PIf32 / 2.0f))), M_PIf32 / 2.0f,
              1e-6);
  EXPECT_NEAR(r2a(a2r(static_cast<float>(-M_PIf32 / 2.0f))), -M_PIf32 / 2.0f,
              1e-6);
  EXPECT_NEAR(r2a(a2r(static_cast<float>(M_PIf32 / 3.0f))), M_PIf32 / 3.0f,
              1e-6);
  EXPECT_NEAR(r2a(a2r(static_cast<float>(-M_PIf32 / 3.0f))), -M_PIf32 / 3.0f,
              1e-6);
  EXPECT_NEAR(r2a(a2r(static_cast<float>(M_PIf32 / 6.0f))), M_PIf32 / 6.0f,
              1e-6);
  EXPECT_NEAR(r2a(a2r(static_cast<float>(-M_PIf32 / 6.0f))), -M_PIf32 / 6.0f,
              1e-6);
  EXPECT_NEAR(r2a(a2r(static_cast<float>(2.0f * M_PIf32 / 3.0f))),
              2.0f * M_PIf32 / 3.0f, 1e-6);
  EXPECT_NEAR(r2a(a2r(static_cast<float>(-2.0f * M_PIf32 / 3.0f))),
              -2.0f * M_PIf32 / 3.0f, 1e-6);

  EXPECT_NEAR(r2a(a2r(static_cast<double>(0.0))), 0.0, 1e-9);
  EXPECT_NEAR(r2a(a2r(static_cast<double>(M_PIf64 / 2.0))), M_PIf64 / 2.0,
              1e-9);
  EXPECT_NEAR(r2a(a2r(static_cast<double>(-M_PIf64 / 2.0))), -M_PIf64 / 2.0,
              1e-9);
  EXPECT_NEAR(r2a(a2r(static_cast<double>(M_PIf64 / 3.0))), M_PIf64 / 3.0,
              1e-9);
  EXPECT_NEAR(r2a(a2r(static_cast<double>(-M_PIf64 / 3.0))), -M_PIf64 / 3.0,
              1e-9);
  EXPECT_NEAR(r2a(a2r(static_cast<double>(M_PIf64 / 6.0))), M_PIf64 / 6.0,
              1e-9);
  EXPECT_NEAR(r2a(a2r(static_cast<double>(-M_PIf64 / 6.0))), -M_PIf64 / 6.0,
              1e-9);
  EXPECT_NEAR(r2a(a2r(static_cast<double>(2.0 * M_PIf64 / 3.0))),
              2.0 * M_PIf64 / 3.0, 1e-9);
  EXPECT_NEAR(r2a(a2r(static_cast<double>(-2.0 * M_PIf64 / 3.0))),
              -2.0 * M_PIf64 / 3.0, 1e-9);
}  // TEST(geometry_2d, a2r_r2a)

TEST(geometry_2d, v2t)
{
  EXPECT_TRUE(v2t(Vector3f(0, 0, 0)).translation().isApprox(Vector2f::Zero()));
  EXPECT_TRUE(v2t(Vector3f(0, 0, 0)).linear().isApprox(Matrix2f::Identity()));

  EXPECT_FLOAT_EQ(v2t(Vector3f(2, 3, 0)).translation().x(), 2.0f);
  EXPECT_FLOAT_EQ(v2t(Vector3f(2, 3, 0)).translation().y(), 3.0f);

  Matrix2f Rf;
  Rf << 0.0f, -1.0f, 1.0f, 0.0f;
  EXPECT_TRUE(v2t(Vector3f(0, 0, M_PIf32 / 2.0f)).linear().isApprox(Rf));

  EXPECT_TRUE(v2t(Vector3d(0, 0, 0)).translation().isApprox(Vector2d::Zero()));
  EXPECT_TRUE(v2t(Vector3d(0, 0, 0)).linear().isApprox(Matrix2d::Identity()));

  EXPECT_FLOAT_EQ(v2t(Vector3d(2, 3, 0)).translation().x(), 2.0);
  EXPECT_FLOAT_EQ(v2t(Vector3d(2, 3, 0)).translation().y(), 3.0);

  Matrix2d Rd;
  Rd << 0, -1, 1, 0;
  EXPECT_TRUE(v2t(Vector3d(0, 0, M_PIf64 / 2.0)).linear().isApprox(Rd));
}  // TEST(geometry_2d, v2t)

TEST(geometry_2d, t2v)
{
  EXPECT_TRUE(t2v(Isometry2f::Identity()).isApprox(Vector3f::Zero()));
  EXPECT_TRUE(t2v(Isometry2d::Identity()).isApprox(Vector3d::Zero()));

  EXPECT_FLOAT_EQ(t2v(v2t(Vector3f(1, 2, M_PIf32 / 3.0f)))(0), 1.0f);
  EXPECT_FLOAT_EQ(t2v(v2t(Vector3f(1, 2, M_PIf32 / 3.0f)))(1), 2.0f);
  EXPECT_NEAR(t2v(v2t(Vector3f(1, 2, M_PIf32 / 3.0f)))(2), M_PIf32 / 3.0f,
              1e-6);

  EXPECT_FLOAT_EQ(t2v(v2t(Vector3d(1, 2, M_PIf64 / 3.0)))(0), 1.0);
  EXPECT_FLOAT_EQ(t2v(v2t(Vector3d(1, 2, M_PIf64 / 3.0)))(1), 2.0);
  EXPECT_NEAR(t2v(v2t(Vector3d(1, 2, M_PIf64 / 3.0)))(2), M_PIf64 / 3.0, 1e-9);
}  // TEST(geometry_2d, t2v)

TEST(geometry_2d, a2q)
{
  EXPECT_NEAR(a2q<float>(0.f).w(), 1.f, 1e-6f);
  EXPECT_NEAR(a2q<float>(0.f).x(), 0.f, 1e-6f);
  EXPECT_NEAR(a2q<float>(0.f).y(), 0.f, 1e-6f);
  EXPECT_NEAR(a2q<float>(0.f).z(), 0.f, 1e-6f);

  EXPECT_NEAR(a2q<float>(M_PIf32).w(), 0.f, 1e-6f);
  EXPECT_NEAR(a2q<float>(M_PIf32).x(), 0.f, 1e-6f);
  EXPECT_NEAR(a2q<float>(M_PIf32).y(), 0.f, 1e-6f);
  EXPECT_NEAR(a2q<float>(M_PIf32).z(), 1.f, 1e-6f);

  EXPECT_NEAR(a2q<float>(M_PI_2f32).w(), sqrt(2.f) * 0.5f, 1e-6f);
  EXPECT_NEAR(a2q<float>(M_PI_2f32).x(), 0.f, 1e-6f);
  EXPECT_NEAR(a2q<float>(M_PI_2f32).y(), 0.f, 1e-6f);
  EXPECT_NEAR(a2q<float>(M_PI_2f32).z(), sqrt(2.f) * 0.5f, 1e-6f);

  EXPECT_NEAR(a2q<float>(-M_PI_2f32).w(), sqrt(2.f) * 0.5f, 1e-6f);
  EXPECT_NEAR(a2q<float>(-M_PI_2f32).x(), 0.f, 1e-6f);
  EXPECT_NEAR(a2q<float>(-M_PI_2f32).y(), 0.f, 1e-6f);
  EXPECT_NEAR(a2q<float>(-M_PI_2f32).z(), -sqrt(2.f) * 0.5f, 1e-6f);

  EXPECT_NEAR(a2q<double>(0.0).w(), 1.0, 1e-9);
  EXPECT_NEAR(a2q<double>(0.0).x(), 0.0, 1e-9);
  EXPECT_NEAR(a2q<double>(0.0).y(), 0.0, 1e-9);
  EXPECT_NEAR(a2q<double>(0.0).z(), 0.0, 1e-9);

  EXPECT_NEAR(a2q<double>(M_PIf64).w(), 0.0, 1e-9);
  EXPECT_NEAR(a2q<double>(M_PIf64).x(), 0.0, 1e-9);
  EXPECT_NEAR(a2q<double>(M_PIf64).y(), 0.0, 1e-9);
  EXPECT_NEAR(a2q<double>(M_PIf64).z(), 1.0, 1e-9);

  EXPECT_NEAR(a2q<double>(M_PI_2f64).w(), sqrt(2.0) * 0.5, 1e-9);
  EXPECT_NEAR(a2q<double>(M_PI_2f64).x(), 0.0, 1e-9);
  EXPECT_NEAR(a2q<double>(M_PI_2f64).y(), 0.0, 1e-9);
  EXPECT_NEAR(a2q<double>(M_PI_2f64).z(), sqrt(2.0) * 0.5, 1e-9);

  EXPECT_NEAR(a2q<double>(-M_PI_2f64).w(), sqrt(2.0) * 0.5, 1e-9);
  EXPECT_NEAR(a2q<double>(-M_PI_2f64).x(), 0.0, 1e-9);
  EXPECT_NEAR(a2q<double>(-M_PI_2f64).y(), 0.0, 1e-9);
  EXPECT_NEAR(a2q<double>(-M_PI_2f64).z(), -sqrt(2.0) * 0.5, 1e-9);
}  // TEST(geometry_2d, a2q)

TEST(geometry_2d, q2a)
{
  EXPECT_NEAR(q2a<float>(a2q<float>(0.f)), 0.f, 1e-6f);
  EXPECT_NEAR(q2a<float>(a2q<float>(0.99f * M_PIf32)), 0.99f * M_PIf32, 1e-6f);
  EXPECT_NEAR(q2a<float>(a2q<float>(-0.99f * M_PIf32)), -0.99f * M_PIf32,
              1e-6f);
  EXPECT_NEAR(q2a<float>(a2q<float>(M_PI_2f32)), M_PI_2f32, 1e-6f);
  EXPECT_NEAR(q2a<float>(a2q<float>(-M_PI_2f32)), -M_PI_2f32, 1e-6f);

  EXPECT_NEAR(q2a<double>(a2q<double>(0.0)), 0.0, 1e-9);
  EXPECT_NEAR(q2a<double>(a2q<double>(0.99 * M_PIf64)), 0.99 * M_PIf64, 1e-9);
  EXPECT_NEAR(q2a<double>(a2q<double>(-0.99 * M_PIf64)), -0.99 * M_PIf64,
              1e-9f);
  EXPECT_NEAR(q2a<double>(a2q<double>(M_PI_2f64)), M_PI_2f64, 1e-9);
  EXPECT_NEAR(q2a<double>(a2q<double>(-M_PI_2f64)), -M_PI_2f64, 1e-9);
}  // TEST(geometry_2d, q2a)

TEST(geometry_2d, q2D2a)
{
  EXPECT_NEAR(q2D2a<float>(a2q<float>(0.f)), 0.f, 1e-6f);
  EXPECT_NEAR(q2D2a<float>(a2q<float>(0.99f * M_PIf32)), 0.99f * M_PIf32,
              1e-6f);
  EXPECT_NEAR(q2D2a<float>(a2q<float>(-0.99f * M_PIf32)), -0.99f * M_PIf32,
              1e-6f);
  EXPECT_NEAR(q2D2a<float>(a2q<float>(M_PI_2f32)), M_PI_2f32, 1e-6f);
  EXPECT_NEAR(q2D2a<float>(a2q<float>(-M_PI_2f32)), -M_PI_2f32, 1e-6f);

  EXPECT_NEAR(q2D2a<double>(a2q<double>(0.0)), 0.0, 1e-9);
  EXPECT_NEAR(q2D2a<double>(a2q<double>(0.99 * M_PIf64)), 0.99 * M_PIf64, 1e-9);
  EXPECT_NEAR(q2D2a<double>(a2q<double>(-0.99 * M_PIf64)), -0.99 * M_PIf64,
              1e-9f);
  EXPECT_NEAR(q2D2a<double>(a2q<double>(M_PI_2f64)), M_PI_2f64, 1e-9);
  EXPECT_NEAR(q2D2a<double>(a2q<double>(-M_PI_2f64)), -M_PI_2f64, 1e-9);
}  // TEST(geometry_2d, q2D2a)

}  // namespace testing
