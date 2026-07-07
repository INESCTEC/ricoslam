#include <gtest/gtest.h>

#include <inesctec_mrdt_slam_distmap_2d/data_structures/correspondence.hpp>

namespace testing
{

using namespace inesctec_mrdt_slam_distmap_2d;
using namespace data_structures;

TEST(data_structures_correspondence, constructor)
{
  Correspondence crp1;

  EXPECT_EQ(crp1.m_idx_fixed, -1);
  EXPECT_EQ(crp1.m_idx_moving, -1);
  EXPECT_FLOAT_EQ(crp1.m_response, 0.f);

  Correspondence crp2(1, 3, 1.0);

  EXPECT_EQ(crp2.m_idx_fixed, 1);
  EXPECT_EQ(crp2.m_idx_moving, 3);
  EXPECT_FLOAT_EQ(crp2.m_response, 1.f);
}  // TEST(data_structures_correspondence, constructor)

TEST(data_structures_correspondence, vector)
{
  CorrespondenceVector correspondences(100);

  EXPECT_EQ(correspondences.size(), 100);

  for (const Correspondence& crp : correspondences)
  {
    EXPECT_EQ(crp.m_idx_fixed, -1);
    EXPECT_EQ(crp.m_idx_moving, -1);
    EXPECT_FLOAT_EQ(crp.m_response, 0.f);
  }
}  // TEST(data_structures_correspondence, vector)

}  // namespace testing
