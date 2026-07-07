#include <gtest/gtest.h>

#include <inesctec_mrdt_slam_distmap_2d/data_structures/traverse_line.hpp>
#include <inesctec_mrdt_slam_distmap_2d/geometry/defs.hpp>
#include <inesctec_mrdt_slam_distmap_2d/system_utils/time.hpp>
#include <utility>
#include <vector>

#define TEST_TRAVERSE_LINE_EXPECT_EQ(gt, line)       \
  {                                                  \
    EXPECT_EQ(gt.size(), line.size());               \
    if (gt.size() == line.size())                    \
    {                                                \
      auto gt_it = gt.begin();                       \
      auto line_it = line.begin();                   \
                                                     \
      for (size_t idx = 0; idx < line.size(); idx++) \
      {                                              \
        EXPECT_EQ(gt_it->first, line_it->first);     \
        EXPECT_EQ(gt_it->second, line_it->second);   \
      }                                              \
    }                                                \
  }

namespace testing
{

using namespace inesctec_mrdt_slam_distmap_2d;
using namespace data_structures;

TEST(data_structures_traverse_line, basic)
{
  TraverseLineFillMultimapAction::MultimapType gt, line;
  TraverseLineFillMultimapAction fill_action(line);

  // basic functionality (horizontal / vertical lines)
  gt.clear();
  line.clear();
  gt.insert({{0, 0}, {1, 0}, {2, 0}, {3, 0}, {4, 0}, {5, 0}, {6, 0}});
  traverseLine(0, 0, 6, 0, fill_action);
  TEST_TRAVERSE_LINE_EXPECT_EQ(gt, line);

  gt.clear();
  line.clear();
  gt.insert({{0, 0}, {0, 1}, {0, 2}, {0, 3}, {0, 4}, {0, 5}, {0, 6}});
  traverseLine(0, 0, 0, 6, fill_action);
  TEST_TRAVERSE_LINE_EXPECT_EQ(gt, line);

  gt.clear();
  line.clear();
  gt.insert({{0, 0}, {-1, 0}, {-2, 0}, {-3, 0}, {-4, 0}, {-5, 0}, {-6, 0}});
  traverseLine(0, 0, -6, 0, fill_action);
  TEST_TRAVERSE_LINE_EXPECT_EQ(gt, line);

  gt.clear();
  line.clear();
  gt.insert({{0, 0}, {0, -1}, {0, -2}, {0, -3}, {0, -4}, {0, -5}, {0, -6}});
  traverseLine(0, 0, 0, -6, fill_action);
  TEST_TRAVERSE_LINE_EXPECT_EQ(gt, line);

  gt.clear();
  line.clear();
  gt.insert({{0, 9}, {1, 9}, {2, 9}, {3, 9}, {4, 9}, {5, 9}, {6, 9}});
  traverseLine(0, 9, 6, 9, fill_action);
  TEST_TRAVERSE_LINE_EXPECT_EQ(gt, line);

  gt.clear();
  line.clear();
  gt.insert({{5, 0}, {5, 1}, {5, 2}, {5, 3}, {5, 4}, {5, 5}, {5, 6}});
  traverseLine(5, 0, 5, 6, fill_action);
  TEST_TRAVERSE_LINE_EXPECT_EQ(gt, line);

  gt.clear();
  line.clear();
  gt.insert({{0, 2}, {-1, 2}, {-2, 2}, {-3, 2}, {-4, 2}, {-5, 2}, {-6, 2}});
  traverseLine(0, 2, -6, 2, fill_action);
  TEST_TRAVERSE_LINE_EXPECT_EQ(gt, line);

  gt.clear();
  line.clear();
  gt.insert({{7, 0}, {7, -1}, {7, -2}, {7, -3}, {7, -4}, {7, -5}, {7, -6}});
  traverseLine(7, 0, 7, -6, fill_action);
  TEST_TRAVERSE_LINE_EXPECT_EQ(gt, line);

  gt.clear();
  line.clear();
  gt.insert({{-3, 9}, {-2, 9}, {-1, 9}, {0, 9}, {1, 9}, {2, 9}, {3, 9}});
  traverseLine(-3, 9, 3, 9, fill_action);
  TEST_TRAVERSE_LINE_EXPECT_EQ(gt, line);

  gt.clear();
  line.clear();
  gt.insert({{5, -3}, {5, -2}, {5, -1}, {5, 0}, {5, 1}, {5, 2}, {5, 3}});
  traverseLine(5, -3, 5, 3, fill_action);
  TEST_TRAVERSE_LINE_EXPECT_EQ(gt, line);

  gt.clear();
  line.clear();
  gt.insert({{-3, 2}, {-2, 2}, {-1, 2}, {0, 2}, {1, 2}, {2, 2}, {3, 2}});
  traverseLine(-3, 2, 3, 2, fill_action);
  TEST_TRAVERSE_LINE_EXPECT_EQ(gt, line);

  gt.clear();
  line.clear();
  gt.insert({{7, -3}, {7, -2}, {7, -1}, {7, 0}, {7, 1}, {7, 2}, {7, 3}});
  traverseLine(7, -3, 7, 3, fill_action);
  TEST_TRAVERSE_LINE_EXPECT_EQ(gt, line);

  gt.clear();
  line.clear();
  gt.insert({{3, 9}, {2, 9}, {1, 9}, {0, 9}, {-1, 9}, {-2, 9}, {-3, 9}});
  traverseLine(3, 9, -3, 9, fill_action);
  TEST_TRAVERSE_LINE_EXPECT_EQ(gt, line);

  gt.clear();
  line.clear();
  gt.insert({{5, 3}, {5, 2}, {5, 1}, {5, 0}, {5, -1}, {5, -2}, {5, -3}});
  traverseLine(5, 3, 5, -3, fill_action);
  TEST_TRAVERSE_LINE_EXPECT_EQ(gt, line);

  gt.clear();
  line.clear();
  gt.insert({{3, 2}, {2, 2}, {1, 2}, {0, 2}, {-1, 2}, {-2, 2}, {-3, 2}});
  traverseLine(3, 2, -3, 2, fill_action);
  TEST_TRAVERSE_LINE_EXPECT_EQ(gt, line);

  gt.clear();
  line.clear();
  gt.insert({{7, 3}, {7, 2}, {7, 1}, {7, 0}, {7, -1}, {7, -2}, {7, -3}});
  traverseLine(7, 3, 7, -3, fill_action);
  TEST_TRAVERSE_LINE_EXPECT_EQ(gt, line);
}  // TEST(data_structures_traverse_line, basic)

TEST(data_structures_traverse_line, diagonal)
{
  TraverseLineFillMultimapAction::MultimapType gt, line;
  TraverseLineFillMultimapAction fill_action(line);

  // diagonal lines (45, 135, -45, -135)
  gt.clear();
  line.clear();
  gt.insert({{-3, -3}, {-2, -2}, {-1, -1}, {0, 0}, {1, 1}, {2, 2}, {3, 3}});
  traverseLine(-3, -3, 3, 3, fill_action);
  TEST_TRAVERSE_LINE_EXPECT_EQ(gt, line);

  gt.clear();
  line.clear();
  gt.insert({{3, -3}, {2, -2}, {1, -1}, {0, 0}, {-1, 1}, {-2, 2}, {-3, 3}});
  traverseLine(3, -3, -3, 3, fill_action);
  TEST_TRAVERSE_LINE_EXPECT_EQ(gt, line);

  gt.clear();
  line.clear();
  gt.insert({{-3, 3}, {-2, 2}, {-1, 1}, {0, 0}, {1, -1}, {2, -2}, {3, -3}});
  traverseLine(-3, 3, 3, -3, fill_action);
  TEST_TRAVERSE_LINE_EXPECT_EQ(gt, line);

  gt.clear();
  line.clear();
  gt.insert({{3, 3}, {2, 2}, {1, 1}, {0, 0}, {-1, -1}, {-2, -2}, {-3, -3}});
  traverseLine(3, 3, -3, -3, fill_action);
  TEST_TRAVERSE_LINE_EXPECT_EQ(gt, line);
}  // TEST(data_structures_traverse_line, diagonal)

TEST(data_structures_traverse_line, octant_claude)
{
  TraverseLineFillMultimapAction::MultimapType gt, line;
  TraverseLineFillMultimapAction fill_action(line);

  // Octant 1: 0 < slope < 1, x increasing, y increasing
  gt.clear();
  line.clear();
  gt.insert({{0, 0}, {1, 0}, {2, 1}, {3, 1}, {4, 2}, {5, 2}});
  traverseLine(0, 0, 5, 2, fill_action);
  TEST_TRAVERSE_LINE_EXPECT_EQ(gt, line);

  // Octant 2: slope > 1, x increasing, y increasing
  gt.clear();
  line.clear();
  gt.insert({{0, 0}, {0, 1}, {1, 2}, {1, 3}, {2, 4}, {2, 5}});
  traverseLine(0, 0, 2, 5, fill_action);
  TEST_TRAVERSE_LINE_EXPECT_EQ(gt, line);

  // Octant 3: slope < -1, x decreasing, y increasing
  gt.clear();
  line.clear();
  gt.insert({{5, 0}, {5, 1}, {4, 2}, {4, 3}, {3, 4}, {3, 5}});
  traverseLine(5, 0, 3, 5, fill_action);
  TEST_TRAVERSE_LINE_EXPECT_EQ(gt, line);

  // Octant 4: -1 < slope < 0, x decreasing, y increasing
  gt.clear();
  line.clear();
  gt.insert({{5, 0}, {4, 0}, {3, 1}, {2, 1}, {1, 2}, {0, 2}});
  traverseLine(5, 0, 0, 2, fill_action);
  TEST_TRAVERSE_LINE_EXPECT_EQ(gt, line);

  // Octant 5: -1 < slope < 0, x decreasing, y decreasing
  gt.clear();
  line.clear();
  gt.insert({{5, 5}, {4, 5}, {3, 4}, {2, 4}, {1, 3}, {0, 3}});
  traverseLine(5, 5, 0, 3, fill_action);
  TEST_TRAVERSE_LINE_EXPECT_EQ(gt, line);

  // Octant 6: slope < -1, x decreasing, y decreasing
  gt.clear();
  line.clear();
  gt.insert({{5, 5}, {5, 4}, {4, 3}, {4, 2}, {3, 1}, {3, 0}});
  traverseLine(5, 5, 3, 0, fill_action);
  TEST_TRAVERSE_LINE_EXPECT_EQ(gt, line);

  // Octant 7: slope > 1, x increasing, y decreasing
  gt.clear();
  line.clear();
  gt.insert({{0, 5}, {0, 4}, {1, 3}, {1, 2}, {2, 1}, {2, 0}});
  traverseLine(0, 5, 2, 0, fill_action);
  TEST_TRAVERSE_LINE_EXPECT_EQ(gt, line);

  // Octant 8: 0 < slope < 1, x increasing, y decreasing
  gt.clear();
  line.clear();
  gt.insert({{0, 5}, {1, 5}, {2, 4}, {3, 4}, {4, 3}, {5, 3}});
  traverseLine(0, 5, 5, 3, fill_action);
  TEST_TRAVERSE_LINE_EXPECT_EQ(gt, line);
}  // TEST(data_structures_traverse_line, octant)

TEST(data_structures_traverse_line, octant_wiki)
{
  TraverseLineFillMultimapAction::MultimapType gt, line;
  TraverseLineFillMultimapAction fill_action(line);

  // Octant 1: 0 < slope < 1, x increasing, y increasing
  gt.clear();
  line.clear();
  gt.insert({{1, 1},
             {2, 1},
             {3, 2},
             {4, 2},
             {5, 3},
             {6, 3},
             {7, 3},
             {8, 4},
             {9, 4},
             {10, 5},
             {11, 5}});
  traverseLine(1, 1, 11, 5, fill_action);
  TEST_TRAVERSE_LINE_EXPECT_EQ(gt, line);

  // Octant 2: slope > 1, x increasing, y increasing
  gt.clear();
  line.clear();
  gt.insert({{1, 1},
             {1, 2},
             {2, 3},
             {2, 4},
             {3, 5},
             {3, 6},
             {3, 7},
             {4, 8},
             {4, 9},
             {5, 10},
             {5, 11}});
  traverseLine(1, 1, 5, 11, fill_action);
  TEST_TRAVERSE_LINE_EXPECT_EQ(gt, line);

  // Octant 3: slope < -1, x decreasing, y increasing
  gt.clear();
  line.clear();
  gt.insert({{-1, 1},
             {-1, 2},
             {-2, 3},
             {-2, 4},
             {-3, 5},
             {-3, 6},
             {-3, 7},
             {-4, 8},
             {-4, 9},
             {-5, 10},
             {-5, 11}});
  traverseLine(-1, 1, -5, 11, fill_action);
  TEST_TRAVERSE_LINE_EXPECT_EQ(gt, line);

  // Octant 4: -1 < slope < 0, x decreasing, y increasing
  gt.clear();
  line.clear();
  gt.insert({{-1, 1},
             {-2, 1},
             {-3, 2},
             {-4, 2},
             {-5, 3},
             {-6, 3},
             {-7, 3},
             {-8, 4},
             {-9, 4},
             {-10, 5},
             {-11, 5}});
  traverseLine(-1, 1, -11, 5, fill_action);
  TEST_TRAVERSE_LINE_EXPECT_EQ(gt, line);

  // Octant 5: -1 < slope < 0, x decreasing, y decreasing
  gt.clear();
  line.clear();
  gt.insert({{-1, -1},
             {-2, -1},
             {-3, -2},
             {-4, -2},
             {-5, -3},
             {-6, -3},
             {-7, -3},
             {-8, -4},
             {-9, -4},
             {-10, -5},
             {-11, -5}});
  traverseLine(-1, -1, -11, -5, fill_action);
  TEST_TRAVERSE_LINE_EXPECT_EQ(gt, line);

  // Octant 6: slope < -1, x decreasing, y decreasing
  gt.clear();
  line.clear();
  gt.insert({{-1, -1},
             {-1, -2},
             {-2, -3},
             {-2, -4},
             {-3, -5},
             {-3, -6},
             {-3, -7},
             {-4, -8},
             {-4, -9},
             {-5, -10},
             {-5, -11}});
  traverseLine(-1, -1, -5, -11, fill_action);
  TEST_TRAVERSE_LINE_EXPECT_EQ(gt, line);

  // Octant 7: slope > 1, x increasing, y decreasing
  gt.clear();
  line.clear();
  gt.insert({{1, -1},
             {1, -2},
             {2, -3},
             {2, -4},
             {3, -5},
             {3, -6},
             {3, -7},
             {4, -8},
             {4, -9},
             {5, -10},
             {5, -11}});
  traverseLine(1, -1, 5, -11, fill_action);
  TEST_TRAVERSE_LINE_EXPECT_EQ(gt, line);

  // Octant 8: 0 < slope < 1, x increasing, y decreasing
  gt.clear();
  line.clear();
  gt.insert({{1, -1},
             {2, -1},
             {3, -2},
             {4, -2},
             {5, -3},
             {6, -3},
             {7, -3},
             {8, -4},
             {9, -4},
             {10, -5},
             {11, -5}});
  traverseLine(1, -1, 11, -5, fill_action);
  TEST_TRAVERSE_LINE_EXPECT_EQ(gt, line);
}  // TEST(data_structures_traverse_line, octant_wiki)

TEST(data_structures_traverse_line, edge)
{
  TraverseLineFillMultimapAction::MultimapType gt, line;
  TraverseLineFillMultimapAction fill_action(line);

  // Start and end are the same point
  gt.clear();
  line.clear();
  gt.insert({{3, 3}});
  traverseLine(3, 3, 3, 3, fill_action);
  TEST_TRAVERSE_LINE_EXPECT_EQ(gt, line);

  // Adjacent points (horizontal)
  gt.clear();
  line.clear();
  gt.insert({{3, 3}, {4, 3}});
  traverseLine(3, 3, 4, 3, fill_action);
  TEST_TRAVERSE_LINE_EXPECT_EQ(gt, line);

  // Adjacent points (vertical)
  gt.clear();
  line.clear();
  gt.insert({{3, 3}, {3, 4}});
  traverseLine(3, 3, 3, 4, fill_action);
  TEST_TRAVERSE_LINE_EXPECT_EQ(gt, line);

  // Adjacent points (diagonal)
  gt.clear();
  line.clear();
  gt.insert({{3, 3}, {4, 4}});
  traverseLine(3, 3, 4, 4, fill_action);
  TEST_TRAVERSE_LINE_EXPECT_EQ(gt, line);
}  // TEST(data_structures_traverse_line, critical)

TEST(data_structures_traverse_line, timing)
{
  constexpr float radius = 100.f / 0.001f;
  constexpr int num_rays = 360;
  constexpr float angle_res_deg = 360 / num_rays;

  float angle_res = geometry::degToRad(angle_res_deg);

  TraverseLineFillMultimapAction::MultimapType line_multimap;
  TraverseLineFillMultimapAction fill_multimap_action(line_multimap);

  PRINT_AND_LOG_TIME(
      "radius: 100.0 / 0.001 ; 361 rays; th: -pi .. pi; "
      "TraverseLineFillMultimapAction",

      for (int ray = 0; ray <= num_rays; ray++) {
        float angle = -M_PIf32 + ray * angle_res;
        int x = static_cast<int>(round(radius * cos(angle)));
        int y = static_cast<int>(round(radius * sin(angle)));

        line_multimap.clear();
        traverseLine(0, 0, x, y, fill_multimap_action);
      });

  TraverseLineFillVectorAction::VectorType line_vector;
  TraverseLineFillVectorAction fill_vector_action(line_vector);

  PRINT_AND_LOG_TIME(
      "radius: 100.0 / 0.001 ; 361 rays; th: -pi .. pi; "
      "TraverseLineFillVectorAction",

      for (int ray = 0; ray <= num_rays; ray++) {
        float angle = -M_PIf32 + ray * angle_res;
        int x = static_cast<int>(round(radius * cos(angle)));
        int y = static_cast<int>(round(radius * sin(angle)));

        line_vector.clear();
        traverseLine(0, 0, x, y, fill_vector_action);
      });

  TraverseLineEmptyAction empty_action;

  PRINT_AND_LOG_TIME(
      "radius: 100.0 / 0.001 ; 361 rays; th: -pi .. pi; "
      "TraverseLineEmptyAction",

      for (int ray = 0; ray <= num_rays; ray++) {
        float angle = -M_PIf32 + ray * angle_res;
        int x = static_cast<int>(round(radius * cos(angle)));
        int y = static_cast<int>(round(radius * sin(angle)));

        traverseLine(0, 0, x, y, empty_action);
      });
}  // TEST(data_structures_traverse_line, timing)

TEST(data_structures_traverse_line, timing_srrg)
{
  constexpr float radius = 100.f / 0.001f;
  constexpr int num_rays = 360;
  constexpr float angle_res_deg = 360 / num_rays;

  float angle_res = geometry::degToRad(angle_res_deg);

  TraverseLineFillMultimapAction::MultimapType line_multimap;
  TraverseLineFillMultimapAction fill_multimap_action(line_multimap);

  PRINT_AND_LOG_TIME(
      "radius: 100.0 / 0.001 ; 361 rays; th: -pi .. pi; "
      "TraverseLineFillMultimapAction",

      for (int ray = 0; ray <= num_rays; ray++) {
        float angle = -M_PIf32 + ray * angle_res;
        int x = static_cast<int>(round(radius * cos(angle)));
        int y = static_cast<int>(round(radius * sin(angle)));

        line_multimap.clear();
        traverseLineSRRG(0, 0, x, y, fill_multimap_action);
      });

  TraverseLineFillVectorAction::VectorType line_vector;
  TraverseLineFillVectorAction fill_vector_action(line_vector);

  PRINT_AND_LOG_TIME(
      "radius: 100.0 / 0.001 ; 361 rays; th: -pi .. pi; "
      "TraverseLineFillVectorAction",

      for (int ray = 0; ray <= num_rays; ray++) {
        float angle = -M_PIf32 + ray * angle_res;
        int x = static_cast<int>(round(radius * cos(angle)));
        int y = static_cast<int>(round(radius * sin(angle)));

        line_vector.clear();
        traverseLineSRRG(0, 0, x, y, fill_vector_action);
      });

  TraverseLineEmptyAction empty_action;

  PRINT_AND_LOG_TIME(
      "radius: 100.0 / 0.001 ; 361 rays; th: -pi .. pi; "
      "TraverseLineEmptyAction",

      for (int ray = 0; ray <= num_rays; ray++) {
        float angle = -M_PIf32 + ray * angle_res;
        int x = static_cast<int>(round(radius * cos(angle)));
        int y = static_cast<int>(round(radius * sin(angle)));

        traverseLineSRRG(0, 0, x, y, empty_action);
      });
}  // TEST(data_structures_traverse_line, timing_srrg)

TEST(data_structures_traverse_line, timing_karto)
{
  constexpr float radius = 100.f / 0.001f;
  constexpr int num_rays = 360;
  constexpr float angle_res_deg = 360 / num_rays;

  float angle_res = geometry::degToRad(angle_res_deg);

  TraverseLineFillMultimapAction::MultimapType line_multimap;
  TraverseLineFillMultimapAction fill_multimap_action(line_multimap);

  PRINT_AND_LOG_TIME(
      "radius: 100.0 / 0.001 ; 361 rays; th: -pi .. pi; "
      "TraverseLineFillMultimapAction",

      for (int ray = 0; ray <= num_rays; ray++) {
        float angle = -M_PIf32 + ray * angle_res;
        int x = static_cast<int>(round(radius * cos(angle)));
        int y = static_cast<int>(round(radius * sin(angle)));

        line_multimap.clear();
        traverseLineKarto(0, 0, x, y, fill_multimap_action);
      });

  TraverseLineFillVectorAction::VectorType line_vector;
  TraverseLineFillVectorAction fill_vector_action(line_vector);

  PRINT_AND_LOG_TIME(
      "radius: 100.0 / 0.001 ; 361 rays; th: -pi .. pi; "
      "TraverseLineFillVectorAction",

      for (int ray = 0; ray <= num_rays; ray++) {
        float angle = -M_PIf32 + ray * angle_res;
        int x = static_cast<int>(round(radius * cos(angle)));
        int y = static_cast<int>(round(radius * sin(angle)));

        line_vector.clear();
        traverseLineKarto(0, 0, x, y, fill_vector_action);
      });

  TraverseLineEmptyAction empty_action;

  PRINT_AND_LOG_TIME(
      "radius: 100.0 / 0.001 ; 361 rays; th: -pi .. pi; "
      "TraverseLineEmptyAction",

      for (int ray = 0; ray <= num_rays; ray++) {
        float angle = -M_PIf32 + ray * angle_res;
        int x = static_cast<int>(round(radius * cos(angle)));
        int y = static_cast<int>(round(radius * sin(angle)));

        traverseLineKarto(0, 0, x, y, empty_action);
      });
}  // TEST(data_structures_traverse_line, timing_karto)

}  // namespace testing
