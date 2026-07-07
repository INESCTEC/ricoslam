#pragma once

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <map>
#include <utility>
#include <vector>

namespace inesctec_mrdt_slam_distmap_2d
{

namespace data_structures
{

/**
 * @brief empty action for the traverse line algorithm
 */
struct TraverseLineEmptyAction
{
  inline bool operator()(int, int) { return true; }
};  // struct TraverseLineEmptyAction

/**
 * @brief print path action for the traverse line algorithm
 */
struct TraverseLinePrintPathAction
{
  inline bool operator()(int x, int y)
  {
    std::cout << "(" << x << "," << y << ")" << std::endl;
    return true;
  }
};  // struct TraverseLinePrintPathAction

/**
 * @brief fill action for vector container of std::pair<int,int> elements when
 *        traversing a line
 *        (faster than multimap insertion, given that later is sorted...)
 */
struct TraverseLineFillVectorAction
{
  using VectorType = std::vector<std::pair<int, int>>;

  VectorType& m_line;

  TraverseLineFillVectorAction(VectorType& line) : m_line(line) {}

  inline bool operator()(int x, int y)
  {
    m_line.emplace_back(std::make_pair(x, y));
    return true;
  }
};  // struct TraverseLineFillVectorAction

/**
 * @brief fill action for multimap associative container when traversing a line
 */
struct TraverseLineFillMultimapAction
{
  using MultimapType = std::multimap<int, int>;

  MultimapType& m_line;

  TraverseLineFillMultimapAction(MultimapType& line) : m_line(line) {}

  inline bool operator()(int x, int y)
  {
    m_line.emplace(std::make_pair(x, y));
    return true;
  }
};  // struct TraverseLineFillMultimapAction

/**
 * @brief Bresenham-based traverse line algorithm
 *
 * Sources:
 * - https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
 * - https://zingl.github.io/Bresenham.pdf (1.7 Program to plot a line)
 *
 * @tparam ActionType hitting cell action type when traversing the line
 * @param[in] x0 x coordinate of the line's starting point
 * @param[in] y0 y coordinate of the line's starting point
 * @param[in] x1 x coordinate of the line's final point
 * @param[in] y1 y coordinate of the line's final point
 * @param[in] action hitting cell action object when traversing the line
 *                   (need to define the bool operator()(int,int))
 * @return true algorithm traversed the whole line between the two endpoints
 *              (based on the action's feedback from bool operator()(int,int))
 * @return false otherise
 */
template <typename ActionType>
bool traverseLine(int x0, int y0, int x1, int y1, ActionType& action)
{
  int dx = abs(x1 - x0);
  int dy = -abs(y1 - y0);
  int xi = x0 < x1 ? 1 : -1;
  int yi = y0 < y1 ? 1 : -1;

  int err = dx + dy;
  int err2;

  while (true)
  {
    if (!action(x0, y0))
    {
      return false;
    }

    err2 = 2 * err;

    if (err2 >= dy)
    {
      if (x0 == x1)
      {
        break;
      }

      err += dy;
      x0 += xi;
    }

    if (err2 <= dx)
    {
      if (y0 == y1)
      {
        break;
      }

      err += dx;
      y0 += yi;
    }
  }

  return true;
}  // bool traverseLine<ActionType>(int, int, int, int, ActionType&)

/**
 * @brief Bresenham-based traverse line algorithm (SRRG implementation)
 *
 * Unit tests failed:
 * - {{5, 0}, {5, 1}, {4, 2}, {4, 3}, {3, 4}, {3, 5}}
 *   traverseLineSRRG(5, 0, 3, 5, fill_action);
 * - {{5, 5}, {5, 4}, {4, 3}, {4, 2}, {3, 1}, {3, 0}}
 *   traverseLineSRRG(5, 5, 3, 0, fill_action);
 * - {{-1,1},{-1,2},{-2,3},{-2,4},{-3,5},{-3,6},{-3,7},{-4,8},{-4,9},{-5,10},
 *   {-5,11}}
 *   traverseLineSRRG(-1, 1, -5, 11, fill_action);
 * - {{-1,-1},{-1,-2},{-2,-3},{-2,-4},{-3,-5},{-3,-6},{-3,-7},{-4,-8},{-4,-9},
 *   {-5,-10},{-5,-11}}
 *   traverseLineSRRG(-1, -1, -5, -11, fill_action);
 *
 * @note when testing this function with the unit test, it fails on two tests
 *       (seems related to rounding y coordinates along the rasterized line...)
 * @tparam ActionType hitting cell action type when traversing the line
 * @param[in] x0 x coordinate of the line's starting point
 * @param[in] y0 y coordinate of the line's starting point
 * @param[in] x1 x coordinate of the line's final point
 * @param[in] y1 y coordinate of the line's final point
 * @param[in] action hitting cell action object when traversing the line
 *                   (need to define the bool operator()(int,int))
 * @return true algorithm traversed the whole line between the two endpoints
 *              (based on the action's feedback from bool operator()(int,int))
 * @return false otherise
 */
template <typename ActionType>
bool traverseLineSRRG(int x0, int y0, int x1, int y1, ActionType& action)
{
  bool swapped = false;
  if (abs(y1 - y0) > abs(x1 - x0))
  {
    swapped = true;
    std::swap(x0, y0);
    std::swap(x1, y1);
  }
  int dx = (x1 - x0);
  int dy = (y1 - y0);
  int inc = (x1 - x0) >= 0 ? 1 : -1;
  float slope = 0.f;
  if (dx)
  {
    slope = (float) dy / (float) dx;
  }
  dx += inc;

  for (int d = 0; d != dx; d += inc)
  {
    int x = x0 + d;
    int y = y0 + d * slope;
    if (swapped)
    {
      if (!action(y, x))
      {
        return false;
      }
    }
    else
    {
      if (!action(x, y))
      {
        return false;
      }
    }
  }
  return true;
}  // bool traverseLineSRRG<ActionType>(int, int, int, int, ActionType&)

/**
 * @brief Bresenham-based traverse line algorithm (Karto implementation)
 *
 * Source:
 * https://docs.ros.org/en/diamondback/api/karto/html/classkarto_1_1Grid.html#a8b737b0a00fa16fc1a8337e9b13eee6c
 * https://docs.ros.org/en/diamondback/api/karto/html/Karto_8h_source.html#l04474
 * void TraceLine(kt_int32s, kt_int32s, kt_int32s, kt_int32s, Functor* = NULL)
 *
 * Unit tests failed:
 * - {{0, 0}, {-1, 0}, {-2, 0}, {-3, 0}, {-4, 0}, {-5, 0}, {-6, 0}}
 *   traverseLineKarto(0, 0, -6, 0, fill_action);
 * - {{7, 0}, {7, -1}, {7, -2}, {7, -3}, {7, -4}, {7, -5}, {7, -6}}
 *   traverseLineKarto(7, 0, 7, -6, fill_action);
 * - {{5, 3}, {5, 2}, {5, 1}, {5, 0}, {5, -1}, {5, -2}, {5, -3}}
 *   traverseLineKarto(5, 3, 5, -3, fill_action);
 * - {{7, 3}, {7, 2}, {7, 1}, {7, 0}, {7, -1}, {7, -2}, {7, -3}}
 *   traverseLineKarto(7, 3, 7, -3, fill_action);
 * - {{5, 5}, {5, 4}, {4, 3}, {4, 2}, {3, 1}, {3, 0}}
 *   traverseLineKarto(5, 5, 3, 0, fill_action);
 * - {{0, 5}, {0, 4}, {1, 3}, {1, 2}, {2, 1}, {2, 0}}
 *   traverseLineKarto(0, 5, 2, 0, fill_action);
 * - {{-1,-1},{-1,-2},{-2,-3},{-2,-4},{-3,-5},{-3,-6},{-3,-7},{-4,-8},{-4,-9},
 *   {-5,-10},{-5, -11}}
 *   traverseLineKarto(-1, -1, -5, -11, fill_action);
 * - {{1,-1},{1,-2},{2,-3},{2,-4},{3,-5},{3,-6},{3,-7},{4,-8},{4,-9},{5,-10},
 *   {5,-11}}
 *   traverseLineKarto(1, -1, 5, -11, fill_action);
 *
 * @tparam ActionType hitting cell action type when traversing the line
 * @param[in] x0 x coordinate of the line's starting point
 * @param[in] y0 y coordinate of the line's starting point
 * @param[in] x1 x coordinate of the line's final point
 * @param[in] y1 y coordinate of the line's final point
 * @param[in] action hitting cell action object when traversing the line
 *                   (need to define the bool operator()(int,int))
 * @return true algorithm traversed the whole line between the two endpoints
 *              (based on the action's feedback from bool operator()(int,int))
 * @return false otherise
 */
template <typename ActionType>
bool traverseLineKarto(int x0, int y0, int x1, int y1, ActionType& action)
{
  bool steep = abs(y1 - y0) > abs(x1 - x0);
  if (steep)
  {
    std::swap(x0, y0);
    std::swap(x1, y1);
  }
  if (x0 > x1)
  {
    std::swap(x0, x1);
    std::swap(y0, y1);
  }
  int deltaX = x1 - x0;
  int deltaY = abs(y1 - y0);
  int error = 0;
  int ystep;
  int y = y0;
  if (y0 < y1)
  {
    ystep = 1;
  }
  else
  {
    ystep = -1;
  }
  int pointX;
  int pointY;
  for (int x = x0; x <= x1; x++)
  {
    if (steep)
    {
      pointX = y;
      pointY = x;
    }
    else
    {
      pointX = x;
      pointY = y;
    }
    error += deltaY;
    if (2 * error >= deltaX)
    {
      y += ystep;
      error -= deltaX;
    }

    if (!action(pointX, pointY))
    {
      return false;
    }
  }
  return true;
}  // bool traverseLineKarto<ActionType>(int, int, int, int, ActionType&)

}  // namespace data_structures

}  // namespace inesctec_mrdt_slam_distmap_2d
