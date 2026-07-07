#pragma once

#include <vector>

namespace inesctec_mrdt_slam_distmap_2d
{

namespace data_structures
{

/**
 * @brief correspondence generic class (to specifically define the fixed and
 *        moving indexes, instead of using std::pair)
 */
class Correspondence
{
 public:

  int m_idx_fixed = -1;    //!< fixed element index in fixed data container
  int m_idx_moving = -1;   //!< moving element index in moving data container
  float m_response = 0.f;  //!< correspondence response

 public:

  Correspondence(int idx_fixed = -1, int idx_moving = -1, float response = 0.f)
      : m_idx_fixed(idx_fixed), m_idx_moving(idx_moving), m_response(response)
  {
  }

};  // struct Correspondence

using CorrespondenceVector = std::vector<Correspondence>;

}  // namespace data_structures

}  // namespace inesctec_mrdt_slam_distmap_2d
