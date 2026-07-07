#pragma once

#include <srrg2_core/srrg_boss/id_context.h>
#include <srrg2_core/srrg_boss/object_data.h>
#include <srrg2_core/srrg_boss/serializable.h>

#include <cstdint>
#include <memory>
#include <string>
#include <utility>

namespace inesctec_mrdt_slam_distmap_2d
{

namespace slam
{

namespace msg
{

/**
 * @brief messages header (timestamp + frame id)
 */
struct Header : public srrg2_core::Serializable
{
 public:

  double m_t = -1;              //!< timestamp (s)
  std::string m_frame_id = "";  //!< frame id

 public:

  /**
   * @brief default constructor
   */
  Header() = default;

  /**
   * @brief construct a header message
   * @param[in] t timestamp (s)
   * @param[in] frame_id frame id
   */
  Header(double t, const std::string& frame_id) : m_t(t), m_frame_id(frame_id)
  {
  }

  /**
   * @brief construct a header message using move semantics on the parameters
   * @param[in] t timestamp (s)
   * @param[in] frame_id frame id (rvalue)
   */
  Header(double t, std::string&& frame_id)
      : m_t(t), m_frame_id(std::move(frame_id))
  {
  }

  /**
   * @brief move constructor
   * @param[in] other header message (rvalue)
   */
  Header(Header&& other) noexcept
      : m_t(other.m_t), m_frame_id(std::move(other.m_frame_id))
  {
  }

  /**
   * @brief move assignment operator
   * @param[in] other header message (rvalue)
   * @return Header& final header message
   */
  inline Header& operator=(Header&& other) noexcept
  {
    if (this != &other)
    {
      m_t = other.m_t;
      m_frame_id = std::move(other.m_frame_id);
    }
    return *this;
  }

  /**
   * @brief copy constructor
   * @param[in] other header message
   */
  Header(const Header& other) : m_t(other.m_t), m_frame_id(other.m_frame_id) {}

  /**
   * @brief copy assignment operator
   * @param[in] other header message
   * @return Header& final header message
   */
  inline Header& operator=(const Header& other)
  {
    if (this != &other)
    {
      m_t = other.m_t;
      m_frame_id = other.m_frame_id;
    }
    return *this;
  }

  friend std::ostream& operator<<(std::ostream& os, const Header& msg)
  {
    os << "t    (s): " << msg.m_t << std::endl
       << "frame_id: " << msg.m_frame_id << std::endl;

    return os;
  }

 public:

  /**
   * @brief SRRG BOSS serialization method
   * @param[in,out] data current serialized data
   * @param[in] context serialization context
   */
  void serialize(srrg2_core::ObjectData& data,
                 srrg2_core::IdContext& context) final;

  /**
   * @brief SRRG BOSS deserialization method
   * @param[in] data loaded serialized data
   * @param[in] context serialization context
   */
  void deserialize(srrg2_core::ObjectData& data,
                   srrg2_core::IdContext& context) final;
};  // struct Header

using HeaderPtr = std::shared_ptr<Header>;
using HeaderConstPtr = std::shared_ptr<const Header>;

}  // namespace msg

}  // namespace slam

}  // namespace inesctec_mrdt_slam_distmap_2d
