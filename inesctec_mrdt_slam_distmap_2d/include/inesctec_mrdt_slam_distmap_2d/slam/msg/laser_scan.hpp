#pragma once

#include <srrg2_core/srrg_boss/id_context.h>
#include <srrg2_core/srrg_boss/object_data.h>
#include <srrg2_core/srrg_boss/serializable.h>

#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include "inesctec_mrdt_slam_distmap_2d/geometry/defs.hpp"
#include "inesctec_mrdt_slam_distmap_2d/slam/msg/header.hpp"

namespace inesctec_mrdt_slam_distmap_2d
{

namespace slam
{

namespace msg
{

/**
 * @brief 2D laser scanner message with range and intensity data
 */
struct LaserScan : public srrg2_core::Serializable
{
 public:

  Header m_header;              //!< timestamp (acquisition time of first ray)
                                //!< angles around positive Z axis of frame_id
  float m_angle_min = 0.f;      //!< start angle (rad)
  float m_angle_max = 0.f;      //!< end angle (rad)
  float m_angle_inc = -1.f;     //!< angle between measurements (rad)
  float m_time_inc = -1.f;      //!< time between measurements (s) -
                                //!< if laser moving will be used to interpolate
                                //!< position of 3d points
  float m_scan_time = -1.f;     //!< time between scans (s)
  float m_range_min = -1.f;     //!< minimum range value (m)
  float m_range_max = -1.f;     //!< maximum range value (m)
  std::vector<float> m_ranges;  //!< range data (m) (values < range_min or >
                                //!< range_max should be discarded)
  std::vector<float>
      m_intensities;  //!< intensity data (device-specific) - if laser does not
                      //!< provide intensities, leave the array empty.

 public:

  /**
   * @brief default constructor
   */
  LaserScan() = default;

  /**
   * @brief construct a laser scan message
   * @param[in] header timestamp (acquisition time of first ray)
   *                   angles around positive Z axis of frame_id
   * @param[in] angle_min start angle (rad)
   * @param[in] angle_max end angle (rad)
   * @param[in] angle_inc angle between measurements (rad)
   * @param[in] time_inc time between measurements (s)
   * @param[in] scan_time time between scans (s)
   * @param[in] range_min minimum range value (m)
   * @param[in] range_max maximum range value (m)
   * @param[in] ranges range data (m)
   * @param[in] intensities intensity data (device-specific)
   */
  LaserScan(const Header& header, float angle_min, float angle_max,
            float angle_inc, float time_inc, float scan_time, float range_min,
            float range_max, const std::vector<float>& ranges,
            const std::vector<float>& intensities)
      : m_header(header),
        m_angle_min(angle_min),
        m_angle_max(angle_max),
        m_angle_inc(angle_inc),
        m_time_inc(time_inc),
        m_scan_time(scan_time),
        m_range_min(range_min),
        m_range_max(range_max),
        m_ranges(ranges),
        m_intensities(intensities)
  {
  }

  /**
   * @brief construct a laser scan message using move semantics on the
   *        parameters
   * @param[in] header timestamp (acquisition time of first ray)
   *                   angles around positive Z axis of frame_id
   * @param[in] angle_min start angle (rad)
   * @param[in] angle_max end angle (rad)
   * @param[in] angle_inc angle between measurements (rad)
   * @param[in] time_inc time between measurements (s)
   * @param[in] scan_time time between scans (s)
   * @param[in] range_min minimum range value (m)
   * @param[in] range_max maximum range value (m)
   * @param[in] ranges range data (m)
   * @param[in] intensities intensity data (device-specific)
   */
  LaserScan(Header&& header, float angle_min, float angle_max, float angle_inc,
            float time_inc, float scan_time, float range_min, float range_max,
            std::vector<float>&& ranges, std::vector<float>&& intensities)
      : m_header(std::move(header)),
        m_angle_min(angle_min),
        m_angle_max(angle_max),
        m_angle_inc(angle_inc),
        m_time_inc(time_inc),
        m_scan_time(scan_time),
        m_range_min(range_min),
        m_range_max(range_max),
        m_ranges(std::move(ranges)),
        m_intensities(std::move(intensities))
  {
  }

  /**
   * @brief move constructor
   * @param[in] other laser scan message (rvalue)
   */
  LaserScan(LaserScan&& other) noexcept
      : m_header(std::move(other.m_header)),
        m_angle_min(other.m_angle_min),
        m_angle_max(other.m_angle_max),
        m_angle_inc(other.m_angle_inc),
        m_time_inc(other.m_time_inc),
        m_scan_time(other.m_scan_time),
        m_range_min(other.m_range_min),
        m_range_max(other.m_range_max),
        m_ranges(std::move(other.m_ranges)),
        m_intensities(std::move(other.m_intensities))
  {
  }

  /**
   * @brief move assignment operator
   * @param[in] other laser scan message (rvalue)
   * @return LaserScan& final laser scan message
   */
  inline LaserScan& operator=(LaserScan&& other) noexcept
  {
    if (this != &other)
    {
      m_header = std::move(other.m_header);
      m_angle_min = other.m_angle_min;
      m_angle_max = other.m_angle_max;
      m_angle_inc = other.m_angle_inc;
      m_time_inc = other.m_time_inc;
      m_scan_time = other.m_scan_time;
      m_range_min = other.m_range_min;
      m_range_max = other.m_range_max;
      m_ranges = std::move(other.m_ranges);
      m_intensities = std::move(other.m_intensities);
    }
    return *this;
  }

  /**
   * @brief copy constructor
   * @param[in] other laser scan message
   */
  LaserScan(const LaserScan& other)
      : m_header(other.m_header),
        m_angle_min(other.m_angle_min),
        m_angle_max(other.m_angle_max),
        m_angle_inc(other.m_angle_inc),
        m_time_inc(other.m_time_inc),
        m_scan_time(other.m_scan_time),
        m_range_min(other.m_range_min),
        m_range_max(other.m_range_max),
        m_ranges(other.m_ranges),
        m_intensities(other.m_intensities)
  {
  }

  /**
   * @brief copy assignment operator
   * @param[in] other laser scan message
   * @return LaserScan& final laser scan message
   */
  inline LaserScan& operator=(const LaserScan& other)
  {
    if (this != &other)
    {
      m_header = other.m_header;
      m_angle_min = other.m_angle_min;
      m_angle_max = other.m_angle_max;
      m_angle_inc = other.m_angle_inc;
      m_time_inc = other.m_time_inc;
      m_scan_time = other.m_scan_time;
      m_range_min = other.m_range_min;
      m_range_max = other.m_range_max;
      m_ranges = other.m_ranges;
      m_intensities = other.m_intensities;
    }
    return *this;
  }

  /**
   * @brief convert ray (angle, distance) values to point 2D w.r.t. laser frame
   * @tparam Scalar point scalar type
   * @param[out] pt point w.r.t. laser coordinate frame (m)
   * @param[in] angle ray angle (rad)
   * @param[in] dist ray distance (m)
   */
  template <typename Scalar>
  inline static void angleDistToPt(geometry::Vector2_<Scalar>& pt,
                                   const Scalar angle, const Scalar dist)
  {
    pt.x() = dist * cos(angle);
    pt.y() = dist * sin(angle);
  }

  /**
   * @brief convert point 2D w.r.t. laser to ray angle (rad)
   * @tparam Scalar point scalar type
   * @param[in] pt point w.r.t. laser coordinate frame (m)
   * @return Scalar ray angle (rad)
   */
  template <typename Scalar>
  inline static Scalar ptToAngle(const geometry::Vector2_<Scalar>& pt)
  {
    return atan2(pt.y(), pt.x());
  }

  /**
   * @brief convert point 2D w.r.t. laser to ray distance (m)
   * @tparam Scalar point scalar type
   * @param[in] pt point w.r.t. laser coordinate frame (m)
   * @return Scalar ray distance (m)
   */
  template <typename Scalar>
  inline static Scalar ptToDist(const geometry::Vector2_<Scalar>& pt)
  {
    return std::sqrt(pt.x() * pt.x() + pt.y() * pt.y());
  }

  /**
   * @brief convert point 2D w.r.t. laser frame to ray (angle, distance) values
   * @tparam Scalar point scalar type
   * @param[out] angle ray angle (rad)
   * @param[out] dist ray distance (m)
   * @param[in] pt point w.r.t. laser coordinate frame (m)
   */
  template <typename Scalar>
  inline static void ptToAngleDist(Scalar& angle, Scalar& dist,
                                   const geometry::Vector2_<Scalar>& pt)
  {
    angle = ptToAngle(pt);
    dist = ptToDist(pt);
  }

  /**
   * @brief convert ray angle to index utility method
   * @tparam Scalar angle scalar type
   * @param[in] angle ray angle (rad)
   * @return int ray index
   */
  template <typename Scalar>
  inline int angleToRayIdx(const Scalar angle) const
  {
    /* return geometry::wrapToRange(
        static_cast<int>(std::round((angle - m_angle_min) / m_angle_inc)),
        static_cast<int>(m_ranges.size())); */
    return static_cast<int>(std::round((angle - m_angle_min) / m_angle_inc));
  }

  /**
   * @brief convert point 2D w.r.t. laser frame to ray index
   * @tparam Scalar point scalar type
   * @param[in] pt point w.r.t. laser coordinate frame (m)
   * @return int ray index
   */
  template <typename Scalar>
  inline int ptToRayIdx(const geometry::Vector2_<Scalar>& pt) const
  {
    return angleToRayIdx(ptToAngle(pt));
  }

  /**
   * @brief check if a laser scan message has a 360º Field-of-View (FoV) or not
   *        (fov >= 2 x minimum angle - angle resolution * 0.25)
   * @return true message has a 360º FOV
   * @return false otherwise
   */
  inline bool is360() const
  {
    float fov = std::fabs(m_angle_min) * 2.f;

    return (fov >= 2.0f * M_PIf32 - m_angle_inc * 0.25f);
  }

  /**
   * @brief invert 2D laser scan data
   *        (utility function when laser upside down)
   */
  inline void invert()
  {
    std::swap(m_angle_min, m_angle_max);

    m_angle_min = geometry::wrapToPi(-m_angle_min);
    m_angle_max = geometry::wrapToPi(-m_angle_max);

    std::reverse(m_ranges.begin(), m_ranges.end());
    std::reverse(m_intensities.begin(), m_intensities.end());
  }

  friend std::ostream& operator<<(std::ostream& os, const LaserScan& msg)
  {
    os << "header.t    (s): " << msg.m_header.m_t << std::endl
       << "header.frame_id: " << msg.m_header.m_frame_id << std::endl
       << "angle_min (º): " << geometry::radToDeg(msg.m_angle_min) << std::endl
       << "angle_max (º): " << geometry::radToDeg(msg.m_angle_max) << std::endl
       << "angle_inc (º): " << geometry::radToDeg(msg.m_angle_inc) << std::endl
       << "time_inc  (s): " << msg.m_time_inc << std::endl
       << "scan_time (s): " << msg.m_scan_time << std::endl
       << "range_min (m): " << msg.m_range_min << std::endl
       << "range_max (m): " << msg.m_range_max << std::endl
       << "ranges (m): [" << msg.m_ranges.size() << "] ";

    for (const auto& r : msg.m_ranges)
    {
      os << r << " ";
    }

    os << std::endl << "intensities: [" << msg.m_intensities.size() << "] ";

    for (const auto& i : msg.m_intensities)
    {
      os << i << " ";
    }

    os << std::endl;

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

};  // struct LaserScan

using LaserScanPtr = std::shared_ptr<LaserScan>;
using LaserScanConstPtr = std::shared_ptr<const LaserScan>;

}  // namespace msg

}  // namespace slam

}  // namespace inesctec_mrdt_slam_distmap_2d
