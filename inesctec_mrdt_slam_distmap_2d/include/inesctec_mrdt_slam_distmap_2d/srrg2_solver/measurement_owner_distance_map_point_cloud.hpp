#pragma once

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <memory>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

// SRRG Software
#include <srrg2_core/srrg_boss/object_data.h>
#include <srrg2_core/srrg_pcl/instances.h>
#include <srrg2_core/srrg_pcl/point_types.h>
#include <srrg2_core/srrg_pcl/point_types_data.h>
#include <srrg2_solver/solver_core/measurement_owner.h>

#include "inesctec_mrdt_slam_distmap_2d/data_structures/distance_map_nano_static.hpp"
#include "inesctec_mrdt_slam_distmap_2d/data_structures/distance_map_sparse_static.hpp"
#include "inesctec_mrdt_slam_distmap_2d/data_structures/distance_map_static.hpp"
#include "inesctec_mrdt_slam_distmap_2d/geometry/geometry_2d.hpp"
#include "inesctec_mrdt_slam_distmap_2d/pcl/point_cloud.hpp"
#include "inesctec_mrdt_slam_distmap_2d/pcl/point_types.hpp"
#include "inesctec_mrdt_slam_distmap_2d/slam/msg/laser_scan.hpp"

namespace srrg2_solver
{

/**
 * @brief measurement holder structure to consider as a single measurement both
 *        the distance map, the point cloud, and the laser message(s) used to
 *        create the local point cloud
 *
 * distance map does not hold the point cloud. consequently, in order to save
 * the original point cloud, we need to create this container (also to avoid
 * consideration of all combinations between distance maps and point cloud
 * types...).
 *
 * additionally, the laser scan message(s) are required to generate 2D occupancy
 * grid maps.
 *
 * @tparam DistanceMapTypePtr_ smart pointer of the distance map object type
 * @tparam PointCloudTypePtr_  smart pointer of the point cloud object type
 * @tparam EstimateType_ estimate type (laser message w.r.t. local frame)
 */
template <typename DistanceMapTypePtr_, typename PointCloudTypePtr_,
          typename EstimateType_>
struct DistanceMapPointCloudLaserMsgsContainer_
{
 public:

  using ThisType = DistanceMapPointCloudLaserMsgsContainer_<
      DistanceMapTypePtr_, PointCloudTypePtr_, EstimateType_>;

  using EstimateType = EstimateType_;

  using DistanceMapTypePtr = DistanceMapTypePtr_;
  using DistanceMapType = typename DistanceMapTypePtr::element_type;
  using PointCloudTypePtr = PointCloudTypePtr_;
  using PointCloudType = typename PointCloudTypePtr::element_type;
  using ScalarType = typename DistanceMapType::Scalar;

  using LaserScanMsgType = inesctec_mrdt_slam_distmap_2d::slam::msg::LaserScan;
  using LaserScanMsgTypePtr = std::shared_ptr<LaserScanMsgType>;
  using LaserScanMsgContainerType =
      std::vector<std::pair<LaserScanMsgTypePtr, EstimateType>>;
  using LaserScanMsgContainerTypePtr =
      std::shared_ptr<LaserScanMsgContainerType>;

 public:

  DistanceMapTypePtr m_dist_map =
      std::make_shared<DistanceMapType>();  //!< distance map container
  PointCloudTypePtr m_pc =
      std::make_shared<PointCloudType>();  //!< original point cloud container

  LaserScanMsgContainerTypePtr m_msgs =
      std::make_shared<LaserScanMsgContainerType>();  //!< laser scan messages

  EstimateType m_sensor_in_robot =
      EstimateType::Identity();  //!< calibrated SE(2) pose of the sensor w.r.t.
                                 //!< the robot coordinate frame

 public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  DistanceMapPointCloudLaserMsgsContainer_() = default;

  inline const DistanceMapTypePtr& getDistanceMapPtr() const
  {
    return m_dist_map;
  }
  inline DistanceMapType* getDistanceMap() const { return m_dist_map.get(); }
  inline const PointCloudTypePtr& getPointCloudPtr() const { return m_pc; }
  inline PointCloudType* getPointCloud() const { return m_pc.get(); }
  inline const LaserScanMsgContainerTypePtr& getLaserScanMsgsPtr() const
  {
    return m_msgs;
  }
  inline LaserScanMsgContainerType* getLaserScanMsgs() const
  {
    return m_msgs.get();
  }

  inline void setSensorInRobot(const EstimateType& sensor_in_robot)
  {
    m_sensor_in_robot = sensor_in_robot;
  }

  inline const EstimateType& getSensorInRobot() const
  {
    return m_sensor_in_robot;
  }

};  // struct DistanceMapPointCloudLaserMsgsContainer_<DistanceMapTypePtr_,
    // PointCloudTypePtr_, EstimateType_>

template <typename DistanceMapTypePtr_, typename PointCloudTypePtr_,
          typename EstimateType_>
using DistanceMapPointCloudLaserMsgsContainerPtr_ =
    std::shared_ptr<DistanceMapPointCloudLaserMsgsContainer_<
        DistanceMapTypePtr_, PointCloudTypePtr_, EstimateType_>>;

/**
 * @brief measurement owner of a distance map and a point cloud
 *        (coordinates-only data) compatible with the implementation of SRRG
 *        software that enables variables and/or factors to have measurement
 *        owners that can be serialized or desesrialized upon saving the factor
 *        graph
 * @tparam DistanceMapTypePtr_ smart pointer of the distance map object type
 * @tparam PointCloudTypePtr_  smart pointer of the point cloud object type
 * @tparam EstimateType_ estimate type (laser message w.r.t. local frame)
 */
template <typename DistanceMapTypePtr_, typename PointCloudTypePtr_,
          typename EstimateType_>
struct MeasurementOwnerDistanceMapPointCloudLaserMsgs_
    : public MeasurementOwner_<DistanceMapPointCloudLaserMsgsContainerPtr_<
          DistanceMapTypePtr_, PointCloudTypePtr_, EstimateType_>>
{
 public:

  using ThisType = MeasurementOwnerDistanceMapPointCloudLaserMsgs_<
      DistanceMapTypePtr_, PointCloudTypePtr_, EstimateType_>;
  using BaseType =
      MeasurementOwner_<DistanceMapPointCloudLaserMsgsContainerPtr_<
          DistanceMapTypePtr_, PointCloudTypePtr_, EstimateType_>>;

  using MeasurementType = DistanceMapPointCloudLaserMsgsContainerPtr_<
      DistanceMapTypePtr_, PointCloudTypePtr_, EstimateType_>;
  using MeasurementObjType = typename MeasurementType::element_type;

  using DistanceMapTypePtr = DistanceMapTypePtr_;
  using DistanceMapType = typename DistanceMapTypePtr::element_type;

  using PointCloudTypePtr = PointCloudTypePtr_;
  using PointCloudType = typename PointCloudTypePtr::element_type;
  using PointCloudPointType = typename PointCloudType::PointType;
  using PointCloudPointBaseType = typename PointCloudPointType::BaseType;

  using LaserScanMsgType = typename MeasurementObjType::LaserScanMsgType;
  using LaserScanMsgTypePtr = typename MeasurementObjType::LaserScanMsgTypePtr;
  using LaserScanMsgContainerType =
      typename MeasurementObjType::LaserScanMsgContainerType;
  using LaserScanMsgContainerTypePtr =
      typename MeasurementObjType::LaserScanMsgContainerTypePtr;

  using EstimateType = EstimateType_;

 public:

  virtual ~MeasurementOwnerDistanceMapPointCloudLaserMsgs_() = default;

  /****************************************************************************
   * @brief SRRG SOFTWARE BOSS SERIALIZATION METHODS
   ****************************************************************************/

  void serializeMeasurement(srrg2_core::ObjectData& data,
                            srrg2_core::IdContext& context) override
  {
    if (!this->_measurement)
    {
      return;
    }

    data.setEigen<EstimateType>("sensor_in_robot",
                                this->_measurement->m_sensor_in_robot);

    if (this->_measurement->m_dist_map)
    {
      srrg2_core::ObjectData* measurement_boss_obj = new srrg2_core::ObjectData;

      this->_measurement->m_dist_map->serialize(*measurement_boss_obj, context);

      data.setField("distance_map", measurement_boss_obj);
    }

    if (this->_measurement->m_msgs)
    {
      if (this->_measurement->m_msgs->size() > 0)
      {
        srrg2_core::ObjectData* msgs_data = new srrg2_core::ObjectData;

        for (size_t idx = 0; idx < this->_measurement->m_msgs->size(); idx++)
        {
          std::string idx_str = "(" + std::to_string(idx) + ")";

          srrg2_core::ObjectData* msg_data = new srrg2_core::ObjectData;

          this->_measurement->m_msgs->operator[](idx).first->serialize(
              *msg_data, context);
          msg_data->setEigen(
              "pose", this->_measurement->m_msgs->operator[](idx).second);

          msgs_data->setField(idx_str, msg_data);
        }

        data.setField("msgs", msgs_data);
      }
    }

    if (!this->_measurement->m_pc)
    {
      return;
    }
    if (this->_measurement->m_pc->size() == 0)
    {
      return;
    }

    serializePointCloud(data, context);
  }

  template <
      typename T = PointCloudPointBaseType,
      std::enable_if_t<
          std::is_same<
              T, inesctec_mrdt_slam_distmap_2d::pcl::PointDerived_<
                     inesctec_mrdt_slam_distmap_2d::pcl::Point_<
                         PointCloudPointType::Dim,
                         typename PointCloudPointType::ComponentsType>,
                     inesctec_mrdt_slam_distmap_2d::pcl::PointFieldDirection_<
                         typename PointCloudPointType::ComponentsType,
                         PointCloudPointType::Dim>>>::value,
          bool> = 0>
  void serializePointCloud(srrg2_core::ObjectData& data,
                           srrg2_core::IdContext& context)
  {
    using PointCloudDataSRRGPointType =
        srrg2_core::PointNormal_<PointCloudPointType::Dim,
                                 typename PointCloudPointType::ComponentsType>;
    using PointCloudDataSRRGContainerType =
        srrg2_core::VectorData_<srrg2_core::PointCloud_<std::vector<
            PointCloudDataSRRGPointType,
            Eigen::aligned_allocator<PointCloudDataSRRGPointType>>>>;

    srrg2_core::BLOBReference<PointCloudDataSRRGContainerType> pc_blob;

    pc_blob.set(new PointCloudDataSRRGContainerType);
    pc_blob.get()->resize(this->_measurement->m_pc->size());

    for (size_t idx = 0; idx < this->_measurement->m_pc->size(); idx++)
    {
      pc_blob.get()->operator[](idx).coordinates() =
          this->_measurement->m_pc->operator[](idx).coordinates();
      pc_blob.get()->operator[](idx).normal() =
          this->_measurement->m_pc->operator[](idx).normal();
    }

    srrg2_core::ObjectData* measurement_boss_obj = new srrg2_core::ObjectData;

    pc_blob.serialize(*measurement_boss_obj, context);

    data.setField("pc", measurement_boss_obj);
  }

  template <
      typename T = PointCloudPointBaseType,
      std::enable_if_t<
          !std::is_same<
              T, inesctec_mrdt_slam_distmap_2d::pcl::PointDerived_<
                     inesctec_mrdt_slam_distmap_2d::pcl::Point_<
                         PointCloudPointType::Dim,
                         typename PointCloudPointType::ComponentsType>,
                     inesctec_mrdt_slam_distmap_2d::pcl::PointFieldDirection_<
                         typename PointCloudPointType::ComponentsType,
                         PointCloudPointType::Dim>>>::value,
          bool> = 0>
  void serializePointCloud(srrg2_core::ObjectData& data,
                           srrg2_core::IdContext& context)
  {
    using PointCloudDataSRRGPointType =
        srrg2_core::Point_<PointCloudPointType::Dim,
                           typename PointCloudPointType::ComponentsType>;
    using PointCloudDataSRRGContainerType =
        srrg2_core::VectorData_<srrg2_core::PointCloud_<std::vector<
            PointCloudDataSRRGPointType,
            Eigen::aligned_allocator<PointCloudDataSRRGPointType>>>>;

    srrg2_core::BLOBReference<PointCloudDataSRRGContainerType> pc_blob;

    pc_blob.set(new PointCloudDataSRRGContainerType);
    pc_blob.get()->resize(this->_measurement->m_pc->size());

    for (size_t idx = 0; idx < this->_measurement->m_pc->size(); idx++)
    {
      pc_blob.get()->operator[](idx).coordinates() =
          this->_measurement->m_pc->operator[](idx).coordinates();
    }

    srrg2_core::ObjectData* measurement_boss_obj = new srrg2_core::ObjectData;

    pc_blob.serialize(*measurement_boss_obj, context);

    data.setField("pc", measurement_boss_obj);
  }

  void deserializeMeasurement(srrg2_core::ObjectData& data,
                              srrg2_core::IdContext& context) override
  {
    if (data.getField("sensor_in_robot"))
    {
      if (!this->_measurement)
      {
        this->_measurement = std::make_shared<MeasurementObjType>();
      }

      this->_measurement->m_sensor_in_robot =
          data.getEigen<EstimateType>("sensor_in_robot");
    }

    if (data.getField("distance_map"))
    {
      if (!this->_measurement)
      {
        this->_measurement = std::make_shared<MeasurementObjType>();
      }

      srrg2_core::ObjectData* measurement_boss_obj =
          dynamic_cast<ObjectData*>(data.getField("distance_map"));

      if (measurement_boss_obj)
      {
        this->_measurement->m_dist_map = std::make_shared<DistanceMapType>();
        this->_measurement->m_dist_map->deserialize(*measurement_boss_obj,
                                                    context);
      }
    }

    if (data.getField("msgs"))
    {
      if (!this->_measurement)
      {
        this->_measurement = std::make_shared<MeasurementObjType>();
      }

      srrg2_core::ObjectData* msgs_data =
          dynamic_cast<srrg2_core::ObjectData*>(data.getField("msgs"));

      this->_measurement->m_msgs =
          std::make_shared<LaserScanMsgContainerType>();

      size_t idx = 0;
      std::string idx_str = "(" + std::to_string(idx) + ")";

      while (msgs_data->getField(idx_str))
      {
        LaserScanMsgTypePtr msg = std::make_shared<LaserScanMsgType>();

        srrg2_core::ObjectData* msg_data =
            dynamic_cast<srrg2_core::ObjectData*>(msgs_data->getField(idx_str));

        if (!msg_data)
        {
          std::cerr << "ERROR " << idx_str << std::endl;
          idx_str = "(" + std::to_string(++idx) + ")";
          continue;
        }

        msg->deserialize(*msg_data, context);

        EstimateType pose_in_local =
            static_cast<EstimateType>(msg_data->getEigen<EstimateType>("pose"));

        this->_measurement->m_msgs->push_back(
            std::make_pair(msg, pose_in_local));

        idx_str = "(" + std::to_string(++idx) + ")";
      }
    }

    if (!data.getField("pc"))
    {
      return;
    }

    if (!this->_measurement)
    {
      this->_measurement = std::make_shared<MeasurementObjType>();
    }

    deserializePointCloud(data, context);
  }

  template <
      typename T = PointCloudPointBaseType,
      std::enable_if_t<
          std::is_same<
              T, inesctec_mrdt_slam_distmap_2d::pcl::PointDerived_<
                     inesctec_mrdt_slam_distmap_2d::pcl::Point_<
                         PointCloudPointType::Dim,
                         typename PointCloudPointType::ComponentsType>,
                     inesctec_mrdt_slam_distmap_2d::pcl::PointFieldDirection_<
                         typename PointCloudPointType::ComponentsType,
                         PointCloudPointType::Dim>>>::value,
          bool> = 0>
  void deserializePointCloud(srrg2_core::ObjectData& data,
                             srrg2_core::IdContext& context)
  {
    using PointCloudDataSRRGPointType =
        srrg2_core::PointNormal_<PointCloudPointType::Dim,
                                 typename PointCloudPointType::ComponentsType>;
    using PointCloudDataSRRGContainerType =
        srrg2_core::VectorData_<srrg2_core::PointCloud_<std::vector<
            PointCloudDataSRRGPointType,
            Eigen::aligned_allocator<PointCloudDataSRRGPointType>>>>;

    srrg2_core::BLOBReference<PointCloudDataSRRGContainerType> pc_blob;

    pc_blob.set(new PointCloudDataSRRGContainerType);

    srrg2_core::ObjectData* measurement_boss_obj =
        dynamic_cast<srrg2_core::ObjectData*>(data.getField("pc"));

    if (!measurement_boss_obj)
    {
      return;
    }

    pc_blob.deserialize(*measurement_boss_obj, context);

    this->_measurement->m_pc = std::make_shared<PointCloudType>();
    this->_measurement->m_pc->resize(pc_blob.get()->size());

    for (size_t idx = 0; idx < pc_blob.get()->size(); idx++)
    {
      this->_measurement->m_pc->operator[](idx).coordinates() =
          pc_blob.get()->operator[](idx).coordinates();
      this->_measurement->m_pc->operator[](idx).normal() =
          pc_blob.get()->operator[](idx).normal();
    }
  }

  template <
      typename T = PointCloudPointBaseType,
      std::enable_if_t<
          !std::is_same<
              T, inesctec_mrdt_slam_distmap_2d::pcl::PointDerived_<
                     inesctec_mrdt_slam_distmap_2d::pcl::Point_<
                         PointCloudPointType::Dim,
                         typename PointCloudPointType::ComponentsType>,
                     inesctec_mrdt_slam_distmap_2d::pcl::PointFieldDirection_<
                         typename PointCloudPointType::ComponentsType,
                         PointCloudPointType::Dim>>>::value,
          bool> = 0>
  void deserializePointCloud(srrg2_core::ObjectData& data,
                             srrg2_core::IdContext& context)
  {
    using PointCloudDataSRRGPointType =
        srrg2_core::Point_<PointCloudPointType::Dim,
                           typename PointCloudPointType::ComponentsType>;
    using PointCloudDataSRRGContainerType =
        srrg2_core::VectorData_<srrg2_core::PointCloud_<std::vector<
            PointCloudDataSRRGPointType,
            Eigen::aligned_allocator<PointCloudDataSRRGPointType>>>>;

    srrg2_core::BLOBReference<PointCloudDataSRRGContainerType> pc_blob;

    pc_blob.set(new PointCloudDataSRRGContainerType);

    srrg2_core::ObjectData* measurement_boss_obj =
        dynamic_cast<srrg2_core::ObjectData*>(data.getField("pc"));

    if (!measurement_boss_obj)
    {
      return;
    }

    pc_blob.deserialize(*measurement_boss_obj, context);

    this->_measurement->m_pc = std::make_shared<PointCloudType>();
    this->_measurement->m_pc->resize(pc_blob.get()->size());

    for (size_t idx = 0; idx < pc_blob.get()->size(); idx++)
    {
      this->_measurement->m_pc->operator[](idx).coordinates() =
          pc_blob.get()->operator[](idx).coordinates();
    }
  }

};  // struct
    // MeasurementOwnerDistanceMapPointCloudLaserMsgs_<DistanceMapTypePtr_,
    // PointCloudTypePtr_, EstimateType_> :
    // MeasurementOwner_<DistanceMapPointCloudContainerPtr_<DistanceMapTypePtr_,
    // PointCloudTypePtr_, EstimateType_>>

}  // namespace srrg2_solver
