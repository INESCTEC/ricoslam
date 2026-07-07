#include "inesctec_mrdt_slam_distmap_2d_ros/tracker_ros2.hpp"

// ROS
#if defined(ROS_DISTRO_foxy)
#include <tf2/exceptions.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2/exceptions.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <rclcpp/qos.hpp>

namespace inesctec_mrdt_slam_distmap_2d_ros
{

using namespace inesctec_mrdt_slam_distmap_2d;

TrackerROS2::TrackerROS2(const std::string& name,
                         const rclcpp::NodeOptions& options)
    : BaseType::TrackerROS2API(name, options)
{
  // Quality of Service (to make it the same for all publishers and subscribers)
  rclcpp::QoS qos(10);
  rclcpp::SensorDataQoS sensor_qos;

  // TF
  m_tf2_sub_ = std::make_unique<tf2_ros::TransformListener>(
      *this->m_tf2_buffer_, this, false);
  m_tf2_pub_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Process pre-defined laser pose available in ROS param
  if (m_param_.m_laser_pub_tf_static)
  {
    tf2::Quaternion laser_q;
    laser_q.setRPY(0.0, 0.0, m_param_.m_laser_pose.z());

    geometry_msgs::msg::TransformStamped laser_tf_msg;

    tf2::Transform laser_tf(
        laser_q,
        tf2::Vector3(m_param_.m_laser_pose.x(), m_param_.m_laser_pose.y(), 0.));

    laser_tf_msg.header.stamp = this->now();
    laser_tf_msg.header.frame_id = m_param_.m_base_frame;
    laser_tf_msg.child_frame_id = m_param_.m_laser_frame;
    laser_tf_msg.transform = tf2::toMsg(laser_tf);

    m_tf2_pub_static_ =
        std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    m_tf2_pub_static_->sendTransform(laser_tf_msg);
  }

  // ROS API
  if (m_param_.m_tracker_pub_odom_msg)
  {
    m_pub_odom_ =
        this->create_publisher<nav_msgs::msg::Odometry>("~/odom", qos);
  }

  if (m_param_.m_ext_odom_pub_msg)
  {
    m_pub_ext_odom_ =
        this->create_publisher<nav_msgs::msg::Odometry>("~/ext_odom_pub", qos);
  }

  if (!m_param_.m_ext_odom_use)
  {
    m_sub_scan_only_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "~/scan", sensor_qos,
        std::bind(&BaseType::subScan, static_cast<BaseType*>(this),
                  std::placeholders::_1));
  }
  else
  {
    m_sub_scan_ = std::make_shared<
        message_filters::Subscriber<sensor_msgs::msg::LaserScan>>();

#if defined(ROS_DISTRO_foxy) || defined(ROS_DISTRO_humble) || \
    defined(ROS_DISTRO_jazzy)
    m_sub_scan_->subscribe(this, "~/scan", sensor_qos.get_rmw_qos_profile());
#else
    m_sub_scan_->subscribe(this, "~/scan", sensor_qos);
#endif

    if (m_param_.m_ext_odom_impl.compare("tf") == 0)
    {
      m_sub_scan_tf_ =
          std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(
              *m_sub_scan_, *m_tf2_buffer_, m_param_.m_ext_odom_frame,
              static_cast<uint32_t>(sensor_qos.get_rmw_qos_profile().depth),
              this->get_node_logging_interface(),
              this->get_node_clock_interface());

      m_sub_scan_tf_->registerCallback(std::bind(&BaseType::subScan,
                                                 static_cast<BaseType*>(this),
                                                 std::placeholders::_1));
    }
    else
    {
      m_sub_odom_ = std::make_shared<
          message_filters::Subscriber<nav_msgs::msg::Odometry>>();

#if defined(ROS_DISTRO_foxy) || defined(ROS_DISTRO_humble) || \
    defined(ROS_DISTRO_jazzy)
      m_sub_odom_->subscribe(this, "~/ext_odom",
                             sensor_qos.get_rmw_qos_profile());
#else
      m_sub_odom_->subscribe(this, "~/ext_odom", sensor_qos);
#endif

      m_sub_scan_odom_sync_ = std::make_shared<MsgFilterScanOdomSyncType>(
          MsgFilterScanOdomSyncPolicyType(
              static_cast<uint32_t>(sensor_qos.get_rmw_qos_profile().depth)),
          *m_sub_odom_, *m_sub_scan_);

      m_sub_scan_odom_sync_->registerCallback(
          std::bind(&BaseType::subOdomScan, static_cast<BaseType*>(this),
                    std::placeholders::_1, std::placeholders::_2));
    }
  }
}

void TrackerROS2::pubOdom(const std_msgs::msg::Header& header)
{
  EstimateType odom_t = m_tracker_->getRobotInOdom();
  EstimateVectorType odom_v = geometry::t2v(odom_t);

  tf2::Quaternion odom_q;
  odom_q.setRPY(0.0, 0.0, odom_v.z());

  if (m_param_.m_tracker_pub_odom_msg)
  {
    nav_msgs::msg::Odometry odom_msg;

    odom_msg.header.frame_id = m_param_.m_tracker_odom_frame;
    odom_msg.header.stamp = header.stamp;
    odom_msg.child_frame_id = m_param_.m_base_frame;

    odom_msg.pose.pose.position.x = odom_v.x();
    odom_msg.pose.pose.position.y = odom_v.y();
    odom_msg.pose.pose.position.z = 0.;

    odom_msg.pose.pose.orientation = tf2::toMsg(odom_q);

    m_pub_odom_->publish(odom_msg);
  }

  if (m_param_.m_tracker_pub_tf)
  {
    geometry_msgs::msg::TransformStamped tracker_tf_msg;
    tf2::Transform tracker_tf;

    if (m_param_.m_laser_tf_static_yaml)
    {
      EstimateType odom_in_sensor_t = odom_t * m_tracker_->getSensorInRobot();
      EstimateVectorType odom_in_sensor_v = geometry::t2v(odom_in_sensor_t);

      tracker_tf_msg.header.frame_id = m_param_.m_laser_frame;
      tracker_tf_msg.child_frame_id = m_param_.m_tracker_odom_frame;

      tf2::Quaternion odom_in_sensor_q;
      odom_in_sensor_q.setRPY(
          m_tracker_->getParam().m_laser_invert_scan ? M_PI : 0.0, 0.0,
          odom_in_sensor_v.z());

      tracker_tf = tf2::Transform(
          odom_in_sensor_q,
          tf2::Vector3(odom_in_sensor_v.x(), odom_in_sensor_v.y(), 0.));

      tracker_tf = tracker_tf.inverse();
    }
    else
    {
      tracker_tf =
          tf2::Transform(odom_q, tf2::Vector3(odom_v.x(), odom_v.y(), 0.));

      if (m_param_.m_tracker_odom_invert_tf)
      {
        tracker_tf_msg.header.frame_id = m_param_.m_base_frame;
        tracker_tf_msg.child_frame_id = m_param_.m_tracker_odom_frame;

        tracker_tf = tracker_tf.inverse();
      }
      else
      {
        tracker_tf_msg.header.frame_id = m_param_.m_tracker_odom_frame;
        tracker_tf_msg.child_frame_id = m_param_.m_base_frame;
      }
    }

    tracker_tf_msg.header.stamp = header.stamp;

    tracker_tf_msg.transform = tf2::toMsg(tracker_tf);

    m_tf2_pub_->sendTransform(tracker_tf_msg);
  }
}

void TrackerROS2::pubExtOdom(
    const geometry_msgs::msg::TransformStamped& ext_odom_tf)
{
  nav_msgs::msg::Odometry ext_odom_msg;

  ext_odom_msg.header = ext_odom_tf.header;
  ext_odom_msg.child_frame_id = ext_odom_tf.child_frame_id;

  ext_odom_msg.pose.pose.position.x = ext_odom_tf.transform.translation.x;
  ext_odom_msg.pose.pose.position.y = ext_odom_tf.transform.translation.y;
  ext_odom_msg.pose.pose.position.z = ext_odom_tf.transform.translation.z;
  ext_odom_msg.pose.pose.orientation = ext_odom_tf.transform.rotation;

  m_pub_ext_odom_->publish(ext_odom_msg);
}

void TrackerROS2::pubExtOdomTF(const nav_msgs::msg::Odometry& ext_odom)
{
  geometry_msgs::msg::TransformStamped ext_odom_tf_msg;

  tf2::Transform ext_odom_tf(
      tf2::Quaternion(
          ext_odom.pose.pose.orientation.x, ext_odom.pose.pose.orientation.y,
          ext_odom.pose.pose.orientation.z, ext_odom.pose.pose.orientation.w),
      tf2::Vector3(ext_odom.pose.pose.position.x, ext_odom.pose.pose.position.y,
                   ext_odom.pose.pose.position.z));

  if (m_param_.m_ext_odom_invert_tf)
  {
    ext_odom_tf_msg.header.frame_id = ext_odom.child_frame_id;
    ext_odom_tf_msg.child_frame_id = ext_odom.header.frame_id;

    ext_odom_tf = ext_odom_tf.inverse();
  }
  else
  {
    ext_odom_tf_msg.header.frame_id = ext_odom.header.frame_id;
    ext_odom_tf_msg.child_frame_id = ext_odom.child_frame_id;
  }

  ext_odom_tf_msg.header.stamp = ext_odom.header.stamp;

  ext_odom_tf_msg.transform = tf2::toMsg(ext_odom_tf);

  m_tf2_pub_->sendTransform(ext_odom_tf_msg);
}

}  // namespace inesctec_mrdt_slam_distmap_2d_ros