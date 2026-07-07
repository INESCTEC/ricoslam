#include "inesctec_mrdt_slam_distmap_2d_ros/tracker_ros1.hpp"

// ROS
#include <tf2/exceptions.h>
#include <tf2/utils.h>

namespace inesctec_mrdt_slam_distmap_2d_ros
{

using namespace inesctec_mrdt_slam_distmap_2d;

TrackerROS1::TrackerROS1()
    : BaseType::TrackerROS1API(), m_tf2_sub_(m_tf2_buffer_)
{
  // Process pre-defined laser pose available in ROS param
  if (m_param_.m_laser_pub_tf_static)
  {
    tf2::Quaternion laser_q;
    laser_q.setRPY(0.0, 0.0, m_param_.m_laser_pose.z());

    geometry_msgs::TransformStamped laser_tf_msg;

    tf2::Transform laser_tf(
        laser_q,
        tf2::Vector3(m_param_.m_laser_pose.x(), m_param_.m_laser_pose.y(), 0.));

    laser_tf_msg.header.stamp = ros::Time::now();
    laser_tf_msg.header.frame_id = m_param_.m_base_frame;
    laser_tf_msg.child_frame_id = m_param_.m_laser_frame;
    laser_tf_msg.transform = tf2::toMsg(laser_tf);

    m_tf2_pub_static_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>();
    m_tf2_pub_static_->sendTransform(laser_tf_msg);
  }

  // ROS API
  if (m_param_.m_tracker_pub_odom_msg)
  {
    m_pub_odom_ = std::make_unique<ros::Publisher>();
    *m_pub_odom_ = m_nh_priv_.advertise<nav_msgs::Odometry>("odom", 10, false);
  }

  if (m_param_.m_ext_odom_pub_msg)
  {
    m_pub_ext_odom_ = std::make_unique<ros::Publisher>();
    *m_pub_ext_odom_ =
        m_nh_priv_.advertise<nav_msgs::Odometry>("ext_odom_pub", 10, false);
  }

  if (!m_param_.m_ext_odom_use)
  {
    m_sub_scan_only_ = std::make_unique<ros::Subscriber>();
    *m_sub_scan_only_ = m_nh_priv_.subscribe<sensor_msgs::LaserScan>(
        "scan", 10, &BaseType::subScan, static_cast<BaseType*>(this));
  }
  else
  {
    m_sub_scan_ =
        std::make_unique<message_filters::Subscriber<sensor_msgs::LaserScan>>();

    m_sub_scan_->subscribe(m_nh_priv_, "scan", 10);

    if (m_param_.m_ext_odom_impl.compare("tf") == 0)
    {
      m_sub_scan_tf_ =
          std::make_unique<tf2_ros::MessageFilter<sensor_msgs::LaserScan>>(
              *m_sub_scan_, m_tf2_buffer_, m_param_.m_ext_odom_frame, 10,
              m_nh_priv_);

      m_sub_scan_tf_->registerCallback(&BaseType::subScan,
                                       static_cast<BaseType*>(this));
    }
    else
    {
      m_sub_odom_ =
          std::make_unique<message_filters::Subscriber<nav_msgs::Odometry>>();

      m_sub_odom_->subscribe(m_nh_priv_, "ext_odom", 10);

      m_sub_scan_odom_sync_ = std::make_unique<MsgFilterScanOdomSyncType>(
          MsgFilterScanOdomSyncPolicyType(10), *m_sub_odom_, *m_sub_scan_);

      m_sub_scan_odom_sync_->registerCallback(&BaseType::subOdomScan,
                                              static_cast<BaseType*>(this));
    }
  }
}

void TrackerROS1::pubOdom(const std_msgs::Header& header)
{
  EstimateType odom_t = m_tracker_->getRobotInOdom();
  EstimateVectorType odom_v = geometry::t2v(odom_t);

  tf2::Quaternion odom_q;
  odom_q.setRPY(0.0, 0.0, odom_v.z());

  if (m_param_.m_tracker_pub_odom_msg)
  {
    nav_msgs::Odometry odom_msg;

    odom_msg.header.frame_id = m_param_.m_tracker_odom_frame;
    odom_msg.header.seq = header.seq;
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
    geometry_msgs::TransformStamped tracker_tf_msg;
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

    m_tf2_pub_.sendTransform(tracker_tf_msg);
  }
}

void TrackerROS1::pubExtOdom(const geometry_msgs::TransformStamped& ext_odom_tf)
{
  nav_msgs::Odometry ext_odom_msg;

  ext_odom_msg.header = ext_odom_tf.header;
  ext_odom_msg.child_frame_id = ext_odom_tf.child_frame_id;

  ext_odom_msg.pose.pose.position.x = ext_odom_tf.transform.translation.x;
  ext_odom_msg.pose.pose.position.y = ext_odom_tf.transform.translation.y;
  ext_odom_msg.pose.pose.position.z = ext_odom_tf.transform.translation.z;
  ext_odom_msg.pose.pose.orientation = ext_odom_tf.transform.rotation;

  m_pub_ext_odom_->publish(ext_odom_msg);
}

void TrackerROS1::pubExtOdomTF(const nav_msgs::Odometry::ConstPtr& ext_odom)
{
  geometry_msgs::TransformStamped ext_odom_tf_msg;

  tf2::Transform ext_odom_tf(tf2::Quaternion(ext_odom->pose.pose.orientation.x,
                                             ext_odom->pose.pose.orientation.y,
                                             ext_odom->pose.pose.orientation.z,
                                             ext_odom->pose.pose.orientation.w),
                             tf2::Vector3(ext_odom->pose.pose.position.x,
                                          ext_odom->pose.pose.position.y,
                                          ext_odom->pose.pose.position.z));

  if (m_param_.m_ext_odom_invert_tf)
  {
    ext_odom_tf_msg.header.frame_id = ext_odom->child_frame_id;
    ext_odom_tf_msg.child_frame_id = ext_odom->header.frame_id;

    ext_odom_tf = ext_odom_tf.inverse();
  }
  else
  {
    ext_odom_tf_msg.header.frame_id = ext_odom->header.frame_id;
    ext_odom_tf_msg.child_frame_id = ext_odom->child_frame_id;
  }

  ext_odom_tf_msg.header.stamp = ext_odom->header.stamp;

  ext_odom_tf_msg.transform = tf2::toMsg(ext_odom_tf);

  m_tf2_pub_.sendTransform(ext_odom_tf_msg);
}

}  // namespace inesctec_mrdt_slam_distmap_2d_ros
