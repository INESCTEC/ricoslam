#include <gtest/gtest.h>

#include <inesctec_mrdt_slam_distmap_2d/system_utils/ros_log_offline.hpp>

namespace testing
{

TEST(system_utils_ros_log_offline, offlstdout)
{
  OFFLDEBUG("[%s] OFFLDEBUG", "system_utils_ros_log_offline::offlstdout");
  OFFLINFO("[%s] OFFLINFO", "system_utils_ros_log_offline::offlstdout");
  OFFLWARN("[%s] OFFLWARN", "system_utils_ros_log_offline::offlstdout");
  OFFLERROR("[%s] OFFLERROR", "system_utils_ros_log_offline::offlstdout");
  OFFLFATAL("[%s] OFFLFATAL", "system_utils_ros_log_offline::offlstdout");

  OFFLDEBUG("[system_utils_ros_log_offline::offlstdout] OFFLDEBUG (one arg)");
  OFFLINFO("[system_utils_ros_log_offline::offlstdout] OFFLINFO (one arg)");
  OFFLWARN("[system_utils_ros_log_offline::offlstdout] OFFLWARN (one arg)");
  OFFLERROR("[system_utils_ros_log_offline::offlstdout] OFFLERROR (one arg)");
  OFFLFATAL("[system_utils_ros_log_offline::offlstdout] OFFLFATAL (one arg)");
}  // TEST(system_utils_ros_log_offline, offlstdout)

}  // namespace testing
