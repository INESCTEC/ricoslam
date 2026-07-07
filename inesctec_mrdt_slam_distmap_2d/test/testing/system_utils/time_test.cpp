#include <gtest/gtest.h>

#include <inesctec_mrdt_slam_distmap_2d/system_utils/time.hpp>
#include <thread>

namespace testing
{

TEST(system_utils_time, print_and_log_time)
{
  PRINT_AND_LOG_TIME("sleep 1s",
                     std::this_thread::sleep_for(std::chrono::seconds(1)));
  PRINT_AND_LOG_TIME("sleep 100ms", std::this_thread::sleep_for(
                                        std::chrono::milliseconds(100)));
  PRINT_AND_LOG_TIME(
      "sleep 10ms", std::this_thread::sleep_for(std::chrono::milliseconds(10)));
  PRINT_AND_LOG_TIME("sleep 1ms",
                     std::this_thread::sleep_for(std::chrono::milliseconds(1)));
  PRINT_AND_LOG_TIME("sleep 100us", std::this_thread::sleep_for(
                                        std::chrono::microseconds(100)));
  PRINT_AND_LOG_TIME(
      "sleep 10us", std::this_thread::sleep_for(std::chrono::microseconds(10)));
  PRINT_AND_LOG_TIME("sleep 1us",
                     std::this_thread::sleep_for(std::chrono::microseconds(1)));
}  // TEST(system_utils_time, print_and_log_time)

}  // namespace testing
