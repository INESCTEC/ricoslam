#include <iostream>
#include <thread>

#include "inesctec_mrdt_slam_distmap_2d/system_utils/ros_log_offline.hpp"
#include "inesctec_mrdt_slam_distmap_2d_ros/slam_ros2_online.hpp"

using OGLViewerType =
    inesctec_mrdt_slam_distmap_2d_ros::SLAMROS2Online::SLAMType::OGLViewerType;

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  try
  {
    rclcpp::NodeOptions opts;

    opts.allow_undeclared_parameters(true);
    opts.automatically_declare_parameters_from_overrides(true);

    std::shared_ptr<inesctec_mrdt_slam_distmap_2d_ros::SLAMROS2Online> slam =
        std::make_shared<inesctec_mrdt_slam_distmap_2d_ros::SLAMROS2Online>(
            "slam_online", opts);

    if (!slam->isOpenGLDbg())
    {
      rclcpp::spin(slam);
    }
    else
    {
      std::shared_ptr<QApplication> qapp =
          std::make_shared<QApplication>(argc, argv);
      std::shared_ptr<OGLViewerType> srrg_viewer =
          std::make_shared<OGLViewerType>(argc, argv, qapp.get(),
                                          BUFFER_SIZE_50MEGABYTE, 3, 25, false);

      slam->getSLAM()->setupDbgOGL(qapp, srrg_viewer);

      std::thread ros_thread(
          [&qapp, &slam]()
          {
            std::cout << "just sleeping for a little second... "
                         "debug must get ready!"
                      << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
            rclcpp::spin(slam);
            qapp->closeAllWindows();
          });

      srrg_viewer->startViewerServer();

      ros_thread.join();
    }

    return 0;
  }
  catch (const std::exception& e)
  {
    OFFLFATAL("Fatal error: %s", e.what());

    rclcpp::shutdown();

    return -1;
  }
}
