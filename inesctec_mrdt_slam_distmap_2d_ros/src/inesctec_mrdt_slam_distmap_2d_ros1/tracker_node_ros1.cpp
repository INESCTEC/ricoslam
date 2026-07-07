#include <iostream>
#include <thread>

#include "inesctec_mrdt_slam_distmap_2d/system_utils/ros_log_offline.hpp"
#include "inesctec_mrdt_slam_distmap_2d_ros/tracker_ros1.hpp"

using OGLViewerType =
    inesctec_mrdt_slam_distmap_2d_ros::TrackerROS1::TrackerType::OGLViewerType;

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "tracker");

  try
  {
    inesctec_mrdt_slam_distmap_2d_ros::TrackerROS1 tracker;

    if (!tracker.isOpenGLDbg())
    {
      ros::spin();
    }
    else
    {
      std::shared_ptr<QApplication> qapp =
          std::make_shared<QApplication>(argc, argv);
      std::shared_ptr<OGLViewerType> srrg_viewer =
          std::make_shared<OGLViewerType>(argc, argv, qapp.get(),
                                          BUFFER_SIZE_50MEGABYTE, 3, 25, false);

      tracker.getTracker()->setupDbgOGL(qapp, srrg_viewer);

      std::thread ros_thread(
          [&qapp]()
          {
            std::cout << "just sleeping for a little second... "
                         "debug must get ready!"
                      << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
            ros::spin();
            qapp->closeAllWindows();
          });

      srrg_viewer->startViewerServer();

      ros_thread.join();
    }

    return 0;
  }
  catch (const std::exception& e)
  {
    ROS_FATAL("[%s] Fatal error: %s", ros::this_node::getName().c_str(),
              e.what());
    OFFLFATAL("[%s] Fatal error: %s", ros::this_node::getName().c_str(),
              e.what());

    ros::spinOnce();
    ros::requestShutdown();

    return -1;
  }
}
