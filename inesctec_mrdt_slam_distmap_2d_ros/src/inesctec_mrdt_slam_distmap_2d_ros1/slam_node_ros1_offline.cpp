#include <iostream>
#include <thread>

// Boost
#include <boost/exception/diagnostic_information.hpp>
#include <boost/program_options.hpp>

#include "inesctec_mrdt_slam_distmap_2d/system_utils/ros_log_offline.hpp"
#include "inesctec_mrdt_slam_distmap_2d_ros/slam_ros1_offline.hpp"

using OGLViewerType =
    inesctec_mrdt_slam_distmap_2d_ros::SLAMROS1Offline::SLAMType::OGLViewerType;

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "slam_offline", ros::init_options::NoSigintHandler);

  inesctec_mrdt_slam_distmap_2d_ros::SLAMROS1Offline::ParamOffline param;

  boost::program_options::options_description opts_desc(
      "Usage: rosrun inesctec_mrdt_slam_distmap_2d_ros slam_offline "
      "-b BAGFILE1 [BAGFILE2 BAGFILE3 ...]\n\nOptions");

  opts_desc.add_options()("help,h", "Display this information.")(
      "bags,b",
      boost::program_options::value<std::vector<std::string>>(&param.m_bags)
          ->required()
          ->multitoken()
          ->value_name("BAGFILE"),
      "ROS bag files to process")(
      "odomtopic",
      boost::program_options::value<std::string>(&param.m_odom_topic)
          ->default_value("/odom")
          ->value_name("TOPIC"),
      "Topic name for the robot's odometry data (used for initialization)")(
      "scantopic",
      boost::program_options::value<std::string>(&param.m_scan_topic)
          ->default_value("/scan")
          ->value_name("TOPIC"),
      "Topic name for the 2D laser scanner data")(
      "start,s",
      boost::program_options::value<double>(&param.m_time_start)
          ->default_value(0.0)
          ->value_name("SEC"),
      "start SEC seconds into the bag files")(
      "duration,d",
      boost::program_options::value<double>(&param.m_time_duration)
          ->value_name("SEC"),
      "play only SEC seconds from the bag files")(
      "log",
      boost::program_options::value<std::string>(&param.m_log_filename)
          ->value_name("FILENAME"),
      "log the robot estimated data (odom, pose) into TUM files");

  /* boost::program_options::positional_options_description opts_pos;
  opts_pos.add("bags", -1); */

  boost::program_options::variables_map opts_vm;

  try
  {
    boost::program_options::store(
        boost::program_options::command_line_parser(argc, argv)
            .options(opts_desc)
            .allow_unregistered()
            // .positional(opts_pos)
            .run(),
        opts_vm);

    if (opts_vm.count("help"))
    {
      std::cout << opts_desc << std::endl;
      return 0;
    }
  }
  catch (boost::exception& e)
  {
    OFFLFATAL("[%s] something went wrong.... (error: %s)\n",
              ros::this_node::getName().c_str(),
              boost::diagnostic_information(e).c_str());
    std::cout << opts_desc << std::endl;
    return 0;
  }
  catch (...)
  {
    OFFLFATAL("[%s] unexpected behavior.....\n",
              ros::this_node::getName().c_str());
    std::cout << opts_desc << std::endl;
    return -1;
  }

  try
  {
    boost::program_options::notify(opts_vm);
  }
  catch (boost::exception& e)
  {
    OFFLFATAL("[%s] something went wrong.... (error: %s)\n",
              ros::this_node::getName().c_str(),
              boost::diagnostic_information(e).c_str());
    std::cout << opts_desc << std::endl;
    return -1;
  }
  catch (...)
  {
    OFFLFATAL("[%s] unexpected behavior.....\n",
              ros::this_node::getName().c_str());
    std::cout << opts_desc << std::endl;
    return -1;
  }

  if (opts_vm.count("duration"))
  {
    param.m_has_duration = true;
  }
  else
  {
    param.m_has_duration = false;
  }
  if (opts_vm.count("log"))
  {
    param.m_enable_log = true;
  }
  else
  {
    param.m_enable_log = false;
  }

  try
  {
    inesctec_mrdt_slam_distmap_2d_ros::SLAMROS1Offline slam(param);

    if (!slam.isOpenGLDbg())
    {
      slam.run();
    }
    else
    {
      std::shared_ptr<QApplication> qapp =
          std::make_shared<QApplication>(argc, argv);
      std::shared_ptr<OGLViewerType> srrg_viewer =
          std::make_shared<OGLViewerType>(argc, argv, qapp.get(),
                                          BUFFER_SIZE_50MEGABYTE, 3, 25, false);

      slam.getSLAM()->setupDbgOGL(qapp, srrg_viewer);

      std::thread ros_thread(
          [&qapp, &slam]()
          {
            std::cout << "just sleeping for a little second... "
                         "debug must get ready!"
                      << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
            slam.run();
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
