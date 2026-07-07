#include <rosbag/bag.h>
#include <rosbag/query.h>
#include <rosbag/view.h>
#include <sensor_msgs/LaserScan.h>
#include <srrg2_core/srrg_pcl/instances.h>
#include <srrg2_core/srrg_viewer/viewer_canvas.h>
#include <srrg2_qgl_viewport/viewer_core_shared_qgl.h>
#include <srrg2_solver/solver_core/factor_graph.h>
#include <srrg2_solver/solver_core/instances.h>
#include <srrg2_solver/solver_core/internals/linear_solvers/instances.h>
#include <srrg2_solver/solver_core/internals/linear_solvers/sparse_block_linear_solver_cholesky_cholmod.h>
#include <srrg2_solver/solver_core/iteration_algorithm_ddl.h>
#include <srrg2_solver/solver_core/iteration_algorithm_gn.h>
#include <srrg2_solver/solver_core/iteration_algorithm_lm.h>
#include <srrg2_solver/solver_core/solver.h>

#include <array>
#include <boost/exception/diagnostic_information.hpp>
#include <boost/program_options.hpp>
#include <filesystem>
#include <fstream>
#include <inesctec_mrdt_slam_distmap_2d/data_structures/instances.hpp>
#include <inesctec_mrdt_slam_distmap_2d/data_structures/tum_pose.hpp>
#include <inesctec_mrdt_slam_distmap_2d/geometry/geometry_2d.hpp>
#include <inesctec_mrdt_slam_distmap_2d/pcl/normal_computator.hpp>
#include <inesctec_mrdt_slam_distmap_2d/slam/aligner.hpp>
#include <inesctec_mrdt_slam_distmap_2d/slam/aligner_distance_map_point_plane.hpp>
#include <inesctec_mrdt_slam_distmap_2d/slam/aligner_distance_map_point_point.hpp>
#include <inesctec_mrdt_slam_distmap_2d/slam/loop_finder_visitor.hpp>
#include <inesctec_mrdt_slam_distmap_2d/slam/msg/laser_scan.hpp>
#include <inesctec_mrdt_slam_distmap_2d/srrg2_solver/instances.hpp>
#include <inesctec_mrdt_slam_distmap_2d/srrg2_solver/se2_distance_map_pose_pose_bundle_error_factor.hpp>
#include <inesctec_mrdt_slam_distmap_2d/srrg2_solver/se2_distance_map_pose_pose_bundle_plane_error_factor.hpp>
#include <inesctec_mrdt_slam_distmap_2d/srrg2_solver/solver_actions.hpp>
#include <inesctec_mrdt_slam_distmap_2d/srrg2_solver/variable_se2_distance_map.hpp>
#include <inesctec_mrdt_slam_distmap_2d/system_utils/ros_log_offline.hpp>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <type_traits>
#include <vector>

#include "inesctec_mrdt_slam_distmap_2d_ros/ogba_app_ros1.hpp"

using namespace inesctec_mrdt_slam_distmap_2d_ros;
using namespace inesctec_mrdt_slam_distmap_2d;
using namespace data_structures;
using namespace slam;
using namespace std::chrono_literals;

using OGLViewerType = OgbaAppROS1::OGLViewerType;

int main(int argc, char* argv[])
{
  OgbaAppROS1::Param param;

  boost::program_options::options_description opts_desc(
      "Usage: ./offline_global_bundle_adjustment --bags BAGFILE1 [BAGFILE2 "
      "BAGFILE3 ...] --log LOG_FILENAME --config CONFIG_FILE\n\nOptions");

  opts_desc.add_options()("help,h", "Display this information.")(
      "bags,b",
      boost::program_options::value<std::vector<std::string>>(
          &param.m_filenames_bags)
          ->required()
          ->multitoken()
          ->value_name("BAGFILE [...]"),
      "ROS 1 bag files to process with laser scan data")(
      "scantopic",
      boost::program_options::value<std::string>(&param.m_scan_topic)
          ->default_value("/scan")
          ->value_name("TOPIC"),
      "Topic name for the 2D laser scanner data")(
      "log,l",
      boost::program_options::value<std::string>(&param.m_filename_log)
          ->required()
          ->value_name("FILENAME"),
      "TUM data file with pose data of the robot or laser")(
      "config,c",
      boost::program_options::value<std::string>(&param.m_ogba_yaml)
          ->required()
          ->value_name("CONFIG_FILE"),
      "YAML configuration file for the OGBA framework")(
      "sensor",
      boost::program_options::value<std::vector<double>>(
          &param.m_sensor_in_robot_val)
          ->multitoken()
          ->default_value(std::vector<double>{0., 0., 0.}, "0 0 0")
          ->value_name("x y yaw (m m deg)"),
      "sensor pose w.r.t. robot frame (if not set, identity isometry  used)")(
      "start,s",
      boost::program_options::value<double>(&param.m_time_start)
          ->default_value(0.0)
          ->value_name("SEC"),
      "start SEC seconds into the bag files")(
      "duration,d",
      boost::program_options::value<double>(&param.m_time_duration)
          ->value_name("SEC"),
      "play only SEC seconds from the bag files");

  boost::program_options::variables_map opts_vm;

  try
  {
    boost::program_options::store(
        boost::program_options::command_line_parser(argc, argv)
            .options(opts_desc)
            .allow_unregistered()
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
    OFFLFATAL("something went wrong.... (error: %s)\n",
              boost::diagnostic_information(e).c_str());
    std::cout << opts_desc << std::endl;
    return 0;
  }
  catch (...)
  {
    OFFLFATAL("unexpected behavior.....\n");
    std::cout << opts_desc << std::endl;
    return -1;
  }

  try
  {
    boost::program_options::notify(opts_vm);
  }
  catch (boost::exception& e)
  {
    OFFLFATAL("something went wrong.... (error: %s)\n",
              boost::diagnostic_information(e).c_str());
    std::cout << opts_desc << std::endl;
    return -1;
  }
  catch (...)
  {
    OFFLFATAL("unexpected behavior.....\n");
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

  OgbaAppROS1 app(param);

  std::shared_ptr<QApplication> qapp =
      std::make_shared<QApplication>(argc, argv);
  std::shared_ptr<OGLViewerType> srrg_viewer = std::make_shared<OGLViewerType>(
      argc, argv, qapp.get(), BUFFER_SIZE_50MEGABYTE, 3, 25, false);

  app.setupDbgOGL(qapp, srrg_viewer);

  std::thread compute_thread(
      [&qapp, &srrg_viewer, &app]()
      {
        app.getPoses();
        app.computeCovisibilityGraph();
        app.compute();

        while (!srrg_viewer->isRunning())
        {
          std::cout << "just sleeping for a little second... "
                       "debug is getting ready!"
                    << std::endl;
          std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        while (srrg_viewer->isRunning())
        {
          app.updateDbgOPGLCanvas();

          std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        qapp->closeAllWindows();

        std::cout << "going out... bye!!!" << std::endl;
      });

  srrg_viewer->startViewerServer();

  compute_thread.join();

  return 0;
}
