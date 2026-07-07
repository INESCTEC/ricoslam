#include <srrg2_core/srrg_pcl/instances.h>
#include <srrg2_core/srrg_viewer/viewer_canvas.h>
#include <srrg2_qgl_viewport/viewer_core_shared_qgl.h>
#include <srrg2_solver/solver_core/factor_graph.h>

#include <array>
#include <boost/exception/diagnostic_information.hpp>
#include <boost/program_options.hpp>
#include <filesystem>
#include <inesctec_mrdt_slam_distmap_2d/data_structures/derived_types_pack_executor.hpp>
#include <inesctec_mrdt_slam_distmap_2d/data_structures/instances.hpp>
#include <inesctec_mrdt_slam_distmap_2d/slam/grid_mapper.hpp>
#include <inesctec_mrdt_slam_distmap_2d/slam/merger.hpp>
#include <inesctec_mrdt_slam_distmap_2d/srrg2_solver/instances.hpp>
#include <inesctec_mrdt_slam_distmap_2d/srrg2_solver/variable_se2_distance_map.hpp>
#include <inesctec_mrdt_slam_distmap_2d/system_utils/ros_log_offline.hpp>
#include <iostream>
#include <string>
#include <thread>

using namespace inesctec_mrdt_slam_distmap_2d;
using namespace data_structures;
using namespace slam;

using FactorGraphType = srrg2_solver::FactorGraph;
using FactorGraphTypePtr = std::shared_ptr<FactorGraphType>;

using OGLViewerType = srrg2_qgl_viewport::ViewerCoreSharedQGL;

namespace inesctec_mrdt_slam_distmap_2d
{
namespace apps
{

using MyTypes = DerivedTypesPackExecutor_<
    srrg2_solver::VariableSE2DistanceMapStaticPoint2fVectorCloudRight,
    srrg2_solver::
        VariableSE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudRight,
    srrg2_solver::VariableSE2DistanceMapSparseStaticPoint2fVectorCloudRight,
    srrg2_solver::
        VariableSE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudRight,
    srrg2_solver::VariableSE2DistanceMapNanoStaticPoint2fVectorCloudRight,
    srrg2_solver::VariableSE2DistanceMapStaticPointNormal2fVectorCloudRight,
    srrg2_solver::
        VariableSE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRight,
    srrg2_solver::
        VariableSE2DistanceMapSparseStaticPointNormal2fVectorCloudRight,
    srrg2_solver::
        VariableSE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRight,
    srrg2_solver::VariableSE2DistanceMapNanoStaticPointNormal2fVectorCloudRight,
    srrg2_solver::VariableSE2DistanceMapStaticPoint2fVectorCloudLeft,
    srrg2_solver::
        VariableSE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudLeft,
    srrg2_solver::VariableSE2DistanceMapSparseStaticPoint2fVectorCloudLeft,
    srrg2_solver::
        VariableSE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudLeft,
    srrg2_solver::VariableSE2DistanceMapNanoStaticPoint2fVectorCloudLeft,
    srrg2_solver::VariableSE2DistanceMapStaticPointNormal2fVectorCloudLeft,
    srrg2_solver::
        VariableSE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudLeft,
    srrg2_solver::
        VariableSE2DistanceMapSparseStaticPointNormal2fVectorCloudLeft,
    srrg2_solver::
        VariableSE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudLeft,
    srrg2_solver::VariableSE2DistanceMapNanoStaticPointNormal2fVectorCloudLeft>;

const std::array<std::string, 20> my_types_str(
    {"VariableSE2DistanceMapNanoStaticPointNormal2fVectorCloudLeft",
     "VariableSE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudLef"
     "t",
     "VariableSE2DistanceMapSparseStaticPointNormal2fVectorCloudLeft",
     "VariableSE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudLeft",
     "VariableSE2DistanceMapStaticPointNormal2fVectorCloudLeft",
     "VariableSE2DistanceMapNanoStaticPoint2fVectorCloudLeft",
     "VariableSE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudLeft",
     "VariableSE2DistanceMapSparseStaticPoint2fVectorCloudLeft",
     "VariableSE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudLeft",
     "VariableSE2DistanceMapStaticPoint2fVectorCloudLeft",
     "VariableSE2DistanceMapNanoStaticPointNormal2fVectorCloudRight",
     "VariableSE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRig"
     "ht",
     "VariableSE2DistanceMapSparseStaticPointNormal2fVectorCloudRight",
     "VariableSE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRight",
     "VariableSE2DistanceMapStaticPointNormal2fVectorCloudRight",
     "VariableSE2DistanceMapNanoStaticPoint2fVectorCloudRight",
     "VariableSE2DistanceMapSparseStaticWith2ndDerivPoint2fVectorCloudRight",
     "VariableSE2DistanceMapSparseStaticPoint2fVectorCloudRight",
     "VariableSE2DistanceMapStaticWith2ndDerivPoint2fVectorCloudRight",
     "VariableSE2DistanceMapStaticPoint2fVectorCloudRight"});

void printFactorGraphStats(FactorGraphType* const graph);

// does not matter... we only need a standard point cloud-based representation
// with no normals to generate a 2D occupancy grid map
using EmptyVariableType =
    srrg2_solver::VariableSE2DistanceMapNanoStaticPoint2fVectorCloudRight;
using MergerType = slam::Merger_<
    typename EmptyVariableType::EstimateType,
    typename EmptyVariableType::MeasurementOwnerType::DistanceMapType,
    typename EmptyVariableType::MeasurementOwnerType::PointCloudType>;
using MergerTypeParam = MergerType::Param;
using MergerTypePointCloudType = MergerType::PointCloudType;

}  // namespace apps
}  // namespace inesctec_mrdt_slam_distmap_2d

int main(int argc, char* argv[])
{
  std::string filename;
  inesctec_mrdt_slam_distmap_2d::slam::GridMapper::Param param;

  apps::MergerTypeParam merger_param;
  merger_param.m_enable_avg = false;
  merger_param.m_weight_new = -1.f;
  merger_param.m_weight_old = -1.f;

  boost::program_options::options_description opts_desc(
      "Usage: ./generate_occgrid2d_from_srrg -f FILENAME\n\nOptions");

  opts_desc.add_options()("help,h", "Display this information.")(
      "file,f",
      boost::program_options::value<std::string>(&filename)
          ->required()
          ->value_name("FILENAME"),
      "JSON file from SRRG factor graph serialization")(
      "res,r",
      boost::program_options::value<float>(&param.m_res)
          ->default_value(0.05)
          ->value_name("RES"),
      "map resolution (m/px)")(
      "rumin",
      boost::program_options::value<float>(&param.m_usable_range_min)
          ->default_value(0.10)
          ->value_name("RANGE_USABLE_MIN"),
      "minimum usable range (m)")(
      "rumax",
      boost::program_options::value<float>(&param.m_usable_range_max)
          ->default_value(10.00)
          ->value_name("RANGE_USABLE_MAX"),
      "maximum usable range (m)")(
      "rmax",
      boost::program_options::value<float>(&param.m_range_max)
          ->default_value(12.00)
          ->value_name("RANGE_MAX"),
      "maximum range (m) (free space)")(
      "pfree",
      boost::program_options::value<float>(&param.m_thresh_free)
          ->default_value(-1.0)
          ->value_name("FREE"),
      "free occupancy probability threshold (0..1) (if enabled)")(
      "pocc",
      boost::program_options::value<float>(&param.m_thresh_occ)
          ->default_value(0.85)
          ->value_name("OCC"),
      "occupied probability threshold (0..1)")(
      "raythrough",
      boost::program_options::value<float>(&merger_param.m_ray_min_dist_through)
          ->default_value(-1.0)
          ->value_name("RAY_THROUGH"),
      "ray minimum distance to consider a see through (m) (-1 not used)")(
      "raynew",
      boost::program_options::value<float>(&merger_param.m_ray_min_dist_add_new)
          ->default_value(-1.0)
          ->value_name("RAY_NEW"),
      "ray minimum distance to consider a new point not mapped yet (m) (-1 not "
      "used)");

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

  bool enable_merger = (merger_param.m_ray_min_dist_add_new >= 0) ||
                       (merger_param.m_ray_min_dist_through >= 0);

  std::filesystem::path filepath(filename);

  if (!std::filesystem::exists(filepath))
  {
    OFFLFATAL("file %s does not exist", filename.c_str());
    return -1;
  }

  std::filesystem::path dir = filepath.parent_path();
  std::string stem = filepath.stem().string();
  std::string ext = filepath.extension().string();

  try
  {
    srrg2_solver::point_cloud_registerTypes();
    srrg2_solver::inesctec_mrdt_slam_distmap_2d_data_structures_registerTypes();
    srrg2_solver::inesctec_mrdt_slam_distmap_2d_srrg2_solver_registerTypes();

    std::cout << "Grid Mapper parametrization:" << std::endl
              << param << std::endl;

    FactorGraphTypePtr graph = srrg2_solver::FactorGraph::read(filename);

    if (!graph)
    {
      OFFLFATAL("invalid graph deserialization...");
      return -1;
    }

    apps::printFactorGraphStats(graph.get());

    GridMapper mapper = GridMapper(param);

    mapper.setGraph(graph.get());

    int total_num_scans = 0;

    for (const auto& var : graph->variables())
    {
      srrg2_solver::VariableBase* v = var.second;

      apps::MyTypes::executeWithMatchingType(
          v,
          [&mapper, &total_num_scans](auto* ptr)
          {
            auto sensor_in_robot = ptr->measurement()->m_sensor_in_robot;

            for (const auto& msg_pair : *ptr->measurement()->m_msgs)
            {
              mapper.pushLaserScanMsg(msg_pair.first, sensor_in_robot,
                                      msg_pair.second, ptr->graphId());
              ++total_num_scans;
            }
          });
    }

    std::cout << "- #scans (total): " << total_num_scans << std::endl;

    apps::MergerTypePointCloudType map;

    std::vector<std::pair<size_t, size_t>> map_to_scan_pt;

    if (enable_merger)
    {
      apps::MergerType merger(merger_param);

      mapper.setMapToScanPt(&map_to_scan_pt);

      for (size_t idx = 0; idx < mapper.getProcessedScans().size(); idx++)
      {
        const auto& msg = mapper.getProcessedScans()[idx];

        srrg2_solver::VariableBase* var = graph->variable(msg->m_local_map_id);

        if (!var)
        {
          throw std::runtime_error("GridMapper::updateMap | graph variable " +
                                   std::to_string(msg->m_local_map_id) +
                                   " not found when preprocessing the scans");
        }

        srrg2_solver::VariableSE2Base* v =
            dynamic_cast<srrg2_solver::VariableSE2Base*>(var);

        if (!v)
        {
          throw std::runtime_error(
              "GridMapper::updateMap | invalid variable dynamic cast to "
              "VariableSE2Base when preprocessing the scans");
        }

        const auto robot_in_global = v->estimate() * msg->m_pose_in_local_map;

        float range_min =
            param.m_usable_range_min < 0
                ? msg->m_scan->m_range_min
                : std::max(param.m_usable_range_min, msg->m_scan->m_range_min);
        float range_max =
            param.m_range_max > 0
                ? std::min(param.m_range_max, msg->m_scan->m_range_max)
                : msg->m_scan->m_range_max;

        merger.merge(*mapper.getProcessedScans()[idx]->m_scan, idx,
                     mapper.getProcessedScans()[idx]->m_sensor_in_robot,
                     robot_in_global, range_min, range_max, map,
                     map_to_scan_pt);
      }

      std::cout << "- #map pts      : " << map.size() << std::endl;
      std::cout << "- merger config : " << std::endl << merger_param;
    }

    mapper.updateMap();

    std::cout << "2D occupancy grid map generated successfully..." << std::endl;

    UCharImage img, occ;
    FloatImage occ_f;

    mapper.getGridMapContainer()->toImageOccupancyGridImage(img);
    mapper.getGridMapContainer()->toImageOccupancyGridMap(occ);
    mapper.getGridMapContainer()->toOccupancyGridMapWithUnknown(occ_f);

    // cv::imshow("occupancy grid map", img);
    // cv::waitKey(100);
    cv::imwrite(filepath.replace_extension("png").string(), img);
    cv::imwrite((dir / (stem + "_occ.png")).string(), occ);
    cv::imwrite((dir / (stem + "_occ_mat.tiff")).string(), occ_f);
    // cv::waitKey(0);

    std::string mapdatafile = filepath.replace_extension("pgm").string();

    std::cout << "Writing map occupancy data to " << mapdatafile << std::endl;

    FILE* out = fopen(mapdatafile.c_str(), "w");

    if (!out)
    {
      OFFLERROR("Couldn't save map file to %s", mapdatafile.c_str());
      return -1;
    }

    fprintf(out, "P5\n# CREATOR: map_saver.cpp %.3f m/pix\n%d %d\n255\n",
            mapper.getGridMapContainer()->getRes(),
            mapper.getGridMapContainer()->getWidth(),
            mapper.getGridMapContainer()->getHeight());

    for (int y = mapper.getGridMapContainer()->getHeight() - 1; y >= 0; --y)
    {
      for (int x = 0; x < mapper.getGridMapContainer()->getWidth(); ++x)
      {
        auto* occ_map = mapper.getGridMapContainer()->getOccupancyMap();
        auto* freq_map = mapper.getGridMapContainer()->getFrequencyMap();

        if ((*freq_map)(x, y)(1) != 0)
        {
          if (param.m_thresh_free >= 0.f)
          {
            if ((*occ_map)(x, y) <= param.m_thresh_free)
            {  // [0,free]
              fputc(254, out);
            }
            else if ((*occ_map)(x, y) >= param.m_thresh_occ)
            {  // [occ,255]
              fputc(000, out);
            }
            else
            {  // ]free,occ[ --> unknwon (cell values retrieved from map_server)
              fputc(205, out);
            }
          }
          else
          {
            if ((*occ_map)(x, y) >= param.m_thresh_occ)
            {
              fputc(000, out);
            }
            else
            {
              fputc(254, out);
            }
          }
        }
        else
        {
          fputc(205, out);
        }
      }
    }

    fclose(out);

    std::string mapmetadatafile = filepath.replace_extension("yaml").string();

    FILE* yaml = fopen(mapmetadatafile.c_str(), "w");

    float yaw = geometry::r2a<float>(
        mapper.getGridMapContainer()->getOrigin().linear());

    fprintf(yaml,
            "image: %s\n"
            "resolution: %f\n"
            "origin: [%f, %f, %f]\n"
            "negate: 0\n"
            "occupied_thresh: 0.65\n"
            "free_thresh: 0.192\n"
            "orig_occupied_thresh: %f\n"
            "orig_free_thresh: %f\n"
            "usable_range_min: %f\n"
            "usable_range_max: %f\n"
            "maximum_range: %f\n",
            mapdatafile.c_str(), mapper.getGridMapContainer()->getRes(),
            mapper.getGridMapContainer()->getOrigin().translation().x(),
            mapper.getGridMapContainer()->getOrigin().translation().y(), yaw,
            param.m_thresh_occ, param.m_thresh_free, param.m_usable_range_min,
            param.m_usable_range_max, param.m_range_max);

    if (enable_merger)
    {
      fprintf(yaml,
              "ray_min_dist_through: %f\n"
              "ray_min_dist_add_new: %f\n\n",
              merger_param.m_ray_min_dist_through,
              merger_param.m_ray_min_dist_add_new);
    }
    else
    {
      fprintf(yaml, "\n");
    }

    fclose(yaml);

    std::shared_ptr<QApplication> qapp =
        std::make_shared<QApplication>(argc, argv);
    std::shared_ptr<OGLViewerType> srrg_viewer =
        std::make_shared<OGLViewerType>(argc, argv, qapp.get(),
                                        BUFFER_SIZE_50MEGABYTE, 3, 25, false);
    std::shared_ptr<srrg2_core::ViewerCanvas> canvas_graph =
        srrg_viewer->getCanvas("graph");
    std::shared_ptr<srrg2_core::ViewerCanvas> canvas_map = nullptr;

    canvas_graph->setName("graph");
    canvas_graph->_setup();

    if (enable_merger)
    {
      canvas_map = srrg_viewer->getCanvas("map");
      canvas_map->setName("map");
      canvas_map->_setup();
    }

    std::thread render_thread(
        [&qapp, &mapper, &srrg_viewer, &canvas_graph, &canvas_map, &map]()
        {
          std::cout << "just sleeping for a little second... "
                       "debug must get ready!"
                    << std::endl;
          std::this_thread::sleep_for(std::chrono::seconds(1));

          srrg2_core::PointNormal3fVectorCloud map_srrg_pc;

          if (canvas_map)
          {
            map_srrg_pc.resize(map.size());

            for (size_t idx = 0; idx < map.size(); idx++)
            {
              map_srrg_pc[idx].coordinates().x() = map[idx].coordinates().x();
              map_srrg_pc[idx].coordinates().y() = map[idx].coordinates().y();
              map_srrg_pc[idx].coordinates().z() = 0;
            }
          }

          while (srrg_viewer->isRunning())
          {
            mapper.updateDbgOPGLCanvas(canvas_graph);

            if (canvas_map)
            {
              canvas_map->putReferenceSystem(0.10);

              canvas_map->pushColor();
              canvas_map->setColor(srrg2_core::ColorPalette::color4fBlack());
              canvas_map->putPoints(map_srrg_pc);
              canvas_map->popAttribute();

              canvas_map->flush();
            }

            std::this_thread::sleep_for(std::chrono::seconds(1));
          }

          qapp->closeAllWindows();
        });

    srrg_viewer->startViewerServer();

    render_thread.join();
  }
  catch (const std::exception& e)
  {
    OFFLFATAL("Fatal error: %s", e.what());
    return -1;
  }

  return 0;
}

namespace inesctec_mrdt_slam_distmap_2d
{

namespace apps
{

void printFactorGraphStats(FactorGraphType* const graph)
{
  int idx_variable_type =
      MyTypes::findMatchingType(graph->variables().begin().value());

  std::cout << "Factor graph metadata:" << std::endl
            << "- #variables    : " << graph->variables().size() << std::endl
            << "- #factors      : " << graph->factors().size() << std::endl
            << "- variable type : "
            << (idx_variable_type == -1 ? "NOT FOUND"
                                        : my_types_str[idx_variable_type])
            << std::endl;
}

}  // namespace apps

}  // namespace inesctec_mrdt_slam_distmap_2d
