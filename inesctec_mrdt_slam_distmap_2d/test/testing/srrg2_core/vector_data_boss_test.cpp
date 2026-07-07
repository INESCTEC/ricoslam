#include <gtest/gtest.h>
#include <srrg2_core/srrg_boss/blob.h>
#include <srrg2_core/srrg_boss/deserializer.h>
#include <srrg2_core/srrg_boss/object_data.h>
#include <srrg2_core/srrg_boss/serializer.h>
#include <srrg2_core/srrg_pcl/instances.h>
#include <srrg2_core/srrg_pcl/point_cloud.h>
#include <srrg2_core/srrg_pcl/point_types.h>
#include <srrg2_core/srrg_pcl/point_types_data.h>
#include <srrg2_core/srrg_system_utils/parse_command_line.h>
#include <srrg2_core/srrg_system_utils/system_utils.h>

#include <inesctec_mrdt_slam_distmap_2d/geometry/defs.hpp>
#include <inesctec_mrdt_slam_distmap_2d/test/point_cloud_examples.hpp>

using namespace srrg2_core;
using namespace inesctec_mrdt_slam_distmap_2d;

const char* banner[] = {
    "usage: rosrun inesctec_mrdt_slam_distmap_2d "
    "inesctec_mrdt_slam_distmap_2d_test_example_srrg2_core_vector_data_boss "
    "[-w]",
    0};

int main(int argc, char** argv)
{
  point_cloud_registerTypes();

  std::random_device rd;
  std::mt19937 rng(rd());

  ParseCommandLine cmd(argv, banner);

  ArgumentFlag write(&cmd, "w", "write",
                     "Serialize class, if not set it will deserialize");

  std::string filename = std::string(__FILE__) + ".json";

  cmd.parse();

  if (write.isSet())
  {
    Serializer ser;
    std::cerr << "writing on file [" << filename << "]" << std::endl;

    ser.setFilePath(filename);

    float max_range = 12.0f;

    float ang_res = geometry::degToRad(1.0f);
    float fov = 2 * M_PIf32;

    Point2fVectorBLOBReference pc_blob;

    pc_blob.set(new Point2fVectorData);
    pc_blob.get()->clear();

    test::getLaser2DPointCloudRandom(*pc_blob.get(), ang_res, fov, max_range,
                                     rng);

    ser.writeObject(pc_blob);
  }
  else
  {
    Deserializer des;
    std::cerr << "reading from file [" << filename << "]" << std::endl;

    des.setFilePath(filename);

    SerializablePtr o;

    std::vector<SerializablePtr> v;

    while ((o = des.readObjectShared()))
    {
      v.push_back(o);
    }

    std::cerr << "number of elements: " << v.size() << std::endl;

    for (auto o : v)
    {
      std::cerr << "Object pointer : " << o << std::endl;
      std::cerr << "Type : " << o->className() << std::endl;

      std::shared_ptr<Point2fVectorBLOBReference> ao =
          std::dynamic_pointer_cast<Point2fVectorBLOBReference>(o);

      if (ao)
      {
        std::cerr << "Point cloud size: " << ao->get()->size() << std::endl;

        for (size_t idx = 0; idx < ao->get()->size(); idx++)
        {
          std::cerr << "pt " << static_cast<int>(idx) << ": "
                    << ao->get()->operator[](idx).coordinates().transpose()
                    << std::endl;
        }
      }
    }
  }

}  // int main(int argc, char** argv)
