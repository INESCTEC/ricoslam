#include "inesctec_mrdt_slam_2d_datasets/carmen_to_rosbag_parser.hpp"





int main(int argc, char* argv[])
{
  boost::program_options::options_description opts(
      "Usage: rosrun inesctec_mrdt_slam_2d_datasets carmen_to_rosbag [options]"
      "\n\nOptions");

  boost::program_options::variables_map opts_vm;

  opts.add_options()("help,h", "Display this information.");



  inesctec_mrdt_slam_2d_datasets::CARMENToROSbagParser parser;

  parser.addOptions(opts);



  try
  {
    boost::program_options::store(
        boost::program_options::parse_command_line(argc, argv, opts),
        opts_vm);

    if (opts_vm.count("help"))
    {
      std::cout << opts << std::endl;
      return 0;
    }
  }
  catch (boost::exception& e)
  {
    std::cout << "something went wrong... (error: "
              << boost::diagnostic_information(e) << ")"
              << std::endl << std::endl;
    std::cout << opts << std::endl;
    return 0;
  }
  catch (...)
  {
    std::cout << "unexpected behavior..." << std::endl << std::endl;
    std::cout << opts << std::endl;
    return -1;
  }

  try
  {
    boost::program_options::notify(opts_vm);
  }
  catch (boost::exception& e)
  {
    std::cout << "something went wrong... (error: "
              << boost::diagnostic_information(e) << ")" << std::endl << std::endl;
    std::cout << opts << std::endl;
    return -1;
  }
  catch (...)
  {
    std::cout << "unexpected behavior..." << std::endl << std::endl;
    std::cout << opts << std::endl;
    return -1;
  }



  try
  {
    parser.load();
  }
  catch(const std::exception& e)
  {
    std::cerr << "\033[1m\033[31m"
                  << "[inesctec_mrdt_slam_2d_datasets] Fatal error: " << e.what()
              << "\033[0m" << std::endl;
  }
  catch (...)
  {
    std::cerr << "\033[1m\033[31m"
                  << "[inesctec_mrdt_slam_2d_datasets] Unexpected fatal error..."
              << "\033[0m" << std::endl;
  }

  return 0;
}
