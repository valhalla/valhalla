#include "argparse_utils.h"
#include "mjolnir/add_predicted_speeds.h"

#include <boost/property_tree/ptree.hpp>
#include <cxxopts.hpp>

#include <filesystem>
#include <iostream>
#include <string>

namespace vm = valhalla::midgard;
namespace vb = valhalla::baldr;
namespace vj = valhalla::mjolnir;
namespace bpt = boost::property_tree;

int main(int argc, char** argv) {
  const auto program = std::filesystem::path(__FILE__).stem().string();
  // args
  std::filesystem::path traffic_tile_dir;
  bool summary = false;
  boost::property_tree::ptree config;
  try {
    // clang-format off
    cxxopts::Options options(
      program,
      program + " " + VALHALLA_PRINT_VERSION + "\n\n"
      "adds predicted traffic to valhalla tiles.\n");
    options.add_options()
      ("h,help", "Print this help message.")
      ("v,version", "Print the version of this software.")
      ("j,concurrency", "Number of threads to use.", cxxopts::value<unsigned int>())
      ("c,config", "Path to the json configuration file.", cxxopts::value<std::string>())
      ("i,inline-config", "Inline json config.", cxxopts::value<std::string>())
      ("s,summary", "Output summary information about traffic coverage for the tile set", cxxopts::value<bool>(summary))
      ("t,traffic-tile-dir", "positional argument", cxxopts::value<std::string>());
    // clang-format on
    options.parse_positional({"traffic-tile-dir"});
    options.positional_help("Traffic tile dir");
    auto result = options.parse(argc, argv);
    if (!parse_common_args(program, options, result, &config, "mjolnir.logging", true))
      return EXIT_SUCCESS;
    if (!result.count("traffic-tile-dir")) {
      std::cout << "You must provide a tile directory to read the csv tiles from.\n";
      return EXIT_SUCCESS;
    }
    traffic_tile_dir = std::filesystem::path(result["traffic-tile-dir"].as<std::string>());
  } catch (cxxopts::exceptions::exception& e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
  } catch (std::exception& e) {
    std::cerr << "Unable to parse command line options because: " << e.what() << "\n"
              << "This is a bug, please report it at " PACKAGE_BUGREPORT << "\n";
    return EXIT_FAILURE;
  }
  // Prepare traffic tiles
  // Get tile directory from config
  auto tile_dir = config.get<std::string>("mjolnir.tile_dir");
  // Process traffic tiles
  vj::ProcessTrafficTiles(tile_dir, traffic_tile_dir, summary, config);

  return EXIT_SUCCESS;
}
