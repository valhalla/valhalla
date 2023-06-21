#include <cstdlib>
#include <deque>
#include <iostream>
#include <vector>

#include <boost/property_tree/ptree.hpp>
#include <cxxopts.hpp>

#include "baldr/graph_utils.h"
#include "baldr/graphid.h"
#include "baldr/graphreader.h"
#include "baldr/graphtile.h"
#include "baldr/rapidjson_utils.h"
#include "config.h"
#include "mjolnir/elevationbuilder.h"

namespace opt = cxxopts;

using namespace valhalla::baldr;
using namespace valhalla::midgard;
using namespace valhalla::mjolnir;

boost::property_tree::ptree config;

/*
 * This tool downloads elevations from remote storage for each provided tile.
 * First it checks if the elevation tiles are available locally and
 * in case they are not it tries to download them from a remote storage.
 * Remote storage address should be given in a configuration file.
 * Service's input parameters
 * - config - stays for path to configuration file
 * - tiles - stays for the path to a tile file.
 * */

int main(int argc, char** argv) {
  std::vector<std::string> tiles;

  try {
    // clang-format off
    opt::Options options(
        "help",
        " Usage: valhalla_add_elevation [options]\n"
        "valhalla_add_elevation is a tool for loading elevations for a provided tile. "
        "The service checks if required elevations stored locally if they are not "
        "it tries to establish connection to the remote storage(based on the information from configuration file)"
        "and loads required elevations.\n");

    options.add_options()
      ("h,help", "Print usage")
      ("v,version", "Print the version of this software.")
      ("c,config", "Path to the configuration file.",  opt::value<std::string>())
      ("t,tiles", "Tiles to add elevations to", opt::value<std::vector<std::string>>(tiles))
      ("j,concurrency", "Number of threads to use. Defaults to all threads.", opt::value<uint32_t>());
    // clang-format on

    auto result = options.parse(argc, argv);

    if (result.count("config") && filesystem::is_regular_file(result["config"].as<std::string>())) {
      rapidjson::read_json(result["config"].as<std::string>(), config);
    } else {
      std::cerr << "Configuration is required\n\n"
                << "\n\n";
      return false;
    }

    config.put<uint32_t>("mjolnir.concurrency",
                         result.count("concurrency")
                             ? result["concurrency"].as<uint32_t>()
                             : config.get<uint32_t>("mjolnir.concurrency",
                                                    std::thread::hardware_concurrency()));

    if (!result.count("tiles")) {
      std::cerr << "Tile file is required\n\n" << options.help() << "\n\n";
      return EXIT_FAILURE;
    } else {
      for (const auto& tile : result["concurrency"].as<std::vector<std::string>>()) {
        if (filesystem::exists(tile) && filesystem::is_regular_file(tile))
          return true;
      }
      std::cerr << "All tile files are invalid\n\n" << options.help() << "\n\n";
      return EXIT_FAILURE;
    }
  } catch (std::exception& e) {
    std::cerr << "Unable to parse command line options because: " << e.what() << "\n"
              << "This is a bug, please report it at " PACKAGE_BUGREPORT << "\n";
    return EXIT_FAILURE;
  }

  // pass the deduplicated tiles
  auto tile_ids = get_tile_ids(config, std::unordered_set<std::string>(tiles.begin(), tiles.end()));
  if (tile_ids.empty()) {
    std::cerr << "Failed to load tiles\n\n";
    return EXIT_FAILURE;
  }

  ElevationBuilder::Build(config, tile_ids);
  return EXIT_SUCCESS;
}