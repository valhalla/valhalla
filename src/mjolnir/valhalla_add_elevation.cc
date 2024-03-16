#include <cstdlib>
#include <deque>
#include <iostream>
#include <vector>

#include <boost/property_tree/ptree.hpp>
#include <cxxopts.hpp>

#include "baldr/graphid.h"
#include "baldr/graphreader.h"
#include "baldr/graphtile.h"
#include "baldr/rapidjson_utils.h"
#include "mjolnir/elevationbuilder.h"

#include "argparse_utils.h"

namespace opt = cxxopts;

using namespace valhalla::baldr;
using namespace valhalla::midgard;
using namespace valhalla::mjolnir;

namespace {
std::deque<GraphId> get_tile_ids(const boost::property_tree::ptree& pt,
                                 const std::unordered_set<std::string>& tiles) {
  if (tiles.empty())
    return {};

  auto tile_dir = pt.get_optional<std::string>("mjolnir.tile_dir");
  if (!tile_dir || !filesystem::exists(*tile_dir)) {
    LOG_WARN("Tile storage directory does not exist");
    return {};
  }

  std::unordered_set<std::string> tiles_set{tiles.begin(), tiles.end()};

  std::deque<GraphId> tilequeue;
  GraphReader reader(pt.get_child("mjolnir"));
  std::for_each(std::begin(tiles), std::end(tiles), [&](const auto& tile) {
    auto tile_id = GraphTile::GetTileId(*tile_dir + tile);
    GraphId local_tile_id(tile_id.tileid(), tile_id.level(), tile_id.id());
    if (!reader.DoesTileExist(local_tile_id)) {
      LOG_WARN("Provided tile doesn't belong to the tile directory from config file");
      return;
    }

    tilequeue.push_back(tile_id);
  });

  return tilequeue;
}
} // namespace

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
  const auto program = filesystem::path(__FILE__).stem().string();
  // args
  std::vector<std::string> tiles;
  boost::property_tree::ptree config;

  try {
    // clang-format off
    opt::Options options(
        program,
        std::string(program) + " " + VALHALLA_VERSION + "\n\n"
        "a tool for loading elevations for a provided tile. "
        "The service checks if required elevations stored locally if they are not "
        "it tries to establish connection to the remote storage (based on the information from configuration file)"
        "and loads required elevations.\n");

    options.add_options()
      ("h,help", "Print usage")
      ("v,version", "Print the version of this software.")
      ("c,config", "Path to the configuration file.",  opt::value<std::string>())
      ("i,inline-config", "Inline JSON config", cxxopts::value<std::string>())
      ("t,tiles", "Tiles to add elevations to", opt::value<std::vector<std::string>>(tiles))
      ("j,concurrency", "Number of threads to use. Defaults to all threads.", opt::value<uint32_t>());
    // clang-format on

    const auto result = options.parse(argc, argv);
    if (!parse_common_args(program, options, result, config, "mjolnir.logging", true))
      return EXIT_SUCCESS;

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
  } catch (cxxopts::exceptions::exception& e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
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