#include <fstream>
#include <iostream>
#include <vector>

#include "baldr/rapidjson_utils.h"
#include <boost/filesystem/operations.hpp>
#include <boost/optional.hpp>
#include <boost/program_options.hpp>
#include <boost/property_tree/ptree.hpp>

#include "baldr/graphreader.h"
#include "baldr/graphtile.h"
#include "baldr/tilehierarchy.h"
#include "midgard/aabb2.h"
#include "midgard/pointll.h"
#include "midgard/tiles.h"

#include "config.h"

namespace bpo = boost::program_options;

using namespace valhalla::baldr;
using namespace valhalla::midgard;

// Iterate over all Valhalla tiles within the coverage (specified in the
// mjolnir properties) and identify the 1 degree tiles that include
// any Valhalla edge. Returns a list of the base (SW) lat,lng of the tiles
std::vector<PointLL> get_land_tiles(const boost::property_tree::ptree& pt) {
  AABB2<PointLL> world(-180.0f, -90.0f, 180.f, 90.f);
  Tiles<PointLL> one_degree_tiles(world, 1.0f);
  Tiles<PointLL> local_tiles(world, 0.25f);

  std::set<int> land_tile_ids;
  boost::property_tree::ptree hierarchy_properties = pt.get_child("mjolnir");
  auto local_level = TileHierarchy::levels().rbegin()->second.level;
  GraphReader reader(hierarchy_properties);
  auto local_tile_ids = reader.GetTileSet(local_level);
  for (const auto& tile_id : local_tile_ids) {
    // Convert the local tile to a 1 degree tile ID. Add to the set.
    PointLL center = local_tiles.Center(tile_id.tileid());
    land_tile_ids.insert(one_degree_tiles.TileId(center));
  }

  // Iterate through the set and convert to SW corner lat,lngs
  std::vector<PointLL> ll;
  for (auto id : land_tile_ids) {
    ll.push_back(one_degree_tiles.Base(id));
  }
  return ll;
}

boost::filesystem::path config_file_path;

bool ParseArguments(int argc, char* argv[]) {
  std::vector<std::string> input_files;

  bpo::options_description options(
      " Usage: valhalla_land_tiles [options]\n"
      "valhalla_land_tiles is a program that creates a list of base lat,lng for 1 degree "
      "tiles that include Valhalla data.\n");

  options.add_options()("help,h", "Print this help message.")("version,v",
                                                              "Print the version of this software.")(
      "config,c",
      boost::program_options::value<boost::filesystem::path>(&config_file_path)->required(),
      "Path to the json configuration file.")
      // positional arguments
      ("input_files",
       boost::program_options::value<std::vector<std::string>>(&input_files)->multitoken());

  bpo::positional_options_description pos_options;
  pos_options.add("input_files", 16);

  bpo::variables_map vm;
  try {
    bpo::store(bpo::command_line_parser(argc, argv).options(options).positional(pos_options).run(),
               vm);
    bpo::notify(vm);

  } catch (std::exception& e) {
    std::cerr << "Unable to parse command line options because: " << e.what() << "\n"
              << "This is a bug, please report it at " PACKAGE_BUGREPORT << "\n";
    return false;
  }
  if (vm.count("version")) {
    std::cout << "ways_to_edges " << VALHALLA_VERSION << "\n";
    return true;
  }

  if (vm.count("config")) {
    if (boost::filesystem::is_regular_file(config_file_path)) {
      return true;
    } else {
      std::cerr << "Configuration file is required\n\n" << options << "\n\n";
    }
  }
  return false;
}

int main(int argc, char** argv) {
  // Parse command line arguments
  if (!ParseArguments(argc, argv)) {
    return EXIT_FAILURE;
  }

  // Get the config to see which coverage we are using
  boost::property_tree::ptree pt;
  rapidjson::read_json(config_file_path.string(), pt);

  // Get the base lat,lng of all 1 degree tiles that include a Valhalla tile
  auto lls = get_land_tiles(pt);

  // Open output file, iterate through the list of land tiles and output
  // the elevation filename required for this tile. Uses N,S for latitude
  // and E,W for longitude and requires 2 digits for latitude and 3 digits
  // for longitude (padded with 0s).
  std::ofstream landtiles;
  landtiles.open("elevation_tiles.txt");
  for (const auto& ll : lls) {
    if (ll.lat() >= 0.0f) {
      landtiles << "N" << std::setfill('0') << std::setw(2) << ll.lat();
    } else {
      landtiles << "S" << std::setfill('0') << std::setw(2) << -ll.lat();
    }
    if (ll.lng() >= 0.0f) {
      landtiles << "E" << std::setfill('0') << std::setw(3) << ll.lng();
    } else {
      landtiles << "W" << std::setfill('0') << std::setw(3) << -ll.lng();
    }
    landtiles << std::endl;
  }
  landtiles.close();
}
