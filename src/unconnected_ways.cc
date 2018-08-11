#include "baldr/rapidjson_utils.h"
#include <boost/format.hpp>
#include <boost/optional.hpp>
#include <boost/program_options.hpp>
#include <boost/property_tree/ptree.hpp>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <set>
#include <string>
#include <tuple>
#include <vector>

#include "config.h"

#include "baldr/graphreader.h"
#include "baldr/pathlocation.h"
#include "midgard/aabb2.h"
#include "midgard/logging.h"
#include "midgard/pointll.h"

using namespace valhalla::midgard;
using namespace valhalla::baldr;

namespace bpo = boost::program_options;

// Main method for testing a single path
int main(int argc, char* argv[]) {
  bpo::options_description options(
      "unconnected_ways " VERSION "\n"
      "\n"
      " Usage: unconnected_ways [options]\n"
      "\n"
      "unconnected_ways is a simple command line test tool to find unconnected ways\n"
      "within a bounding box.\n"
      "\n"
      "\n");

  std::string minll, maxll, config;
  options.add_options()("help,h", "Print this help message.")("version,v",
                                                              "Print the version of this software.")(
      "min,n", boost::program_options::value<std::string>(&minll),
      "minll: lat,lng")("max,x", boost::program_options::value<std::string>(&maxll), "maxll: lat,lng")
      // positional arguments
      ("config,c", bpo::value<std::string>(&config), "Valhalla configuration file");

  bpo::positional_options_description pos_options;
  pos_options.add("config", 1);
  bpo::variables_map vm;

  try {
    bpo::store(bpo::command_line_parser(argc, argv).options(options).positional(pos_options).run(),
               vm);
    bpo::notify(vm);
  } catch (std::exception& e) {
    std::cerr << "Unable to parse command line options because: " << e.what() << "\n"
              << "This is a bug, please report it at " PACKAGE_BUGREPORT << "\n";
    return EXIT_FAILURE;
  }

  if (vm.count("help")) {
    std::cout << options << "\n";
    return EXIT_SUCCESS;
  }

  if (vm.count("version")) {
    std::cout << "unconnected_ways " << VERSION << "\n";
    return EXIT_SUCCESS;
  }

  // argument checking and verification
  AABB2<PointLL> bb;
  boost::property_tree::ptree json_ptree;
  for (auto arg : std::vector<std::string>{"min", "max", "config"}) {
    if (vm.count(arg) == 0) {
      std::cerr << "The <" << arg << "> mandatory argument was not provided\n";
      std::cerr << options << "\n";
      return EXIT_FAILURE;
    }
    Location minloc = Location::FromCsv(minll);
    Location maxloc = Location::FromCsv(maxll);
    bb = AABB2<PointLL>(minloc.latlng_, maxloc.latlng_);
  }

  // Parse the config
  boost::property_tree::ptree pt;
  rapidjson::read_json(config.c_str(), pt);

  // configure logging
  boost::optional<boost::property_tree::ptree&> logging_subtree =
      pt.get_child_optional("thor.logging");
  if (logging_subtree) {
    auto logging_config =
        valhalla::midgard::ToMap<const boost::property_tree::ptree&,
                                 std::unordered_map<std::string, std::string>>(logging_subtree.get());
    valhalla::midgard::logging::Configure(logging_config);
  }

  // Get graph reader
  valhalla::baldr::GraphReader reader(pt.get_child("mjolnir.hierarchy"));

  // Get list of local tiles needed
  auto tile_hierarchy = reader.GetTileHierarchy();
  auto local_level = tile_hierarchy.levels().rbegin()->second.level;
  auto tiles = tile_hierarchy.levels().rbegin()->second.tiles;
  std::vector<int32_t> tilelist = tiles.TileList(bb);

  // Find unconnected way ids within the tiles
  std::set<uint64_t> wayids;
  for (auto local_tile : tilelist) {
    GraphId tile_id(local_tile, local_level, 0);
    const GraphTile* tile = reader.GetGraphTile(tile_id);
    const DirectedEdge* de = tile->directededge(0);
    for (uint32_t n = 0; n < tile->header()->directededgecount(); n++, de++) {
      if (de->unreachable()) {
        wayids.insert(tile->edgeinfo(de->edgeinfo_offset())->wayid());
      }
    }
  }

  // Log the list of unreachable ways
  LOG_INFO("unreachable ways:");
  for (auto w : wayids) {
    LOG_INFO(std::to_string(w));
  }

  return EXIT_SUCCESS;
}
