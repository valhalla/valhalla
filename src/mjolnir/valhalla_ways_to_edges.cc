#include <cstdint>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <unordered_map>
#include <vector>

#include "config.h"

#include "baldr/rapidjson_utils.h"
#include <boost/filesystem/operations.hpp>
#include <boost/optional.hpp>
#include <boost/program_options.hpp>
#include <boost/property_tree/ptree.hpp>
#include <ostream>

#include "baldr/directededge.h"
#include "baldr/edgeinfo.h"
#include "baldr/graphreader.h"
#include "baldr/graphtile.h"
#include "baldr/tilehierarchy.h"

namespace bpo = boost::program_options;

using namespace valhalla::baldr;
using namespace valhalla::midgard;

boost::filesystem::path config_file_path;
std::vector<std::string> input_files;

// Structure holding an edge Id and forward flag
struct EdgeAndDirection {
  bool forward;
  GraphId edgeid;

  EdgeAndDirection(const bool f, const GraphId& id) : forward(f), edgeid(id) {
  }
};

bool ParseArguments(int argc, char* argv[]) {

  bpo::options_description options(
      "ways_to_edges " VALHALLA_VERSION "\n"
      "\n"
      " Usage: ways_to_edges [options]\n"
      "\n"
      "ways_to_edges is a program that creates a list of edges for each OSM way "
      "on the local level tiles."
      "\n"
      "\n");

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

  if (vm.count("help")) {
    std::cout << options << "\n";
    return true;
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

// Main application to create a list wayids and directed edges belonging
// to ways that are driveable.
int main(int argc, char** argv) {
  // Parse command line arguments
  if (!ParseArguments(argc, argv)) {
    return EXIT_FAILURE;
  }

  // Get the config to see which coverage we are using
  boost::property_tree::ptree pt;
  rapidjson::read_json(config_file_path.c_str(), pt);

  // Get something we can use to fetch tiles
  auto tile_properties = pt.get_child("mjolnir");
  auto local_level = TileHierarchy::levels().rbegin()->second.level;
  auto tiles = TileHierarchy::levels().rbegin()->second.tiles;

  // Create an unordered map of OSM ways Ids and their associated graph edges
  std::unordered_map<uint64_t, std::vector<EdgeAndDirection>> ways_edges;

  // Iterate through tiles at the local level
  GraphReader reader(pt.get_child("mjolnir"));
  for (uint32_t id = 0; id < tiles.TileCount(); id++) {
    // If tile exists add it to the queue
    GraphId edge_id(id, local_level, 0);
    if (!reader.DoesTileExist(tile_properties, edge_id)) {
      continue;
    }

    const GraphTile* tile = reader.GetGraphTile(edge_id);
    for (uint32_t n = 0; n < tile->header()->directededgecount(); n++, ++edge_id) {
      const DirectedEdge* edge = tile->directededge(edge_id);
      if (edge->IsTransitLine() || edge->use() == Use::kTransitConnection ||
          edge->use() == Use::kEgressConnection || edge->use() == Use::kPlatformConnection) {
        continue;
      }

      // Skip if the edge does not allow auto use
      if (!(edge->forwardaccess() & kAutoAccess)) {
        continue;
      }

      // Get the way Id
      uint64_t wayid = tile->edgeinfo(edge->edgeinfo_offset()).wayid();
      ways_edges[wayid].push_back({edge->forward(), edge_id});
    }
  }

  std::ofstream ways_file;
  std::string fname = pt.get<std::string>("mjolnir.tile_dir") + "/way_edges.txt";
  ways_file.open(fname, std::ofstream::out | std::ofstream::trunc);
  for (auto way : ways_edges) {
    ways_file << way.first;
    for (auto edge : way.second) {
      ways_file << "," << (uint32_t)edge.forward << "," << (uint64_t)edge.edgeid;
    }
    ways_file << std::endl;
  }
  ways_file.close();

  return EXIT_SUCCESS;
}
