#include <cstdint>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <unordered_map>
#include <vector>

#include <boost/property_tree/ptree.hpp>
#include <cxxopts.hpp>

#include "baldr/directededge.h"
#include "baldr/edgeinfo.h"
#include "baldr/graphreader.h"
#include "baldr/graphtile.h"
#include "baldr/rapidjson_utils.h"
#include "baldr/tilehierarchy.h"
#include "filesystem.h"
#include "midgard/logging.h"

#include "argparse_utils.h"

using namespace valhalla::baldr;
using namespace valhalla::midgard;

// Structure holding an edge Id and forward flag
struct EdgeAndDirection {
  bool forward;
  GraphId edgeid;

  EdgeAndDirection(const bool f, const GraphId& id) : forward(f), edgeid(id) {
  }
};

// Main application to create a list wayids and directed edges belonging
// to ways that are drivable.
int main(int argc, char** argv) {
  const auto program = filesystem::path(__FILE__).stem().string();
  // args
  boost::property_tree::ptree config;

  try {
    // clang-format off
    cxxopts::Options options(
      program,
      program + " " + VALHALLA_VERSION + "\n\n"
      "a program that creates a list of edges for each auto-drivable OSM way.\n\n");

    options.add_options()
      ("h,help", "Print this help message.")
      ("i,inline-config", "Inline JSON config", cxxopts::value<std::string>())
      ("v,version", "Print the version of this software.")
      ("c,config", "Path to the json configuration file.", cxxopts::value<std::string>());
    // clang-format on

    auto result = options.parse(argc, argv);
    if (!parse_common_args(program, options, result, config, "mjolnir.logging"))
      return EXIT_SUCCESS;

  } catch (cxxopts::exceptions::exception& e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
  } catch (std::exception& e) {
    std::cerr << "Unable to parse command line options because: " << e.what() << "\n"
              << "This is a bug, please report it at " PACKAGE_BUGREPORT << "\n";
    return EXIT_FAILURE;
  }

  // Create an unordered map of OSM ways Ids and their associated graph edges
  std::unordered_map<uint64_t, std::vector<EdgeAndDirection>> ways_edges;

  GraphReader reader(config.get_child("mjolnir"));
  // Iterate through all tiles
  for (auto edge_id : reader.GetTileSet()) {
    // If tile exists add it to the queue
    if (!reader.DoesTileExist(edge_id)) {
      continue;
    }
    if (reader.OverCommitted()) {
      reader.Trim();
    }

    graph_tile_ptr tile = reader.GetGraphTile(edge_id);
    for (uint32_t n = 0; n < tile->header()->directededgecount(); n++, ++edge_id) {
      const DirectedEdge* edge = tile->directededge(edge_id);
      if (edge->IsTransitLine() || edge->use() == Use::kTransitConnection ||
          edge->use() == Use::kEgressConnection || edge->use() == Use::kPlatformConnection ||
          edge->is_shortcut()) {
        continue;
      }

      // Skip if the edge does not allow auto use
      if (!(edge->forwardaccess() & kAutoAccess)) {
        continue;
      }

      // Get the way Id
      uint64_t wayid = tile->edgeinfo(edge).wayid();
      ways_edges[wayid].push_back({edge->forward(), edge_id});
    }
  }

  std::ofstream ways_file;
  std::string fname = config.get<std::string>("mjolnir.tile_dir") +
                      filesystem::path::preferred_separator + "way_edges.txt";
  ways_file.open(fname, std::ofstream::out | std::ofstream::trunc);
  for (const auto& way : ways_edges) {
    ways_file << way.first;
    for (auto edge : way.second) {
      ways_file << "," << (uint32_t)edge.forward << "," << (uint64_t)edge.edgeid;
    }
    ways_file << std::endl;
  }
  ways_file.close();

  LOG_INFO("Finished with " + std::to_string(ways_edges.size()) + " ways.");

  return EXIT_SUCCESS;
}
