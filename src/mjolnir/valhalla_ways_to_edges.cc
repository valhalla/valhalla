#include <cstdint>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <unordered_map>
#include <vector>

#include "config.h"

#include "baldr/rapidjson_utils.h"
#include <boost/property_tree/ptree.hpp>
#include <cxxopts.hpp>
#include <ostream>

#include "baldr/directededge.h"
#include "baldr/edgeinfo.h"
#include "baldr/graphreader.h"
#include "baldr/graphtile.h"
#include "baldr/tilehierarchy.h"
#include "filesystem.h"
#include "midgard/logging.h"

using namespace valhalla::baldr;
using namespace valhalla::midgard;

filesystem::path config_file_path;
boost::property_tree::ptree pt;

// Structure holding an edge Id and forward flag
struct EdgeAndDirection {
  bool forward;
  GraphId edgeid;

  EdgeAndDirection(const bool f, const GraphId& id) : forward(f), edgeid(id) {
  }
};

bool ParseArguments(int argc, char* argv[]) {
  try {
    // clang-format off
    cxxopts::Options options(
      "valhalla_ways_to_edges",
      "valhalla_ways_to_edges " VALHALLA_VERSION "\n\n"
      "valhalla_ways_to_edges is a program that creates a list of edges for each auto-driveable OSM way.\n\n");

    options.add_options()
      ("h,help", "Print this help message.")
      ("i,inline-config", "Inline JSON config", cxxopts::value<std::string>())
      ("v,version", "Print the version of this software.")
      ("c,config", "Path to the json configuration file.", cxxopts::value<std::string>());
    // clang-format on

    auto result = options.parse(argc, argv);

    if (result.count("help")) {
      std::cout << options.help() << "\n";
      exit(0);
    }

    if (result.count("version")) {
      std::cout << "valhalla_ways_to_edges " << VALHALLA_VERSION << "\n";
      exit(0);
    }

    // Read the config file
    if (result.count("inline-config")) {
      std::stringstream ss;
      ss << result["inline-config"].as<std::string>();
      rapidjson::read_json(ss, pt);
      return true;
    } else if (result.count("config") &&
               filesystem::is_regular_file(
                   config_file_path = filesystem::path(result["config"].as<std::string>()))) {
      rapidjson::read_json(config_file_path.string(), pt);
      return true;
    } else {
      std::cerr << "Configuration is required\n" << options.help() << std::endl;
      return false;
    }
  } catch (const cxxopts::OptionException& e) {
    std::cout << "Unable to parse command line options because: " << e.what() << std::endl;
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

  // Create an unordered map of OSM ways Ids and their associated graph edges
  std::unordered_map<uint64_t, std::vector<EdgeAndDirection>> ways_edges;

  GraphReader reader(pt.get_child("mjolnir"));
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
  std::string fname = pt.get<std::string>("mjolnir.tile_dir") +
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
