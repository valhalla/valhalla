#include <cstdint>
#include <cstdlib>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <unordered_map>
#include <vector>

#include "config.h"

#include "baldr/rapidjson_utils.h"
#include <boost/optional.hpp>
#include <boost/program_options.hpp>
#include <boost/property_tree/ptree.hpp>
#include <ostream>

#include "baldr/directededge.h"
#include "baldr/edgeinfo.h"
#include "baldr/graphreader.h"
#include "baldr/graphtile.h"
#include "baldr/tilehierarchy.h"
#include "filesystem.h"

namespace bpo = boost::program_options;

using namespace valhalla::baldr;
using namespace valhalla::midgard;

filesystem::path config_file_path;

// Structure holding an edge Id and forward flag
// struct EdgeAndDirection {
//   bool forward;
//   GraphId edgeid;

//   EdgeAndDirection(const bool f, const GraphId& id) : forward(f), edgeid(id) {
//   }
// };

bool ParseArguments(int argc, char* argv[]) {

  bpo::options_description options(
      "shortcuts_recost " VALHALLA_VERSION "\n"
      "\n"
      " Usage: shortcuts_recost [options]\n"
      "\n"
      "shortcuts_recost is a program that update the cost of shortcut by using existing traffic and other costs ."
      "\n"
      "\n");

  options.add_options()("help,h", "Print this help message.")("version,v",
                                                              "Print the version of this software.")(
      "config,c", boost::program_options::value<filesystem::path>(&config_file_path)->required(),
      "Path to the json configuration file.");

  bpo::variables_map vm;
  try {
    bpo::store(bpo::command_line_parser(argc, argv).options(options).run(), vm);
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
    std::cout << "shortcuts_recost " << VALHALLA_VERSION << "\n";
    return true;
  }

  if (vm.count("config")) {
    if (filesystem::is_regular_file(config_file_path)) {
      return true;
    } else {
      std::cerr << "Configuration file is required\n\n" << options << "\n\n";
    }
  }

  return false;
}

template <class T> bool write_vector(std::string filename, std::vector<T> vec) {
  std::ofstream file(filename, std::ios::out | std::ios::binary | std::ios::trunc);
  if (!file.is_open()) {
    LOG_ERROR("Failed to open output file: " + filename);
    return false;
  }

  // Write the count and then the via ids
  uint64_t sz = vec.size();
  // file.write(reinterpret_cast<const char*>(&sz), sizeof(T));
  file.write(reinterpret_cast<const char*>(vec.data()), vec.size() * sizeof(T));
  file.close();
  return true;
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
  rapidjson::read_json(config_file_path.string(), pt);
  // rapidjson::read_json("/SDD_datadrive/valhalla/deu/valhalla_deu.json", pt);

  // Create an unordered map of OSM ways Ids and their associated graph edges
  std::unordered_map<GraphId, std::vector<GraphId>> shortcuts_edges;

  uint64_t edge_count = 0;
  GraphReader reader(pt.get_child("mjolnir"));
  // Iterate through all tiles
  for (auto edge_id : reader.GetTileSet()) {
    // If tile exists add it to the queue
    if (!reader.DoesTileExist(edge_id)) {
      continue;
    }
    if (edge_id.level() == 2) {
      continue;
    }

    graph_tile_ptr tile = reader.GetGraphTile(edge_id);
    for (uint32_t n = 0; n < tile->header()->directededgecount(); n++, ++edge_id) {
      const DirectedEdge* edge = tile->directededge(edge_id);
      if (edge->IsTransitLine() || edge->use() == Use::kTransitConnection ||
          edge->use() == Use::kEgressConnection || edge->use() == Use::kPlatformConnection) {
        continue;
      }

      if (!edge->is_shortcut()) {
        continue;
      }

      auto result = reader.RecoverShortcut(edge_id);
      float shortcut_duration = 0;

      if (result.size() < 2)
        continue;

      for (auto id : result) {
        auto tile2 = reader.GetGraphTile(GraphId{id});
        auto edge2 = tile2->directededge(id);

        // auto speed = tile2->GetSpeed(edge2, kNoFlowMask, 1);
        auto speed = tile2->GetSpeed(edge2, 255, 1);

        if (speed == 0) {
          shortcut_duration = 0;
          break;
        } else {
          shortcut_duration += edge2->length() / static_cast<float>(speed);
        }
      } // for

      auto shortcut_speed = tile->GetSpeed(edge, 255, 1);
      decltype(shortcut_speed) new_speed;
      if (shortcut_duration == 0) {
        // closure on road
        new_speed = 0;
      } else {
        // new_speed = static_cast<decltype(shortcut_speed)>(edge->length() / shortcut_time);
        new_speed = static_cast<uint32_t>(std::round(edge->length() / shortcut_duration));
      }

      // if speed has be changed. new speed shall be slower than old speed
      // some shortcut speed has increase a lot after recalculation. root cause is unknown
      // the recalculated length of shortcuts is not changed
      // Therefore, RecoverShortcut works.  
      if (static_cast<int>(shortcut_speed) - static_cast<int>(new_speed) > 2) {
        shortcuts_edges[edge_id].emplace_back(new_speed);
        shortcuts_edges[edge_id].emplace_back(shortcut_speed);
      }

      edge_count++;
    }
  }
  LOG_INFO("Shortcuts in total: " + std::to_string(edge_count));
  LOG_INFO("Write shortcut speed [edge_id, new speed, old speed]]");
  edge_count = 0;
  std::ofstream shortcuts_file;
  std::string fname = pt.get<std::string>("mjolnir.tile_dir") +
                      filesystem::path::preferred_separator + "shortcuts_speed.txt";

  shortcuts_file.open(fname, std::ofstream::out | std::ofstream::trunc);
  for (const auto& shortcut : shortcuts_edges) {
    shortcuts_file << (uint64_t)shortcut.first;
    for (auto speed : shortcut.second) {
      shortcuts_file << " " << (uint64_t)speed;
    }
    shortcuts_file << std::endl;
    edge_count++;
  }
  shortcuts_file.close();
  LOG_INFO("Write shortcut: " + std::to_string(edge_count));
  return EXIT_SUCCESS;
}
