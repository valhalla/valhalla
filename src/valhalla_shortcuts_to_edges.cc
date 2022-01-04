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
bool export_edge_id = true;

// Structure holding an edge Id and forward flag
// struct EdgeAndDirection {
//   bool forward;
//   GraphId edgeid;

//   EdgeAndDirection(const bool f, const GraphId& id) : forward(f), edgeid(id) {
//   }
// };

bool ParseArguments(int argc, char* argv[]) {

  bpo::options_description options(
      "shortcuts_to_edges " VALHALLA_VERSION "\n"
      "\n"
      " Usage: shortcuts_to_edges [options]\n"
      "\n"
      "shortcuts_to_edges is a program that creates a list of edges for each shortcut."
      "\n"
      "\n");

  options.add_options()("help,h", "Print this help message.")("version,v",
                                                              "Print the version of this software.")(
      "config,c", boost::program_options::value<filesystem::path>(&config_file_path)->required(),
      "Path to the json configuration file.")("export_edge_id,e",
                                              boost::program_options::value<bool>(&export_edge_id));

  // bpo::positional_options_description pos_options;
  // pos_options.add("input_files", 16);

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
    std::cout << "shortcuts_to_edges " << VALHALLA_VERSION << "\n";
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

  // bool export_edge_id = export_edge_id;
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

      edge_count++;
      auto result = reader.RecoverShortcut(edge_id);
      float shortcut_time = 0;

      if (result.size() < 2)
        continue;

      for (auto id : result) {
        if (export_edge_id) {
          // insert edge id
          shortcuts_edges[edge_id].emplace_back(id);
        } else {
          // insert link_id
          auto tile2 = reader.GetGraphTile(GraphId{id});
          auto edge2 = tile2->directededge(id);
          uint64_t wayid = tile2->edgeinfo(edge2).wayid();
          shortcuts_edges[edge_id].emplace_back(wayid);
        }
      } // for
    }
  }
  LOG_INFO("Shortcuts in total: " + std::to_string(edge_count));

  edge_count = 0;
  std::ofstream shortcuts_file;
  std::string fname = pt.get<std::string>("mjolnir.tile_dir") + filesystem::path::preferred_separator;
  if (export_edge_id) {
    fname += "shortcuts_edges.txt";
  } else {
    fname += "shortcuts_links.txt";
  }

  shortcuts_file.open(fname, std::ofstream::out | std::ofstream::trunc);
  for (const auto& shortcut : shortcuts_edges) {
    shortcuts_file << (uint64_t)shortcut.first;

    for (auto edge : shortcut.second) {
      if (export_edge_id) {
        if (shortcut.first.tileid() != edge.tileid()) {
          // LOG_ERROR("shortcut " + std::to_string((uint64_t)shortcut.first) +
          //           " contains different tiles");
          edge_count++;
        }
      }
      shortcuts_file << " " << (uint64_t)edge;
    }
    shortcuts_file << std::endl;
  }
  shortcuts_file.close();

  if (export_edge_id) {
    LOG_INFO("Shortcuts in different tiles: " + std::to_string(edge_count));
  }
  return EXIT_SUCCESS;
}
