#include <cstdint>
#include <fstream>
#include <iostream>
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
#include "midgard/logging.h"

namespace bpo = boost::program_options;

using namespace valhalla::baldr;
using namespace valhalla::midgard;

boost::filesystem::path config_file_path;
std::vector<std::string> input_files;

const uint32_t kMinutesPerHour = 60;
const uint32_t kMinutesPerDay = 24 * kMinutesPerHour;

/**
 * Structure to define speed along a way
 */
struct WaySpeed {
  uint8_t forward; // Speed in forward direction (kph)
  uint8_t reverse; // Speed in reverse direction (kph)
};

// Structure holding an edge Id and forward flag
struct EdgeAndDirection {
  bool forward;
  GraphId edgeid;

  EdgeAndDirection(const bool f, const GraphId& id) : forward(f), edgeid(id) {
  }
};

bool ParseArguments(int argc, char* argv[]) {

  bpo::options_description options(
      "valhalla_build_speeds " VALHALLA_VERSION "\n"
      "\n"
      " Usage: valhalla_build_speeds [options]\n"
      "\n"
      "valhalla_build_speeds is a program that reads speed data associated to OSM ways "
      "and creates a speed table on the local level tiles."
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
    std::cout << "valhalla_build_speeds " << VALHALLA_VERSION << "\n";
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

std::unordered_map<uint64_t, std::vector<EdgeAndDirection>>
ReadWaysToEdges(const std::string& way_edges_file) {
  std::unordered_map<uint64_t, std::vector<EdgeAndDirection>> ways_to_edges;

  // Open the file
  std::ifstream ways_file;
  ways_file.open(way_edges_file);

  // Get the edges (and direction) for each wayId
  std::string line;
  while (std::getline(ways_file, line)) {
    // Split into tokens separated by ","
    std::string num;
    std::stringstream line_stream(line);
    std::vector<EdgeAndDirection> edges;

    // Get the wayId
    std::getline(line_stream, num, ',');
    uint64_t wayid = std::stoll(num);

    uint32_t n = 0;
    bool forward;
    GraphId edgeid;
    while (std::getline(line_stream, num, ',')) {
      if ((n % 2) == 0) {
        forward = std::stoll(num);
      } else {
        edgeid = GraphId(std::stoll(num));
        edges.push_back({forward, edgeid});
      }
      n++;
    }
    ways_to_edges[wayid] = edges;
  }
  ways_file.close();
  return ways_to_edges;
}

/**
 * Read way speeds CSV file and return a mapping of ways to speeds.
 */
std::unordered_map<uint64_t, WaySpeed> ReadWaySpeeds(const std::string& tile_dir) {
  std::string way_speeds_file = tile_dir + "/traffic/way_speeds.csv";
  LOG_INFO("Read Way Speeds file: " + way_speeds_file);
  std::unordered_map<uint64_t, WaySpeed> way_speeds;
  //  way_speeds.reserve(200000);

  std::ifstream ways_file;
  ways_file.open(way_speeds_file);

  // Get the first line (format)
  std::string line;
  std::getline(ways_file, line);

  // Get way speed: wayid, forward speed (kph), reverse speed (kph)
  while (std::getline(ways_file, line)) {
    // Split into tokens separated by ","
    uint32_t n = 0;
    uint64_t wayid;
    uint32_t forward = 0;
    uint32_t reverse = 0;
    std::string num;
    std::stringstream line_stream(line);
    while (std::getline(line_stream, num, ',')) {
      if (n == 0) {
        wayid = std::stoll(num);
      } else if (n == 1) {
        forward = num.empty() ? 0 : std::stoi(num);
      } else if (n == 2) {
        reverse = num.empty() ? 0 : std::stoi(num);
      }
      n++;
    }

    // Add the speed (kph)
    way_speeds[wayid].forward = forward;
    way_speeds[wayid].reverse = reverse;
  }
  return way_speeds;
}

uint8_t GetSpeed(const bool forward, const uint8_t fwd, const uint8_t rev) {
  return (forward) ? fwd : rev;
}

// Main application to create a ppm image file of connectivity.
int main(int argc, char** argv) {
  // Parse command line arguments
  if (!ParseArguments(argc, argv)) {
    return EXIT_FAILURE;
  }

  // Get the config to see which coverage we are using
  boost::property_tree::ptree pt;
  rapidjson::read_json(config_file_path.string(), pt);

  // Get the tile directory from the config
  std::string tile_dir = pt.get<std::string>("mjolnir.tile_dir");

  // Read the way speed CSV file
  auto way_speeds = ReadWaySpeeds(tile_dir);
  if (way_speeds.size() == 0) {
    LOG_ERROR("No speeds in the way speeds csv file");
    return 0;
  }

  // Stats
  uint32_t n = 0;
  for (auto way : way_speeds) {
    if (way_speeds[way.first].forward > 0 || way_speeds[way.first].reverse > 0) {
      n++;
    }
  }
  LOG_INFO(std::to_string(n) + " ways with valid speeds");

  // Read the OSM way to edge association
  std::string way_edges_file = tile_dir + "/way_edges.txt";
  std::unordered_map<uint64_t, std::vector<EdgeAndDirection>> way_edges;
  way_edges = ReadWaysToEdges(way_edges_file);
  LOG_INFO("Done reading ways to edges file");

  // Get Valhalla tiles
  auto local_level = TileHierarchy::levels().rbegin()->second.level;
  auto tiles = TileHierarchy::levels().rbegin()->second.tiles;

  // Create a map of tiles with speed table for the specified entry
  std::unordered_map<uint32_t, std::vector<uint8_t>> tile_speeds;

  // Iterate through the way Ids
  uint8_t speed;
  uint32_t stored_speeds = 0;
  GraphReader reader(pt.get_child("mjolnir"));
  for (auto way : way_speeds) {
    uint32_t wayid = way.first;
    const WaySpeed& speeds = way.second;
    uint8_t fwd = speeds.forward;
    uint8_t rev = speeds.reverse;

    // Iterate through edges for this wayid
    auto itr = way_edges.find(wayid);
    if (itr != way_edges.end()) {
      for (auto edgeitr : itr->second) {
        // Check if this tile has been encountered
        bool forward = edgeitr.forward;
        GraphId edgeid = GraphId(edgeitr.edgeid);
        uint32_t tileid = edgeid.tileid();
        auto tile_itr = tile_speeds.find(tileid);
        if (tile_itr == tile_speeds.end()) {
          // Create a speed list for this tile
          const GraphTile* tile = reader.GetGraphTile(edgeid.Tile_Base());
          if (tile == nullptr) {
            LOG_ERROR("No tile found for " + std::to_string(edgeid.Tile_Base().tileid()) + "," +
                      std::to_string(edgeid.Tile_Base().level()));
          } else {
            std::vector<uint8_t> speeds(tile->header()->directededgecount());
            tile_speeds[tileid] = speeds;
            tile_speeds[tileid][edgeid.id()] = GetSpeed(forward, fwd, rev);
            stored_speeds++;
          }
        } else {
          tile_itr->second[edgeid.id()] = GetSpeed(forward, fwd, rev);
          stored_speeds++;
        }
      }
    }
  }
  LOG_INFO("Number of ways = " + std::to_string(way_speeds.size()));
  LOG_INFO("Number of speed tiles = " + std::to_string(tile_speeds.size()));
  LOG_INFO("Stored speeds = " + std::to_string(stored_speeds));

  // Write out the speed tiles
  for (auto tile_itr : tile_speeds) {
    // Count the number of non-zero entries
    uint32_t n = 0;
    for (auto speed : tile_itr.second) {
      if (speed > 0) {
        n++;
      }
    }
    LOG_INFO("TileID: " + std::to_string(tile_itr.first) + " speeds = " + std::to_string(n) +
             " out of " + std::to_string(tile_itr.second.size()));

    std::ofstream outfile;
    std::string fname = tile_dir + "/traffic/" + std::to_string(tile_itr.first) + ".spd";
    outfile.open(fname, std::ios::binary | std::ios::out);
    outfile.write((char*)(&tile_itr.second.front()), tile_itr.second.size());
    outfile.close();
  }

  return EXIT_SUCCESS;
}
