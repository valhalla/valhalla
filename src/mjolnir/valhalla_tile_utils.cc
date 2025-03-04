#include "baldr/graphreader.h"
#include "baldr/graphtile.h"
#include <filesystem>
namespace fs = std::filesystem;

#include <boost/algorithm/string/replace.hpp>
#include <iostream>
#include <vector>

// Function to get the traffic directory for a given edge ID
int handle_get_traffic_dir(uint64_t edge_id) {
  try {
    valhalla::baldr::GraphId graph_id(edge_id);
    auto tile_path = valhalla::baldr::GraphTile::FileSuffix(graph_id);
    auto dir = fs::path(tile_path);
    auto dir_str = dir.string();
    boost::replace_all(dir_str, ".gph", ".csv");
    std::cout << edge_id << ": " << dir_str << std::endl;
  } catch (const std::exception& e) {
    std::cerr << "Error in handle_get_traffic_dir: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}

// Function to get the tile ID for a given edge ID
int handle_get_tile_id(uint64_t edge_id) {
  try {
    valhalla::baldr::GraphId graph_id(edge_id);
    std::cout << edge_id << ": " << graph_id << std::endl;
  } catch (const std::exception& e) {
    std::cerr << "Error in handle_get_tile_id: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}

// Function to get traffic directories for multiple edge IDs
void handle_get_traffic_dirs(const std::vector<uint64_t>& edge_ids) {
  for (const auto& edge_id : edge_ids) {
    handle_get_traffic_dir(edge_id);
  }
}

// Function to get tile IDs for multiple edge IDs
void handle_get_tile_ids(const std::vector<uint64_t>& edge_ids) {
  for (const auto& edge_id : edge_ids) {
    handle_get_tile_id(edge_id);
  }
}

// Main function to demonstrate usage
int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0] << " <operation> <edge_id(s)> " << std::endl;
    std::cerr << "Operations: get-traffic-dir | get-tile-id | get-traffic-dirs | get-tile-ids" << std::endl;
    return EXIT_FAILURE;
  }

  std::string operation = argv[1];
  std::vector<uint64_t> edge_ids;
  for (int i = 2; i < argc; ++i) {
    edge_ids.push_back(std::stoull(argv[i]));
  }

  if (operation == "get-traffic-dir" && edge_ids.size() == 1) {
    return handle_get_traffic_dir(edge_ids[0]);
  } else if (operation == "get-tile-id" && edge_ids.size() == 1) {
    return handle_get_tile_id(edge_ids[0]);
  } else if (operation == "get-traffic-dirs") {
    handle_get_traffic_dirs(edge_ids);
  } else if (operation == "get-tile-ids") {
    handle_get_tile_ids(edge_ids);
  } else {
    std::cerr << "Unknown operation: " << operation << std::endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
