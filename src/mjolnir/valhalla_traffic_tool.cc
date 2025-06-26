
#include "baldr/graphreader.h"
#include "baldr/predictedspeeds.h"
#include "baldr/rapidjson_utils.h"
#include "config.h"
#include "filesystem.h"
#include "microtar.h"
#include "mjolnir/graphtilebuilder.h"
#include "mjolnir/valhalla_traffic_utils.h"
#include "rapidjson/document.h"

#include <boost/algorithm/string/replace.hpp>
#include <boost/property_tree/ptree.hpp>
#include <cxxopts.hpp>

#include <algorithm>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <filesystem>


int handle_help(cxxopts::Options options) {
  std::cout << options.help() << std::endl;
  return EXIT_SUCCESS;
}


int main(int argc, char** argv) {
  // args
  std::string config_file_path;
  try {
    // clang-format off
    cxxopts::Options options(argv[0], " - Provides utilities for adding traffic to valhalla routing tiles.");

    options.add_options()
        ("h,help", "Print this help message.")
        ("c,config", "Path to the json configuration file.",
            cxxopts::value<std::string>(config_file_path))
				("update-tile-traffic", "Update traffic for a given tile. Usage: --update-tile-traffic <tile_offset>,<timestamp>,[<edge_index>,<overall_speed>,<speed1>,<speed2>,<speed3>,<breakpoint1>,<breakpoint2>,<congestion1>,<congestion2>,<congestion3>,<has_incidents>,...]",
            cxxopts::value<std::vector<std::string>>())
        ("ways-to-edges", "Creates a list of edges for each OSM way with some additional attributes")
        ("tile-offset-index", "Creates an index of tile name with their offset in traffic_extract file");

    // clang-format on

    auto result = options.parse(argc, argv);

    if (result.count("help")) {
      return handle_help(options);
    }


    if (result.count("ways-to-edges")) {
      return handle_ways_to_edges(config_file_path);
    }

    if (result.count("tile-offset-index")) {
      return handle_tile_offset_index(config_file_path);
    }

    std::cout << options.help() << std::endl;

  } catch (const std::exception& e) {
    std::cerr << "Unable to parse command line options because: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }
  return EXIT_FAILURE;
}
