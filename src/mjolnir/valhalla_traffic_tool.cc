
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
#include <filesystem>
#include <fstream>
#include <iostream>

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
        ("build-verification", "Creates a verification file for the traffic.tar file. To be used with verify")
        ("verify", "Verifies the traffic.tar file. To be used before replacing new traffic.tar file to ensure it has not been corrupted. Usage: --verify-path <verify_path> --traffic-path <traffic_path>")
        ("copy-traffic", "Copies traffic data from src to dest file. Usage: --input-traffic-path <input-traffic-path> --traffic-path <traffic_path>")
        ("tile-offset-index", "Creates an index of tile name with their offset in traffic_extract file")
        ("verify-path", "Path to the verification file.",
        cxxopts::value<std::string>())
        ("traffic-path", "Path to the traffic.tar file.",
        cxxopts::value<std::string>())
        ("input-traffic-path", "Path to input traffic.tar file.",
        cxxopts::value<std::string>());

    // clang-format on

    auto result = options.parse(argc, argv);

    if (result.count("help")) {
      return handle_help(options);
    }

    if (result.count("tile-offset-index")) {
      return handle_tile_offset_index(config_file_path);
    }

    if (result.count("build-verification")) {
      return handle_build_verification(config_file_path);
    }

    if (result.count("verify")) {
      if (!result.count("verify-path")) {
        std::cerr << "Please provide a path to verify with --verify-path" << std::endl;
        return EXIT_FAILURE;
      }
      if (!result.count("traffic-path")) {
        std::cerr << "Please provide a path to the traffic.tar file with --traffic-path" << std::endl;
        return EXIT_FAILURE;
      }
      return handle_verify(result["traffic-path"].as<std::string>(),
                           result["verify-path"].as<std::string>());
    }

    if (result.count("copy-traffic")) {
      if (!result.count("input-traffic-path")) {
        std::cerr << "Please provide a path to the input traffic.tar file with --input-traffic-path"
                  << std::endl;
        return EXIT_FAILURE;
      }
      if (!result.count("traffic-path")) {
        std::cerr << "Please provide a path to the traffic.tar file with --traffic-path" << std::endl;
        return EXIT_FAILURE;
      }
      return handle_copy_traffic(result["input-traffic-path"].as<std::string>(),
                                 result["traffic-path"].as<std::string>());
    }

    std::cout << options.help() << std::endl;

  } catch (const std::exception& e) {
    std::cerr << "Unable to parse command line options because: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }
  return EXIT_FAILURE;
}
