#include <cstdlib>
#include <deque>
#include <iostream>
#include <vector>

#include <boost/property_tree/ptree.hpp>
#include <cxxopts.hpp>

#include "baldr/graphreader.h"
#include "baldr/graphtile.h"
#include "baldr/rapidjson_utils.h"
#include "mjolnir/elevationbuilder.h"
#include "mjolnir/valhalla_add_elevation_utils.h"

namespace opt = cxxopts;

using namespace valhalla::baldr;
using namespace valhalla::midgard;
using namespace valhalla::mjolnir;

enum Input { CONFIG, TILES };

/*
 * This tool downloads elevations from remote storage for each provided tile.
 * First it checks if the elevation tiles are available locally and
 * in case they are not it tries to download them from a remote storage.
 * Remote storage address should be given in a configuration file.
 * Service's input parameters
 * - config - stays for path to configuration file
 * - tiles - stays for the path to a tile file.
 * */

std::pair<std::string, std::vector<std::string>> parse_arguments(int argc, char** argv) {
  opt::Options options(
      "help",
      " Usage: valhalla_add_elevation [options]\n"
      "valhalla_add_elevation is a tool for loading elevations for a provided tile. "
      "The service checks if required elevations stored locally if they are not "
      "it tries to establish connection to the remote storage(based on the information from configuration file)"
      "and loads required elevations.\n");

  options.add_options()("h,help",
                        "Print usage")("c,config", "Path to the configuration file.",
                                       opt::value<
                                           std::string>())("t,tiles", "Tiles to add elevations to",
                                                           opt::value<std::vector<std::string>>());

  options.allow_unrecognised_options();
  std::string config_file;
  std::vector<std::string> tiles;
  try {
    auto result = options.parse(argc, argv);

    if (result.count("help")) {
      std::cout << options.help() << "\n";
      return {};
    }

    if (!result.count("config")) {
      std::cerr << "No configuration file provided"
                << "\n\n";
    } else {
      config_file = result["config"].as<std::string>();
    }

    if (!result.count("tiles")) {
      std::cerr << "No tile file provided"
                << "\n\n";
    } else {
      tiles = result["tiles"].as<std::vector<std::string>>();
    }
  } catch (std::exception& e) {
    std::cerr << "Unable to parse command line options. Error: " << e.what() << "\n";
    return {};
  }

  return {config_file, tiles};
}

std::unordered_set<std::string> get_valid_tile_paths(std::vector<std::string>&& tiles) {
  std::unordered_set<std::string> st;
  for (const auto& tile : tiles) {
    if (filesystem::exists(tile) && filesystem::is_regular_file(tile))
      st.insert(tile);
  }

  return st;
}

int main(int argc, char** argv) {
  auto params = parse_arguments(argc, argv);
  if (std::get<Input::CONFIG>(params).empty() && std::get<Input::TILES>(params).empty()) {
    return EXIT_SUCCESS;
  }

  if (std::get<Input::CONFIG>(params).empty() || std::get<Input::TILES>(params).empty()) {
    std::cerr << "Invalid input: " << (std::get<Input::CONFIG>(params).empty() ? "config" : "tile")
              << " was not provided\n\n";
    return EXIT_FAILURE;
  }

  if (!filesystem::exists(std::get<Input::CONFIG>(params)) ||
      !filesystem::is_regular_file(std::get<Input::CONFIG>(params))) {
    std::cerr << "Fail to parse configuration file\n\n";
    return EXIT_FAILURE;
  }

  auto tiles = get_valid_tile_paths(std::move(std::get<Input::TILES>(params)));
  if (tiles.empty()) {
    std::cerr << "All tile files are invalid\n\n";
    return EXIT_FAILURE;
  }

  boost::property_tree::ptree pt;
  rapidjson::read_json(std::get<Input::CONFIG>(params), pt);
  auto tile_ids = valhalla::mjolnir::get_tile_ids(pt, tiles);
  if (tile_ids.empty()) {
    std::cerr << "Failed to load tiles\n\n";
    return EXIT_FAILURE;
  }

  ElevationBuilder::Build(pt, tile_ids);
  return EXIT_SUCCESS;
}