#include <cstdlib>
#include <deque>
#include <iostream>
#include <vector>

#include <boost/program_options.hpp>
#include <boost/property_tree/ptree.hpp>

#include "baldr/graphreader.h"
#include "baldr/graphtile.h"
#include "baldr/rapidjson_utils.h"
#include "config.h"
#include "mjolnir/elevationbuilder.h"
#include "mjolnir/valhalla_add_elevation_utils.h"

namespace bpo = boost::program_options;

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
  std::string config_path;
  std::vector<std::string> tiles;
  bpo::options_description options(
      " Usage: valhalla_add_elevation [options]\n"
      "valhalla_add_elevation is a tool for loading elevations for a provided tile. "
      "The service checks if required elevations stored locally if they are not "
      "it tries to establish connection to the remote storage(based on the information from configuration file)"
      "and loads required elevations.\n");

  options.add_options()("help,h", "Print this help message.")("version,v",
                                                              "Print the version of this software.")(
      "config,c", boost::program_options::value<std::string>(&config_path),
      "Path to the json configuration file.")("tiles,t",
                                              boost::program_options::value<std::vector<std::string>>(
                                                  &tiles)
                                                  ->multitoken());

  bpo::positional_options_description pos_options;
  pos_options.add("tiles", 16);
  bpo::variables_map vm;
  try {
    bpo::store(bpo::command_line_parser(argc, argv).options(options).positional(pos_options).run(),
               vm);
    bpo::notify(vm);

  } catch (std::exception& e) {
    std::cerr << "Unable to parse command line options because: " << e.what() << "\n";
    return {};
  }

  if (vm.count("help")) {
    std::cout << options << "\n";
    return {};
  }
  if (vm.count("version")) {
    std::cout << "valhalla_build_tiles " << VALHALLA_VERSION << "\n";
    return {};
  }

  if (!vm.count("config")) {
    std::cerr << "Configuration file is required\n\n" << options << "\n\n";
    return {};
  }

  if (tiles.empty()) {
    std::cerr << "Tile file is required\n\n" << options << "\n\n";
    return {};
  }

  return {config_path, tiles};
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
  if (std::get<Input::CONFIG>(params).empty() || std::get<Input::TILES>(params).empty()) {
    std::cerr << "Invalid input " << (std::get<Input::CONFIG>(params).empty() ? "config" : "tile")
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
  ElevationBuilder::Build(pt, valhalla::mjolnir::get_tile_ids(pt, tiles));

  return EXIT_SUCCESS;
}