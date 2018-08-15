#include <string>
#include <vector>

#include "config.h"
#include "mjolnir/util.h"

using namespace valhalla::mjolnir;

#include "baldr/rapidjson_utils.h"
#include <boost/filesystem/operations.hpp>
#include <boost/optional.hpp>
#include <boost/program_options.hpp>
#include <boost/property_tree/ptree.hpp>
#include <iostream>

#include "midgard/logging.h"
#include "midgard/util.h"

namespace bpo = boost::program_options;

int main(int argc, char** argv) {
  // Program options
  boost::filesystem::path config_file_path;
  std::string inline_config;
  std::vector<std::string> input_files;
  bpo::options_description options(
      "valhalla_build_tiles " VALHALLA_VERSION "\n\n"
      "Usage: valhalla_build_tiles [options] <protocolbuffer_input_file>\n\n"
      "valhalla_build_tiles is a program that creates the route graph from an osm.pbf "
      "extract. Sample json configs are located in ../conf directory.\n\n");

  options.add_options()("help,h", "Print this help message.")("version,v",
                                                              "Print the version of this software.")(
      "config,c", boost::program_options::value<boost::filesystem::path>(&config_file_path),
      "Path to the json configuration file.")("inline-config,i",
                                              boost::program_options::value<std::string>(
                                                  &inline_config),
                                              "Inline json config.")
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
    std::cerr << "Unable to parse command line options because: " << e.what() << "\n";
    return EXIT_FAILURE;
  }

  // Print out help or version and return
  if (vm.count("help")) {
    std::cout << options << "\n";
    return EXIT_SUCCESS;
  }
  if (vm.count("version")) {
    std::cout << "valhalla_build_tiles " << VALHALLA_VERSION << "\n";
    return EXIT_SUCCESS;
  }
  if (input_files.size() == 0) {
    std::cerr << "Input file is required\n\n" << options << "\n\n";
    return EXIT_FAILURE;
  }

  // Read the config file
  boost::property_tree::ptree pt;
  if (vm.count("inline-config")) {
    std::stringstream ss;
    ss << inline_config;
    rapidjson::read_json(ss, pt);
  } else if (vm.count("config") && boost::filesystem::is_regular_file(config_file_path)) {
    rapidjson::read_json(config_file_path.string(), pt);
  } else {
    std::cerr << "Configuration is required\n\n" << options << "\n\n";
    return EXIT_FAILURE;
  }

  // configure logging
  boost::optional<boost::property_tree::ptree&> logging_subtree =
      pt.get_child_optional("mjolnir.logging");
  if (logging_subtree) {
    auto logging_config =
        valhalla::midgard::ToMap<const boost::property_tree::ptree&,
                                 std::unordered_map<std::string, std::string>>(logging_subtree.get());
    valhalla::midgard::logging::Configure(logging_config);
  }

  // build some tiles
  pt.get_child("mjolnir").erase("tile_extract");
  pt.get_child("mjolnir").erase("tile_url");
  build_tile_set(pt, input_files);

  return EXIT_SUCCESS;
}
