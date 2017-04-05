#include <string>
#include <vector>

#include "mjolnir/graphvalidator.h"
#include "mjolnir/pbfgraphparser.h"
#include "mjolnir/graphbuilder.h"
#include "mjolnir/transitbuilder.h"
#include "mjolnir/graphenhancer.h"
#include "mjolnir/hierarchybuilder.h"
#include "mjolnir/shortcutbuilder.h"
#include "mjolnir/restrictionbuilder.h"
#include "baldr/tilefshierarchy.h"
#include "config.h"

using namespace valhalla::mjolnir;

#include <ostream>
#include <boost/program_options.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/optional.hpp>

#include "midgard/point2.h"
#include "midgard/aabb2.h"
#include "midgard/polyline2.h"
#include "midgard/logging.h"

namespace bpo = boost::program_options;

int main(int argc, char** argv) {
  // Program options
  boost::filesystem::path config_file_path;
  std::vector<std::string> input_files;
  bpo::options_description options(
    "valhalla_build_tiles " VERSION "\n\n"
    "Usage: valhalla_build_tiles [options] <protocolbuffer_input_file>\n\n"
    "valhalla_build_tiles is a program that creates the route graph from an osm.pbf "
    "extract. Sample json configs are located in ../conf directory.\n\n");

  options.add_options()
      ("help,h", "Print this help message.")
      ("version,v", "Print the version of this software.")
      ("config,c",
        boost::program_options::value<boost::filesystem::path>(&config_file_path),
        "Path to the json configuration file.")
      // positional arguments
      ("input_files", boost::program_options::value<std::vector<std::string> >(&input_files)->multitoken());

  bpo::positional_options_description pos_options;
  pos_options.add("input_files", 16);
  bpo::variables_map vm;
  try {
    bpo::store(bpo::command_line_parser(argc, argv).options(options).positional(pos_options).run(), vm);
    bpo::notify(vm);

  } catch (std::exception &e) {
    std::cerr << "Unable to parse command line options because: " << e.what() << "\n";
    return EXIT_FAILURE;
  }

  // Print out help or version and return
  if (vm.count("help")) {
    std::cout << options << "\n";
    return EXIT_SUCCESS;
  }
  if (vm.count("version")) {
    std::cout << "valhalla_build_tiles " << VERSION << "\n";
    return EXIT_SUCCESS;
  }

  // Exit if no config file or no input pbf files
  if (!vm.count("config") || !boost::filesystem::is_regular_file(config_file_path)) {
    std::cerr << "Configuration file is required\n\n" << options << "\n\n";
    return EXIT_FAILURE;
  }
  if (input_files.size() == 0) {
    std::cerr << "Input file is required\n\n" << options << "\n\n";
    return EXIT_FAILURE;
  }

  // Read the config file
  boost::property_tree::ptree pt;
  boost::property_tree::read_json(config_file_path.c_str(), pt);

  //configure logging
  boost::optional<boost::property_tree::ptree&> logging_subtree = pt.get_child_optional("mjolnir.logging");
  if(logging_subtree) {
    auto logging_config = valhalla::midgard::ToMap<const boost::property_tree::ptree&, std::unordered_map<std::string, std::string> >(logging_subtree.get());
    valhalla::midgard::logging::Configure(logging_config);
  }

  //set up the directories and purge old tiles
  pt.get_child("mjolnir").erase("tile_extract");
  auto tile_dir = pt.get<std::string>("mjolnir.tile_dir");
  valhalla::baldr::TileFsHierarchy hierarchy(tile_dir);
  for(const auto& level : hierarchy.levels()) {
    auto level_dir = tile_dir + "/" + std::to_string(level.first);
    if(boost::filesystem::exists(level_dir) && !boost::filesystem::is_empty(level_dir)) {
      LOG_WARN("Non-empty " + level_dir + " will be purged of tiles");
      boost::filesystem::remove_all(level_dir);
    }
  }

  //check for transit level.
  auto level_dir = tile_dir + "/" + std::to_string(hierarchy.levels().rbegin()->second.level+1);
  if(boost::filesystem::exists(level_dir) && !boost::filesystem::is_empty(level_dir)) {
    LOG_WARN("Non-empty " + level_dir + " will be purged of tiles");
    boost::filesystem::remove_all(level_dir);
  }

  boost::filesystem::create_directories(tile_dir);

  // Read the OSM protocol buffer file. Callbacks for nodes, ways, and
  // relations are defined within the PBFParser class
  auto osm_data = PBFGraphParser::Parse(pt.get_child("mjolnir"), input_files, "ways.bin",
                                        "way_nodes.bin", "access.bin", "complex_restrictions.bin");

  // Build the graph using the OSMNodes and OSMWays from the parser
  GraphBuilder::Build(pt, osm_data, "ways.bin", "way_nodes.bin", "complex_restrictions.bin");

  // Enhance the local level of the graph. This adds information to the local
  // level that is usable across all levels (density, administrative
  // information (and country based attribution), edge transition logic, etc.
  GraphEnhancer::Enhance(pt, "access.bin");

  // Add transit
  TransitBuilder::Build(pt);

  // Builds additional hierarchies based on the config file. Connections
  // (directed edges) are formed between nodes at adjacent levels.
  HierarchyBuilder::Build(pt);

  // Build shortcuts
  ShortcutBuilder::Build(pt);

  // Build the Complex Restrictions
  RestrictionBuilder::Build(pt,"complex_restrictions.bin", osm_data.end_map);

  // Validate the graph and add information that cannot be added until
  // full graph is formed.
  GraphValidator::Validate(pt);

  return EXIT_SUCCESS;
}

