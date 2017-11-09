#include "mjolnir/util.h"

#include "mjolnir/graphvalidator.h"
#include "mjolnir/pbfgraphparser.h"
#include "mjolnir/osmpbfparser.h"
#include "mjolnir/graphbuilder.h"
#include "mjolnir/transitbuilder.h"
#include "mjolnir/graphenhancer.h"
#include "mjolnir/hierarchybuilder.h"
#include "mjolnir/shortcutbuilder.h"
#include "mjolnir/restrictionbuilder.h"
#include "midgard/point2.h"
#include "midgard/aabb2.h"
#include "midgard/polyline2.h"
#include "midgard/logging.h"
#include "baldr/tilehierarchy.h"

#include <boost/filesystem/operations.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

namespace valhalla {
namespace mjolnir {

/**
 * Splits a tag into a vector of strings.  Delim defaults to ;
 */
std::vector<std::string> GetTagTokens(const std::string& tag_value,
                                      char delim) {
  std::vector<std::string> tokens;
  boost::algorithm::split(tokens, tag_value,
                          std::bind1st(std::equal_to<char>(), delim),
                          boost::algorithm::token_compress_on);
  return tokens;
}

// remove double quotes.
std::string remove_double_quotes(const std::string& s) {
  std::string ret;
  for (auto c : s) {
    if (c != '"') {
      ret += c;
    }
  }
  return ret;
}

void build_tile_set(const boost::property_tree::ptree& config, const std::vector<std::string>& input_files, const std::string& bin_file_prefix, bool free_protobuf) {
  //cannot allow this when building tiles
  if(config.get_child("mjolnir").get_optional<std::string>("tile_extract"))
    throw std::runtime_error("Tiles cannot be directly built into a tar extract");

  //set up the directories and purge old tiles
  auto tile_dir = config.get<std::string>("mjolnir.tile_dir");
  for(const auto& level : valhalla::baldr::TileHierarchy::levels()) {
    auto level_dir = tile_dir + "/" + std::to_string(level.first);
    if(boost::filesystem::exists(level_dir) && !boost::filesystem::is_empty(level_dir)) {
      LOG_WARN("Non-empty " + level_dir + " will be purged of tiles");
      boost::filesystem::remove_all(level_dir);
    }
  }

  //check for transit level.
  auto level_dir = tile_dir + "/" +
      std::to_string(valhalla::baldr::TileHierarchy::levels().rbegin()->second.level+1);
  if(boost::filesystem::exists(level_dir) && !boost::filesystem::is_empty(level_dir)) {
    LOG_WARN("Non-empty " + level_dir + " will be purged of tiles");
    boost::filesystem::remove_all(level_dir);
  }

  boost::filesystem::create_directories(tile_dir);

  // Read the OSM protocol buffer file. Callbacks for nodes, ways, and
  // relations are defined within the PBFParser class
  auto osm_data = PBFGraphParser::Parse(config.get_child("mjolnir"), input_files, bin_file_prefix + "ways.bin",
      bin_file_prefix + "way_nodes.bin", bin_file_prefix + "access.bin", bin_file_prefix + "complex_restrictions.bin");

  // Optionally free all protobuf memory but also you cant use the protobuffer lib after this!
  if(free_protobuf)
    OSMPBF::Parser::free();

  // Build the graph using the OSMNodes and OSMWays from the parser
  GraphBuilder::Build(config, osm_data, bin_file_prefix + "ways.bin", bin_file_prefix + "way_nodes.bin",
      bin_file_prefix + "complex_restrictions.bin");

  // Enhance the local level of the graph. This adds information to the local
  // level that is usable across all levels (density, administrative
  // information (and country based attribution), edge transition logic, etc.
  GraphEnhancer::Enhance(config, bin_file_prefix + "access.bin");

  // Add transit
  TransitBuilder::Build(config);

  // Builds additional hierarchies if specified within config file. Connections
  // (directed edges) are formed between nodes at adjacent levels.
  auto build_hierarchy = config.get<bool>("mjolnir.hierarchy", true);
  if (build_hierarchy) {
    HierarchyBuilder::Build(config);

    // Build shortcuts if specified in the config file. Shortcuts can only be
    // applied if hierarchies are also generated.
    auto build_shortcuts = config.get<bool>("mjolnir.shortcuts", true);
    if (build_shortcuts) {
      ShortcutBuilder::Build(config);
    } else {
      LOG_INFO("Skipping shortcut builder");
    }
  } else {
    LOG_INFO("Skipping hierarchy builder and shortcut builder");
  }

  // Build the Complex Restrictions
  RestrictionBuilder::Build(config, bin_file_prefix + "complex_restrictions.bin", osm_data.end_map);

  // Validate the graph and add information that cannot be added until
  // full graph is formed.
  GraphValidator::Validate(config);

}

}
}
