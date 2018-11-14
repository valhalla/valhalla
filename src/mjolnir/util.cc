#include "mjolnir/util.h"

#include "baldr/tilehierarchy.h"
#include "filesystem.h"
#include "midgard/aabb2.h"
#include "midgard/logging.h"
#include "midgard/point2.h"
#include "midgard/polyline2.h"
#include "mjolnir/graphbuilder.h"
#include "mjolnir/graphenhancer.h"
#include "mjolnir/graphfilter.h"
#include "mjolnir/graphvalidator.h"
#include "mjolnir/hierarchybuilder.h"
#include "mjolnir/osmpbfparser.h"
#include "mjolnir/pbfgraphparser.h"
#include "mjolnir/restrictionbuilder.h"
#include "mjolnir/shortcutbuilder.h"
#include "mjolnir/transitbuilder.h"

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/property_tree/ptree.hpp>

namespace valhalla {
namespace mjolnir {

/**
 * Splits a tag into a vector of strings.  Delim defaults to ;
 */
std::vector<std::string> GetTagTokens(const std::string& tag_value, char delim) {
  std::vector<std::string> tokens;
  boost::algorithm::split(tokens, tag_value, std::bind1st(std::equal_to<char>(), delim),
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

// Compute a curvature metric given an edge shape
uint32_t compute_curvature(const std::list<PointLL>& shape) {
  // Edges with just 2 shape points have no curvature.
  // TODO - perhaps a post-process to "average" curvature along adjacent edges
  // and smooth curvature on connected edges may be desirable?
  if (shape.size() == 2) {
    return 0;
  }

  // Iterate through sets of shape vertices and compute a radius of curvature.
  // Apply a score to each section.
  uint32_t n = 0;
  float total_score = 0.0f;
  auto p1 = shape.begin();
  auto p2 = p1;
  p2++;
  auto p3 = p2;
  p3++;
  for (; p3 != shape.end(); ++p1, ++p2, ++p3) {
    float radius = p1->Curvature(*p2, *p3);
    if (!std::isnan(radius)) {
      // Compute a score and cap it at 25 (that way one sharp turn doesn't
      // impact the total edge more than it should)
      float score = (radius > 1000.0f) ? 0.0f : 1500.0f / radius;
      total_score += (score > 25.0f) ? 25.0f : score;
      n++;
    }
  }
  float average_score = (n == 0) ? 0.0f : total_score / n;
  return average_score > 15.0f ? 15 : static_cast<uint32_t>(average_score);
}

// Do the 2 shape vectors match (either direction).
bool shapes_match(const std::vector<PointLL>& shape1, const std::vector<PointLL>& shape2) {
  if (shape1.size() != shape2.size()) {
    return false;
  }

  if (shape1.front() == shape2.front()) {
    // Compare shape in forward direction
    auto iter1 = shape1.begin();
    auto iter2 = shape2.begin();
    for (; iter2 != shape2.end(); iter2++, iter1++) {
      if (*iter1 != *iter2) {
        return false;
      }
    }
    return true;
  } else if (shape1.front() == shape2.back()) {
    // Compare shape (reverse direction for shape2)
    auto iter1 = shape1.begin();
    auto iter2 = shape2.rbegin();
    for (; iter2 != shape2.rend(); iter2++, iter1++) {
      if (*iter1 != *iter2) {
        return false;
      }
    }
    return true;
  } else {
    LOG_WARN("Neither end of the shape matches");
    return false;
  }
}

void build_tile_set(const boost::property_tree::ptree& config,
                    const std::vector<std::string>& input_files,
                    const std::string& bin_file_prefix,
                    bool free_protobuf) {
  // cannot allow this when building tiles
  if (config.get_child("mjolnir").get_optional<std::string>("tile_extract")) {
    throw std::runtime_error("Tiles cannot be directly built into a tar extract");
  }

  // set up the directories and purge old tiles
  auto tile_dir = config.get<std::string>("mjolnir.tile_dir");
  for (const auto& level : valhalla::baldr::TileHierarchy::levels()) {
    auto level_dir = tile_dir + filesystem::path::preferred_separator + std::to_string(level.first);
    if (boost::filesystem::exists(level_dir) && !boost::filesystem::is_empty(level_dir)) {
      LOG_WARN("Non-empty " + level_dir + " will be purged of tiles");
      boost::filesystem::remove_all(level_dir);
    }
  }

  // check for transit level.
  auto level_dir =
      tile_dir + filesystem::path::preferred_separator +
      std::to_string(valhalla::baldr::TileHierarchy::levels().rbegin()->second.level + 1);
  if (boost::filesystem::exists(level_dir) && !boost::filesystem::is_empty(level_dir)) {
    LOG_WARN("Non-empty " + level_dir + " will be purged of tiles");
    boost::filesystem::remove_all(level_dir);
  }

  boost::filesystem::create_directories(tile_dir);

  // Read the OSM protocol buffer file. Callbacks for nodes, ways, and
  // relations are defined within the PBFParser class
  auto osm_data =
      PBFGraphParser::Parse(config.get_child("mjolnir"), input_files, bin_file_prefix + "ways.bin",
                            bin_file_prefix + "way_nodes.bin", bin_file_prefix + "access.bin",
                            bin_file_prefix + "complex_from_restrictions.bin",
                            bin_file_prefix + "complex_to_restrictions.bin");

  // Optionally free all protobuf memory but also you cant use the protobuffer lib after this!
  if (free_protobuf) {
    OSMPBF::Parser::free();
  }

  // Build the graph using the OSMNodes and OSMWays from the parser
  GraphBuilder::Build(config, osm_data, bin_file_prefix + "ways.bin",
                      bin_file_prefix + "way_nodes.bin",
                      bin_file_prefix + "complex_from_restrictions.bin",
                      bin_file_prefix + "complex_to_restrictions.bin");

  // Enhance the local level of the graph. This adds information to the local
  // level that is usable across all levels (density, administrative
  // information (and country based attribution), edge transition logic, etc.
  GraphEnhancer::Enhance(config, bin_file_prefix + "access.bin");

  // Perform optional edge filtering (remove edges and nodes for specific access modes)
  GraphFilter::Filter(config);

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
  RestrictionBuilder::Build(config, bin_file_prefix + "complex_from_restrictions.bin",
                            "complex_to_restrictions.bin");

  // Validate the graph and add information that cannot be added until
  // full graph is formed.
  GraphValidator::Validate(config);
}

} // namespace mjolnir
} // namespace valhalla
