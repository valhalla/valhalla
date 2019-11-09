#include "mjolnir/util.h"

#include "baldr/tilehierarchy.h"
#include "filesystem.h"
#include "midgard/aabb2.h"
#include "midgard/logging.h"
#include "midgard/point2.h"
#include "midgard/polyline2.h"
#include "mjolnir/bssbuilder.h"
#include "mjolnir/elevationbuilder.h"
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

using namespace valhalla::midgard;

namespace {

// Temporary files used during tile building
const std::string ways_file = "ways.bin";
const std::string way_nodes_file = "way_nodes.bin";
const std::string nodes_file = "nodes.bin";
const std::string edges_file = "edges.bin";
const std::string access_file = "access.bin";
const std::string bss_nodes_file = "bss_nodes.bin";
const std::string cr_from_file = "complex_from_restrictions.bin";
const std::string cr_to_file = "complex_to_restrictions.bin";
const std::string new_to_old_file = "new_nodes_to_old_nodes.bin";
const std::string old_to_new_file = "old_nodes_to_new_nodes.bin";

} // namespace

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

bool build_tile_set(const boost::property_tree::ptree& config,
                    const std::vector<std::string>& input_files,
                    const BuildStage start_stage,
                    const BuildStage end_stage) {
  auto remove_temp_file = [](const std::string& fname) {
    if (boost::filesystem::exists(fname)) {
      boost::filesystem::remove(fname);
    }
  };

  // cannot allow this when building tiles
  if (config.get_child("mjolnir").get_optional<std::string>("tile_extract")) {
    throw std::runtime_error("Tiles cannot be directly built into a tar extract");
  }

  // Get the tile directory (make sure it ends with the preferred separator
  std::string tile_dir = config.get<std::string>("mjolnir.tile_dir");
  if (tile_dir.back() != filesystem::path::preferred_separator) {
    tile_dir.push_back(filesystem::path::preferred_separator);
  }

  // During the initialize stage the tile directory will be purged (if it already exists)
  // and will be created if it does not already exist
  if (start_stage == BuildStage::kInitialize) {
    // set up the directories and purge old tiles if starting at the parsing stage
    for (const auto& level : valhalla::baldr::TileHierarchy::levels()) {
      auto level_dir = tile_dir + std::to_string(level.first);
      if (boost::filesystem::exists(level_dir) && !boost::filesystem::is_empty(level_dir)) {
        LOG_WARN("Non-empty " + level_dir + " will be purged of tiles");
        boost::filesystem::remove_all(level_dir);
      }
    }

    // check for transit level.
    auto level_dir =
        tile_dir +
        std::to_string(valhalla::baldr::TileHierarchy::levels().rbegin()->second.level + 1);
    if (boost::filesystem::exists(level_dir) && !boost::filesystem::is_empty(level_dir)) {
      LOG_WARN("Non-empty " + level_dir + " will be purged of tiles");
      boost::filesystem::remove_all(level_dir);
    }

    // Create the directory if it does not exist
    boost::filesystem::create_directories(tile_dir);
  }

  // Set up the temporary (*.bin) files used during processing
  std::string ways_bin = tile_dir + ways_file;
  std::string way_nodes_bin = tile_dir + way_nodes_file;
  std::string nodes_bin = tile_dir + nodes_file;
  std::string edges_bin = tile_dir + edges_file;
  std::string access_bin = tile_dir + access_file;
  std::string bss_nodes_bin = tile_dir + bss_nodes_file;
  std::string cr_from_bin = tile_dir + cr_from_file;
  std::string cr_to_bin = tile_dir + cr_to_file;
  std::string new_to_old_bin = tile_dir + new_to_old_file;
  std::string old_to_new_bin = tile_dir + old_to_new_file;

  // OSMData class
  OSMData osm_data;

  // Parse OSM data
  if (start_stage <= BuildStage::kParse && BuildStage::kParse <= end_stage) {
    // Read the OSM protocol buffer file. Callbacks for nodes, ways, and
    // relations are defined within the PBFParser class
    osm_data =
        PBFGraphParser::Parse(config.get_child("mjolnir"), input_files, ways_bin, way_nodes_bin,
                              access_bin, cr_from_bin, cr_to_bin, bss_nodes_bin);

    // Free all protobuf memory - cannot use the protobuffer lib after this!
    OSMPBF::Parser::free();

    // Write the OSMData to files if parsing is the end stage
    if (end_stage <= BuildStage::kEnhance) {
      osm_data.write_to_temp_files(tile_dir);
    }
  }

  // Build Valhalla routing tiles
  if (start_stage <= BuildStage::kBuild && BuildStage::kBuild <= end_stage) {
    // Read OSMData from files if building tiles is the first stage
    if (start_stage == BuildStage::kBuild) {
      osm_data.read_from_temp_files(tile_dir);
    }

    // Build the graph using the OSMNodes and OSMWays from the parser
    GraphBuilder::Build(config, osm_data, ways_bin, way_nodes_bin, nodes_bin, edges_bin, cr_from_bin,
                        cr_to_bin);
  }

  // Enhance the local level of the graph. This adds information to the local
  // level that is usable across all levels (density, administrative
  // information (and country based attribution), edge transition logic, etc.
  if (start_stage <= BuildStage::kEnhance && BuildStage::kEnhance <= end_stage) {
    // Read OSMData names from file if building tiles is the first stage
    if (start_stage == BuildStage::kEnhance) {
      osm_data.read_from_unique_names_file(tile_dir);
    }
    GraphEnhancer::Enhance(config, osm_data, access_bin);
  }

  // Perform optional edge filtering (remove edges and nodes for specific access modes)
  if (start_stage <= BuildStage::kFilter && BuildStage::kFilter <= end_stage) {
    GraphFilter::Filter(config);
  }

  // Add transit
  if (start_stage <= BuildStage::kTransit && BuildStage::kTransit <= end_stage) {
    TransitBuilder::Build(config);
  }

  // Build bike share stations
  if (start_stage <= BuildStage::kBss && BuildStage::kBss <= end_stage) {
    BssBuilder::Build(config, bss_nodes_bin);
  }

  // Builds additional hierarchies if specified within config file. Connections
  // (directed edges) are formed between nodes at adjacent levels.
  auto build_hierarchy = config.get<bool>("mjolnir.hierarchy", true);
  if (build_hierarchy) {
    if (start_stage <= BuildStage::kHierarchy && BuildStage::kHierarchy <= end_stage) {
      HierarchyBuilder::Build(config, new_to_old_bin, old_to_new_bin);
    }

    // Build shortcuts if specified in the config file. Shortcuts can only be
    // applied if hierarchies are also generated.
    auto build_shortcuts = config.get<bool>("mjolnir.shortcuts", true);
    if (build_shortcuts) {
      if (start_stage <= BuildStage::kShortcuts && BuildStage::kShortcuts <= end_stage) {
        ShortcutBuilder::Build(config);
      }
    } else {
      LOG_INFO("Skipping shortcut builder");
    }
  } else {
    LOG_INFO("Skipping hierarchy builder and shortcut builder");
  }

  // Add elevation to the tiles
  if (start_stage <= BuildStage::kElevation && BuildStage::kElevation <= end_stage) {
    ElevationBuilder::Build(config);
  }

  // Build the Complex Restrictions
  // ComplexRestrictions must be done after elevation. The reason is that building
  // elevation into the tiles reads each tile and serializes the data to "builders"
  // within the tile. However, there is no serialization currently available for complex restrictions.
  if (start_stage <= BuildStage::kRestrictions && BuildStage::kRestrictions <= end_stage) {
    RestrictionBuilder::Build(config, cr_from_bin, cr_to_bin);
  }

  // Validate the graph and add information that cannot be added until full graph is formed.
  if (start_stage <= BuildStage::kValidate && BuildStage::kValidate <= end_stage) {
    GraphValidator::Validate(config);
  }

  // Cleanup bin files
  if (start_stage <= BuildStage::kCleanup && BuildStage::kCleanup <= end_stage) {
    LOG_INFO("Cleaning up temporary *.bin files within " + tile_dir);
    remove_temp_file(ways_bin);
    remove_temp_file(way_nodes_bin);
    remove_temp_file(nodes_bin);
    remove_temp_file(edges_bin);
    remove_temp_file(access_bin);
    remove_temp_file(bss_nodes_bin);
    remove_temp_file(cr_from_bin);
    remove_temp_file(cr_to_bin);
    remove_temp_file(new_to_old_bin);
    remove_temp_file(old_to_new_bin);
    OSMData::cleanup_temp_files(tile_dir);
  }
  return true;
}

} // namespace mjolnir
} // namespace valhalla
