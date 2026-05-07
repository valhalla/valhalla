#ifndef VALHALLA_MJOLNIR_UTIL_H_
#define VALHALLA_MJOLNIR_UTIL_H_

#include <valhalla/baldr/directededge.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphtileptr.h>
#include <valhalla/baldr/nodeinfo.h>
#include <valhalla/midgard/pointll.h>

#include <boost/property_tree/ptree_fwd.hpp>

#include <array>
#include <atomic>
#include <map>
#include <mutex>
#include <span>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace valhalla {
namespace mjolnir {

// Stages of the Valhalla tile building pipeline
enum class BuildStage : int8_t {
  kInvalid = -1,
  kInitialize = 0,
  kParseWays = 1,
  kParseRelations = 2,
  kParseNodes = 3,
  kConstructEdges = 4,
  kBuild = 5,
  kEnhance = 6,
  kFilter = 7,
  kTransit = 8,
  kBss = 9,
  kHierarchy = 10,
  kShortcuts = 11,
  kRestrictions = 12,
  kElevation = 13,
  kValidate = 14,
  kCleanup = 15
};

constexpr uint8_t kMinor = 1;
constexpr uint8_t kStopSign = 2;
constexpr uint8_t kYieldSign = 4;

// Convert string to BuildStage
inline BuildStage string_to_buildstage(const std::string& s) {
  static const std::unordered_map<std::string, BuildStage> stringToBuildStage =
      {{"initialize", BuildStage::kInitialize},
       {"parseways", BuildStage::kParseWays},
       {"parserelations", BuildStage::kParseRelations},
       {"parsenodes", BuildStage::kParseNodes},
       {"constructedges", BuildStage::kConstructEdges},
       {"build", BuildStage::kBuild},
       {"enhance", BuildStage::kEnhance},
       {"filter", BuildStage::kFilter},
       {"transit", BuildStage::kTransit},
       {"bss", BuildStage::kBss},
       {"hierarchy", BuildStage::kHierarchy},
       {"shortcuts", BuildStage::kShortcuts},
       {"restrictions", BuildStage::kRestrictions},
       {"elevation", BuildStage::kElevation},
       {"validate", BuildStage::kValidate},
       {"cleanup", BuildStage::kCleanup}};

  auto i = stringToBuildStage.find(s);
  return (i == stringToBuildStage.cend()) ? BuildStage::kInvalid : i->second;
}

// Convert BuildStage to string
inline std::string to_string(BuildStage stg) {
  static const std::unordered_map<uint8_t, std::string> BuildStageStrings =
      {{static_cast<int8_t>(BuildStage::kInitialize), "initialize"},
       {static_cast<int8_t>(BuildStage::kParseWays), "parseways"},
       {static_cast<int8_t>(BuildStage::kParseRelations), "parserelations"},
       {static_cast<int8_t>(BuildStage::kParseNodes), "parsenodes"},
       {static_cast<int8_t>(BuildStage::kConstructEdges), "constructedges"},
       {static_cast<int8_t>(BuildStage::kBuild), "build"},
       {static_cast<int8_t>(BuildStage::kEnhance), "enhance"},
       {static_cast<int8_t>(BuildStage::kFilter), "filter"},
       {static_cast<int8_t>(BuildStage::kTransit), "transit"},
       {static_cast<int8_t>(BuildStage::kBss), "bss"},
       {static_cast<int8_t>(BuildStage::kHierarchy), "hierarchy"},
       {static_cast<int8_t>(BuildStage::kShortcuts), "shortcuts"},
       {static_cast<int8_t>(BuildStage::kRestrictions), "restrictions"},
       {static_cast<int8_t>(BuildStage::kElevation), "elevation"},
       {static_cast<int8_t>(BuildStage::kValidate), "validate"},
       {static_cast<int8_t>(BuildStage::kCleanup), "cleanup"}};

  auto i = BuildStageStrings.find(static_cast<int8_t>(stg));
  return (i == BuildStageStrings.cend()) ? "null" : i->second;
}

// Counters for warnings that fire per-item in hot loops during tile building.
// Instead of logging each occurrence (which produces millions of lines on planet builds),
// we accumulate counts and log a summary at the end. Full per-item detail is still
// available at LOG_DEBUG level. Counters are atomic for thread safety.
//
// To add a new counter: add an enum value before kCount and a corresponding entry in
// build_stats::meta (same order). The static_assert in util.cc will catch mismatches.
struct build_stats {
  enum counter : uint8_t {
    kExceededElevationDiff,
    kExceededMaxAssignerSpeed,
    kExceededMaxDensity,
    kExceededMaxLocalEdgeCount,
    kExceededMaxNames,
    kExceededMaxNodesPerWay,
    kExceededMaxOSMSpeed,
    kExceededMaxOSMSpeedLimit,
    kExceededMaxOSMTruckSpeed,
    kExceededMaxShapeSize,
    kExceededMaxShortcutEdges,
    kExceededMaxVias,
    kExceededTurnRestrictionMask,
    kFailedFerryReclassBoth,
    kFailedFerryReclassInbound,
    kFailedFerryReclassOutbound,
    kFailedLaneConnectivity,
    kFailedNodeInitialization,
    kFailedOSMTimeRange,
    kFailedOSMTimeRangeUnknown,
    kInvalidHovType,
    kInvalidLevel,
    kInvalidOSMTag,
    kMissingAccessTags,
    // Graph summary counters (not warnings — totals for the built graph)
    kCountComplexTurnRestrictions,
    kCountEdges,
    kCountNodes,
    kCountShortcutEdgesLevel0,
    kCountShortcutEdgesLevel1,
    kCountShortcutsLevel0,
    kCountShortcutsLevel1,
    kCountSimpleTurnRestrictions,
    kCountTiles,
    kCount // sentinel — must be last
  };

  struct meta_entry {
    const char* statsd_key;
    const char* log_label;
    BuildStage stage; // the stage that owns this counter (for statsd emission)
    bool is_warning;  // true = LOG_WARN, false = LOG_INFO
  };
  static constexpr meta_entry meta[] = {
      // Warning counters (alphabetical)
      {"exceeded_elevation_diff", "edges with elevation exceeding max difference",
       BuildStage::kElevation, true},
      {"exceeded_max_assigner_speed", "SpeedAssigner edges clamped to max", BuildStage::kEnhance,
       true},
      {"exceeded_max_density", "nodes exceeding max density", BuildStage::kEnhance, true},
      {"exceeded_max_local_edge_count", "nodes exceeding max local edge count", BuildStage::kEnhance,
       true},
      {"exceeded_max_names", "edges exceeding max names", BuildStage::kBuild, true},
      {"exceeded_max_nodes_per_way", "ways exceeding max nodes per way", BuildStage::kParseWays,
       true},
      {"exceeded_max_osm_speed", "ways with speed clamped to max", BuildStage::kParseWays, true},
      {"exceeded_max_osm_speed_limit", "ways with speed limit clamped to max", BuildStage::kParseWays,
       true},
      {"exceeded_max_osm_truck_speed", "ways with truck speed clamped to max", BuildStage::kParseWays,
       true},
      {"exceeded_max_shape_size", "edges exceeding max encoded shape size", BuildStage::kBuild, true},
      {"exceeded_max_shortcut_edges", "nodes exceeding max shortcut edges", BuildStage::kShortcuts,
       true},
      {"exceeded_max_vias", "restrictions exceeding max vias", BuildStage::kRestrictions, true},
      {"exceeded_turn_restriction_mask", "simple turn restriction masks exceeding limit",
       BuildStage::kBuild, true},
      {"failed_ferry_reclass_both",
       "ferry connections completely failing to reclassify edges to the next level 0 edge",
       BuildStage::kBuild, true},
      {"failed_ferry_reclass_inbound",
       "inbound ferry connections failing to reclassify edges to the next level 0 edge",
       BuildStage::kBuild, true},
      {"failed_ferry_reclass_outbound",
       "outbound ferry connections failing to reclassify edges to the next level 0 edge",
       BuildStage::kBuild, true},
      {"failed_lane_connectivity", "lane connectivity import failures", BuildStage::kBuild, true},
      {"failed_node_initialization", "nodes with uninitialized coordinates",
       BuildStage::kConstructEdges, true},
      {"failed_osm_time_range", "OSM time range raises either invalid_argument or out_of_range",
       BuildStage::kParseWays, true},
      {"failed_osm_time_range_unknown", "OSM time range causes an unknown runtime_error",
       BuildStage::kParseWays, true},
      {"invalid_hov_type", "ways with invalid HOV type", BuildStage::kParseWays, true},
      {"invalid_level", "ways with invalid level tags", BuildStage::kParseWays, true},
      {"invalid_osm_tag", "invalid OSM tag parse errors", BuildStage::kParseWays, true},
      {"missing_access_tags", "edges with missing access tags", BuildStage::kEnhance, true},
      // Graph summary counters (alphabetical)
      {"count_complex_turn_restrictions", "complex turn restrictions", BuildStage::kRestrictions,
       false},
      {"count_edges", "amount of edges at Validate", BuildStage::kValidate, false},
      {"count_nodes", "amount of nodes at Validate", BuildStage::kValidate, false},
      {"count_shortcut_edges_level_0", "level 0 edges in shortcuts", BuildStage::kShortcuts, false},
      {"count_shortcut_edges_level_1", "level 1 edges in shortcuts", BuildStage::kShortcuts, false},
      {"count_shortcuts_level_0", "level 0 shortcuts", BuildStage::kShortcuts, false},
      {"count_shortcuts_level_1", "level 1 shortcuts", BuildStage::kShortcuts, false},
      {"count_simple_turn_restrictions", "simple turn restrictions", BuildStage::kBuild, false},
      {"count_tiles", "amount of tiles with edges/nodes in them", BuildStage::kValidate, false},
  };

  static_assert(std::size(meta) == kCount, "build_stats::meta and counter enum are out of sync");

  void increment(counter c, uint32_t by = 1) {
    counters_[c] += by;
  }

  // Increment the shortcut count and edge count for the given hierarchy level.
  void increment_shortcuts(uint8_t level, uint32_t shortcuts, uint32_t edges) {
    counter scl = level == 0 ? kCountShortcutsLevel0 : kCountShortcutsLevel1;
    counter ecl = level == 0 ? kCountShortcutEdgesLevel0 : kCountShortcutEdgesLevel1;
    counters_[scl] += shortcuts;
    counters_[ecl] += edges;
  }

  static build_stats& get() {
    static build_stats instance;
    return instance;
  }

  // Record a timing to be emitted with the next log_stage() call.
  void record_timing(const std::string& key, uint64_t seconds);

  // Log and emit to statsd what changed since last snapshot.
  // Also emits any timings recorded since the last call.
  void log_stage(BuildStage stage, const boost::property_tree::ptree& config) const;

private:
  std::array<std::atomic<uint32_t>, kCount> counters_{};

  // are modified in const log_stage
  mutable std::vector<std::pair<std::string, uint64_t>> pending_timings_;
  mutable std::mutex timings_mutex_;
};

// A little struct to hold stats information during each threads work
struct enhancer_stats {
  float max_density; //(km/km2)
  uint32_t not_thru;
  uint32_t no_country_found;
  uint32_t internalcount;
  uint32_t turnchannelcount;
  uint32_t rampcount;
  uint32_t pencilucount;
  uint32_t density_counts[16];
  void operator()(const enhancer_stats& other) {
    if (max_density < other.max_density) {
      max_density = other.max_density;
    }
    not_thru += other.not_thru;
    no_country_found += other.no_country_found;
    internalcount += other.internalcount;
    turnchannelcount += other.turnchannelcount;
    rampcount += other.rampcount;
    pencilucount += other.pencilucount;
    for (uint32_t i = 0; i < 16; i++) {
      density_counts[i] += other.density_counts[i];
    }
  }
};

/**
 * Splits a tag into a vector of strings.
 * @param  tag_value  tag to split
 * @param  delim      defaults to ;
 * @return the vector of strings
 */
std::vector<std::string> GetTagTokens(const std::string& tag_value, char delim = ';');

/**
 * Splits a tag into a vector of strings.
 * @param  tag_value  tag to split
 * @param  delim      delimiter
 * @return the vector of strings
 */
std::vector<std::string> GetTagTokens(const std::string& tag_value, const std::string& delim_str);

/**
 * Remove double quotes.
 * @param  s
 * @return string string with no quotes.
 */
std::string remove_double_quotes(const std::string& s);

/**
 * Do the 2 supplied shape vectors match (either direction).
 * @param shape1 First shape vector.
 * @param shape2 Second shape vector.
 * @return Returns true if the shapes match. Checks if one is reverse direction than the other.
 */
bool shapes_match(const std::vector<midgard::PointLL>& shape1,
                  const std::vector<midgard::PointLL>& shape2);

/**
 * Get the index of the opposing edge at the end node. This is on the local hierarchy,
 * before adding transition and shortcut edges. Make sure that even if the end nodes
 * and lengths match that the correct edge is selected (match shape) since some loops
 * can have the same length and end node.
 * @param  endnodetile   Graph tile at the end node.
 * @param  startnode     Start node of the directed edge.
 * @param  tile          Graph tile of the edge
 * @param  directededge  Directed edge to match.
 */
uint32_t GetOpposingEdgeIndex(const baldr::graph_tile_ptr& endnodetile,
                              const baldr::GraphId& startnode,
                              const baldr::graph_tile_ptr& tile,
                              const baldr::DirectedEdge& edge);

/**
 * Process edge transitions from all other incoming edges onto the
 * specified outbound directed edge.
 * @param  idx            Index of the directed edge - the to edge.
 * @param  directededge   Directed edge builder - set values.
 * @param  edges          Other directed edges at the node.
 * @param  ntrans         Number of transitions (either number of edges or max)
 * @param  headings       Headings of directed edges.
 */
void ProcessEdgeTransitions(const uint32_t idx,
                            baldr::DirectedEdge& directededge,
                            const baldr::DirectedEdge* edges,
                            const uint32_t ntrans,
                            const baldr::NodeInfo& nodeinfo,
                            enhancer_stats& stats);
/**
 * Compute a curvature metric given an edge shape.
 * @param  shape  Shape of an edge (list of lat,lon vertices).
 * @return Returns a curvature measure [0-15] where higher numbers indicate
 *         more curved and tighter turns.
 */
uint32_t compute_curvature(const std::vector<midgard::PointLL>& shape);

/**
 * Build an entire valhalla tileset give a config file and some input pbfs. The
 * tile building process is split into stages. This method allows either the entire
 * tile building pipeline to run (default) or a subset of the stages to run.
 * @param config        Used to tell the function where and how to build the tiles
 * @param input_files   Tells what osm pbf files to build the tiles from
 * @param start_stage   Starting stage of the pipeline to run
 * @param end_stage     End stage of the pipeline to run
 * @param release_osmpbf_memory Free PBF parsing libs after use.  Saves RAM, but makes libprotobuf
 * unusable afterwards.  Set to false if you need to perform protobuf operations after building tiles.
 * @return Returns true if no errors occur, false if an error occurs.
 */
bool build_tile_set(const boost::property_tree::ptree& config,
                    const std::vector<std::string>& input_files,
                    const BuildStage start_stage = BuildStage::kInitialize,
                    const BuildStage end_stage = BuildStage::kValidate);

// The tile manifest is a JSON-serializable index of tiles to be processed during the build stage of
// valhalla_build_tiles'. It can be used to distribute shard keys when building tiles with
// parallelized, distributed batch processing. For example, a workflow orchestrator can partition
// and distribute this manifest to workers in a 'build' stage worker pool.
//
// This file written out during 'constructedges' and a prerequisite for the 'build' stage run by
// valhalla_build_tiles.
//
// Example manifest :
//
// {
//   "tiles": [
//     {
//       "node_index": 0,
//       "graphid": {
//         "value": 5970538,
//         "id": 0,
//         "tile_id": 746317,
//         "level": 2
//       }
//     }
//   ]
// }
struct TileManifest {
  std::map<baldr::GraphId, size_t> tileset;

  std::string ToString() const;

  void LogToFile(const std::string& filename) const;

  static TileManifest ReadFromFile(const std::string& filename);
};
} // namespace mjolnir
} // namespace valhalla
#endif // VALHALLA_MJOLNIR_UTIL_H_
