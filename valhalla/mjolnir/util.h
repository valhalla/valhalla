#ifndef VALHALLA_MJOLNIR_UTIL_H_
#define VALHALLA_MJOLNIR_UTIL_H_

#include <boost/property_tree/ptree.hpp>
#include <list>
#include <map>
#include <string>
#include <unordered_map>
#include <vector>

#include <sqlite3.h>

#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/rapidjson_utils.h>
#include <valhalla/midgard/logging.h>
#include <valhalla/midgard/pointll.h>

namespace valhalla {
namespace mjolnir {

using boost::property_tree::ptree;

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

/**
 * Splits a tag into a vector of strings.
 * @param  tag_value  tag to split
 * @param  delim      defaults to ;
 * @return the vector of strings
 */
std::vector<std::string> GetTagTokens(const std::string& tag_value, char delim = ';');

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
 * Compute a curvature metric given an edge shape.
 * @param  shape  Shape of an edge (list of lat,lon vertices).
 * @return Returns a curvature measure [0-15] where higher numbers indicate
 *         more curved and tighter turns.
 */
uint32_t compute_curvature(const std::list<midgard::PointLL>& shape);

/**
 * Loads spatialite extension for sqlite
 * @param db_handle   the handle to the open sqlite database
 * @return returns true if the module was successfully loaded
 */
bool load_spatialite(sqlite3* db_handle);

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
bool build_tile_set(const ptree& config,
                    const std::vector<std::string>& input_files,
                    const BuildStage start_stage = BuildStage::kInitialize,
                    const BuildStage end_stage = BuildStage::kValidate,
                    const bool release_osmpbf_memory = true);

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

  std::string ToString() const {
    baldr::json::ArrayPtr array = baldr::json::array({});
    for (const auto& tile : tileset) {
      const baldr::json::Value& graphid = tile.first.json();
      const baldr::json::MapPtr& item = baldr::json::map(
          {{"graphid", graphid}, {"node_index", static_cast<uint64_t>(tile.second)}});
      array->emplace_back(item);
    }
    std::stringstream manifest;
    manifest << *baldr::json::map({{"tiles", array}});
    return manifest.str();
  }

  void LogToFile(const std::string& filename) const {
    std::ofstream handle;
    handle.open(filename);
    handle << ToString();
    handle.close();
    LOG_INFO("Writing tile manifest to " + filename);
  }

  static TileManifest ReadFromFile(const std::string& filename) {
    ptree manifest;
    rapidjson::read_json(filename, manifest);
    LOG_INFO("Reading tile manifest from " + filename);
    std::map<baldr::GraphId, size_t> tileset;
    for (const auto& tile_info : manifest.get_child("tiles")) {
      const ptree& graph_id = tile_info.second.get_child("graphid");
      const baldr::GraphId id(graph_id.get<uint64_t>("value"));
      const size_t node_index = tile_info.second.get<size_t>("node_index");
      tileset.insert({id, node_index});
    }
    LOG_INFO("Reading " + std::to_string(tileset.size()) + " tiles from tile manifest file " +
             filename);
    return TileManifest{tileset};
  }
};
} // namespace mjolnir
} // namespace valhalla
#endif // VALHALLA_MJOLNIR_UTIL_H_
