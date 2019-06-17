#ifndef VALHALLA_MJOLNIR_UTIL_H_
#define VALHALLA_MJOLNIR_UTIL_H_

#include <boost/property_tree/ptree.hpp>
#include <list>
#include <string>
#include <unordered_map>
#include <vector>

#include <valhalla/midgard/pointll.h>

namespace valhalla {
namespace mjolnir {

// NOTE - for now parse and build must be run at the same time!
// Stages of the Valhalla tile building pipeline
enum class BuildStage : int8_t {
  kInvalid = -1,
  kInitialize = 0,
  kParse = 1,
  kBuild = 2,
  kEnhance = 3,
  kFilter = 4,
  kTransit = 5,
  kBss = 6,
  kHierarchy = 7,
  kShortcuts = 8,
  kRestrictions = 9,
  kElevation = 10,
  kValidate = 11,
  kCleanup = 12
};

// Convert string to BuildStage
inline BuildStage string_to_buildstage(const std::string& s) {
  static const std::unordered_map<std::string, BuildStage> stringToBuildStage =
      {{"initialize", BuildStage::kInitialize},
       {"parse", BuildStage::kParse},
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
       {static_cast<int8_t>(BuildStage::kParse), "parse"},
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
 * Build an entire valhalla tileset give a config file and some input pbfs. The
 * tile building process is split into stages. This method allows either the entire
 * tile building pipeline to run (default) or a subset of the stages to run.
 * @param config        Used to tell the function where and how to build the tiles
 * @param input_files   Tells what osm pbf files to build the tiles from
 * @param start_stage   Starting stage of the pipeline to run
 * @param end_stage     End stage of the pipeline to run
 * @return Returns true if no errors occur, false if an error occurs.
 */
bool build_tile_set(const boost::property_tree::ptree& config,
                    const std::vector<std::string>& input_files,
                    const BuildStage start_stage = BuildStage::kInitialize,
                    const BuildStage end_stage = BuildStage::kValidate);

} // namespace mjolnir
} // namespace valhalla
#endif // VALHALLA_MJOLNIR_UTIL_H_
