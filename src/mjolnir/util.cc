#include "mjolnir/util.h"
#include "baldr/rapidjson_utils.h"
#include "baldr/tilehierarchy.h"
#include "midgard/logging.h"
#include "mjolnir/bssbuilder.h"
#include "mjolnir/elevationbuilder.h"
#include "mjolnir/graphbuilder.h"
#include "mjolnir/graphenhancer.h"
#include "mjolnir/graphfilter.h"
#include "mjolnir/graphvalidator.h"
#include "mjolnir/hierarchybuilder.h"
#include "mjolnir/pbfgraphparser.h"
#include "mjolnir/restrictionbuilder.h"
#include "mjolnir/shortcutbuilder.h"
#include "mjolnir/transitbuilder.h"
#include "scoped_timer.h"

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/property_tree/ptree.hpp>

#include <filesystem>
#include <regex>

using namespace valhalla::baldr;
using namespace valhalla::midgard;
using namespace valhalla::mjolnir;

namespace {

// Temporary files used during tile building
const std::string ways_file = "ways.bin";
const std::string way_nodes_file = "way_nodes.bin";
const std::string nodes_file = "nodes.bin";
const std::string edges_file = "edges.bin";
const std::string tile_manifest_file = "tile_manifest.json";
const std::string access_file = "access.bin";
const std::string pronunciation_file = "pronunciation.bin";
const std::string bss_nodes_file = "bss_nodes.bin";
const std::string linguistic_node_file = "linguistics_node.bin";
const std::string cr_from_file = "complex_from_restrictions.bin";
const std::string cr_to_file = "complex_to_restrictions.bin";
const std::string new_to_old_file = "new_nodes_to_old_nodes.bin";
const std::string old_to_new_file = "old_nodes_to_new_nodes.bin";
const std::string intersections_file = "intersections.bin";
const std::string shapes_file = "shapes.bin";

/**
 * Returns true if edge transition is a pencil point u-turn, false otherwise.
 * A pencil point intersection happens when a doubly-digitized road transitions
 * to a singly-digitized road - which looks like a pencil point - for example:
 *        -----\____
 *        -----/
 *
 * @param  from_index  Index of the 'from' directed edge.
 * @param  to_index  Index of the 'to' directed edge.
 * @param  directededge  Directed edge builder.
 * @param  edges  Directed edges outbound from a node.
 * @param  node_info  Node info builder used for name consistency.
 * @param  turn_degree  The turn degree between the 'from' and 'to' edge.
 *
 * @return true if edge transition is a pencil point u-turn, false otherwise.
 */
bool IsPencilPointUturn(uint32_t from_index,
                        uint32_t to_index,
                        const DirectedEdge& directededge,
                        const DirectedEdge* edges,
                        const NodeInfo& node_info,
                        uint32_t turn_degree) {
  // Logic for drive on right
  if (node_info.drive_on_right()) {
    // If the turn is a sharp left (179 < turn < 211)
    //    or short distance (< 50m) and wider sharp left (179 < turn < 226)
    // and oneway edgesb
    // and an intersecting right road exists
    // and no intersecting left road exists
    // and the from and to edges have a common base name
    // then it is a left pencil point u-turn
    if ((((turn_degree > 179) && (turn_degree < 211)) ||
         (((edges[from_index].length() < 50) || (directededge.length() < 50)) &&
          (turn_degree > 179) && (turn_degree < 226))) &&
        (!(edges[from_index].forwardaccess() & kAutoAccess) &&
         (edges[from_index].reverseaccess() & kAutoAccess)) &&
        ((directededge.forwardaccess() & kAutoAccess) &&
         !(directededge.reverseaccess() & kAutoAccess)) &&
        directededge.edge_to_right(from_index) && !directededge.edge_to_left(from_index) &&
        edges[to_index].name_consistency(from_index)) {
      return true;
    }

  }
  // Logic for drive on left
  else {
    // If the turn is a sharp right (149 < turn < 181)
    //    or short distance (< 50m) and wider sharp right (134 < turn < 181)
    // and oneway edges
    // and no intersecting right road exists
    // and an intersecting left road exists
    // and the from and to edges have a common base name
    // then it is a right pencil point u-turn
    if ((((turn_degree > 149) && (turn_degree < 181)) ||
         (((edges[from_index].length() < 50) || (directededge.length() < 50)) &&
          (turn_degree > 134) && (turn_degree < 181))) &&
        (!(edges[from_index].forwardaccess() & kAutoAccess) &&
         (edges[from_index].reverseaccess() & kAutoAccess)) &&
        ((directededge.forwardaccess() & kAutoAccess) &&
         !(directededge.reverseaccess() & kAutoAccess)) &&
        !directededge.edge_to_right(from_index) && directededge.edge_to_left(from_index) &&
        edges[to_index].name_consistency(from_index)) {
      return true;
    }
  }

  return false;
}

/**
 * Returns true if edge transition is a cycleway u-turn, false otherwise.
 *
 * @param  from_index  Index of the 'from' directed edge.
 * @param  to_index  Index of the 'to' directed edge.
 * @param  directededge  Directed edge builder.
 * @param  edges  Directed edges outbound from a node.
 * @param  node_info  Node info builder used for name consistency.
 * @param  turn_degree  The turn degree between the 'from' and 'to' edge.
 *
 * @return true if edge transition is a cycleway u-turn, false otherwise.
 */
bool IsCyclewayUturn(uint32_t from_index,
                     uint32_t to_index,
                     const DirectedEdge& directededge,
                     const DirectedEdge* edges,
                     const NodeInfo& node_info,
                     uint32_t turn_degree) {

  // we only deal with Cycleways
  if (edges[from_index].use() != Use::kCycleway || edges[to_index].use() != Use::kCycleway) {
    return false;
  }

  // Logic for drive on right
  if (node_info.drive_on_right()) {
    // If the turn is a sharp left (179 < turn < 211)
    //    or short distance (< 50m) and wider sharp left (179 < turn < 226)
    // and an intersecting right road exists
    // and an intersecting left road exists
    // then it is a cycleway u-turn
    if ((((turn_degree > 179) && (turn_degree < 211)) ||
         (((edges[from_index].length() < 50) || (directededge.length() < 50)) &&
          (turn_degree > 179) && (turn_degree < 226))) &&
        directededge.edge_to_right(from_index) && directededge.edge_to_left(from_index)) {
      return true;
    }
  }
  // Logic for drive on left
  else {
    // If the turn is a sharp right (149 < turn < 181)
    //    or short distance (< 50m) and wider sharp right (134 < turn < 181)
    // and an intersecting right road exists
    // and an intersecting left road exists
    // then it is a right cyclewayt u-turn
    if ((((turn_degree > 149) && (turn_degree < 181)) ||
         (((edges[from_index].length() < 50) || (directededge.length() < 50)) &&
          (turn_degree > 134) && (turn_degree < 181))) &&
        directededge.edge_to_right(from_index) && directededge.edge_to_left(from_index)) {
      return true;
    }
  }
  return false;
}

/**
 * Gets the stop likelihoood / impact at an intersection when transitioning
 * from one edge to another. This depends on the difference between the
 * classifications/importance of the from and to edge and the highest
 * classification of the remaining edges at the intersection. Low impact
 * values occur when the from and to roads are higher class roads than other
 * roads. There is less likelihood of having to stop in these cases (or stops
 * will usually be shorter duration). Where traffic lights are (or might be)
 * present it is more likely that a favorable "green" is present in the
 * direction of the higher classification. If classifications are all equal
 * the stop impact will depend on the classification. All directions are
 * likely to stop and duration is likely longer with higher classification
 * roads (e.g. a 4 way stop of tertiary roads is likely to be shorter than
 * a 4 way stop (with traffic light) at an intersection of 4 primary roads.
 * Higher stop impacts occur when the from and to edges are lower class
 * than the others. There is almost certainly a stop (stop sign, traffic
 * light) and longer waits are likely when a low class road crosses
 * a higher class road. Special cases occur for links (ramps/turn channels)
 * and parking aisles.
 * @param  from  Index of the from directed edge.
 * @param  to    Index of the to directed edge.
 * @param  directededge   Directed edge builder - set values.
 * @param  edges Directed edges outbound from a node.
 * @param  count Number of outbound directed edges to consider.
 * @param  node_info  Node info builder used for name consistency.
 * @param  turn_degree  The turn degree between the 'from' and 'to' edge.
 *
 * @return  Returns stop impact ranging from 0 (no likely impact) to
 *          7 - large impact.
 */
uint32_t GetStopImpact(uint32_t from,
                       uint32_t to,
                       const DirectedEdge& directededge,
                       const DirectedEdge* edges,
                       const uint32_t count,
                       const NodeInfo& nodeinfo,
                       uint32_t turn_degree,
                       enhancer_stats& stats) {

  ///////////////////////////////////////////////////////////////////////////
  // Special cases.

  // Handle Roundabouts
  if (edges[from].roundabout() && edges[to].roundabout()) {
    return 0;
  }

  // Handle Pencil point u-turn
  if (IsPencilPointUturn(from, to, directededge, edges, nodeinfo, turn_degree)) {
    stats.pencilucount++;
    return 7;
  }

  // Handle Cycleway u-turn
  if (IsCyclewayUturn(from, to, directededge, edges, nodeinfo, turn_degree)) {
    return 7;
  }

  ///////////////////////////////////////////////////////////////////////////

  // Get the highest classification of other roads at the intersection
  bool all_ramps = true;
  const DirectedEdge* edge = &edges[0];
  // kUnclassified,  kResidential, and kServiceOther are grouped
  // together for the stop_impact logic.
  RoadClass bestrc = RoadClass::kUnclassified;
  for (uint32_t i = 0; i < count; i++, edge++) {
    // Check the road if it is drivable TO the intersection and is neither
    // the "to" nor "from" edge. Treat roundabout edges as two levels lower
    // classification (higher value) to reduce the stop impact.
    if (i != to && i != from && (edge->reverseaccess() & kAutoAccess)) {
      if (edge->roundabout()) {
        uint32_t c = static_cast<uint32_t>(edge->classification()) + 2;
        if (c < static_cast<uint32_t>(bestrc)) {
          bestrc = static_cast<RoadClass>(c);
        }
      } else if (edge->classification() < bestrc) {
        bestrc = edge->classification();
      }
    }

    // Check if not a ramp or turn channel
    if (!edge->link()) {
      all_ramps = false;
    }
  }

  // kUnclassified,  kResidential, and kServiceOther are grouped
  // together for the stop_impact logic.
  RoadClass from_rc = edges[from].classification();
  if (from_rc > RoadClass::kUnclassified) {
    from_rc = RoadClass::kUnclassified;
  }

  // High stop impact from a turn channel onto a turn channel unless
  // the other edge a low class road (walkways often intersect
  // turn channels)
  if (edges[from].use() == Use::kTurnChannel && edges[to].use() == Use::kTurnChannel &&
      bestrc < RoadClass::kUnclassified) {
    return 7;
  }

  // Set stop impact to the difference in road class (make it non-negative)
  int impact = static_cast<int>(from_rc) - static_cast<int>(bestrc);
  uint32_t stop_impact = (impact < -3) ? 0 : impact + 3;

  // TODO: possibly increase stop impact at large intersections (more edges)
  // or if several are high class
  // Reduce stop impact from a turn channel or when only links
  // (ramps and turn channels) are involved. Exception - sharp turns.
  Turn::Type turn_type = Turn::GetType(turn_degree);
  bool is_sharp = (turn_type == Turn::Type::kSharpLeft || turn_type == Turn::Type::kSharpRight ||
                   turn_type == Turn::Type::kReverse);
  bool is_slight = (turn_type == Turn::Type::kStraight || turn_type == Turn::Type::kSlightRight ||
                    turn_type == Turn::Type::kSlightLeft);
  if (all_ramps) {
    if (is_sharp) {
      stop_impact += 2;
    } else if (is_slight) {
      stop_impact /= 2;
    } else if (stop_impact != 0) { // make sure we do not subtract 1 from 0
      stop_impact -= 1;
    }
  } else if (edges[from].use() == Use::kRamp && edges[to].use() == Use::kRamp &&
             bestrc < RoadClass::kUnclassified) {
    // Ramp may be crossing a road (not a path or service road)
    if (nodeinfo.traffic_signal() || edges[from].traffic_signal() || edges[from].stop_sign()) {
      stop_impact = 4;
    } else if (count > 3) {
      stop_impact += 2;
    }
  } else if (edges[from].use() == Use::kRamp && edges[to].use() != Use::kRamp &&
             !edges[from].internal() && !edges[to].internal()) {
    // Increase stop impact on merge
    if (is_sharp) {
      stop_impact += 3;
    } else if (is_slight) {
      stop_impact += 1;
    } else {
      stop_impact += 2;
    }

  } else if (edges[from].use() == Use::kTurnChannel) {
    // Penalize sharp turns
    if (is_sharp) {
      stop_impact += 2;
    } else if (edges[to].use() == Use::kRamp) {
      stop_impact += 1;
    } else if (is_slight) {
      stop_impact /= 2;
    } else if (stop_impact != 0) { // make sure we do not subtract 1 from 0
      stop_impact -= 1;
    }
  } else if (edges[from].use() == Use::kParkingAisle && edges[to].use() == Use::kParkingAisle) {
    // decrease stop impact inside parking lots
    if (stop_impact != 0)
      stop_impact -= 1;
  }
  // add to the stop impact when transitioning from higher to lower class road and we are not on a TC
  // or ramp penalize lefts when driving on the right.
  else if (nodeinfo.drive_on_right() &&
           (turn_type == Turn::Type::kSharpLeft || turn_type == Turn::Type::kLeft) &&
           from_rc != edges[to].classification() && edges[to].use() != Use::kRamp &&
           edges[to].use() != Use::kTurnChannel) {
    if (nodeinfo.traffic_signal() || edges[from].traffic_signal() || edges[from].stop_sign()) {
      stop_impact += 2;
    } else if (abs(static_cast<int>(from_rc) - static_cast<int>(edges[to].classification())) > 1)
      stop_impact++;
    // penalize rights when driving on the left.
  } else if (!nodeinfo.drive_on_right() &&
             (turn_type == Turn::Type::kSharpRight || turn_type == Turn::Type::kRight) &&
             from_rc != edges[to].classification() && edges[to].use() != Use::kRamp &&
             edges[to].use() != Use::kTurnChannel) {
    if (nodeinfo.traffic_signal() || edges[from].traffic_signal() || edges[from].stop_sign()) {
      stop_impact += 2;
    } else if (abs(static_cast<int>(from_rc) - static_cast<int>(edges[to].classification())) > 1)
      stop_impact++;
  }
  // Clamp to kMaxStopImpact
  return (stop_impact <= kMaxStopImpact) ? stop_impact : kMaxStopImpact;
}

} // namespace

namespace valhalla {
namespace mjolnir {

/**
 * Splits a tag into a vector of strings.  Delim defaults to ;
 */
std::vector<std::string> GetTagTokens(const std::string& tag_value, char delim) {
  std::vector<std::string> tokens;
  boost::algorithm::split(
      tokens, tag_value,
      [delim](auto&& PH1) { return std::equal_to<char>()(delim, std::forward<decltype(PH1)>(PH1)); },
      boost::algorithm::token_compress_off);

  return tokens;
}

std::vector<std::string> GetTagTokens(const std::string& tag_value, const std::string& delim_str) {
  std::regex regex_str(delim_str);
  std::vector<std::string> tokens(std::sregex_token_iterator(tag_value.begin(), tag_value.end(),
                                                             regex_str, -1),
                                  std::sregex_token_iterator());
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

/**
 * Compute a curvature metric given an edge shape. The final value is from 0 to 15 it is computed by
 * taking each pair of 3 points in the shape and finding the radius of the circle for which all 3
 * points lie on it. The larger the radius the less curvy a set of 3 points is. The function is not
 * robust to the ordering of the points which means some pathological cases can seem straight but
 * actually be curvy however this is uncommon in real data sets. Each radius of 3 consecutive points
 * is measured and capped at a maximum value, the radii are averaged together and a final score
 * between 0 and 15 is stored.
 *
 * @param shape   the shape whose curviness we want to measure
 * @return value between 0 and 15 representing the average curviness of the input shape. lower
 *         values indicate less curvy shapes and higher values indicate curvier shapes
 */
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

/**
 * Get the index of the opposing edge at the end node. This is on the local hierarchy,
 * before adding transition and shortcut edges. Make sure that even if the end nodes
 * and lengths match that the correct edge is selected (match shape) since some loops
 * can have the same length and end node.
 */
uint32_t GetOpposingEdgeIndex(const graph_tile_ptr& endnodetile,
                              const baldr::GraphId& startnode,
                              const graph_tile_ptr& tile,
                              const baldr::DirectedEdge& edge) {
  // Get the nodeinfo at the end of the edge
  const baldr::NodeInfo* nodeinfo = endnodetile->node(edge.endnode().id());

  // Iterate through the directed edges and return when the end node matches the specified
  // node, the length matches, and the shape matches (or edgeinfo offset matches)
  const baldr::DirectedEdge* directededge = endnodetile->directededge(nodeinfo->edge_index());
  for (uint32_t i = 0; i < nodeinfo->edge_count(); i++, directededge++) {
    if (directededge->endnode() == startnode && directededge->length() == edge.length()) {
      // If in the same tile and the edgeinfo offset matches then the shape and names will match
      if (endnodetile == tile && directededge->edgeinfo_offset() == edge.edgeinfo_offset()) {
        return i;
      } else {
        // Need to compare shape if not in the same tile or different EdgeInfo (could be different
        // names in opposing directions)
        if (shapes_match(tile->edgeinfo(&edge).shape(),
                         endnodetile->edgeinfo(directededge).shape())) {
          return i;
        }
      }
    }
  }
  LOG_ERROR("Could not find opposing edge index");
  return baldr::kMaxEdgesPerNode;
}

/**
 * Process edge transitions from all other incoming edges onto the
 * specified outbound directed edge.
 */
void ProcessEdgeTransitions(const uint32_t idx,
                            baldr::DirectedEdge& directededge,
                            const baldr::DirectedEdge* edges,
                            const uint32_t ntrans,
                            const baldr::NodeInfo& nodeinfo,
                            enhancer_stats& stats) {
  for (uint32_t i = 0; i < ntrans; i++) {
    // Get the turn type (reverse the heading of the from directed edge since
    // it is incoming
    uint32_t from_heading = ((nodeinfo.heading(i) + 180) % 360);
    uint32_t turn_degree = GetTurnDegree(from_heading, nodeinfo.heading(idx));
    directededge.set_turntype(i, baldr::Turn::GetType(turn_degree));

    // Set the edge_to_left and edge_to_right flags
    uint32_t right_count = 0;
    uint32_t left_count = 0;
    if (ntrans > 2) {
      for (uint32_t j = 0; j < ntrans; ++j) {
        // Skip the from and to edges; also skip roads under construction
        if (j == i || j == idx || edges[j].use() == baldr::Use::kConstruction) {
          continue;
        }

        // Get the turn degree from incoming edge i to j and check if right
        // or left of the turn degree from incoming edge i onto idx
        uint32_t degree = GetTurnDegree(from_heading, nodeinfo.heading(j));
        if (turn_degree > 180) {
          if (degree > turn_degree || degree < 180) {
            ++right_count;
          } else if (degree < turn_degree && degree > 180) {
            ++left_count;
          }
        } else {
          if (degree > turn_degree && degree < 180) {
            ++right_count;
          } else if (degree < turn_degree || degree > 180) {
            ++left_count;
          }
        }
      }
    }
    directededge.set_edge_to_left(i, (left_count > 0));
    directededge.set_edge_to_right(i, (right_count > 0));

    // Get stop impact
    // NOTE: stop impact uses the right and left edges so this logic must
    // come after the right/left edge logic
    uint32_t stopimpact =
        GetStopImpact(i, idx, directededge, edges, ntrans, nodeinfo, turn_degree, stats);
    directededge.set_stopimpact(i, stopimpact);
  }
}

bool build_tile_set(const boost::property_tree::ptree& original_config,
                    const std::vector<std::string>& input_files,
                    const BuildStage start_stage,
                    const BuildStage end_stage) {
  SCOPED_TIMER();
  auto remove_temp_file = [](const std::string& fname) {
    if (std::filesystem::exists(fname)) {
      std::filesystem::remove(fname);
    }
  };

  if (input_files.size() > 1) {
    LOG_WARN(
        "Tile building using more than one osm.pbf extract is discouraged. Consider merging the extracts into one file. See this issue for more info: https://github.com/valhalla/valhalla/issues/3925 ");
  }

  // Take out tile_extract and tile_url from property tree as tiles must only use the tile_dir
  auto config = original_config;
  config.get_child("mjolnir").erase("tile_extract");
  config.get_child("mjolnir").erase("tile_url");
  config.get_child("mjolnir").erase("traffic_extract");

  // Get the tile directory (make sure it ends with the preferred separator
  std::string tile_dir = config.get<std::string>("mjolnir.tile_dir");
  if (tile_dir.back() != std::filesystem::path::preferred_separator) {
    tile_dir.push_back(std::filesystem::path::preferred_separator);
  }

  // During the initialize stage the tile directory will be purged (if it already exists)
  // and will be created if it does not already exist
  if (start_stage == BuildStage::kInitialize) {
    // set up the directories and purge old tiles if starting at the parsing stage
    for (const auto& level : valhalla::baldr::TileHierarchy::levels()) {
      auto level_dir = tile_dir + std::to_string(level.level);
      if (std::filesystem::exists(level_dir) && !std::filesystem::is_empty(level_dir)) {
        LOG_WARN("Non-empty " + level_dir + " will be purged of tiles");
        std::filesystem::remove_all(level_dir);
      }
    }

    // check for transit level.
    auto level_dir =
        tile_dir + std::to_string(valhalla::baldr::TileHierarchy::GetTransitLevel().level);
    if (std::filesystem::exists(level_dir) && !std::filesystem::is_empty(level_dir)) {
      LOG_WARN("Non-empty " + level_dir + " will be purged of tiles");
      std::filesystem::remove_all(level_dir);
    }

    // Create the directory if it does not exist
    std::filesystem::create_directories(tile_dir);
  }

  // Set up the temporary (*.bin) files used during processing
  std::string ways_bin = tile_dir + ways_file;
  std::string way_nodes_bin = tile_dir + way_nodes_file;
  std::string nodes_bin = tile_dir + nodes_file;
  std::string edges_bin = tile_dir + edges_file;
  std::string tile_manifest = tile_dir + tile_manifest_file;
  std::string access_bin = tile_dir + access_file;
  std::string bss_nodes_bin = tile_dir + bss_nodes_file;
  std::string linguistic_node_bin = tile_dir + linguistic_node_file;
  std::string cr_from_bin = tile_dir + cr_from_file;
  std::string cr_to_bin = tile_dir + cr_to_file;
  std::string new_to_old_bin = tile_dir + new_to_old_file;
  std::string old_to_new_bin = tile_dir + old_to_new_file;

  // OSMData class
  OSMData osm_data{0};

  // Parse the ways
  if (start_stage <= BuildStage::kParseWays && BuildStage::kParseWays <= end_stage) {
    // Read the OSM protocol buffer file. Callbacks for ways are defined within the PBFParser class
    osm_data = PBFGraphParser::ParseWays(config.get_child("mjolnir"), input_files, ways_bin,
                                         way_nodes_bin, access_bin);

    // Write the OSMData to files if the end stage is less than enhancing
    if (end_stage <= BuildStage::kEnhance) {
      osm_data.write_to_temp_files(tile_dir);
    }
  }

  // Parse OSM data
  if (start_stage <= BuildStage::kParseRelations && BuildStage::kParseRelations <= end_stage) {

    // Read the OSM protocol buffer file. Callbacks for relations are defined within the PBFParser
    // class
    PBFGraphParser::ParseRelations(config.get_child("mjolnir"), input_files, cr_from_bin, cr_to_bin,
                                   osm_data);

    // Write the OSMData to files if the end stage is less than enhancing
    if (end_stage <= BuildStage::kEnhance) {
      osm_data.write_to_temp_files(tile_dir);
    }
  }

  // Parse OSM data
  if (start_stage <= BuildStage::kParseNodes && BuildStage::kParseNodes <= end_stage) {
    // Read the OSM protocol buffer file. Callbacks for nodes
    // are defined within the PBFParser class
    PBFGraphParser::ParseNodes(config.get_child("mjolnir"), input_files, way_nodes_bin, bss_nodes_bin,
                               linguistic_node_bin, osm_data);

    // Write the OSMData to files if the end stage is less than enhancing
    if (end_stage <= BuildStage::kEnhance) {
      osm_data.write_to_temp_files(tile_dir);
    }
  }

  // Construct edges
  std::map<baldr::GraphId, size_t> tiles;
  if (start_stage <= BuildStage::kConstructEdges && BuildStage::kConstructEdges <= end_stage) {

    // Read OSMData from files if construct edges is the first stage
    if (start_stage == BuildStage::kConstructEdges)
      osm_data.read_from_temp_files(tile_dir);

    tiles = GraphBuilder::BuildEdges(config, ways_bin, way_nodes_bin, nodes_bin, edges_bin);
    // Output manifest
    TileManifest manifest{tiles};
    manifest.LogToFile(tile_manifest);
  }

  // Build Valhalla routing tiles
  if (start_stage <= BuildStage::kBuild && BuildStage::kBuild <= end_stage) {
    if (start_stage == BuildStage::kBuild) {
      // Read OSMData from files if building tiles is the first stage
      osm_data.read_from_temp_files(tile_dir);
      if (std::filesystem::exists(tile_manifest)) {
        tiles = TileManifest::ReadFromFile(tile_manifest).tileset;
      } else {
        // TODO: Remove this backfill in the future, and make calling constructedges stage
        // explicitly required in the future.
        LOG_WARN("Tile manifest not found, rebuilding edges and manifest");
        tiles = GraphBuilder::BuildEdges(config, ways_bin, way_nodes_bin, nodes_bin, edges_bin);
      }
    }

    // Build the graph using the OSMNodes and OSMWays from the parser
    GraphBuilder::Build(config, osm_data, ways_bin, way_nodes_bin, nodes_bin, edges_bin, cr_from_bin,
                        cr_to_bin, linguistic_node_bin, tiles);
  }

  // Enhance the local level of the graph. This adds information to the local
  // level that is usable across all levels (density, administrative
  // information (and country based attribution), edge transition logic, etc.
  if (start_stage <= BuildStage::kEnhance && BuildStage::kEnhance <= end_stage) {
    // Read OSMData names from file if enhancing tiles is the first stage
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
    if (start_stage == BuildStage::kBss) {
      osm_data.read_from_unique_names_file(tile_dir);
    }
    BssBuilder::Build(config, osm_data, bss_nodes_bin);
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
    remove_temp_file(linguistic_node_bin);
    remove_temp_file(new_to_old_bin);
    remove_temp_file(old_to_new_bin);
    remove_temp_file(tile_manifest);
    OSMData::cleanup_temp_files(tile_dir);
  }
  return true;
}

std::string TileManifest::ToString() const {
  rapidjson::writer_wrapper_t writer(4096);
  writer.start_object();
  writer.start_array("tiles");
  for (const auto& tile : tileset) {
    writer.start_object();
    writer.start_object("graphid");
    tile.first.json(writer);
    writer.end_object();
    writer("node_index", static_cast<uint64_t>(tile.second));
    writer.end_object();
  }
  writer.end_array();
  writer.end_object();
  return writer.get_buffer();
}

void TileManifest::LogToFile(const std::string& filename) const {
  std::ofstream handle;
  handle.open(filename);
  handle << ToString();
  handle.close();
  LOG_INFO("Writing tile manifest to " + filename);
}

TileManifest TileManifest::ReadFromFile(const std::string& filename) {
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

} // namespace mjolnir
} // namespace valhalla
