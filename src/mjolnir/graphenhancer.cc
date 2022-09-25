#include "mjolnir/graphenhancer.h"
#include "mjolnir/admin.h"
#include "mjolnir/countryaccess.h"
#include "mjolnir/graphtilebuilder.h"
#include "mjolnir/util.h"
#include "speed_assigner.h"

#include <cinttypes>
#include <future>
#include <limits>
#include <list>
#include <memory>
#include <mutex>
#include <queue>
#include <set>
#include <stdexcept>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <boost/format.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/io/wkt/wkt.hpp>
#include <boost/geometry/multi/geometries/multi_polygon.hpp>

#include "baldr/datetime.h"
#include "baldr/graphconstants.h"
#include "baldr/graphid.h"
#include "baldr/graphreader.h"
#include "baldr/graphtile.h"
#include "baldr/streetnames.h"
#include "baldr/streetnames_factory.h"
#include "baldr/tilehierarchy.h"
#include "midgard/aabb2.h"
#include "midgard/constants.h"
#include "midgard/distanceapproximator.h"
#include "midgard/logging.h"
#include "midgard/pointll.h"
#include "midgard/sequence.h"
#include "midgard/util.h"
#include "mjolnir/osmaccess.h"

using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::mjolnir;

namespace {

// Geometry types for admin queries
typedef boost::geometry::model::d2::point_xy<double> point_type;
typedef boost::geometry::model::polygon<point_type> polygon_type;
typedef boost::geometry::model::multi_polygon<polygon_type> multi_polygon_type;

// Number of tries when determining not thru edges
constexpr uint32_t kMaxNoThruTries = 256;

// Radius (km) to use for density
constexpr float kDensityRadius = 2.0f;
constexpr float kDensityRadius2 = kDensityRadius * kDensityRadius;
constexpr float kDensityLatDeg = (kDensityRadius * kMetersPerKm) / kMetersPerDegreeLat;

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

// Get the turn types.  This is used the determine if we should enhance or update
// Turn lanes based on the turns at this node.
void GetTurnTypes(const DirectedEdge& directededge,
                  std::set<Turn::Type>& outgoing_turn_type,
                  graph_tile_ptr tile,
                  GraphReader& reader,
                  std::mutex& lock) {
  // Get the heading value at the end of incoming edge based on edge shape
  auto incoming_shape = tile->edgeinfo(&directededge).shape();
  if (directededge.forward()) {
    std::reverse(incoming_shape.begin(), incoming_shape.end());
  }
  uint32_t heading = std::round(
      PointLL::HeadingAlongPolyline(incoming_shape, GetOffsetForHeading(directededge.classification(),
                                                                        directededge.use())));
  heading = ((heading + 180) % 360);

  // Get the tile at the end node. and find inbound heading of the candidate
  // edge to the end node.
  if (tile->id() != directededge.endnode().Tile_Base()) {
    lock.lock();
    tile = reader.GetGraphTile(directededge.endnode());
    lock.unlock();
  }
  const NodeInfo* node = tile->node(directededge.endnode());

  // Iterate through outbound edges and get turn degrees from the candidate
  // edge onto outbound driveable edges.
  const auto* diredge = tile->directededge(node->edge_index());
  for (uint32_t i = 0; i < node->edge_count(); i++, diredge++) {
    // Skip opposing directed edge and any edge that is not a road. Skip any
    // edges that are not driveable outbound.
    if (i == directededge.opp_local_idx() || !(diredge->forwardaccess() & kAutoAccess) ||
        (directededge.restrictions() & (1 << diredge->localedgeidx())) != 0) {
      continue;
    }

    // Get the heading of the outbound edge (unfortunately GraphEnhancer may
    // not have yet computed and stored headings for this node).
    auto shape = tile->edgeinfo(diredge).shape();
    if (!diredge->forward()) {
      std::reverse(shape.begin(), shape.end());
    }
    uint32_t to_heading =
        std::round(PointLL::HeadingAlongPolyline(shape, GetOffsetForHeading(diredge->classification(),
                                                                            diredge->use())));
    // Store outgoing turn type for any driveable edges
    uint32_t turndegree = GetTurnDegree(heading, to_heading);
    outgoing_turn_type.insert(Turn::GetType(turndegree));
  }
}

// Update the rightmost lane as needed.
void EnhanceRightLane(const DirectedEdge& directededge,
                      const graph_tile_ptr& tilebuilder,
                      GraphReader& reader,
                      std::mutex& lock,
                      std::vector<uint16_t>& enhanced_tls) {
  std::set<Turn::Type> outgoing_turn_type;
  GetTurnTypes(directededge, outgoing_turn_type, tilebuilder, reader, lock);

  size_t index = enhanced_tls.size() - 1;
  uint16_t tl = enhanced_tls[index];

  if (outgoing_turn_type.find(Turn::Type::kSlightRight) != outgoing_turn_type.end()) {
    // assume the slight right is the through if kStraight is not found
    if (tl == kTurnLaneThrough &&
        outgoing_turn_type.find(Turn::Type::kStraight) != outgoing_turn_type.end()) {
      tl |= kTurnLaneSlightRight;
    }
  }

  if (outgoing_turn_type.find(Turn::Type::kRight) != outgoing_turn_type.end())
    tl |= kTurnLaneRight;
  if (outgoing_turn_type.find(Turn::Type::kSharpRight) != outgoing_turn_type.end())
    tl |= kTurnLaneSharpRight;
  enhanced_tls[index] = tl;
}

// Update the leftmost lane as needed.
void EnhanceLeftLane(const DirectedEdge& directededge,
                     const graph_tile_ptr& tilebuilder,
                     GraphReader& reader,
                     std::mutex& lock,
                     std::vector<uint16_t>& enhanced_tls) {
  std::set<Turn::Type> outgoing_turn_type;
  GetTurnTypes(directededge, outgoing_turn_type, tilebuilder, reader, lock);

  uint16_t tl = enhanced_tls[0];
  if (outgoing_turn_type.find(Turn::Type::kSlightLeft) != outgoing_turn_type.end()) {
    // assume the slight left is the through if kStraight is not found
    if (tl == kTurnLaneThrough &&
        outgoing_turn_type.find(Turn::Type::kStraight) != outgoing_turn_type.end()) {
      tl |= kTurnLaneSlightLeft;
    }
  }
  if (outgoing_turn_type.find(Turn::Type::kLeft) != outgoing_turn_type.end())
    tl |= kTurnLaneLeft;
  if (outgoing_turn_type.find(Turn::Type::kSharpLeft) != outgoing_turn_type.end())
    tl |= kTurnLaneSharpLeft;
  enhanced_tls[0] = tl;
}

// update turn lanes vector
bool ProcessLanes(bool isLeft, bool endOnTurn, std::vector<uint16_t>& enhanced_tls) {

  bool bUpdated = false;
  uint16_t previous = 0u;

  if (isLeft) {
    std::vector<uint16_t>::iterator it = enhanced_tls.begin();
    for (; it != enhanced_tls.end(); it++) {
      if (((*it & kTurnLaneLeft) || (*it & kTurnLaneSharpLeft) || (*it & kTurnLaneSlightLeft) ||
           (*it & kTurnLaneThrough)) &&
          (previous == 0u || (previous & kTurnLaneLeft) || (previous & kTurnLaneSharpLeft) ||
           (previous & kTurnLaneSlightLeft) || (previous & kTurnLaneThrough))) {
        previous = *it;
      } else if (previous && (*it == kTurnLaneEmpty || *it == kTurnLaneNone)) {
        *it = kTurnLaneThrough;

        if (!endOnTurn)
          bUpdated = true; // should end on a through

      } else if (previous && ((*it & kTurnLaneRight) || (*it & kTurnLaneSharpRight) ||
                              (*it & kTurnLaneSlightRight))) {
        if (endOnTurn)
          bUpdated = true;
        else
          bUpdated = false; // should end on a through
        break;
      } else { // anything else then no update.
        bUpdated = false;
        break;
      }
    }
  } else {
    std::vector<uint16_t>::reverse_iterator r_it = enhanced_tls.rbegin(); // note reverse iterator
    for (; r_it != enhanced_tls.rend(); r_it++) {
      if (((*r_it & kTurnLaneRight) || (*r_it & kTurnLaneSharpRight) ||
           (*r_it & kTurnLaneSlightRight) || (*r_it & kTurnLaneThrough)) &&
          (previous == 0u || (previous & kTurnLaneRight) || (previous & kTurnLaneSharpRight) ||
           (previous & kTurnLaneSlightRight) || (previous & kTurnLaneThrough))) {
        previous = *r_it;
      } else if (previous && (*r_it == kTurnLaneEmpty || *r_it == kTurnLaneNone)) {
        *r_it = kTurnLaneThrough;
        bUpdated = true;
      } else { // if it is anything else then no update.
        bUpdated = false;
        break;
      }
    }
  }
  return bUpdated;
}

// Enhance the Turn lanes (if needed) and add them to the tile.
void UpdateTurnLanes(const OSMData& osmdata,
                     const uint32_t idx,
                     DirectedEdge& directededge,
                     graph_tile_builder_ptr& tilebuilder,
                     GraphReader& reader,
                     std::mutex& lock,
                     std::vector<TurnLanes>& turn_lanes) {

  // Lambda to check if the turn set includes a right turn type
  const auto has_turn_right = [](std::set<Turn::Type>& turn_types) {
    return turn_types.find(Turn::Type::kSlightRight) != turn_types.end() ||
           turn_types.find(Turn::Type::kRight) != turn_types.end() ||
           turn_types.find(Turn::Type::kSharpRight) != turn_types.end();
  };
  // Lambda to check if the turn set includes a left turn type
  const auto has_turn_left = [](std::set<Turn::Type>& turn_types) {
    return turn_types.find(Turn::Type::kSlightLeft) != turn_types.end() ||
           turn_types.find(Turn::Type::kLeft) != turn_types.end() ||
           turn_types.find(Turn::Type::kSharpLeft) != turn_types.end();
  };

  if (directededge.turnlanes()) {
    auto index = tilebuilder->turnlanes_offset(idx);
    std::string turnlane_tags = osmdata.name_offset_map.name(index);
    std::string str = TurnLanes::GetTurnLaneString(turnlane_tags);
    std::vector<uint16_t> enhanced_tls = TurnLanes::lanemasks(str);

    bool bUpdated = false;
    // handle [left, none, none, right] --> [left, straight, straight, right]
    // handle [straight, none, [straight, right], right] --> [straight, straight, [straight, right],
    // right]
    bUpdated = ProcessLanes(true, true, enhanced_tls);

    if (!bUpdated) {
      // handle [left, [straight, left], none, straight] --> [left, [straight, left], straight,
      // straight]
      // handle [left, none, none] --> [left, straight, straight]
      enhanced_tls = TurnLanes::lanemasks(str);

      std::set<Turn::Type> outgoing_turn_type;
      GetTurnTypes(directededge, outgoing_turn_type, tilebuilder, reader, lock);
      if (outgoing_turn_type.empty()) {
        directededge.set_turnlanes(false);
        return;
      }

      bUpdated = ProcessLanes(true, false, enhanced_tls);

      if (bUpdated) {
        // Should have a left.
        if (has_turn_left(outgoing_turn_type)) {
          // check for a right.
          if (!directededge.start_restriction())
            EnhanceRightLane(directededge, tilebuilder, reader, lock, enhanced_tls);
        }
      }
    }

    if (!bUpdated) {
      // handle [none, none, right] --> [straight, straight, right]
      enhanced_tls = TurnLanes::lanemasks(str);
      if (((enhanced_tls.back() & kTurnLaneRight) || (enhanced_tls.back() & kTurnLaneSharpRight) ||
           (enhanced_tls.back() & kTurnLaneSlightRight)) &&
          (enhanced_tls.front() == kTurnLaneEmpty || enhanced_tls.front() == kTurnLaneNone)) {

        std::set<Turn::Type> outgoing_turn_type;
        GetTurnTypes(directededge, outgoing_turn_type, tilebuilder, reader, lock);
        if (outgoing_turn_type.empty()) {
          directededge.set_turnlanes(false);
          return;
        }

        bUpdated = ProcessLanes(false, false, enhanced_tls);

        if (bUpdated) {
          // Should have a right.  check for a left.
          if (has_turn_right(outgoing_turn_type)) {
            // check for a left
            if (!directededge.start_restriction())
              EnhanceLeftLane(directededge, tilebuilder, reader, lock, enhanced_tls);
          }
        }
      }
    }

    if (!bUpdated) {
      // handle [straight, straight, none] --> [straight, straight, straight]
      enhanced_tls = TurnLanes::lanemasks(str);
      if ((enhanced_tls.front() & kTurnLaneThrough) &&
          (enhanced_tls.back() == kTurnLaneEmpty || enhanced_tls.back() == kTurnLaneNone)) {
        uint16_t previous = 0u;
        for (auto it = enhanced_tls.begin(); it != enhanced_tls.end(); it++) {
          if ((*it & kTurnLaneThrough) && (previous == 0u || (previous & kTurnLaneThrough))) {
            previous = *it;
          } else if (previous && (*it == kTurnLaneEmpty || *it == kTurnLaneNone)) {
            *it = kTurnLaneThrough;
            bUpdated = true;
          } else { // if it is anything else then no update.
            bUpdated = false;
            break;
          }
        }

        if (bUpdated && !directededge.start_restriction()) {
          // check for a right.
          EnhanceRightLane(directededge, tilebuilder, reader, lock, enhanced_tls);
          // check for a left
          EnhanceLeftLane(directededge, tilebuilder, reader, lock, enhanced_tls);
        }
      }
    }

    if (!bUpdated) {
      // handle [none, straight, straight] --> [straight, straight, straight]
      enhanced_tls = TurnLanes::lanemasks(str);
      uint16_t previous = 0u;
      if ((enhanced_tls.back() & kTurnLaneThrough) &&
          (enhanced_tls.front() == kTurnLaneEmpty || enhanced_tls.front() == kTurnLaneNone)) {
        for (auto r_it = enhanced_tls.rbegin(); r_it != enhanced_tls.rend(); r_it++) {
          if ((enhanced_tls.back() & kTurnLaneThrough) &&
              (previous == 0u || (previous & kTurnLaneThrough))) {
            previous = *r_it;
          } else if (previous && (*r_it == kTurnLaneEmpty || *r_it == kTurnLaneNone)) {
            *r_it = kTurnLaneThrough;
            bUpdated = true;
          } else { // if it is anything else then no update.
            bUpdated = false;
            break;
          }
        }

        if (bUpdated && !directededge.start_restriction()) {
          // check for a right.
          EnhanceRightLane(directededge, tilebuilder, reader, lock, enhanced_tls);
          // check for a left
          EnhanceLeftLane(directededge, tilebuilder, reader, lock, enhanced_tls);
        }
      }
    }
    // if anything was updated, we have to get the new string from the updated vector.
    if (bUpdated)
      str = TurnLanes::GetTurnLaneString(TurnLanes::turnlane_string(enhanced_tls));

    uint32_t offset = tilebuilder->AddName(str);
    turn_lanes.emplace_back(idx, offset);
  }
}

#ifdef UNREACHABLE
// TODO - may want to keep this to add to a postprocess to find unreachable areas

// Number of iterations to try to determine if an edge is unreachable
// by driving. If a search terminates before this without reaching
// a secondary road then the edge is considered unreachable.
constexpr uint32_t kUnreachableIterations = 20;

/**
 * Tests if the directed edge is unreachable by driving. If a driveable
 * edge cannot reach higher class roads and a search cannot expand after
 * a set number of iterations the edge is considered unreachable.
 * @param  reader        Graph reader
 * @param  lock          Mutex for locking while tiles are retrieved
 * @param  directededge  Directed edge to test.
 * @return  Returns true if the edge is found to be unreachable.
 */
bool IsUnreachable(GraphReader& reader, std::mutex& lock, DirectedEdge& directededge) {
  // Only check driveable edges. If already on a higher class road consider
  // the edge reachable
  if (!(directededge.forwardaccess() & kAutoAccess) ||
      directededge.classification() < RoadClass::kTertiary) {
    return false;
  }

  // Add the end node to the expand list
  std::unordered_set<GraphId> visitedset; // Set of visited nodes
  std::unordered_set<GraphId> expandset;  // Set of nodes to expand
  expandset.insert(directededge.endnode());

  // Expand until we either find a tertiary or higher classification,
  // expand more than kUnreachableIterations nodes, or cannot expand
  // any further. To reduce calls to lock and unlock keep a record of
  // current tile and only read a new tile when needed.
  uint32_t n = 0;
  GraphId prior_tile;
  graph_tile_ptr tile;
  while (n < kUnreachableIterations) {
    if (expandset.empty()) {
      // Have expanded all nodes without reaching a higher classification
      // driveable road - consider this unreachable
      return true;
    }

    // Get all driveable edges from the node on the expandlist
    const GraphId expandnode = *expandset.cbegin();
    expandset.erase(expandset.begin());
    visitedset.insert(expandnode);
    if (expandnode.Tile_Base() != prior_tile) {
      lock.lock();
      tile = reader.GetGraphTile(expandnode);
      lock.unlock();
      prior_tile = expandnode.Tile_Base();
    }
    const NodeInfo* nodeinfo = tile->node(expandnode);
    const DirectedEdge* diredge = tile->directededge(nodeinfo->edge_index());
    for (uint32_t i = 0; i < nodeinfo->edge_count(); i++, diredge++) {
      if ((diredge->forwardaccess() & kAutoAccess)) {
        if (diredge->classification() < RoadClass::kTertiary) {
          return false;
        }

        if (visitedset.find(diredge->endnode()) == visitedset.end()) {
          // Add to the expand set if not in the visited set
          expandset.insert(diredge->endnode());
        }
      }
    }
    n++;
  }
  return false;
}
#endif

// Test if this is a "not thru" edge. These are edges that enter a region that
// has no exit other than the edge entering the region
bool IsNotThruEdge(GraphReader& reader,
                   std::mutex& lock,
                   const GraphId& startnode,
                   DirectedEdge& directededge) {
  // Add the end node to the expand list
  std::unordered_set<GraphId> visitedset; // Set of visited nodes
  std::vector<GraphId> expandset; // Set of nodes to expand - uses a vector for cache friendliness

  // Pre-reserve space in the expandset.  Most of the time, we'll
  // expand fewer nodes than this, so we'll avoid vector reallocation
  // delays.  If a particular startnode expands further than this, it's
  // fine, it'll just be a little slower because the expandset
  // vector will need to grow.
  constexpr int MAX_EXPECTED_OUTGOING_EDGES = 10;
  expandset.reserve(kMaxNoThruTries * MAX_EXPECTED_OUTGOING_EDGES);

  // Instead of removing elements from the expandset, we'll just keep
  // a pointer to the "start" position - it's faster to just leave
  // the nodes we've visited in memory at the start of the vector
  // than to constantly keep the vector "correct" by removing items.
  // Because we know that the expandset is of relatively small limited size,
  // this approach is faster than the more generic erase(begin()) option
  std::size_t expand_pos = 0;
  expandset.push_back(directededge.endnode());

  // Expand edges until exhausted, the maximum number of expansions occur,
  // or end up back at the starting node. No node can be visited twice.
  // To reduce calls to lock and unlock keep a record of current tile and
  // only read a new tile when needed.
  GraphId prior_tile;
  graph_tile_ptr tile;
  for (uint32_t n = 0; n < kMaxNoThruTries; n++) {
    // If expand list is exhausted this is "not thru"
    if (expand_pos == expandset.size()) {
      return true;
    }

    // Get the node off of the expand list and add it to the visited list.
    // Expand edges from this node.  Post-increment the index to the next
    // item.
    const GraphId expandnode = expandset[expand_pos++];
    visitedset.insert(expandnode);
    if (expandnode.Tile_Base() != prior_tile) {
      lock.lock();
      tile = reader.GetGraphTile(expandnode);
      lock.unlock();
      prior_tile = expandnode.Tile_Base();
    }
    const NodeInfo* nodeinfo = tile->node(expandnode);
    const DirectedEdge* diredge = tile->directededge(nodeinfo->edge_index());
    for (uint32_t i = 0; i < nodeinfo->edge_count(); i++, diredge++) {
      // Do not allow use of the opposing start edge. Check more than just
      // endnode since many simple, 2-edge loops would have 2 edges coming
      // back to the same endnode
      if (n == 0 && diredge->endnode() == startnode &&
          diredge->forwardaccess() == directededge.reverseaccess() &&
          diredge->reverseaccess() == directededge.forwardaccess() &&
          diredge->length() == directededge.length()) {
        if ((startnode.tileid() == expandnode.tileid()) &&
            diredge->edgeinfo_offset() == directededge.edgeinfo_offset()) {
          continue;
        }
      }

      // Return false if we get back to the start node or hit an
      // edge with higher classification
      if (diredge->classification() < RoadClass::kTertiary || diredge->endnode() == startnode) {
        return false;
      }

      // Add to the end node to expand set if not already visited set
      if (visitedset.find(diredge->endnode()) == visitedset.end()) {
        expandset.push_back(diredge->endnode());
      }
    }
  }
  return false;
}

// Process stop and yields where a stop sign exists at an intersection node.
void SetStopYieldSignInfo(const graph_tile_ptr& start_tile,
                          GraphReader& reader,
                          std::mutex& lock,
                          const NodeInfo& startnodeinfo,
                          DirectedEdge& directededge) {

  if (directededge.stop_sign() || directededge.yield_sign()) {
    uint32_t ntrans = startnodeinfo.local_edge_count();
    for (uint32_t k = 0; k < ntrans; k++) {
      const DirectedEdge* fromedge = start_tile->directededge(startnodeinfo.edge_index() + k);
      // get the temporarily set deadend flag to indicate if the stop or yield should be at the
      // minor roads
      if (directededge.deadend()) {
        // if we are a higher class road unset the yield or stop
        if (fromedge->classification() > directededge.classification() ||
            (fromedge->classification() == directededge.classification() &&
             (fromedge->use() == Use::kRamp || fromedge->use() == Use::kTurnChannel))) {

          directededge.set_stop_sign(false);
          directededge.set_yield_sign(false);
          directededge.set_deadend(false);
          return;
        }
      }
    }

    if (!(directededge.forwardaccess() & kAutoAccess)) {
      directededge.set_stop_sign(false);
      directededge.set_yield_sign(false);
      directededge.set_deadend(false);
      return;
    }
  }

  // Get the tile at the startnode
  graph_tile_ptr tile = start_tile;
  // Get the tile at the end node
  if (tile->id() != directededge.endnode().Tile_Base()) {
    lock.lock();
    tile = reader.GetGraphTile(directededge.endnode());
    lock.unlock();
  }
  const NodeInfo* nodeinfo = tile->node(directededge.endnode());
  if (nodeinfo->transition_index()) {

    bool minor = (nodeinfo->transition_index() & kMinor);
    bool stop = (nodeinfo->transition_index() & kStopSign);
    bool yield = (nodeinfo->transition_index() & kYieldSign);
    bool inbound = false;
    RoadClass rc = directededge.classification();

    if (stop || yield) {

      // Iterate through inbound edges
      const DirectedEdge* diredge = tile->directededge(nodeinfo->edge_index());
      for (uint32_t i = 0; i < nodeinfo->edge_count(); i++, diredge++) {
        // Skip the candidate directed edge and any non-road edges. Skip any edges
        // that are not driveable inbound.
        if (!diredge->is_road() || !(diredge->reverseaccess() & kAutoAccess)) {
          continue;
        }

        if (minor) {
          if (rc > diredge->classification()) {
            rc = diredge->classification();
          }
        }
      }

      if (minor) {
        if ((directededge.forwardaccess() & kAutoAccess) &&
            (directededge.classification() > rc ||
             (directededge.classification() == rc &&
              (directededge.use() == Use::kRamp || directededge.use() == Use::kTurnChannel)))) {
          directededge.set_stop_sign(stop);
          directededge.set_yield_sign(yield);
        }
      } else if (directededge.forwardaccess() & kAutoAccess) {
        directededge.set_stop_sign(stop);
        directededge.set_yield_sign(yield);
      }
    }
  }
  // remove the temporarily set deadend flag
  directededge.set_deadend(false);
}

// Test if the edge is internal to an intersection.
bool IsIntersectionInternal(const graph_tile_ptr& start_tile,
                            GraphReader& reader,
                            std::mutex& lock,
                            const NodeInfo& startnodeinfo,
                            const DirectedEdge& directededge,
                            const uint32_t idx) {
  // Internal intersection edges must be short and cannot be a roundabout.
  // Also they must be a road use (not footway, cycleway, etc.).
  // TODO - consider whether alleys, cul-de-sacs, and other road uses
  // are candidates to be marked as internal intersection edges.
  // Returns false if any connecting edge is a roundabout.
  if (directededge.length() > kMaxInternalLength || directededge.roundabout() ||
      directededge.use() > Use::kCycleway) {
    return false;
  }

  // Lambda to check if the turn set includes a right turn type
  const auto has_turn_right = [](std::set<Turn::Type>& turn_types) {
    return turn_types.find(Turn::Type::kRight) != turn_types.end() ||
           turn_types.find(Turn::Type::kSharpRight) != turn_types.end();
  };
  // Lambda to check if the turn set includes a left turn type
  const auto has_turn_left = [](std::set<Turn::Type>& turn_types) {
    return turn_types.find(Turn::Type::kLeft) != turn_types.end() ||
           turn_types.find(Turn::Type::kSharpLeft) != turn_types.end();
  };

  // Get the tile at the startnode
  graph_tile_ptr tile = start_tile;

  // Exclude trivial "loops" where only 2 edges at start of candidate edge and
  // the end of the candidate edge is the start of the incoming edge to the
  // candidate
  if (startnodeinfo.edge_count() == 2) {
    const DirectedEdge* diredge = tile->directededge(startnodeinfo.edge_index());
    for (uint32_t i = 0; i < startnodeinfo.edge_count(); i++, diredge++) {
      // This is a loop if the non-candidate edge ends at the end node of the
      // candidate directed edge
      if (i != idx && diredge->endnode() == directededge.endnode()) {
        return false;
      }
    }
  }

  // Iterate through inbound edges and get turn degrees from driveable inbound
  // edges onto the candidate edge.
  bool oneway_inbound = false;
  uint32_t heading = startnodeinfo.heading(idx);
  std::set<Turn::Type> incoming_turn_type;
  const DirectedEdge* diredge = tile->directededge(startnodeinfo.edge_index());
  for (uint32_t i = 0; i < startnodeinfo.edge_count(); i++, diredge++) {
    // Skip the candidate directed edge and any non-road edges. Skip any edges
    // that are not driveable inbound.
    if (i == idx || !diredge->is_road() || !(diredge->reverseaccess() & kAutoAccess)) {
      continue;
    }

    // Return false if this is a roundabout connection.
    if (diredge->roundabout()) {
      return false;
    }

    // Store the turn type of incoming driveable edges.
    uint32_t from_heading = ((startnodeinfo.heading(i) + 180) % 360);
    uint32_t turndegree = GetTurnDegree(from_heading, heading);
    incoming_turn_type.insert(Turn::GetType(turndegree));

    // Flag if this is oneway, not a link, and not nearly straight turn
    // onto candidate edge.
    if (!(diredge->forwardaccess() & kAutoAccess) && !diredge->link() &&
        !(turndegree < 30 || turndegree > 330)) {
      oneway_inbound = true;
    }
  }

  // Must have an inbound oneway, excluding edges that are nearly straight
  // turn type onto the directed edge.
  if (!oneway_inbound) {
    return false;
  }

  // Get the tile at the end node. and find inbound heading of the candidate
  // edge to the end node.
  if (tile->id() != directededge.endnode().Tile_Base()) {
    lock.lock();
    tile = reader.GetGraphTile(directededge.endnode());
    lock.unlock();
  }
  const NodeInfo* node = tile->node(directededge.endnode());
  diredge = tile->directededge(node->edge_index());
  for (uint32_t i = 0; i < node->edge_count(); i++, diredge++) {
    // Find the opposing directed edge and its heading
    if (i == directededge.opp_local_idx()) {
      auto shape = tile->edgeinfo(diredge).shape();
      if (!diredge->forward()) {
        std::reverse(shape.begin(), shape.end());
      }
      uint32_t hdg = std::round(
          PointLL::HeadingAlongPolyline(shape, GetOffsetForHeading(diredge->classification(),
                                                                   diredge->use())));

      // Convert to inbound heading
      heading = ((hdg + 180) % 360);
      break;
    }
  }

  // Iterate through outbound edges and get turn degrees from the candidate
  // edge onto outbound driveable edges.
  bool oneway_outbound = false;
  std::set<Turn::Type> outgoing_turn_type;
  diredge = tile->directededge(node->edge_index());
  for (uint32_t i = 0; i < node->edge_count(); i++, diredge++) {
    // Skip opposing directed edge and any edge that is not a road. Skip any
    // edges that are not driveable outbound.
    if (i == directededge.opp_local_idx() || !diredge->is_road() ||
        !(diredge->forwardaccess() & kAutoAccess)) {
      continue;
    }

    // Return false if this is a roundabout connection.
    if (diredge->roundabout()) {
      return false;
    }

    // Get the heading of the outbound edge (unfortunately GraphEnhancer may
    // not have yet computed and stored headings for this node).
    auto shape = tile->edgeinfo(diredge).shape();
    if (!diredge->forward()) {
      std::reverse(shape.begin(), shape.end());
    }
    uint32_t to_heading =
        std::round(PointLL::HeadingAlongPolyline(shape, GetOffsetForHeading(diredge->classification(),
                                                                            diredge->use())));

    // Store outgoing turn type for any driveable edges
    uint32_t turndegree = GetTurnDegree(heading, to_heading);
    outgoing_turn_type.insert(Turn::GetType(turndegree));

    // Flag if this is oneway, not a link, and not nearly straight turn
    // from the candidate edge.
    if (!(diredge->reverseaccess() & kAutoAccess) && !diredge->link() &&
        !(turndegree < 30 || turndegree > 330)) {
      oneway_outbound = true;
    }
  }

  // Must have outbound oneway at end node (exclude edges that are nearly
  // straight turn from directed edge
  if (!oneway_outbound) {
    return false;
  }

  // A further rejection case is if there are incoming edges that
  // have "opposite" turn degrees than outgoing edges or if the outgoing
  // edges have opposing turn degrees.
  if ((has_turn_left(incoming_turn_type) && has_turn_right(outgoing_turn_type)) ||
      (has_turn_right(incoming_turn_type) && has_turn_left(outgoing_turn_type)) ||
      (has_turn_left(outgoing_turn_type) && has_turn_right(outgoing_turn_type))) {
    return false;
  }

  // TODO - determine if we need to add name checks or need to check headings
  // of the inbound and outbound oneway edges

  // Assume this is an intersection internal edge
  return true;
}

/**
 * Get the road density around the specified lat,lng position. This is a
 * value from 0-15 indicating a relative road density. This can be used
 * in costing methods to help avoid dense, urban areas.
 * @param  reader        Graph reader
 * @param  lock          Mutex for locking while tiles are retrieved
 * @param  ll            Lat,lng position
 * @param  maxdensity    (OUT) max density found
 * @param  tiles         Tiling (for getting list of required tiles)
 * @param  local_level   Level of the local tiles.
 * @return  Returns the relative road density (0-15) - higher values are
 *          more dense.
 */
uint32_t GetDensity(GraphReader& reader,
                    std::mutex& lock,
                    const PointLL& ll,
                    enhancer_stats& stats,
                    const Tiles<PointLL>& tiles,
                    uint8_t local_level) {
  // Radius is in km - turn into meters
  float rm = kDensityRadius * kMetersPerKm;
  float mr2 = rm * rm;

  // Use distance approximator for all distance checks
  DistanceApproximator<PointLL> approximator(ll);

  // Get a list of tiles required for a node search within this radius
  float lngdeg = (rm / DistanceApproximator<PointLL>::MetersPerLngDegree(ll.lat()));
  AABB2<PointLL> bbox(Point2(ll.lng() - lngdeg, ll.lat() - kDensityLatDeg),
                      Point2(ll.lng() + lngdeg, ll.lat() + kDensityLatDeg));
  std::vector<int32_t> tilelist = tiles.TileList(bbox);

  // For all tiles needed to find nodes within the radius...find nodes within
  // the radius (squared) and add lengths of directed edges
  float roadlengths = 0.0f;
  for (const auto t : tilelist) {
    // Check all the nodes within the tile. Skip if tile has no nodes (can be
    // an empty tile added for connectivity map logic).
    lock.lock();
    auto newtile = reader.GetGraphTile(GraphId(t, local_level, 0));
    lock.unlock();
    if (!newtile || newtile->header()->nodecount() == 0) {
      continue;
    }
    PointLL base_ll = newtile->header()->base_ll();
    const auto start_node = newtile->node(0);
    const auto end_node = start_node + newtile->header()->nodecount();
    for (auto node = start_node; node < end_node; ++node) {
      // Check if within radius
      if (approximator.DistanceSquared(node->latlng(base_ll)) < mr2) {
        // Get all directed edges and add length
        const DirectedEdge* directededge = newtile->directededge(node->edge_index());
        for (uint32_t i = 0; i < node->edge_count(); i++, directededge++) {
          // Exclude non-roads (parking, walkways, ferries, construction, etc.)
          if (directededge->is_road() || directededge->use() == Use::kRamp ||
              directededge->use() == Use::kTurnChannel || directededge->use() == Use::kAlley ||
              directededge->use() == Use::kEmergencyAccess) {
            roadlengths += directededge->length();
          }
        }
      }
    }
  }

  // Form density measure as km/km^2. Convert roadlengths to km and divide by 2
  // (since 2 directed edges per edge)
  float density = (roadlengths * 0.0005f) / (kPi * kDensityRadius2);
  if (density > stats.max_density) {
    stats.max_density = density;
  }

  // Convert density into a relative value from 0-16.
  uint32_t relative_density = std::round(density * 0.7f);
  if (relative_density > 15) {
    relative_density = 15;
  }
  stats.density_counts[relative_density]++;
  return relative_density;
}

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
    // Check the road if it is driveable TO the intersection and is neither
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
                            DirectedEdge& directededge,
                            const DirectedEdge* edges,
                            const uint32_t ntrans,
                            const NodeInfo& nodeinfo,
                            enhancer_stats& stats) {
  for (uint32_t i = 0; i < ntrans; i++) {
    // Get the turn type (reverse the heading of the from directed edge since
    // it is incoming
    uint32_t from_heading = ((nodeinfo.heading(i) + 180) % 360);
    uint32_t turn_degree = GetTurnDegree(from_heading, nodeinfo.heading(idx));
    directededge.set_turntype(i, Turn::GetType(turn_degree));

    // Set the edge_to_left and edge_to_right flags
    uint32_t right_count = 0;
    uint32_t left_count = 0;
    if (ntrans > 2) {
      for (uint32_t j = 0; j < ntrans; ++j) {
        // Skip the from and to edges; also skip roads under construction
        if (j == i || j == idx || edges[j].use() == Use::kConstruction) {
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
uint32_t GetOpposingEdgeIndex(const graph_tile_ptr& endnodetile,
                              const GraphId& startnode,
                              const graph_tile_ptr& tile,
                              const DirectedEdge& edge) {
  // Get the nodeinfo at the end of the edge
  const NodeInfo* nodeinfo = endnodetile->node(edge.endnode().id());

  // Iterate through the directed edges and return when the end node matches the specified
  // node, the length matches, and the shape matches (or edgeinfo offset matches)
  const DirectedEdge* directededge = endnodetile->directededge(nodeinfo->edge_index());
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
  return kMaxEdgesPerNode;
}

bool ConsistentNames(const std::string& country_code,
                     const std::vector<std::pair<std::string, bool>>& names1,
                     const std::vector<std::pair<std::string, bool>>& names2) {
  std::unique_ptr<StreetNames> street_names1 = StreetNamesFactory::Create(country_code, names1);
  std::unique_ptr<StreetNames> street_names2 = StreetNamesFactory::Create(country_code, names2);

  // Flag as consistent names when neither has names!
  if (street_names1->empty() && street_names2->empty()) {
    return true;
  }

  // Return true (consistent) if the common base names are not empty
  return (!(street_names1->FindCommonBaseNames(*street_names2)->empty()));
}

// We make sure to lock on reading and writing because we dont want to race
// since difference threads, use for the tilequeue as well
void enhance(const boost::property_tree::ptree& pt,
             const OSMData& osmdata,
             const std::string& access_file,
             const boost::property_tree::ptree& hierarchy_properties,
             std::queue<GraphId>& tilequeue,
             std::mutex& lock,
             std::promise<enhancer_stats>& result) {

  auto less_than = [](const OSMAccess& a, const OSMAccess& b) { return a.way_id() < b.way_id(); };
  sequence<OSMAccess> access_tags(access_file, false);

  auto database = pt.get_optional<std::string>("admin");
  bool infer_internal_intersections =
      pt.get<bool>("data_processing.infer_internal_intersections", true);
  bool infer_turn_channels = pt.get<bool>("data_processing.infer_turn_channels", true);
  bool apply_country_overrides = pt.get<bool>("data_processing.apply_country_overrides", true);
  bool use_urban_tag = pt.get<bool>("data_processing.use_urban_tag", false);
  bool use_admin_db = pt.get<bool>("data_processing.use_admin_db", true);
  // Initialize the admin DB (if it exists)
  sqlite3* admin_db_handle = (database && use_admin_db) ? GetDBHandle(*database) : nullptr;
  if (!database && use_admin_db) {
    LOG_WARN("Admin db not found.  Not saving admin information.");
  } else if (!admin_db_handle && use_admin_db) {
    LOG_WARN("Admin db " + *database + " not found.  Not saving admin information.");
  }
  auto admin_conn = make_spatialite_cache(admin_db_handle);

  std::unordered_map<std::string, std::vector<int>> country_access =
      GetCountryAccess(admin_db_handle);

  // Local Graphreader
  GraphReader reader(hierarchy_properties);

  // Config driven speed assignment
  auto speeds_config = pt.get_optional<std::string>("default_speeds_config");
  SpeedAssigner speed_assigner(speeds_config);

  // Get some things we need throughout
  enhancer_stats stats{std::numeric_limits<float>::min(), 0, 0, 0, 0, 0, 0, {}};
  const auto& local_level = TileHierarchy::levels().back().level;
  const auto& tiles = TileHierarchy::levels().back().tiles;

  // Iterate through the tiles in the queue and perform enhancements
  while (true) {
    // Get the next tile Id from the queue and get writeable and readable
    // tile. Lock while we access the tile queue and get the tile.
    lock.lock();
    if (tilequeue.empty()) {
      lock.unlock();
      break;
    }
    GraphId tile_id = tilequeue.front();
    tilequeue.pop();

    // Get a readable tile.If the tile is empty, skip it. Empty tiles are
    // added where ways go through a tile but no end not is within the tile.
    // This allows creation of connectivity maps using the tile set,
    graph_tile_ptr tile = reader.GetGraphTile(tile_id);
    if (!tile || tile->header()->nodecount() == 0) {
      lock.unlock();
      continue;
    }

    // Tile builder - serialize in existing tile so we can add admin names
    graph_tile_builder_ptr tilebuilder{new GraphTileBuilder(reader.tile_dir(), tile_id, true, false)};
    lock.unlock();

    // this will be our updated list of restrictions.
    // need to do some conversions on weights; therefore, we must update
    // the restriction list.
    uint32_t ar_before = tilebuilder->header()->access_restriction_count();
    std::vector<AccessRestriction> access_restrictions;

    uint32_t tl_before = tilebuilder->header()->turnlane_count();
    std::vector<TurnLanes> turn_lanes;

    uint32_t id = tile_id.tileid();
    // First pass - update links (set use to ramp or turn channel) and
    // set opposing local index.
    for (uint32_t i = 0; i < tilebuilder->header()->nodecount(); i++) {
      GraphId startnode(id, local_level, i);
      NodeInfo& nodeinfo = tilebuilder->node_builder(i);

      // Get headings of the edges - set in NodeInfo. Set driveability info
      // on the node as well.
      uint32_t count = nodeinfo.edge_count();
      uint32_t ntrans = std::min(count, kNumberOfEdgeTransitions);
      if (ntrans == 0) {
        throw std::runtime_error("edge transitions set is empty");
      }

      // Headings must be done first so that we can check if a next edge is internal or not
      // while processing turn lanes.
      std::vector<uint32_t> heading(ntrans);
      nodeinfo.set_local_edge_count(ntrans);
      for (uint32_t j = 0; j < ntrans; j++) {
        DirectedEdge& directededge = tilebuilder->directededge_builder(nodeinfo.edge_index() + j);

        auto e_offset = tilebuilder->edgeinfo(&directededge);
        auto shape = e_offset.shape();
        if (!directededge.forward()) {
          std::reverse(shape.begin(), shape.end());
        }
        heading[j] = std::round(
            PointLL::HeadingAlongPolyline(shape, GetOffsetForHeading(directededge.classification(),
                                                                     directededge.use())));

        // Set heading in NodeInfo. TODO - what if 2 edges have nearly the
        // same heading - should one be "adjusted" so the relative direction
        // is maintained.
        nodeinfo.set_heading(j, heading[j]);

        // Set traversability for autos
        Traversability traversability;
        if (directededge.forwardaccess() & kAutoAccess) {
          traversability = (directededge.reverseaccess() & kAutoAccess) ? Traversability::kBoth
                                                                        : Traversability::kForward;
        } else {
          traversability = (directededge.reverseaccess() & kAutoAccess) ? Traversability::kBackward
                                                                        : Traversability::kNone;
        }
        nodeinfo.set_local_driveability(j, traversability);
      }

      for (uint32_t j = 0; j < nodeinfo.edge_count(); j++) {
        DirectedEdge& directededge = tilebuilder->directededge_builder(nodeinfo.edge_index() + j);

        // Get the tile at the end node
        graph_tile_ptr endnodetile;
        if (tile->id() == directededge.endnode().Tile_Base()) {
          endnodetile = tile;
        } else {
          lock.lock();
          endnodetile = reader.GetGraphTile(directededge.endnode());
          lock.unlock();
        }

        // If this edge is a link, update its use (potentially change short
        // links to turn channels)
        if (directededge.use() == Use::kTurnChannel) {
          stats.turnchannelcount++;
        } else if (directededge.use() == Use::kRamp) {
          stats.rampcount++;
        }

        // Set the opposing index on the local level
        directededge.set_opp_local_idx(
            GetOpposingEdgeIndex(endnodetile, startnode, tile, directededge));
      }
    }

    // Second pass - add admin information and edge transition information.
    PointLL base_ll = tilebuilder->header()->base_ll();
    for (uint32_t i = 0; i < tilebuilder->header()->nodecount(); i++) {
      GraphId startnode(id, local_level, i);
      NodeInfo& nodeinfo = tilebuilder->node_builder(i);

      // Get relative road density and local density if the urban tag is not set
      uint32_t density = 0;
      if (!use_urban_tag) {
        density = GetDensity(reader, lock, nodeinfo.latlng(base_ll), stats, tiles, local_level);
        nodeinfo.set_density(density);
      }

      uint32_t admin_index = nodeinfo.admin_index();
      // Set the country code
      std::string country_code = "";
      if (admin_index != 0) {
        country_code = tilebuilder->admins_builder(admin_index).country_iso();
      } else {
        ++stats.no_country_found;
      }

      // Go through directed edges and "enhance" directed edge attributes
      uint32_t driveable_count = 0;
      const DirectedEdge* edges = tilebuilder->directededges(nodeinfo.edge_index());
      for (uint32_t j = 0; j < nodeinfo.edge_count(); j++) {
        DirectedEdge& directededge = tilebuilder->directededge_builder(nodeinfo.edge_index() + j);

        auto e_offset = tilebuilder->edgeinfo(&directededge);
        std::string end_node_code = "";
        std::string end_node_state_code = "";
        uint32_t end_admin_index = 0;
        // TODO: why do we get this information again, we could use the begin node admin above instead
        // Get the tile at the end node
        graph_tile_ptr endnodetile;
        const Admin* admin = nullptr;
        if (tile->id() == directededge.endnode().Tile_Base()) {
          end_admin_index = tile->node(directededge.endnode().id())->admin_index();
          admin = tile->admin(end_admin_index);
        } else {
          lock.lock();
          endnodetile = reader.GetGraphTile(directededge.endnode());
          lock.unlock();
          end_admin_index = endnodetile->node(directededge.endnode().id())->admin_index();
          admin = endnodetile->admin(end_admin_index);
        }
        end_node_code = admin->country_iso();
        end_node_state_code = admin->state_iso();

        // only process country access logic if the iso country codes match.
        if (apply_country_overrides && country_code == end_node_code) {
          // country access logic.
          // if the (auto, bike, foot, etc) tag flag is set in the OSMAccess
          // struct, then this means that it has been set by a user of the
          // OpenStreetMap community.  Therefore, we will not override this tag with the
          // country defaults.  Otherwise, country specific access wins.
          // Currently, overrides only exist for Trunk RC and Uses below.
          OSMAccess target{e_offset.wayid()};
          std::unordered_map<std::string, std::vector<int>>::const_iterator country_iterator =
              country_access.find(country_code);
          if (admin_index != 0 && country_iterator != country_access.end() &&
              directededge.use() != Use::kFerry &&
              (directededge.classification() <= RoadClass::kPrimary ||
               directededge.use() == Use::kBridleway || directededge.use() == Use::kCycleway ||
               directededge.use() == Use::kFootway || directededge.use() == Use::kPath ||
               directededge.use() == Use::kPedestrian ||
               directededge.use() == Use::kPedestrianCrossing ||
               directededge.use() == Use::kSidewalk || directededge.use() == Use::kTrack)) {

            std::vector<int> access = country_access.at(country_code);
            // leaves tile flag indicates that we have an access record for this edge.
            // leaves tile flag is updated later to the real value.
            if (directededge.leaves_tile()) {
              sequence<OSMAccess>::iterator access_it = access_tags.find(target, less_than);
              if (access_it != access_tags.end()) {
                SetCountryAccess(directededge, access, access_it);
              } else {
                LOG_WARN("access tags not found for " + std::to_string(e_offset.wayid()));
              }
            } else {
              SetCountryAccess(directededge, access, target);
            }
            // motorroad default.  Only applies to RC <= kPrimary and has no country override.
            // We just use the defaults which is no bicycles, mopeds and no pedestrians.
            // leaves tile flag indicates that we have an access record for this edge.
            // leaves tile flag is updated later to the real value.
          } else if (country_iterator == country_access.end() &&
                     directededge.classification() <= RoadClass::kPrimary &&
                     directededge.leaves_tile()) {

            OSMAccess target{e_offset.wayid()};
            sequence<OSMAccess>::iterator access_it = access_tags.find(target, less_than);
            if (access_it != access_tags.end()) {
              const OSMAccess& access = access_it;
              if (access.motorroad_tag()) {
                uint32_t forward = directededge.forwardaccess();
                uint32_t reverse = directededge.reverseaccess();

                bool f_oneway_vehicle =
                    (((forward & kAutoAccess) && !(reverse & kAutoAccess)) ||
                     ((forward & kTruckAccess) && !(reverse & kTruckAccess)) ||
                     ((forward & kEmergencyAccess) && !(reverse & kEmergencyAccess)) ||
                     ((forward & kTaxiAccess) && !(reverse & kTaxiAccess)) ||
                     ((forward & kHOVAccess) && !(reverse & kHOVAccess)) ||
                     ((forward & kMopedAccess) && !(reverse & kMopedAccess)) ||
                     ((forward & kMotorcycleAccess) && !(reverse & kMotorcycleAccess)) ||
                     ((forward & kBusAccess) && !(reverse & kBusAccess)));

                bool r_oneway_vehicle =
                    ((!(forward & kAutoAccess) && (reverse & kAutoAccess)) ||
                     (!(forward & kTruckAccess) && (reverse & kTruckAccess)) ||
                     (!(forward & kEmergencyAccess) && (reverse & kEmergencyAccess)) ||
                     (!(forward & kTaxiAccess) && (reverse & kTaxiAccess)) ||
                     (!(forward & kHOVAccess) && (reverse & kHOVAccess)) ||
                     (!(forward & kMopedAccess) && (reverse & kMopedAccess)) ||
                     (!(forward & kMotorcycleAccess) && (reverse & kMotorcycleAccess)) ||
                     (!(forward & kBusAccess) && (reverse & kBusAccess)));

                bool f_oneway_bicycle = ((forward & kBicycleAccess) && !(reverse & kBicycleAccess));
                bool r_oneway_bicycle = (!(forward & kBicycleAccess) && (reverse & kBicycleAccess));

                bool f_oneway_pedestrian =
                    ((forward & kPedestrianAccess) && !(reverse & kPedestrianAccess));
                bool r_oneway_pedestrian =
                    (!(forward & kPedestrianAccess) && (reverse & kPedestrianAccess));

                // motorroad defaults remove ped, wheelchair, moped, and bike access.
                // still check for user tags via access.
                forward = GetAccess(forward,
                                    (forward & ~(kPedestrianAccess | kWheelchairAccess |
                                                 kMopedAccess | kBicycleAccess)),
                                    r_oneway_vehicle, r_oneway_bicycle, r_oneway_pedestrian, access);
                reverse = GetAccess(reverse,
                                    (reverse & ~(kPedestrianAccess | kWheelchairAccess |
                                                 kMopedAccess | kBicycleAccess)),
                                    f_oneway_vehicle, f_oneway_bicycle, f_oneway_pedestrian, access);

                directededge.set_forwardaccess(forward);
                directededge.set_reverseaccess(reverse);
              }
            }
          }
        }

        // Update driveable count (do this after country access logic)
        if ((directededge.forwardaccess() & kAutoAccess) ||
            (directededge.reverseaccess() & kAutoAccess)) {
          driveable_count++;
        }

        // Use::kPedestrian is really a kFootway
        if (directededge.use() == Use::kPedestrian) {
          directededge.set_use(Use::kFootway);
        }

        if (directededge.traffic_signal() && !(directededge.forwardaccess() & kAutoAccess)) {
          directededge.set_traffic_signal(
              false); // oneway edge...no need to have a traffic signal on the edge.
        }

        // Update the named flag
        std::vector<uint8_t> types;
        auto names = tilebuilder->edgeinfo(&directededge).GetNamesAndTypes(types, false);
        directededge.set_named(names.size() > 0);

        // Speed assignment
        speed_assigner.UpdateSpeed(directededge, density, infer_turn_channels, end_node_code,
                                   end_node_state_code);

        // Name continuity - on the directededge.
        uint32_t ntrans = nodeinfo.local_edge_count();
        for (uint32_t k = 0; k < ntrans; k++) {
          DirectedEdge& fromedge = tilebuilder->directededge(nodeinfo.edge_index() + k);
          std::vector<uint8_t> types;
          if (ConsistentNames(country_code, names,
                              tilebuilder->edgeinfo(&fromedge).GetNamesAndTypes(types, false))) {
            directededge.set_name_consistency(k, true);
          }
        }

        // Set edge transitions.
        if (j < kNumberOfEdgeTransitions) {
          ProcessEdgeTransitions(j, directededge, edges, ntrans, nodeinfo, stats);
        }

        // Test if an internal intersection edge. Must do this after setting
        // opposing edge index
        if (infer_internal_intersections &&
            IsIntersectionInternal(tilebuilder, reader, lock, nodeinfo, directededge, j)) {
          directededge.set_internal(true);
        }

        if (directededge.internal()) {
          stats.internalcount++;
        }

        SetStopYieldSignInfo(tilebuilder, reader, lock, nodeinfo, directededge);

        // Enhance and add turn lanes if not an internal edge.
        if (directededge.turnlanes()) {
          // Update turn lanes.
          UpdateTurnLanes(osmdata, nodeinfo.edge_index() + j, directededge, tilebuilder, reader, lock,
                          turn_lanes);
        }

        // Check for not_thru edge (only on low importance edges). Exclude
        // transit edges
        if (directededge.classification() > RoadClass::kTertiary) {
          if (IsNotThruEdge(reader, lock, startnode, directededge)) {
            directededge.set_not_thru(true);
            stats.not_thru++;
          }
        }

        // Update access restrictions (update weight units)
        if (directededge.access_restriction()) {
          auto restrictions =
              tilebuilder->GetAccessRestrictions(nodeinfo.edge_index() + j, kAllAccess);

          // Convert any US weight values from short ton (U.S. customary)
          // to metric and add to the tile's access restriction list
          if (country_code == "US" || country_code == "MM" || country_code == "LR") {
            for (auto& res : restrictions) {
              if (res.type() == AccessType::kMaxWeight || res.type() == AccessType::kMaxAxleLoad) {
                res.set_value(std::round(res.value() * kTonsShortToMetric));
              }
            }
          }
          for (const auto& res : restrictions) {
            access_restrictions.emplace_back(std::move(res));
          }
        }
      }

      // Set the intersection type to false or dead-end (do not override
      // gates or toll-booths or toll gantry or sump buster).
      if (nodeinfo.type() != NodeType::kGate && nodeinfo.type() != NodeType::kTollBooth &&
          nodeinfo.type() != NodeType::kTollGantry && nodeinfo.type() != NodeType::kSumpBuster) {
        if (driveable_count == 1) {
          nodeinfo.set_intersection(IntersectionType::kDeadEnd);
        } else if (nodeinfo.edge_count() == 2) {
          nodeinfo.set_intersection(IntersectionType::kFalse);
        }
      }

      nodeinfo.set_transition_index(0);
    }

    // Replace access restrictions
    if (ar_before != access_restrictions.size()) {
      LOG_ERROR("Mismatch in access restriction count before " + std::to_string(ar_before) +
                ""
                " and after " +
                std::to_string(access_restrictions.size()) +
                " tileid = " + std::to_string(tile_id.tileid()));
    }
    tilebuilder->AddAccessRestrictions(access_restrictions);

    // Replace turnlanes
    if (tl_before != turn_lanes.size()) {
      LOG_TRACE("Mismatch in turn lane count before " + std::to_string(tl_before) +
                ""
                " and after " +
                std::to_string(turn_lanes.size()) + " tileid = " + std::to_string(tile_id.tileid()));
    }
    tilebuilder->AddTurnLanes(turn_lanes);

    // Write the new file
    lock.lock();
    tilebuilder->StoreTileData();
    LOG_TRACE((boost::format("GraphEnhancer completed tile %1%") % tile_id).str());

    // Check if we need to clear the tile cache
    if (reader.OverCommitted()) {
      reader.Trim();
    }
    lock.unlock();
  }

  if (admin_db_handle) {
    sqlite3_close(admin_db_handle);
  }

  // Send back the statistics
  result.set_value(stats);
}

} // namespace

namespace valhalla {
namespace mjolnir {

// Enhance the local level of the graph
void GraphEnhancer::Enhance(const boost::property_tree::ptree& pt,
                            const OSMData& osmdata,
                            const std::string& access_file) {
  LOG_INFO("Enhancing local graph...");

  // A place to hold worker threads and their results, exceptions or otherwise
  std::vector<std::shared_ptr<std::thread>> threads(
      std::max(static_cast<unsigned int>(1),
               pt.get<unsigned int>("mjolnir.concurrency", std::thread::hardware_concurrency())));

  // A place to hold the results of those threads, exceptions or otherwise
  std::list<std::promise<enhancer_stats>> results;

  // Create a randomized queue of tiles to work from
  std::deque<GraphId> tempqueue;
  boost::property_tree::ptree hierarchy_properties = pt.get_child("mjolnir");
  auto local_level = TileHierarchy::levels().back().level;
  GraphReader reader(hierarchy_properties);
  auto local_tiles = reader.GetTileSet(local_level);
  for (const auto& tile_id : local_tiles) {
    tempqueue.emplace_back(tile_id);
  }
  std::random_device rd;
  std::shuffle(tempqueue.begin(), tempqueue.end(), std::mt19937(rd()));
  std::queue<GraphId> tilequeue(tempqueue);

  // An atomic object we can use to do the synchronization
  std::mutex lock;

  // Start the threads
  for (auto& thread : threads) {
    results.emplace_back();
    thread.reset(new std::thread(enhance, std::cref(hierarchy_properties), std::cref(osmdata),
                                 std::cref(access_file), std::ref(hierarchy_properties),
                                 std::ref(tilequeue), std::ref(lock), std::ref(results.back())));
  }

  // Wait for them to finish up their work
  for (auto& thread : threads) {
    thread->join();
  }

  // Check all of the outcomes, to see about maximum density (km/km2)
  enhancer_stats stats{std::numeric_limits<float>::min(), 0, 0, 0, 0, 0, 0, {0}};
  for (auto& result : results) {
    // If something bad went down this will rethrow it
    try {
      auto thread_stats = result.get_future().get();
      stats(thread_stats);
    } catch (std::exception& e) {
      // TODO: throw further up the chain?
    }
  }

  LOG_INFO("Finished with max_density " + std::to_string(stats.max_density));
  LOG_DEBUG("not_thru = " + std::to_string(stats.not_thru));
  LOG_DEBUG("no country found = " + std::to_string(stats.no_country_found));
  LOG_INFO("internal intersection = " + std::to_string(stats.internalcount));
  LOG_DEBUG("Turn Channel Count = " + std::to_string(stats.turnchannelcount));
  LOG_DEBUG("Ramp Count = " + std::to_string(stats.rampcount));
  LOG_DEBUG("Pencil Point Uturn count = " + std::to_string(stats.pencilucount));
#ifdef LOGGING_LEVEL_DEBUG
  for (auto density : stats.density_counts) {
    LOG_DEBUG("Density: " + std::to_string(density));
  }
#endif
}

} // namespace mjolnir
} // namespace valhalla
