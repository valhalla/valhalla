#include "mjolnir/graphenhancer.h"
#include "baldr/graphconstants.h"
#include "baldr/graphid.h"
#include "baldr/graphreader.h"
#include "baldr/graphtile.h"
#include "baldr/streetnames.h"
#include "baldr/streetnames_factory.h"
#include "baldr/tilehierarchy.h"
#include "midgard/aabb2.h"
#include "midgard/constants.h"
#include "midgard/logging.h"
#include "midgard/pointll.h"
#include "midgard/sequence.h"
#include "midgard/util.h"
#include "mjolnir/admin.h"
#include "mjolnir/countryaccess.h"
#include "mjolnir/graphtilebuilder.h"
#include "mjolnir/osmaccess.h"
#include "mjolnir/util.h"
#include "scoped_timer.h"
#include "speed_assigner.h"

#include <ankerl/unordered_dense.h>

#include <cstdint>
#include <functional>
#include <future>
#include <limits>
#include <list>
#include <memory>
#include <mutex>
#include <queue>
#include <random>
#include <set>
#include <stdexcept>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::mjolnir;

namespace {

// Number of tries when determining not thru edges
constexpr uint32_t kMaxNoThruTries = 256;

// Radius (km) to use for density
constexpr float kDensityRadius = 2.0f;
constexpr float kDensityLatDeg = (kDensityRadius * kMetersPerKm) / kMetersPerDegreeLat;

struct DensityCellId {
  // `cell_id` is computed by shifting coordinate values by this amount, meaning that a bigger
  // level produces bigger cells (as fewer bits remain from coordinate).
  // - 15: 362m x 181m cell at 60 latitude, approx 7.5k cells to cover single tile
  // - 14: 181m x 90m cell at 60 latitude, approx 30k cells to cover single tile
  // - 13: 90m x 45m cell at 60 latitude, approx 120k cells to cover single tile
  static constexpr uint32_t kGridLevel = 14; // good compromise between performance and accuracy

  // Cell size in degrees
  static constexpr float kSizeDeg = static_cast<float>(1 << kGridLevel) / 1e7;

  // Count number of cells required to cover the given bounding box
  static size_t cover_count(const AABB2<PointLL>& bbox) {
    const size_t x_cells = static_cast<size_t>(floor(bbox.Width() / kSizeDeg)) + 1;
    const size_t y_cells = static_cast<size_t>(floor(bbox.Height() / kSizeDeg)) + 1;
    return x_cells * y_cells;
  }

  // If lat/lon is represented by 32 bit integer and coordinates are right-shifted by `kGridLevel`
  // to identify a cell, the minimum grid level that keeps cells globally unique is 16, as smaller
  // levels will lead to keeping too many coordinate bits that `id_` can hold. At the same time, as
  // density is being calculated for a single tile (0.25 deg size) and its neighbors, such global
  // uniqueness is not required, allowing to use such a small type for id.
  // Alternatively, `uint64_t` can be used for global uniqueness with levels smaller than 16.
  uint32_t id_;
  static constexpr uint32_t kComponentBits = sizeof(id_) * 8 / 2;
  static constexpr uint32_t kComponentMask = (1 << kComponentBits) - 1;

  DensityCellId(uint32_t id) : id_(id) {
  }

  DensityCellId(const PointLL& ll) {
    uint32_t x = static_cast<uint32_t>((ll.lng() + 180.0) * 1e7);
    uint32_t y = static_cast<uint32_t>((ll.lat() + 90.0) * 1e7);

    x = x >> kGridLevel;
    y = y >> kGridLevel;

    id_ = (x & kComponentMask) | ((y & kComponentMask) << kComponentBits);
  }

  // Neighboring cells ids within the given radius in degrees
  std::vector<DensityCellId> neighbors(float radius_lng_deg, float radius_lat_deg) const {
    const int x_neighbors = std::ceil(radius_lng_deg / DensityCellId::kSizeDeg);
    const int y_neighbors = std::ceil(radius_lat_deg / DensityCellId::kSizeDeg);

    // Convert cell_id back to x, y coordinates
    const int x = id_ & kComponentMask;
    const int y = (id_ >> kComponentBits) & kComponentMask;

    std::vector<DensityCellId> neighbors;
    neighbors.reserve(4 * x_neighbors * y_neighbors);
    for (int nx = -x_neighbors; nx <= x_neighbors; nx++) {
      for (int ny = -y_neighbors; ny <= y_neighbors; ny++) {
        // Keep only neighbors within the radius
        if (nx * nx + ny * ny <= x_neighbors * y_neighbors) {
          uint32_t neighbor_id =
              ((x + nx) & kComponentMask) | (((y + ny) & kComponentMask) << kComponentBits);
          neighbors.push_back(neighbor_id);
        }
      }
    }
    return neighbors;
  }

  bool operator==(const DensityCellId& other) const {
    return id_ == other.id_;
  }

  bool operator!=(const DensityCellId& other) const {
    return !(*this == other);
  }
};

using DensityIndex = ankerl::unordered_dense::map<DensityCellId, uint32_t>;
} // namespace

namespace std {
template <> struct hash<DensityCellId> {
  size_t operator()(const DensityCellId& cell) const {
    return cell.id_;
  }
};
} // namespace std

namespace {

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
  // edge onto outbound drivable edges.
  const auto* diredge = tile->directededge(node->edge_index());
  for (uint32_t i = 0; i < node->edge_count(); i++, diredge++) {
    // Skip opposing directed edge and any edge that is not a road. Skip any
    // edges that are not drivable outbound.
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
    // Store outgoing turn type for any drivable edges
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
 * Tests if the directed edge is unreachable by driving. If a drivable
 * edge cannot reach higher class roads and a search cannot expand after
 * a set number of iterations the edge is considered unreachable.
 * @param  reader        Graph reader
 * @param  lock          Mutex for locking while tiles are retrieved
 * @param  directededge  Directed edge to test.
 * @return  Returns true if the edge is found to be unreachable.
 */
bool IsUnreachable(GraphReader& reader, std::mutex& lock, DirectedEdge& directededge) {
  // Only check drivable edges. If already on a higher class road consider
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
      // drivable road - consider this unreachable
      return true;
    }

    // Get all drivable edges from the node on the expandlist
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
    RoadClass rc = directededge.classification();

    if (stop || yield) {

      // Iterate through inbound edges
      const DirectedEdge* diredge = tile->directededge(nodeinfo->edge_index());
      for (uint32_t i = 0; i < nodeinfo->edge_count(); i++, diredge++) {
        // Skip the candidate directed edge and any non-road edges. Skip any edges
        // that are not drivable inbound.
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

  // Iterate through inbound edges and get turn degrees from drivable inbound
  // edges onto the candidate edge.
  bool oneway_inbound = false;
  uint32_t heading = startnodeinfo.heading(idx);
  std::set<Turn::Type> incoming_turn_type;
  const DirectedEdge* diredge = tile->directededge(startnodeinfo.edge_index());
  for (uint32_t i = 0; i < startnodeinfo.edge_count(); i++, diredge++) {
    // Skip the candidate directed edge and any non-road edges. Skip any edges
    // that are not drivable inbound.
    if (i == idx || !diredge->is_road() || !(diredge->reverseaccess() & kAutoAccess)) {
      continue;
    }

    // Return false if this is a roundabout connection.
    if (diredge->roundabout()) {
      return false;
    }

    // Store the turn type of incoming drivable edges.
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
  // edge onto outbound drivable edges.
  bool oneway_outbound = false;
  std::set<Turn::Type> outgoing_turn_type;
  diredge = tile->directededge(node->edge_index());
  for (uint32_t i = 0; i < node->edge_count(); i++, diredge++) {
    // Skip opposing directed edge and any edge that is not a road. Skip any
    // edges that are not drivable outbound.
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

    // Store outgoing turn type for any drivable edges
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

float NodeRoadlengths(const graph_tile_ptr& tile, const NodeInfo* node) {
  float roadlengths = 0.0f;
  const DirectedEdge* directededge = tile->directededge(node->edge_index());
  for (uint32_t i = 0; i < node->edge_count(); i++, directededge++) {
    // Exclude non-roads (parking, walkways, ferries, construction, etc.)
    if (directededge->is_road() || directededge->use() == Use::kRamp ||
        directededge->use() == Use::kTurnChannel || directededge->use() == Use::kAlley ||
        directededge->use() == Use::kEmergencyAccess) {
      roadlengths += directededge->length();
    }
  }
  return roadlengths;
}

/**
 * Build a density index by accumulating edge lengths in each grid cell and convert
 * to the relative density values (0-15) used by the enhancer.
 * This dramatically speeds up density calculations by pre-computing densities.
 * @param reader        Graph reader
 * @param lock          Mutex for thread-safe tile access
 * @param tile          Current tile being processed
 * @param local_level   Local hierarchy level
 * @param stats         Reference to stats object to update max_density
 * @return              Density grid mapping cell IDs to relative density values (0-15)
 */
DensityIndex BuildDensityIndex(GraphReader& reader,
                               std::mutex& lock,
                               const graph_tile_ptr& tile,
                               const TileLevel& tile_level,
                               enhancer_stats& stats) {
  // To properly count density on the tile edges, the bbox should be extended by the density radius,
  // rounded up to the grid cell size to get fully filled edge cells
  const AABB2<PointLL> tile_bbox = tile->BoundingBox();
  const float lat_cos = cosf(kRadPerDeg * tile_bbox.Center().lat());
  const float density_lng_deg = kDensityLatDeg / lat_cos;
  const AABB2<PointLL> bbox(tile_bbox.minpt().lng() - (density_lng_deg + DensityCellId::kSizeDeg),
                            tile_bbox.minpt().lat() - (kDensityLatDeg + DensityCellId::kSizeDeg),
                            tile_bbox.maxpt().lng() + (density_lng_deg + DensityCellId::kSizeDeg),
                            tile_bbox.maxpt().lat() + (kDensityLatDeg + DensityCellId::kSizeDeg));

  // Grid where each cell contains a sum of lengths of all edges of all nodes within the cell
  ankerl::unordered_dense::map<DensityCellId, float> density_grid;
  density_grid.reserve(DensityCellId::cover_count(bbox));

  // Process current tile separately from neighbors to always have a cell for each node in the tile
  {
    const PointLL base_ll = tile->header()->base_ll();
    const auto start_node = tile->node(0);
    const auto end_node = start_node + tile->header()->nodecount();
    for (auto node = start_node; node < end_node; ++node) {
      density_grid[node->latlng(base_ll)] += NodeRoadlengths(tile, node);
    }
  }

  // process neighboring tiles
  for (const auto& t : tile_level.tiles.TileList(bbox)) {
    const GraphId tile_id(t, tile_level.level, 0);
    if (tile_id == tile->id()) {
      continue; // skip current tile as it was processed above
    }

    lock.lock();
    auto newtile = reader.GetGraphTile(tile_id);
    lock.unlock();
    if (!newtile || newtile->header()->nodecount() == 0) {
      continue;
    }

    const PointLL base_ll = newtile->header()->base_ll();
    const auto start_node = newtile->node(0);
    const auto end_node = start_node + newtile->header()->nodecount();
    for (auto node = start_node; node < end_node; ++node) {
      const PointLL node_ll = node->latlng(base_ll);
      if (bbox.Contains(node_ll)) {
        density_grid[node_ll] += NodeRoadlengths(newtile, node);
      }
    }
  }

  // The difference between cell areas at the top of the 0.25 tile vs cell at the bottom of that tile
  // reaches 1.2% at 70s latitude, which is acceptable here
  const float cell_size_km = DensityCellId::kSizeDeg * kMetersPerDegreeLat / kMetersPerKm;
  const float cell_area = cell_size_km * cell_size_km * lat_cos;

  // Now build the density index where each cell contains density value for nodes in that cell
  DensityIndex density_index;
  density_index.reserve(density_grid.size());
  for (const auto& [cell_id, _] : density_grid) {
    float roadlengths = 0.0f;

    const auto neighbors = cell_id.neighbors(density_lng_deg, kDensityLatDeg);
    for (const auto& neighbor : neighbors) {
      if (auto it = density_grid.find(neighbor); it != density_grid.end()) {
        roadlengths += it->second;
      }
    }

    // Form density measure as km/km^2. Convert roadlengths to km and divide by 2
    // (since 2 directed edges per edge)
    float density = (roadlengths * 0.0005f) / (cell_area * neighbors.size());
    if (density > stats.max_density) {
      stats.max_density = density;
    }

    // Convert to relative density (0-15)
    const uint32_t relative_value = std::round(density * 0.7f);
    density_index[cell_id] = std::min(relative_value, 15u);
  }

  return density_index;
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
  auto admin_db = (database && use_admin_db) ? AdminDB::open(*database) : std::optional<AdminDB>{};
  if (!database && use_admin_db) {
    LOG_WARN("Admin db not found. Not saving admin information.");
  } else if (!admin_db && use_admin_db) {
    LOG_WARN("Admin db " + *database + " not found. Not saving admin information.");
  }

  std::unordered_map<std::string, std::vector<int>> country_access;
  if (admin_db) {
    country_access = GetCountryAccess(*admin_db);
  }

  // Local Graphreader
  GraphReader reader(hierarchy_properties);

  // Config driven speed assignment
  auto speeds_config = pt.get_optional<std::string>("default_speeds_config");
  SpeedAssigner speed_assigner(speeds_config);

  // Get some things we need throughout
  enhancer_stats stats{std::numeric_limits<float>::min(), 0, 0, 0, 0, 0, 0, {}};
  const TileLevel& tile_level = TileHierarchy::levels().back();

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
      GraphId startnode(id, tile_level.level, i);
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

    // Get relative road density and local density if the urban tag is not set
    const auto density_index =
        !use_urban_tag ? BuildDensityIndex(reader, lock, tile, tile_level, stats) : DensityIndex{};

    // Second pass - add admin information and edge transition information.
    const PointLL base_ll = tilebuilder->header()->base_ll();
    for (uint32_t i = 0; i < tilebuilder->header()->nodecount(); i++) {
      GraphId startnode(id, tile_level.level, i);
      NodeInfo& nodeinfo = tilebuilder->node_builder(i);

      uint32_t density = 0;
      if (auto it = density_index.find(nodeinfo.latlng(base_ll)); it != density_index.end()) {
        density = it->second;
        stats.density_counts[density]++;
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
      uint32_t drivable_count = 0;
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

            const std::vector<int>& access = country_access.at(country_code);
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

        // Update drivable count (do this after country access logic)
        if ((directededge.forwardaccess() & kAutoAccess) ||
            (directededge.reverseaccess() & kAutoAccess)) {
          drivable_count++;
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
        auto names = e_offset.GetNames(false);
        directededge.set_named(names.size() > 0);

        // Speed assignment
        speed_assigner.UpdateSpeed(directededge, density, infer_turn_channels, end_node_code,
                                   end_node_state_code);

        // Name continuity - on the directededge.
        uint32_t ntrans = nodeinfo.local_edge_count();
        for (uint32_t k = 0; k < ntrans; k++) {
          DirectedEdge& fromedge = tilebuilder->directededge(nodeinfo.edge_index() + k);
          if (ConsistentNames(country_code, names,
                              tilebuilder->edgeinfo(&fromedge).GetNames(false))) {
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
        if (drivable_count == 1) {
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
    LOG_TRACE("GraphEnhancer completed tile " + std::to_string(tile_id));

    // Check if we need to clear the tile cache
    if (reader.OverCommitted()) {
      reader.Trim();
    }
    lock.unlock();
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
  SCOPED_TIMER();
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
    thread =
        std::make_shared<std::thread>(enhance, std::cref(hierarchy_properties), std::cref(osmdata),
                                      std::cref(access_file), std::ref(hierarchy_properties),
                                      std::ref(tilequeue), std::ref(lock), std::ref(results.back()));
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
