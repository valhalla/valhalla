#include "mjolnir/admin.h"
#include "mjolnir/graphenhancer.h"
#include "mjolnir/graphtilebuilder.h"
#include "mjolnir/countryaccess.h"

#include <memory>
#include <future>
#include <thread>
#include <mutex>
#include <vector>
#include <list>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <cinttypes>
#include <limits>

#include <sqlite3.h>
#include <spatialite.h>
#include <boost/filesystem/operations.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/multi/geometries/multi_polygon.hpp>
#include <boost/geometry/io/wkt/wkt.hpp>

#include "midgard/aabb2.h"
#include "midgard/constants.h"
#include "midgard/distanceapproximator.h"
#include "midgard/logging.h"
#include "midgard/pointll.h"
#include "midgard/sequence.h"
#include "midgard/util.h"
#include "baldr/tilehierarchy.h"
#include "baldr/graphid.h"
#include "baldr/graphconstants.h"
#include "baldr/graphtile.h"
#include "baldr/graphreader.h"
#include "baldr/streetnames.h"
#include "baldr/streetnames_factory.h"
#include "baldr/streetnames_us.h"
#include "baldr/admininfo.h"
#include "baldr/datetime.h"
#include "mjolnir/osmaccess.h"

using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::mjolnir;

namespace {

// Geometry types for admin queries
typedef boost::geometry::model::d2::point_xy<double> point_type;
typedef boost::geometry::model::polygon<point_type> polygon_type;
typedef boost::geometry::model::multi_polygon<polygon_type> multi_polygon_type;

// Number of iterations to try to determine if an edge is unreachable
// by driving. If a search terminates before this without reaching
// a secondary road then the edge is considered unreachable.
constexpr uint32_t kUnreachableIterations = 20;

// Number of tries when determining not thru edges
constexpr uint32_t kMaxNoThruTries = 256;

// Radius (km) to use for density
constexpr float kDensityRadius  = 2.0f;
constexpr float kDensityRadius2 = kDensityRadius * kDensityRadius;
constexpr float kDensityLatDeg  = (kDensityRadius * kMetersPerKm) /
                                      kMetersPerDegreeLat;

// Factors used to adjust speed assignments
constexpr float kTurnChannelFactor = 1.25f;
constexpr float kRampDensityFactor = 0.8f;
constexpr float kRampFactor        = 0.85f;

// A little struct to hold stats information during each threads work
struct enhancer_stats {
  float max_density; //(km/km2)
  uint32_t unreachable;
  uint32_t not_thru;
  uint32_t no_country_found;
  uint32_t internalcount;
  uint32_t turnchannelcount;
  uint32_t rampcount;
  uint32_t pencilucount;
  uint32_t density_counts[16];
  void operator()(const enhancer_stats& other) {
    if(max_density < other.max_density)
      max_density = other.max_density;
    unreachable += other.unreachable;
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
 * Update directed edge speed based on density and other edge parameters like
 * surface type. TODO - add admin specific logic
 * @param  directededge  Directed edge to update.
 * @param  density       Relative road density.
 * @param  urban_rc_speed Array of default speeds vs. road class for urban areas
 */
void UpdateSpeed(DirectedEdge& directededge, const uint32_t density,
                 const uint32_t* urban_rc_speed) {

  // Update speed on ramps (if not a tagged speed) and turn channels
  if (directededge.link()) {
    uint32_t speed = directededge.speed();
    Use use = directededge.use();
    if (use == Use::kTurnChannel) {
      speed = static_cast<uint32_t>((speed * kTurnChannelFactor) + 0.5f);
    } else if ((use == Use::kRamp)
        && (directededge.speed_type() != SpeedType::kTagged)) {
      // If no tagged speed set ramp speed to slightly lower than speed
      // for roads of this classification
      RoadClass rc = directededge.classification();
      if ((rc == RoadClass::kMotorway)
          || (rc == RoadClass::kTrunk)
          || (rc == RoadClass::kPrimary)) {
        speed = (density > 8) ?
            static_cast<uint32_t>((speed * kRampDensityFactor) + 0.5f) :
            static_cast<uint32_t>((speed * kRampFactor) + 0.5f);
      } else {
        speed = static_cast<uint32_t>((speed * kRampFactor) + 0.5f);
      }
    }
    directededge.set_speed(speed);

    // Done processing links so return...
    return;
  }

  // If speed is assigned from an OSM max_speed tag we only update it based
  // on surface type.
  if (directededge.speed_type() == SpeedType::kTagged) {
    // Reduce speed on rough pavements. TODO - do we want to increase
    // more on worse surface types?
    if (directededge.surface() >= Surface::kPavedRough) {
      uint32_t speed = directededge.speed();
      if (speed >= 50) {
         directededge.set_speed(speed - 10);
      } else if (speed > 15) {
        directededge.set_speed(speed - 5);
      }
    }
  } else {
    // Set speed on ferries. Base the speed on the length - assumes
    // that longer lengths generally use a faster ferry boat
    if (directededge.use() == Use::kRailFerry) {
      directededge.set_speed(65);   // 40 MPH
      return;
    } else if (directededge.use() == Use::kFerry) {
      // if duration flag is set do nothing with speed - currently set
      // as the leaves tile flag.
      // leaves tile flag is updated later to the real value.
      if (directededge.leaves_tile()) {
        return;
      } else if (directededge.length() < 2000) {
        directededge.set_speed(10);  // 5 knots
      } else if (directededge.length() < 8000) {
        directededge.set_speed(20);  // 10 knots
      } else {
        directededge.set_speed(30);  // 15 knots
      }
      return;
    }

    // Modify speed for roads in urban regions - anything above 8 (TBD) is
    // assumed to be urban
    if (density > 8) {
      uint32_t rc = static_cast<uint32_t>(directededge.classification());
      directededge.set_speed(urban_rc_speed[rc]);
    }

    // Modify speed based on surface.
    if (directededge.surface() >= Surface::kPavedRough) {
      uint32_t speed = directededge.speed();
      directededge.set_speed(speed / 2);
    }
  }
}

/**
 * Tests if the directed edge is unreachable by driving. If a driveable
 * edge cannot reach higher class roads and a search cannot expand after
 * a set number of iterations the edge is considered unreachable.
 * @param  reader        Graph reader
 * @param  lock          Mutex for locking while tiles are retrieved
 * @param  directededge  Directed edge to test.
 * @return  Returns true if the edge is found to be unreachable.
 */
bool IsUnreachable(GraphReader& reader, std::mutex& lock,
                   DirectedEdge& directededge) {
  // Only check driveable edges. If already on a higher class road consider
  // the edge reachable
  if (!(directededge.forwardaccess() & kAutoAccess) ||
       directededge.classification() < RoadClass::kTertiary) {
    return false;
  }

  // Add the end node to the expand list
  std::unordered_set<GraphId> visitedset;  // Set of visited nodes
  std::unordered_set<GraphId> expandset;   // Set of nodes to expand
  expandset.insert(directededge.endnode());

  // Expand until we either find a tertiary or higher classification,
  // expand more than kUnreachableIterations nodes, or cannot expand
  // any further.
  uint32_t n = 0;
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
    lock.lock();
    const GraphTile* tile = reader.GetGraphTile(expandnode);
    lock.unlock();
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

// Test if this is a "not thru" edge. These are edges that enter a region that
// has no exit other than the edge entering the region
bool IsNotThruEdge(GraphReader& reader, std::mutex& lock,
                   const GraphId& startnode,
                   DirectedEdge& directededge) {
  // Add the end node to the expand list
  std::unordered_set<GraphId> visitedset;  // Set of visited nodes
  std::unordered_set<GraphId> expandset;   // Set of nodes to expand
  expandset.insert(directededge.endnode());

  // Expand edges until exhausted, the maximum number of expansions occur,
  // or end up back at the starting node. No node can be visited twice.
  for (uint32_t n = 0; n < kMaxNoThruTries; n++) {
    // If expand list is exhausted this is "not thru"
    if (expandset.empty())
      return true;

    // Get the node off of the expand list and add it to the visited list.
    // Expand edges from this node.
    const GraphId expandnode = *expandset.cbegin();
    expandset.erase(expandset.begin());
    visitedset.insert(expandnode);
    lock.lock();
    const GraphTile* tile = reader.GetGraphTile(expandnode);
    lock.unlock();
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
      if (diredge->classification() < RoadClass::kTertiary ||
          diredge->endnode() == startnode) {
        return false;
      }

      // Add to the end node to expand set if not already visited set
      if (visitedset.find(diredge->endnode()) == visitedset.end()) {
        expandset.insert(diredge->endnode());
      }
    }
  }
  return false;
}

// Test if the edge is internal to an intersection.
bool IsIntersectionInternal(GraphReader& reader, std::mutex& lock,
                            const GraphId& startnode,
                            NodeInfo& startnodeinfo,
                            DirectedEdge& directededge,
                            const uint32_t idx) {
  // Internal intersection edges must be short and cannot be a roundabout
  if (directededge.length() > kMaxInternalLength ||
      directededge.roundabout()) {
    return false;
  }

  // Must have inbound oneway at start node (exclude edges that are nearly
  // straight turn type onto the directed edge
  bool oneway_inbound = false;
  lock.lock();
  const GraphTile* tile = reader.GetGraphTile(startnode);
  lock.unlock();
  uint32_t heading = startnodeinfo.heading(idx);
  const DirectedEdge* diredge = tile->directededge(startnodeinfo.edge_index());
  for (uint32_t i = 0; i < startnodeinfo.edge_count(); i++, diredge++) {
    // Skip the current directed edge
    // and any inbound edges not oneway
    // and any link edge
    // and any link that is not a road
    if (i == idx
        || (!((diredge->reverseaccess() & kAutoAccess)
            && !(diredge->forwardaccess() & kAutoAccess)))
        || diredge->link()
        || (diredge->use() != Use::kRoad)) {
      continue;
    }

    // Exclude edges that are nearly straight to go onto directed edge
    uint32_t from_heading = ((startnodeinfo.heading(i) + 180) % 360);
    uint32_t turndegree = GetTurnDegree(from_heading, heading);
    if (turndegree < 30 || turndegree > 330) {
      continue;
    }

    // If we are here the edge is a candidate oneway inbound
    oneway_inbound = true;
    break;
  }
  if (!oneway_inbound) {
    return false;
  }

  // Must have outbound oneway at end node (exclude edges that are nearly
  // straight turn from directed edge
  bool oneway_outbound = false;
  if (tile->id() != directededge.endnode().Tile_Base()) {
    lock.lock();
    tile = reader.GetGraphTile(directededge.endnode());
    lock.unlock();
  }
  const NodeInfo* node = tile->node(directededge.endnode());
  diredge = tile->directededge(node->edge_index());
  for (uint32_t i = 0; i < node->edge_count(); i++, diredge++) {
    // Skip opposing directed edge
    // and any outbound edges not oneway
    // and any link edge
    // and any link that is not a road
    if (i == directededge.opp_local_idx()
        || (!((diredge->forwardaccess() & kAutoAccess)
            && !(diredge->reverseaccess() & kAutoAccess)))
        || diredge->link()
        || (diredge->use() != Use::kRoad)) {
      continue;
    }

    // Exclude edges that are nearly straight to go onto directed edge
    // Unfortunately don't have headings at the end node...
    auto shape = tile->edgeinfo(diredge->edgeinfo_offset()).shape();
    if (!diredge->forward())
      std::reverse(shape.begin(), shape.end());
    uint32_t to_heading = std::round(
        PointLL::HeadingAlongPolyline(
            shape,
            GetOffsetForHeading(diredge->classification(), diredge->use())));
    uint32_t turndegree = GetTurnDegree(heading, to_heading);
    if (turndegree < 30 || turndegree > 330) {
      continue;
    }

    // If we are here the edge is oneway outbound
    oneway_outbound = true;
    break;
  }
  if (!oneway_outbound) {
    return false;
  }

  // TODO - determine if we need to add name checks

  // TODO - do we need to check headings of the inbound and outbound
  // oneway edges

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
uint32_t GetDensity(GraphReader& reader, std::mutex& lock, const PointLL& ll,
                    enhancer_stats& stats, const Tiles<PointLL>& tiles,
                    uint8_t local_level) {
  // Radius is in km - turn into meters
  float rm = kDensityRadius * kMetersPerKm;
  float mr2 = rm * rm;

  // Use distance approximator for all distance checks
  DistanceApproximator approximator(ll);

  // Get a list of tiles required for a node search within this radius
  float lngdeg = (rm / DistanceApproximator::MetersPerLngDegree(ll.lat()));
  AABB2<PointLL> bbox(Point2(ll.lng() - lngdeg, ll.lat() - kDensityLatDeg),
                      Point2(ll.lng() + lngdeg, ll.lat() + kDensityLatDeg));
  std::vector<int32_t> tilelist = tiles.TileList(bbox);

  // For all tiles needed to find nodes within the radius...find nodes within
  // the radius (squared) and add lengths of directed edges
  float roadlengths = 0.0f;
  for (auto t : tilelist) {
    // Check all the nodes within the tile. Skip if tile has no nodes (can be
    // an empty tile added for connectivity map logic).
    lock.lock();
    const GraphTile* newtile = reader.GetGraphTile(GraphId(t, local_level, 0));
    lock.unlock();
    if (!newtile || newtile->header()->nodecount() == 0)
      continue;
    const auto start_node = newtile->node(0);
    const auto end_node   = start_node + newtile->header()->nodecount();
    for (auto node = start_node; node < end_node; ++node) {
      // Check if within radius
      if (approximator.DistanceSquared(node->latlng()) < mr2) {
        // Get all directed edges and add length
        const DirectedEdge* directededge = newtile->directededge(node->edge_index());
        for (uint32_t i = 0; i < node->edge_count(); i++, directededge++) {
          // Exclude non-roads (parking, walkways, ferries, etc.)
          if (directededge->use() == Use::kRoad ||
              directededge->use() == Use::kRamp ||
              directededge->use() == Use::kTurnChannel ||
              directededge->use() == Use::kAlley ||
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
  if (density > stats.max_density)
    stats.max_density = density;

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
bool IsPencilPointUturn(uint32_t from_index, uint32_t to_index,
                        const DirectedEdge& directededge,
                        const DirectedEdge* edges,
                        const NodeInfo& node_info,
                        uint32_t turn_degree) {
  // Logic for drive on right
  if (directededge.drive_on_right()) {
    // If the turn is a sharp left (179 < turn < 211)
    //    or short distance (< 50m) and wider sharp left (179 < turn < 226)
    // and oneway edges
    // and an intersecting right road exists
    // and no intersecting left road exists
    // and the from and to edges have a common base name
    // then it is a left pencil point u-turn
    if ((((turn_degree > 179) && (turn_degree < 211))
        || (((edges[from_index].length() < 50) || (directededge.length() < 50))
            && (turn_degree > 179) && (turn_degree < 226)))
      && (!(edges[from_index].forwardaccess() & kAutoAccess)
          && (edges[from_index].reverseaccess() & kAutoAccess))
      && ((directededge.forwardaccess() & kAutoAccess)
          && !(directededge.reverseaccess() & kAutoAccess))
      && directededge.edge_to_right(from_index)
      && !directededge.edge_to_left(from_index)
      && node_info.name_consistency(from_index, to_index)) {
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
    if ((((turn_degree > 149) && (turn_degree < 181))
        || (((edges[from_index].length() < 50) || (directededge.length() < 50))
            && (turn_degree > 134) && (turn_degree < 181)))
      && (!(edges[from_index].forwardaccess() & kAutoAccess)
          && (edges[from_index].reverseaccess() & kAutoAccess))
      && ((directededge.forwardaccess() & kAutoAccess)
          && !(directededge.reverseaccess() & kAutoAccess))
      && !directededge.edge_to_right(from_index)
      && directededge.edge_to_left(from_index)
      && node_info.name_consistency(from_index, to_index)) {
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
bool IsCyclewayUturn(uint32_t from_index, uint32_t to_index,
                        const DirectedEdge& directededge,
                        const DirectedEdge* edges,
                        const NodeInfo& node_info,
                        uint32_t turn_degree) {

  // we only deal with Cycleways
  if (edges[from_index].use() != Use::kCycleway || edges[to_index].use() != Use::kCycleway)
    return false;

  // Logic for drive on right
  if (directededge.drive_on_right()) {
    // If the turn is a sharp left (179 < turn < 211)
    //    or short distance (< 50m) and wider sharp left (179 < turn < 226)
    // and an intersecting right road exists
    // and an intersecting left road exists
    // then it is a cycleway u-turn
    if ((((turn_degree > 179) && (turn_degree < 211))
        || (((edges[from_index].length() < 50) || (directededge.length() < 50))
            && (turn_degree > 179) && (turn_degree < 226)))
      && directededge.edge_to_right(from_index)
      && directededge.edge_to_left(from_index)) {
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
    if ((((turn_degree > 149) && (turn_degree < 181))
        || (((edges[from_index].length() < 50) || (directededge.length() < 50))
            && (turn_degree > 134) && (turn_degree < 181)))
      && directededge.edge_to_right(from_index)
      && directededge.edge_to_left(from_index)) {
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
 * a higher class road. Special cases occur for links (ramps/turn channels).
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
uint32_t GetStopImpact(uint32_t from, uint32_t to,
                       const DirectedEdge& directededge,
                       const DirectedEdge* edges, const uint32_t count,
                       const NodeInfo& nodeinfo, uint32_t turn_degree,
                       enhancer_stats& stats) {

  ///////////////////////////////////////////////////////////////////////////
  // Special cases.

  // Handle Roundabouts
  if (edges[from].roundabout() && edges[to].roundabout()) {
    return 0;
  }

  // Handle Pencil point u-turn
  if (IsPencilPointUturn(from, to, directededge, edges, nodeinfo,
                         turn_degree)) {
    stats.pencilucount++;
    return 7;
  }

  // Handle Cycleway u-turn
  if (IsCyclewayUturn(from, to, directededge, edges, nodeinfo,
                      turn_degree)) {
    return 7;
  }

  ///////////////////////////////////////////////////////////////////////////

  // Get the highest classification of other roads at the intersection
  bool allramps = true;
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
        if (c  < static_cast<uint32_t>(bestrc)) {
          bestrc = static_cast<RoadClass>(c);
        }
      } else if (edge->classification() < bestrc) {
        bestrc = edge->classification();
      }
    }

    // Check if not a ramp or turn channel
    if (!edge->link()) {
      allramps = false;
    }
  }

  // kUnclassified,  kResidential, and kServiceOther are grouped
  // together for the stop_impact logic.
  RoadClass from_rc = edges[from].classification();
  if (from_rc > RoadClass::kUnclassified)
    from_rc = RoadClass::kUnclassified;

  // High stop impact from a turn channel onto a turn channel unless
  // the other edge a low class road (walkways often intersect
  // turn channels)
  if (edges[from].use() == Use::kTurnChannel &&
      edges[to].use() == Use::kTurnChannel  &&
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
  bool is_sharp = (turn_type == Turn::Type::kSharpLeft ||
                   turn_type == Turn::Type::kSharpRight ||
                   turn_type == Turn::Type::kReverse);
  bool is_slight = (turn_type == Turn::Type::kStraight ||
                    turn_type == Turn::Type::kSlightRight ||
                    turn_type == Turn::Type::kSlightLeft);
  if (allramps) {
    if (is_sharp) {
      stop_impact += 2;
    } else if (is_slight) {
      stop_impact /= 2;
    } else {
      stop_impact -= 1;
    }
  } else if (edges[from].use() == Use::kRamp && edges[to].use() == Use::kRamp &&
             bestrc < RoadClass::kUnclassified) {
    // Ramp may be crossing a road (not a path or service road)
    if (nodeinfo.traffic_signal()) {
      stop_impact = 4;
    } else if (count > 3) {
      stop_impact += 2;
    }
  } else if (edges[from].use() == Use::kRamp && edges[to].use() != Use::kRamp) {
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
    } else {
      stop_impact -= 1;
    }
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
          DirectedEdge& directededge, const DirectedEdge* edges,
          const uint32_t ntrans, uint32_t* headings,
          const NodeInfo& nodeinfo,
          enhancer_stats& stats) {
  for (uint32_t i = 0; i < ntrans; i++) {
    // Get the turn type (reverse the heading of the from directed edge since
    // it is incoming
    uint32_t from_heading = ((headings[i] + 180) % 360);
    uint32_t turn_degree  = GetTurnDegree(from_heading, headings[idx]);
    directededge.set_turntype(i, Turn::GetType(turn_degree));

    // Set the edge_to_left and edge_to_right flags
    uint32_t right_count = 0;
    uint32_t left_count  = 0;
    if (ntrans > 2) {
      for (uint32_t j = 0; j < ntrans; ++j) {
        // Skip the from and to edges
        if (j == i || j == idx) {
          continue;
        }

        // Get the turn degree from incoming edge i to j and check if right
        // or left of the turn degree from incoming edge i onto idx
        uint32_t degree = GetTurnDegree(from_heading, headings[j]);
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
    uint32_t stopimpact = GetStopImpact(i, idx, directededge, edges, ntrans,
                                        nodeinfo, turn_degree, stats);
    directededge.set_stopimpact(i, stopimpact);
  }

}

/**
 * Get the index of the opposing edge at the end node. This is
 * on the local hierarchy (before adding transition and shortcut edges).
 * @param  endnodetile   Graph tile at the end node.
 * @param  startnode     Start node of the directed edge.
 * @param  directededge  Directed edge to match.
 */
uint32_t GetOpposingEdgeIndex(const GraphTile* endnodetile,
                              const GraphId& startnode,
                              const DirectedEdge& edge) {
  // Get the tile at the end node and get the node info
  GraphId endnode = edge.endnode();
  const NodeInfo* nodeinfo = endnodetile->node(endnode.id());

  // Get the directed edges and return when the end node matches
  // the specified node and length matches
  const DirectedEdge* directededge = endnodetile->directededge(
              nodeinfo->edge_index());
  for (uint32_t i = 0; i < nodeinfo->edge_count(); i++, directededge++) {
    if (directededge->endnode() == startnode &&
        directededge->length() == edge.length()) {
      return i;
    }
  }
  return kMaxEdgesPerNode;
}

bool ConsistentNames(const std::string& country_code,
                     const std::vector<std::string>& names1,
                     const std::vector<std::string>& names2) {
  std::unique_ptr<StreetNames> street_names1 = StreetNamesFactory::Create(
      country_code, names1);
  std::unique_ptr<StreetNames> street_names2 = StreetNamesFactory::Create(
      country_code, names2);

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
             const std::string& access_file,
             const boost::property_tree::ptree& hierarchy_properties,
             std::queue<GraphId>& tilequeue, std::mutex& lock,
             std::promise<enhancer_stats>& result) {

  auto less_than = [](const OSMAccess& a, const OSMAccess& b){return a.way_id() < b.way_id();};
  sequence<OSMAccess> access_tags(access_file, false);

  auto database = pt.get_optional<std::string>("admin");
  // Initialize the admin DB (if it exists)
  sqlite3 *admin_db_handle = database ? GetDBHandle(*database) : nullptr;
  if (!database)
    LOG_WARN("Admin db not found.  Not saving admin information.");
  else if (!admin_db_handle)
    LOG_WARN("Admin db " + *database + " not found.  Not saving admin information.");

  std::unordered_map<std::string, std::vector<int>> country_access = GetCountryAccess(admin_db_handle);

  // Local Graphreader
  GraphReader reader(hierarchy_properties);

  // Default speeds (kph) in urban areas per road class
  // (TODO - get from property tree)
  // 55 MPH - motorway
  // 45 MPH - trunk
  // 35 MPH - primary
  // 30 MPH - secondary
  // 25 MPH - tertiary
  // 20 MPH - residential and unclassified
  // 15 MPH - service/other
  uint32_t urban_rc_speed[] = { 89, 73, 57, 49, 40, 35, 35, 25 };

  // Get some things we need throughout
  enhancer_stats stats{std::numeric_limits<float>::min(), 0};
  const auto& local_level = TileHierarchy::levels().rbegin()->second.level;
  const auto& tiles = TileHierarchy::levels().rbegin()->second.tiles;

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
    const GraphTile* tile = reader.GetGraphTile(tile_id);
    if (tile->header()->nodecount() == 0) {
      lock.unlock();
      continue;
    }

    // Tile builder - serialize in existing tile so we can add admin names
    GraphTileBuilder tilebuilder(reader.tile_dir(), tile_id, true);
    lock.unlock();

    // this will be our updated list of restrictions.
    // need to do some conversions on weights; therefore, we must update
    // the restriction list.
    uint32_t ar_before = tilebuilder.header()->access_restriction_count();
    std::vector<AccessRestriction> access_restrictions;

    uint32_t id  = tile_id.tileid();
    // First pass - update links (set use to ramp or turn channel) and
    // set opposing local index.
    for (uint32_t i = 0; i < tilebuilder.header()->nodecount(); i++) {
      GraphId startnode(id, local_level, i);
      NodeInfo& nodeinfo = tilebuilder.node_builder(i);

      const DirectedEdge* edges = tile->directededge(nodeinfo.edge_index());
      for (uint32_t j = 0; j <  nodeinfo.edge_count(); j++) {
        DirectedEdge& directededge =
            tilebuilder.directededge_builder(nodeinfo.edge_index() + j);

        // Get the tile at the end node
        const GraphTile* endnodetile = nullptr;
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
               GetOpposingEdgeIndex(endnodetile, startnode, directededge));
      }
    }

    // Second pass - add admin information and edge transition information.
    for (uint32_t i = 0; i < tilebuilder.header()->nodecount(); i++) {
      GraphId startnode(id, local_level, i);
      NodeInfo& nodeinfo = tilebuilder.node_builder(i);

      // Get relative road density and local density
      uint32_t density = GetDensity(reader, lock, nodeinfo.latlng(),
                                    stats, tiles, local_level);
      nodeinfo.set_density(density);

      uint32_t admin_index = nodeinfo.admin_index();
      // Set the country code
      std::string country_code = "";
      if (admin_index != 0)
        country_code = tilebuilder.admins_builder(admin_index).country_iso();
      else stats.no_country_found++;

      // Get headings of the edges - set in NodeInfo. Set driveability info
      // on the node as well.
      uint32_t count = nodeinfo.edge_count();
      uint32_t ntrans = std::min(count, kNumberOfEdgeTransitions);
      uint32_t heading[ntrans];
      nodeinfo.set_local_edge_count(ntrans);
      for (uint32_t j = 0; j < ntrans; j++) {
        DirectedEdge& directededge =
            tilebuilder.directededge_builder(nodeinfo.edge_index() + j);

        auto e_offset = tilebuilder.edgeinfo(directededge.edgeinfo_offset());
        auto shape = e_offset.shape();
        if (!directededge.forward())
          std::reverse(shape.begin(), shape.end());
        heading[j] = std::round(
            PointLL::HeadingAlongPolyline(
                shape,
                GetOffsetForHeading(directededge.classification(),
                                    directededge.use())));

        // Set heading in NodeInfo. TODO - what if 2 edges have nearly the
        // same heading - should one be "adjusted" so the relative direction
        // is maintained.
        nodeinfo.set_heading(j, heading[j]);

        // Set traversability for autos
        Traversability traversability;
        if (directededge.forwardaccess() & kAutoAccess) {
          traversability = (directededge.reverseaccess() & kAutoAccess) ?
              Traversability::kBoth : Traversability::kForward;
        } else {
          traversability = (directededge.reverseaccess() & kAutoAccess) ?
              Traversability::kBackward : Traversability::kNone;
        }
        nodeinfo.set_local_driveability(j, traversability);
      }

      // Go through directed edges and "enhance" directed edge attributes
      uint32_t driveable_count = 0;
      const DirectedEdge* edges = tilebuilder.directededges(nodeinfo.edge_index());
      for (uint32_t j = 0; j <  nodeinfo.edge_count(); j++) {
        DirectedEdge& directededge =
            tilebuilder.directededge_builder(nodeinfo.edge_index() + j);

        auto e_offset = tilebuilder.edgeinfo(directededge.edgeinfo_offset());
        std::string end_node_code = "";
        uint32_t end_admin_index = 0;
        // Get the tile at the end node
        const GraphTile* endnodetile = nullptr;
        if (tile->id() == directededge.endnode().Tile_Base()) {
          end_admin_index = tile->node(directededge.endnode().id())->admin_index();
          end_node_code = tile->admin(end_admin_index)->country_iso();
        } else {
          lock.lock();
          endnodetile = reader.GetGraphTile(directededge.endnode());
          lock.unlock();
          end_admin_index = endnodetile->node(directededge.endnode().id())->admin_index();
          end_node_code = endnodetile->admin(end_admin_index)->country_iso();
        }

        // only process country access logic if the iso country codes match.
        if (country_code == end_node_code) {

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
                  directededge.use() == Use::kTrack || directededge.use() == Use::kFootway ||
                  directededge.use() == Use::kPedestrian || directededge.use() == Use::kBridleway ||
                  directededge.use() == Use::kCycleway || directededge.use() == Use::kPath)) {

            std::vector<int> access = country_access.at(country_code);
            // leaves tile flag indicates that we have an access record for this edge.
            // leaves tile flag is updated later to the real value.
            if (directededge.leaves_tile()) {
              sequence<OSMAccess>::iterator access_it = access_tags.find(target,less_than);
              if (access_it != access_tags.end())
                SetCountryAccess(directededge, access, access_it);
              else LOG_WARN("access tags not found for " + std::to_string(e_offset.wayid()));
            } else SetCountryAccess(directededge, access, target);
          // motorroad default.  Only applies to RC <= kPrimary and has no country override.
          // We just use the defaults which is no bicycles, mopeds and no pedestrians.
          // leaves tile flag indicates that we have an access record for this edge.
          // leaves tile flag is updated later to the real value.
          }else if (country_iterator == country_access.end() &&
              directededge.classification() <= RoadClass::kPrimary &&
              directededge.leaves_tile()) {

            OSMAccess target{e_offset.wayid()};
            sequence<OSMAccess>::iterator access_it = access_tags.find(target,less_than);
            if (access_it != access_tags.end()) {
              const OSMAccess& access = access_it;
              if (access.motorroad_tag()) {
                uint32_t forward = directededge.forwardaccess();
                uint32_t reverse = directededge.reverseaccess();

                bool f_oneway_vehicle = (((forward & kAutoAccess) && !(reverse & kAutoAccess)) ||
                    ((forward & kTruckAccess) && !(reverse & kTruckAccess)) ||
                    ((forward & kEmergencyAccess) && !(reverse & kEmergencyAccess)) ||
                    ((forward & kTaxiAccess) && !(reverse & kTaxiAccess)) ||
                    ((forward & kHOVAccess) && !(reverse & kHOVAccess)) ||
                    ((forward & kMopedAccess) && !(reverse & kMopedAccess)) ||
                    ((forward & kBusAccess) && !(reverse & kBusAccess)));

                bool r_oneway_vehicle = ((!(forward & kAutoAccess) && (reverse & kAutoAccess)) ||
                    (!(forward & kTruckAccess) && (reverse & kTruckAccess)) ||
                    (!(forward & kEmergencyAccess) && (reverse & kEmergencyAccess)) ||
                    (!(forward & kTaxiAccess) && (reverse & kTaxiAccess)) ||
                    (!(forward & kHOVAccess) && (reverse & kHOVAccess)) ||
                    (!(forward & kMopedAccess) && (reverse & kMopedAccess)) ||
                    (!(forward & kBusAccess) && (reverse & kBusAccess)));

                bool f_oneway_bicycle = ((forward & kBicycleAccess) && !(reverse & kBicycleAccess));
                bool r_oneway_bicycle = (!(forward & kBicycleAccess) && (reverse & kBicycleAccess));

                // motorroad defaults remove ped, wheelchair, moped, and bike access.
                // still check for user tags via access.
                forward = GetAccess(forward,
                          (forward & ~(kPedestrianAccess | kWheelchairAccess | kMopedAccess | kBicycleAccess)),
                          r_oneway_vehicle, r_oneway_bicycle, access);
                reverse = GetAccess(reverse,
                          (reverse & ~(kPedestrianAccess | kWheelchairAccess | kMopedAccess | kBicycleAccess)),
                          f_oneway_vehicle, f_oneway_bicycle, access);

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
        if (directededge.use() == Use::kPedestrian)
          directededge.set_use(Use::kFootway);

        // Update speed.
        UpdateSpeed(directededge, density, urban_rc_speed);

        // Update the named flag
        auto names = tilebuilder.edgeinfo(directededge.edgeinfo_offset()).GetNames();
        directededge.set_named(names.size() > 0);

        // Name continuity - set in NodeInfo.
        for (uint32_t k = (j + 1); k < ntrans; k++) {
          DirectedEdge& fromedge = tilebuilder.directededge(
                    nodeinfo.edge_index() + k);
          if (ConsistentNames(country_code, names,
              tilebuilder.edgeinfo(fromedge.edgeinfo_offset()).GetNames())) {
            nodeinfo.set_name_consistency(j, k, true);
          }
        }

        // Set edge transitions and unreachable, not_thru, and internal
        // intersection flags.
        if (j < kNumberOfEdgeTransitions) {
          ProcessEdgeTransitions(j, directededge, edges, ntrans, heading,
                                 nodeinfo, stats);
        }

        // Set unreachable (driving) flag
        if (IsUnreachable(reader, lock, directededge)) {
          directededge.set_unreachable(true);
          stats.unreachable++;
        }

        // Check for not_thru edge (only on low importance edges). Exclude
        // transit edges
        if (directededge.classification() > RoadClass::kTertiary) {
          if (IsNotThruEdge(reader, lock, startnode, directededge)) {
            directededge.set_not_thru(true);
            stats.not_thru++;
          }
        }

        // Test if an internal intersection edge. Must do this after setting
        // opposing edge index
        if (IsIntersectionInternal(reader, lock, startnode, nodeinfo,
                                    directededge, j)) {
          directededge.set_internal(true);
          stats.internalcount++;
        }

        // Update access restrictions (update weight units)
        if (directededge.access_restriction()) {
          auto restrictions = tilebuilder.GetAccessRestrictions(nodeinfo.edge_index() + j, kAllAccess);

          // Convert any US weight values from short ton (U.S. customary)
          // to metric and add to the tile's access restriction list
          if (country_code == "US" || country_code == "MM" || country_code == "LR") {
            for (auto& res : restrictions) {
              if (res.type() == AccessType::kMaxWeight ||
                  res.type() == AccessType::kMaxAxleLoad) {
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
      // gates or toll-booths).
      if (nodeinfo.type() != NodeType::kGate &&
          nodeinfo.type() != NodeType::kTollBooth) {
        if (driveable_count == 1) {
          nodeinfo.set_intersection(IntersectionType::kDeadEnd);
        } else if (nodeinfo.edge_count() == 2) {
          nodeinfo.set_intersection(IntersectionType::kFalse);
        }
      }
    }

    // Replace access restrictions
    if (ar_before != access_restrictions.size()) {
      LOG_ERROR("Mismatch in access restriction count before " + std::to_string(ar_before) + ""
          " and after " + std::to_string(access_restrictions.size()) +
          " tileid = " + std::to_string(tile_id.tileid()));
    }
    tilebuilder.AddAccessRestrictions(access_restrictions);

    // Write the new file
    lock.lock();
    tilebuilder.StoreTileData();
    LOG_TRACE((boost::format("GraphEnhancer completed tile %1%") % tile_id).str());

    // Check if we need to clear the tile cache
    if (reader.OverCommitted()) {
      reader.Clear();
    }
    lock.unlock();
  }

  if (admin_db_handle)
    sqlite3_close (admin_db_handle);

  // Send back the statistics
  result.set_value(stats);
}

}

namespace valhalla {
namespace mjolnir {

// Enhance the local level of the graph
void GraphEnhancer::Enhance(const boost::property_tree::ptree& pt,
                            const std::string& access_file) {
  // A place to hold worker threads and their results, exceptions or otherwise
  std::vector<std::shared_ptr<std::thread> > threads(
    std::max(static_cast<unsigned int>(1),
    pt.get<unsigned int>("concurrency", std::thread::hardware_concurrency())));

  // A place to hold the results of those threads, exceptions or otherwise
  std::list<std::promise<enhancer_stats> > results;

  // Create a randomized queue of tiles to work from
  std::deque<GraphId> tempqueue;
  boost::property_tree::ptree hierarchy_properties = pt.get_child("mjolnir");
  GraphReader reader(hierarchy_properties);
  auto local_level = TileHierarchy::levels().rbegin()->second.level;
  auto tiles = TileHierarchy::levels().rbegin()->second.tiles;
  for (uint32_t id = 0; id < tiles.TileCount(); id++) {
    // If tile exists add it to the queue
    GraphId tile_id(id, local_level, 0);
    if (GraphReader::DoesTileExist(hierarchy_properties, tile_id)) {
      tempqueue.push_back(tile_id);
    }
  }
  std::random_shuffle(tempqueue.begin(), tempqueue.end());
  std::queue<GraphId> tilequeue(tempqueue);

  // An atomic object we can use to do the synchronization
  std::mutex lock;

  // Start the threads
  LOG_INFO("Enhancing local graph...");
  for (auto& thread : threads) {
    results.emplace_back();
    thread.reset(new std::thread(enhance,
                 std::cref(hierarchy_properties),
                 std::cref(access_file),
                 std::ref(hierarchy_properties), std::ref(tilequeue),
                 std::ref(lock), std::ref(results.back())));
  }

  // Wait for them to finish up their work
  for (auto& thread : threads) {
    thread->join();
  }

  // Check all of the outcomes, to see about maximum density (km/km2)
  enhancer_stats stats{std::numeric_limits<float>::min(), 0};
  for (auto& result : results) {
    // If something bad went down this will rethrow it
    try {
      auto thread_stats = result.get_future().get();
      stats(thread_stats);
    }
    catch(std::exception& e) {
      //TODO: throw further up the chain?
    }
  }
  LOG_INFO("Finished with max_density " + std::to_string(stats.max_density) + " and unreachable " + std::to_string(stats.unreachable));
  LOG_DEBUG("not_thru = " + std::to_string(stats.not_thru));
  LOG_DEBUG("no country found = " + std::to_string(stats.no_country_found));
  LOG_DEBUG("internal intersection = " + std::to_string(stats.internalcount));
  LOG_DEBUG("Turn Channel Count = " + std::to_string(stats.turnchannelcount));
  LOG_DEBUG("Ramp Count = " + std::to_string(stats.rampcount));
  LOG_DEBUG("Pencil Point Uturn count = " + std::to_string(stats.pencilucount));
  for (auto density : stats.density_counts) {
    LOG_DEBUG("Density: " + std::to_string(density));
  }
}

}
}
