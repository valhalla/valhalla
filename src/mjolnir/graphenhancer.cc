#include "mjolnir/graphenhancer.h"

#include <valhalla/midgard/aabb2.h>
#include <valhalla/midgard/constants.h>
#include <valhalla/midgard/logging.h>

using namespace valhalla::midgard;
using namespace valhalla::baldr;

namespace valhalla {
namespace mjolnir {

uint32_t unreachable = 0;



// Number of iterations to try to determine if an edge is unreachable
// by driving. If a search terminates before this without reaching
// a secondary road then the edge is considered unreachable.
constexpr uint32_t kUnreachableIterations = 20;

// Meters offset from start/end of shape for finding heading
constexpr float kMetersOffsetForHeading = 30.0f;

GraphEnhancer::GraphEnhancer(const boost::property_tree::ptree& pt)
    : tile_hierarchy_(pt),
      local_level_(tile_hierarchy_.levels().rbegin()->second.level),
      tiles_(tile_hierarchy_.levels().rbegin()->second.tiles),
      graphreader_(tile_hierarchy_),
      maxdensity_(0.0f) {
}

// Enhance the local level of the graph
bool GraphEnhancer::Enhance() {
  auto t1 = std::chrono::high_resolution_clock::now();

  // Iterate through the tiles and perform enhancements
  for (uint32_t id = 0; id < tiles_.TileCount(); id++) {
    // Get the graph tile. Skip if no tile exists (common case)
    GraphId tileid(id, local_level_, 0);
    GraphTileBuilder tilebuilder(tile_hierarchy_, tileid);
    if (tilebuilder.size() == 0) {
      continue;
    }

    // Get read only tile
    const GraphTile* tile = graphreader_.GetGraphTile(tileid);

    // Copy existing header. No need to update any counts or offsets.
    GraphTileHeader existinghdr = *(tilebuilder.header());
    GraphTileHeaderBuilder hdrbuilder =
       static_cast<GraphTileHeaderBuilder&>(existinghdr);

    // Update nodes and directed edges as needed
    std::vector<NodeInfoBuilder> nodes;
    std::vector<DirectedEdgeBuilder> directededges;

    // Iterate through the nodes
    float localdensity = 0.0f;
    uint32_t nodecount = tilebuilder.header()->nodecount();
    for (uint32_t i = 0; i < nodecount; i++) {
      GraphId startnode(id, local_level_, i);
      NodeInfoBuilder nodeinfo = tilebuilder.node(i);

      // Get relative road density and local density
      uint32_t density = GetDensity(nodeinfo.latlng(), localdensity);

      // Enhance node attributes (TODO - admin, timezone)
      nodeinfo.set_density(density);

      // Get headings of the edges
      uint32_t count = nodeinfo.edge_count();
      uint32_t ntrans = std::min(count, kNumberOfEdgeTransitions);
      uint32_t heading[ntrans];
      for (uint32_t j = 0, n = nodeinfo.edge_count(); j < ntrans; j++) {
        DirectedEdgeBuilder& directededge = tilebuilder.directededge(
                              nodeinfo.edge_index() + j);
        auto shape = tile->edgeinfo(directededge.edgeinfo_offset())->shape();
        if (!directededge.forward())
          std::reverse(shape.begin(), shape.end());
        heading[j] = std::round(PointLL::HeadingAlongPolyline(shape,
                                kMetersOffsetForHeading));
      }

      // Go through directed edges and update data
      const DirectedEdge* edges = tile->directededge(nodeinfo.edge_index());
      for (uint32_t j = 0, n = nodeinfo.edge_count(); j < n; j++) {
        DirectedEdgeBuilder& directededge = tilebuilder.directededge(
                                nodeinfo.edge_index() + j);
        // Enhance directed edge attributes. TODO - drive_on_right flag
        // and other admin related processing

        // Update speed.
        UpdateSpeed(directededge, localdensity);

        // Edge transitions.
        if (j < kNumberOfEdgeTransitions) {
          ProcessEdgeTransitions(j, directededge, edges, ntrans, heading);
        }

        // Set unreachable (driving) flag
        if (IsUnreachable(directededge)) {
          directededge.set_unreachable(true);
          unreachable++;
        }

        // Set the opposing index on the local level
        directededge.set_opp_local_idx(GetOpposingEdgeIndex(startnode,
                                              directededge));

        // Add the directed edge
        directededges.emplace_back(std::move(directededge));
      }

      // Add the node to the list
      nodes.emplace_back(std::move(nodeinfo));
    }

    // Write the new file
    tilebuilder.Update(tile_hierarchy_, hdrbuilder, nodes, directededges);
  }

  auto t2 = std::chrono::high_resolution_clock::now();
  uint32_t secs = (std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count()) / 1000;
  LOG_INFO("Enhance local graph took " + std::to_string(secs) + " sec");
  LOG_INFO("maxdensity= " + std::to_string(maxdensity_));
  LOG_INFO("unreachable= " + std::to_string(unreachable));

  return true;
}

// Update speed - especially for classified speeds (those without max_speed
// tags).  TODO (add admin)
void GraphEnhancer::UpdateSpeed(DirectedEdgeBuilder& directededge,
                                const float density) {
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
    // Set speed on ferries
    if (directededge.railferry()) {
      directededge.set_speed(40);
    } else if (directededge.ferry()) {
      if (directededge.length() < 4000) {
        directededge.set_speed(15);
      } else if (directededge.length() < 8000) {
        directededge.set_speed(15);
      } else {
        directededge.set_speed(15);
      }
    }

    // Modify speed
    if (directededge.classification() == RoadClass::kMotorway ||
        directededge.classification() == RoadClass::kTrunk) {
      // Motorway or trunk - allow higher speed for rural than urban
    } else {
      // Modify speed: high density (urban) vs. low density (rural)
    }

    // Modify speed based on surface
  }
}

// Test is an edge is unreachable by driving.
bool GraphEnhancer::IsUnreachable(DirectedEdgeBuilder& directededge) {
  // Only check driveable edges. If already on a higher class road consider
  // the edge reachable
  if (!(directededge.forwardaccess() & kAutoAccess) ||
       directededge.classification() < RoadClass::kTertiaryUnclassified) {
    return false;
  }

  // Add the end node to the exmapnd list
  std::unordered_set<GraphId> visitedset;  // Set of visited nodes
  std::unordered_set<GraphId> expandset;   // Set of nodes to expand
  expandset.insert(directededge.endnode());

  // Expand until we either find a secondary or higher classification,
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
    GraphId expandnode = *expandset.begin();
    expandset.erase(expandset.begin());
    visitedset.insert(expandnode);
    const GraphTile* tile = graphreader_.GetGraphTile(expandnode);
    const NodeInfo* nodeinfo = tile->node(expandnode);
    const DirectedEdge* diredge = tile->directededge(nodeinfo->edge_index());
    for (uint32_t i = 0; i < nodeinfo->edge_count(); i++, diredge++) {
      if ((diredge->forwardaccess() & kAutoAccess)) {
        if (diredge->classification() < RoadClass::kTertiaryUnclassified) {
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

// Get the node density. Computes road density (km/km squared) with
// 5 km of the node. TODO - make this a constant or configurable?
uint32_t GraphEnhancer::GetDensity(const PointLL& ll, float& localdensity) {
  // Specify radius in km  but turn into meters since internal lengths are m
  float radius = 5.0f;
  float km2 = kPi * radius * radius;
  float r2 = radius * 1000.0f * radius * 1000.0f;

  // Local radius - for use in speed assignments.
  float localradius = 1.0f;
  float localkm2 = kPi * localradius * localradius;
  float localr2 = localradius * 1000.0f * localradius * 1000.0f;

  DistanceApproximator approximator(ll);

  // Get a list of tiles required for a node search within this radius
  float latdeg = (radius / kMetersPerDegreeLat) * 0.5f;
  float mpd = DistanceApproximator::MetersPerLngDegree(ll.lat());
  float lngdeg = (radius / mpd) * 0.5f;
  AABB2 bbox(Point2(ll.lng() - lngdeg, ll.lat() - latdeg),
             Point2(ll.lng() + lngdeg, ll.lat() + latdeg));
  std::vector<int32_t> tilelist = tiles_.TileList(bbox);
if (tilelist.size() > 4) {
  std::cout << "Tilelist size = " << tilelist.size() << std::endl;
}
  // For all tiles needed to find nodes within the radius...find nodes within
  // the radius and add lengths of directed edges
  float roadlengths = 0.0f;
  float localroadlengths = 0.0f;
  for (auto t : tilelist) {
    // Check all the nodes within the tile
    const GraphTile* newtile = graphreader_.GetGraphTile(GraphId(t, local_level_, 0));
    const auto start_node = newtile->node(0);
    const auto end_node   = start_node + newtile->header()->nodecount();
    for (auto node = start_node; node < end_node; ++node) {
      // Check if within radius
      float d = approximator.DistanceSquared(node->latlng());
      if (d < r2) {
        // Get all directed edges and add length
        const DirectedEdge* directededge = newtile->directededge(node->edge_index());
        for (uint32_t i = 0; i < node->edge_count(); i++, directededge++) {
          // Exclude non-roads (parking, walkways, etc.)
          if (directededge->use() == Use::kRoad ||
              directededge->use() == Use::kRamp ||
              directededge->use() == Use::kTurnChannel ||
              directededge->use() == Use::kAlley ||
              directededge->use() == Use::kEmergencyAccess ||
              directededge->use() == Use::kCuldesac) {
            roadlengths += directededge->length();
            if (d < localr2) {
              localroadlengths += directededge->length();
            }
          }
        }
      }
    }
  }

  // Form density measure as km/km^2. Convert roadlengths to km and divide by 2
  // (since 2 directed edges per edge)
  float density = (roadlengths * 0.0005f) / km2;
  localdensity = (localroadlengths * 0.0005f) / localkm2;
  if (density > maxdensity_)
     maxdensity_ = density;
  return static_cast<uint32_t>((density / 24.0f) * 16.0f);

/**
  LOG_INFO("LL: " + std::to_string(ll.lat()) + "," + std::to_string(ll.lng()) +
                     " density= " + std::to_string(density) + " km/km2");
                 + " localdensity= " + std::to_string(localdensity) + " km/km2");
*/
}

// Process edge transitions at an intersection / node.
void GraphEnhancer::ProcessEdgeTransitions(const uint32_t idx,
          DirectedEdgeBuilder& directededge, const DirectedEdge* edges,
          const uint32_t ntrans, uint32_t* heading) {
  for (uint32_t k = 0; k < ntrans; k++) {
    // Get the turn type (reverse the heading of the from directed edge since
    // it is incoming
    uint32_t degree = GetTurnDegree(((heading[k] + 180) % 360), heading[idx]);
    directededge.set_turntype(k, Turn::GetType(degree));

    // Get stop impact
    uint32_t stopimpact = GetStopImpact(k, idx, edges, ntrans);
    directededge.set_stopimpact(k, stopimpact);
  }
}

// Gets the stop likelihoood / impact for the transition between 2 edges
// at an intersection
uint32_t GraphEnhancer::GetStopImpact(const uint32_t from, const uint32_t to,
                       const DirectedEdge* edges, const uint32_t count) {
  // Get the highest classification of other roads at the intersection
  const DirectedEdge* edge = &edges[0];
  RoadClass bestrc = RoadClass::kOther;
  for (uint32_t i = 0; i < count; i++, edge++) {
    // Check the road if it is driveable TO the intersection and not
    // either of the 2 edges (from or to)
    if (i != to && i != from && edge->classification() < bestrc &&
        (edge->reverseaccess() & kAutoAccess)) {
      bestrc = edge->classification();
    }
  }

  // Set stop impact to the difference in road class (make it non-negative)
  int impact = static_cast<int>(edges[from].classification()) -
          static_cast<int>(bestrc);
  uint32_t stop_impact = (impact < -3) ? 0 : impact + 3;

  // TODO:Handle special cases (ramps, turn channels, etc.)

  // TODO: Increase stop impact at large intersections (more edges)

  // TODO:Increase stop level based on classification of edges

  // TODO - clamp to max (create a const)
  return (stop_impact < 8) ? stop_impact : 7;
}

// Get the index of the opposing edge at the end node. This is
// on the local hierarchy (before adding transition and shortcut edges).
uint32_t GraphEnhancer::GetOpposingEdgeIndex(const GraphId& startnode,
                                             const DirectedEdge& edge) {
  // Get the tile at the end node and get the node info
  GraphId endnode = edge.endnode();
  const GraphTile* tile = graphreader_.GetGraphTile(endnode);
  const NodeInfo* nodeinfo = tile->node(endnode.id());

  // Get the directed edges and return when the end node matches
  // the specified node and length matches
  const DirectedEdge* directededge = tile->directededge(
              nodeinfo->edge_index());
  for (uint32_t i = 0; i < nodeinfo->edge_count(); i++, directededge++) {
    // End node must match the start node
    if (directededge->endnode() == startnode &&
        directededge->length() == edge.length()) {
      return i;
    }
  }
  return 15;
}


}
}
