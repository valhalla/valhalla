#include "mjolnir/graphenhancer.h"

#include <valhalla/midgard/aabb2.h>
#include <valhalla/midgard/constants.h>
#include <valhalla/midgard/logging.h>

using namespace valhalla::midgard;
using namespace valhalla::baldr;

namespace valhalla {
namespace mjolnir {

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
  for (uint32_t tileid = 0; tileid < tiles_.TileCount(); tileid++) {
    // Get the graph tile. Skip if no tile exists (common case)
    GraphTileBuilder tilebuilder(tile_hierarchy_, GraphId(tileid, local_level_, 0));
    if (tilebuilder.size() == 0) {
      continue;
    }

    // Copy existing header. No need to update any counts or offsets.
    GraphTileHeader existinghdr = *(tilebuilder.header());
    GraphTileHeaderBuilder hdrbuilder =
       static_cast<GraphTileHeaderBuilder&>(existinghdr);

    // Update nodes and directed edges as needed
    std::vector<NodeInfoBuilder> nodes;
    std::vector<DirectedEdgeBuilder> directededges;

    // Iterate through the nodes
    uint32_t nodecount = tilebuilder.header()->nodecount();
    GraphId nodeid(tileid, local_level_, 0);
    for (uint32_t i = 0; i < nodecount; i++) {
      NodeInfoBuilder nodeinfo = tilebuilder.node(i);

      // Enhance node attributes
      uint32_t density = GetNodeDensity(nodeinfo);
      nodeinfo.set_density(density);
 //if (nodeinfo.density() > 0) {
 //  LOG_INFO("NodeInfo density = " + std::to_string(density));
 //}

      // Go through directed edges and update data
      for (uint32_t j = 0, n = nodeinfo.edge_count(); j < n; j++) {
        DirectedEdgeBuilder& directededge = tilebuilder.directededge(
                                nodeinfo.edge_index() + j);

        // TODO - enhance directed edge attributes

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
  return true;
}

// Get the node density. Computes road density (km/km squared) with
// 5 km of the node. TODO - make this a constant or configurable?
uint32_t GraphEnhancer::GetNodeDensity(NodeInfoBuilder& nodeinfo) {
  // Specify radius in km  but turn into meters since internal lengths are m
  float radius = 5.0f;
  float km2 = kPi * radius * radius;
  float r2 = radius * 1000.0f * radius * 1000.0f;
/*
  // Local radius (In case we want the notion of a local radius and larger
  // region density)
  float localradius = 2.0f;  // 2km
  float localkm2 = kPi * localradius * localradius;
  float localr2 = localradius * 1000.0f * localradius * 1000.0f;
*/
  const PointLL& ll = nodeinfo.latlng();
  DistanceApproximator approximator(ll);

  // Get a list of tiles required for a node search within this radius
  float latdeg = (radius / kMetersPerDegreeLat) * 0.5f;
  float mpd = DistanceApproximator::MetersPerLngDegree(ll.lat());
  float lngdeg = (radius / mpd) * 0.5f;
  AABB2 bbox(Point2(ll.lng() - lngdeg, ll.lat() - latdeg),
             Point2(ll.lng() + lngdeg, ll.lat() + latdeg));
  std::vector<int32_t> tilelist = tiles_.TileList(bbox);

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
        for (uint32_t i = 0, n = node->edge_count(); i < n;
                  i++, directededge++) {
          // Exclude non-roads (parking, walkways, etc.)
          if (directededge->use() == Use::kRoad ||
              directededge->use() == Use::kRamp ||
              directededge->use() == Use::kTurnChannel ||
              directededge->use() == Use::kAlley ||
              directededge->use() == Use::kEmergencyAccess ||
              directededge->use() == Use::kCuldesac) {
            roadlengths += directededge->length();
 //           if (d < localr2) {
 //             localroadlengths += directededge->length();
 //           }
          }
        }
      }
    }
  }

  // Form density measure as km/km^2. Convert roadlengths to km and divide by 2
  // (since 2 directed edges per edge)
  float density = (roadlengths * 0.001f * 0.5f) / km2;
  if (density > maxdensity_)
     maxdensity_ = density;
  return static_cast<uint32_t>((density / 24.0f) * 16.0f);

/**
  float density = (roadlengths * 0.001f * 0.5f) / km2;
  float localdensity = (localroadlengths * 0.001f * 0.5f) / localkm2;
  LOG_INFO("LL: " + std::to_string(ll.lat()) + "," + std::to_string(ll.lng()) +
                     " density= " + std::to_string(density) + " km/km2");
                 + " localdensity= " + std::to_string(localdensity) + " km/km2");
  if (density > maxdensity_)
    maxdensity_ = density;
  if (localdensity > maxlocaldensity_)
    maxlocaldensity_ = localdensity;
*/
}

}
}
