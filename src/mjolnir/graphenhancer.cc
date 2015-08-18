#include "mjolnir/graphenhancer.h"
#include "mjolnir/graphtilebuilder.h"

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

#include <valhalla/midgard/distanceapproximator.h>
#include <valhalla/midgard/pointll.h>
#include <valhalla/baldr/tilehierarchy.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphconstants.h>
#include <valhalla/baldr/graphtile.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/streetnames.h>
#include <valhalla/baldr/streetnames_factory.h>
#include <valhalla/baldr/streetnames_us.h>
#include <valhalla/baldr/admininfo.h>
#include <valhalla/midgard/aabb2.h>
#include <valhalla/midgard/constants.h>
#include <valhalla/midgard/logging.h>
#include <valhalla/midgard/util.h>

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

// Meters offset from start/end of shape for finding heading
constexpr float kMetersOffsetForHeading = 30.0f;

// Radius (km) to use for density
constexpr float kDensityRadius = 2.5f;

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
 */
void UpdateSpeed(DirectedEdgeBuilder& directededge, const uint32_t density) {
  // Set speed on ramps / turn channels
  if (directededge.link()) {
    uint32_t speed = directededge.speed();
    Use use = directededge.use();
    if (use == Use::kTurnChannel) {
      speed = static_cast<uint32_t>((speed * 1.25f) + 0.5f);
    } else if (use == Use::kRamp) {
      RoadClass rc = directededge.classification();
      if (rc == RoadClass::kMotorway) {
        speed = 95;
      } else if (rc == RoadClass::kTrunk) {
        speed = 80;
      } else if (rc == RoadClass::kPrimary) {
        speed = 65;
      } else if (rc == RoadClass::kSecondary) {
        speed = 50;
      } else if (rc == RoadClass::kTertiary) {
        speed = 40;
      } else if (rc == RoadClass::kUnclassified) {
        speed = 35;
      } else {
        speed = 25;
      }
    }
    directededge.set_speed(speed);
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
    if (directededge.railferry()) {
      directededge.set_speed(65);  // 40 MPH
      return;
    } else if (directededge.ferry()) {
      if (directededge.length() < 2000) {
        directededge.set_speed(10);  // 5 knots
      } else if (directededge.length() < 8000) {
        directededge.set_speed(20);  // 10 knots
      } else {
        directededge.set_speed(30);  // 15 knots
      }
      return;
    }

    // TODO
    // Modify speed based on urban/rural region
    if (directededge.classification() == RoadClass::kMotorway ||
        directededge.classification() == RoadClass::kTrunk) {
      // Motorway or trunk - allow higher speed for rural than urban
    } else {
      // Modify speed: high density (urban) vs. low density (rural). Assume
      // the mean density is 8 - anything above that we assume is urban
      if (density > 8) {
        if (directededge.classification() == RoadClass::kPrimary) {
          directededge.set_speed(60);  // 35MPH
        } else if (directededge.classification() == RoadClass::kSecondary) {
          directededge.set_speed(50);  // 30 MPH
        } else if (directededge.classification() == RoadClass::kTertiary) {
          directededge.set_speed(40);  // 25 MPH
        } else if (directededge.classification() == RoadClass::kResidential ||
                   directededge.classification() == RoadClass::kUnclassified) {
          directededge.set_speed(35);  // 20 MPH
        } else {
          directededge.set_speed(25);  // 15 MPH (service/alley)
        }
      }
    }

    // Modify speed based on surface
    if (directededge.surface() >= Surface::kPavedRough) {
      uint32_t speed = directededge.speed();
      if (speed >= 50) {
         directededge.set_speed(speed - 10);
      } else if (speed > 15) {
        directededge.set_speed(speed - 5);
      }
    }
  }
}

/**
 * Update the use for a link (either a kRamp or kTurnChannel)
 * TODO - validate logic with some real world cases.
 */
void UpdateLinkUse(const GraphTile* tile,
                   const GraphTile* endnodetile,
                   const NodeInfoBuilder& nodeinfo,
                   DirectedEdgeBuilder& directededge) {
  // Assume link that has highway = motorway or trunk is a ramp.
  // Also, if length is > kMaxTurnChannelLength or there is an exit
  // sign on the edge we assume this is a ramp
  RoadClass rc = directededge.classification();
  if (rc == RoadClass::kMotorway || rc == RoadClass::kTrunk ||
      directededge.length() > kMaxTurnChannelLength ||
      directededge.exitsign()) {
    directededge.set_use(Use::kRamp);
    return;
  }

  // Both end nodes have to connect to a non-link edge and only 1 link
  // edge (assumed to be this edge). If either end node connects only
  // to "links" this likely indicates a split or fork, which are not so
  // prevalent in turn channels.
  uint32_t nonlinkcount = 0;
  uint32_t linkcount = 0;
  const DirectedEdge* edge = tile->directededge(nodeinfo.edge_index());
  for (uint32_t j = 0; j <  nodeinfo.edge_count(); j++, edge++) {
    if (edge->link()) {
      linkcount++;
    } else {
      nonlinkcount++;
    }
  }
  if (nonlinkcount == 0 || linkcount > 1) {
    directededge.set_use(Use::kRamp);
    return;
  }

  // Get node info at the end node
  // Get the tile at the end node and get the node info
  nonlinkcount = 0;
  linkcount = 0;
  GraphId endnode = directededge.endnode();
  const NodeInfo* node = endnodetile->node(endnode.id());
  edge = endnodetile->directededge(node->edge_index());
  for (uint32_t j = 0; j <  node->edge_count(); j++, edge++) {
    if (edge->link()) {
      linkcount++;
    } else {
      nonlinkcount++;
    }
  }
  if (nonlinkcount == 0 || linkcount > 1) {
    directededge.set_use(Use::kRamp);
    return;
  }
  directededge.set_use(Use::kTurnChannel);
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
                   DirectedEdgeBuilder& directededge) {
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
                   DirectedEdgeBuilder& directededge) {
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
      // Do not allow use of the start edge or any transit edges
      if ((n == 0 && diredge->endnode() == startnode) ||
          diredge->use() >= Use::kRail) {
        continue;
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
                            NodeInfoBuilder& startnodeinfo,
                            DirectedEdgeBuilder& directededge,
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
    auto shape = tile->edgeinfo(diredge->edgeinfo_offset())->shape();
    if (!diredge->forward())
      std::reverse(shape.begin(), shape.end());
    uint32_t to_heading = std::round(PointLL::HeadingAlongPolyline(shape,
                                 kMetersOffsetForHeading));
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
                    float& maxdensity, Tiles<PointLL>& tiles, uint8_t local_level) {
  // Radius is in km - turn into meters
  float kr2 = kDensityRadius * kDensityRadius;
  float mr2 = kr2 * 1000000.0f;

  DistanceApproximator approximator(ll);

  // Get a list of tiles required for a node search within this radius
  float latdeg = (kDensityRadius / kMetersPerDegreeLat) * 0.5f;
  float mpd = DistanceApproximator::MetersPerLngDegree(ll.lat());
  float lngdeg = (kDensityRadius / mpd) * 0.5f;
  AABB2<PointLL> bbox(Point2(ll.lng() - lngdeg, ll.lat() - latdeg),
             Point2(ll.lng() + lngdeg, ll.lat() + latdeg));
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
      float d = approximator.DistanceSquared(node->latlng());
      if (d < mr2) {
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
          }
        }
      }
    }
  }

  // Form density measure as km/km^2. Convert roadlengths to km and divide by 2
  // (since 2 directed edges per edge)
  float density = (roadlengths * 0.0005f) / (kPi * kr2);
  if (density > maxdensity)
     maxdensity = density;

  // Convert density into a relative value from 0-16.
  // Median density is ~4 km / km2
  // Less than 1% have density > 20 km / km2
  float mid = 6.0;
  float m2  = 10.0f;
  float max = 18.0f;  // about 4.5M above this
  uint32_t relative_density;
  if (density < mid) {
    relative_density = static_cast<uint32_t>((density / mid) * 10.0f);
  } else if (density < m2) {
    relative_density = static_cast<uint32_t>(((density - mid) / (m2 - mid)) * 3.0f) + 10;
  } else {
    relative_density = static_cast<uint32_t>(((density - m2) / (max - mid)) * 3.0f) + 13;
  }
  return (relative_density < 16) ? relative_density : 15;
}

// Get admininstrative index
uint32_t GetAdminId(const std::unordered_map<uint32_t,multi_polygon_type>& polys, const PointLL& ll) {
  uint32_t index = 0;
  point_type p(ll.lng(), ll.lat());
  for (const auto& poly : polys) {
    if (boost::geometry::covered_by(p, poly.second))
      return poly.first;
  }
  return index;
}

std::unordered_map<uint32_t,multi_polygon_type> GetAdminInfo(sqlite3 *db_handle, std::unordered_map<uint32_t,bool>& drive_on_right,
                                                             const AABB2<PointLL>& aabb, GraphTileBuilder& tilebuilder) {
  std::unordered_map<uint32_t,multi_polygon_type> polys;
  if (!db_handle)
    return polys;

  sqlite3_stmt *stmt = 0;
  uint32_t ret;
  char *err_msg = nullptr;
  uint32_t result = 0;
  uint32_t id = 0;
  bool dor = true;
  std::string geom;
  std::string country_name, state_name, country_iso, state_iso;

  std::string sql = "SELECT state.rowid, country.name, state.name, country.iso_code, ";
  sql += "state.iso_code, state.drive_on_right, st_astext(state.geom) ";
  sql += "from admins state, admins country where ";
  sql += "ST_Intersects(state.geom, BuildMBR(" + std::to_string(aabb.minx()) + ",";
  sql += std::to_string(aabb.miny()) + ", " + std::to_string(aabb.maxx()) + ",";
  sql += std::to_string(aabb.maxy()) + ")) and ";
  sql += "country.rowid = state.parent_admin and state.admin_level=4 ";
  sql += "and state.rowid IN (SELECT rowid FROM SpatialIndex WHERE f_table_name = ";
  sql += "'admins' AND search_frame = BuildMBR(" + std::to_string(aabb.minx()) + ",";
  sql += std::to_string(aabb.miny()) + ", " + std::to_string(aabb.maxx()) + ",";
  sql += std::to_string(aabb.maxy()) + "));";

  ret = sqlite3_prepare_v2(db_handle, sql.c_str(), sql.length(), &stmt, 0);

  if (ret == SQLITE_OK) {
    result = sqlite3_step(stmt);
    if (result == SQLITE_DONE) { //state/prov not found, try to find country

      sql = "SELECT rowid, name, "", iso_code, "", drive_on_right, st_astext(geom) from ";
      sql += " admins where ST_Intersects(geom, BuildMBR(" + std::to_string(aabb.minx()) + ",";
      sql += std::to_string(aabb.miny()) + ", " + std::to_string(aabb.maxx()) + ",";
      sql += std::to_string(aabb.maxy()) + ")) and admin_level=2 ";
      sql += "and rowid IN (SELECT rowid FROM SpatialIndex WHERE f_table_name = ";
      sql += "'admins' AND search_frame = BuildMBR(" + std::to_string(aabb.minx()) + ",";
      sql += std::to_string(aabb.miny()) + ", " + std::to_string(aabb.maxx()) + ",";
      sql += std::to_string(aabb.maxy()) + "));";

      sqlite3_finalize(stmt);
      stmt = 0;
      ret = sqlite3_prepare_v2(db_handle, sql.c_str(), sql.length(), &stmt, 0);

      if (ret == SQLITE_OK) {
        result = 0;
        result = sqlite3_step(stmt);
      }
    }
    while (result == SQLITE_ROW) {

      id = sqlite3_column_int(stmt, 0);

      country_name = "";
      state_name = "";
      country_iso = "";
      state_iso = "";

      if (sqlite3_column_type(stmt, 1) == SQLITE_TEXT)
        country_name = (char*)sqlite3_column_text(stmt, 1);

      if (sqlite3_column_type(stmt, 2) == SQLITE_TEXT)
        state_name = (char*)sqlite3_column_text(stmt, 2);

      if (sqlite3_column_type(stmt, 3) == SQLITE_TEXT)
        country_iso = (char*)sqlite3_column_text(stmt, 3);

      if (sqlite3_column_type(stmt, 4) == SQLITE_TEXT)
        state_iso = (char*)sqlite3_column_text(stmt, 4);

      dor = true;
      if (sqlite3_column_type(stmt, 5) == SQLITE_INTEGER)
        dor = sqlite3_column_int(stmt, 5);

      geom = "";
      if (sqlite3_column_type(stmt, 6) == SQLITE_TEXT)
        geom = (char*)sqlite3_column_text(stmt, 6);

      uint32_t index = tilebuilder.AddAdmin(country_name,state_name,
                                            country_iso,state_iso,"","");
      multi_polygon_type multi_poly;
      boost::geometry::read_wkt(geom, multi_poly);
      polys.emplace(index, multi_poly);
      drive_on_right.emplace(index, dor);

      result = sqlite3_step(stmt);
    }
  }
  if (stmt) {
    sqlite3_finalize(stmt);
    stmt = 0;
  }

  return polys;
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
                        const DirectedEdgeBuilder& directededge,
                        const DirectedEdge* edges,
                        const NodeInfoBuilder& node_info,
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
                       const DirectedEdgeBuilder& directededge,
                       const DirectedEdge* edges, const uint32_t count,
                       const NodeInfoBuilder& nodeinfo, uint32_t turn_degree,
                       enhancer_stats& stats) {

  ///////////////////////////////////////////////////////////////////////////
  // Special cases.

  // Handle Roundabouts
  if (edges[from].roundabout() && edges[to].roundabout()) {
    return 0;
  }

  // No stop impact going from a turn channel. Stop impact will
  // be set entering a turn channel
  if (edges[from].use() == Use::kTurnChannel) {
    return 0;
  }

  // Handle Pencil point u-turn
  if (IsPencilPointUturn(from, to, directededge, edges, nodeinfo,
                         turn_degree)) {
    stats.pencilucount++;
    return 7;
  }
  ///////////////////////////////////////////////////////////////////////////

  // Get the highest classification of other roads at the intersection
  const DirectedEdge* edge = &edges[0];
  RoadClass bestrc = RoadClass::kServiceOther;
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

  // TODO: possibly increase stop impact at large intersections (more edges)
  // or if several are high class

  // TODO:Increase stop level based on classification of edges

  // TODO - clamp to max (create a const)
  return (stop_impact < 8) ? stop_impact : 7;
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
          DirectedEdgeBuilder& directededge, const DirectedEdge* edges,
          const uint32_t ntrans, uint32_t* headings,
          const NodeInfoBuilder& nodeinfo,
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
             boost::property_tree::ptree& hierarchy_properties,
             std::queue<GraphId>& tilequeue, std::mutex& lock,
             std::promise<enhancer_stats>& result) {
  // Initialize the admin DB (if it exists)
  std::string dir = pt.get<std::string>("admin_dir");
  std::string db_name = pt.get<std::string>("db_name");
  std::string database = dir + "/" +  db_name;
  sqlite3 *db_handle = nullptr;
  if (boost::filesystem::exists(database)) {
    spatialite_init(0);
    sqlite3_stmt* stmt = 0;
    char* err_msg = nullptr;
    std::string sql;
    uint32_t ret = sqlite3_open_v2(database.c_str(), &db_handle,
                        SQLITE_OPEN_READONLY, nullptr);
    if (ret != SQLITE_OK) {
      LOG_ERROR("cannot open " + database);
      sqlite3_close(db_handle);
      db_handle = nullptr;
      return;
    }

    // loading SpatiaLite as an extension
    sqlite3_enable_load_extension(db_handle, 1);
    sql = "SELECT load_extension('libspatialite.so')";
    ret = sqlite3_exec(db_handle, sql.c_str(), nullptr, nullptr, &err_msg);
    if (ret != SQLITE_OK) {
      LOG_ERROR("load_extension() error: " + std::string(err_msg));
      sqlite3_free(err_msg);
      sqlite3_close(db_handle);
      return;
    }
    LOG_INFO("SpatiaLite loaded as an extension");
  }
  else
    LOG_WARN("Admin db " + database + " not found.  Not saving admin information.");

  // Local Graphreader
  GraphReader reader(hierarchy_properties);

  // Get some things we need throughout
  enhancer_stats stats{std::numeric_limits<float>::min(), 0};
  lock.lock();
  auto tile_hierarchy = reader.GetTileHierarchy();
  auto local_level = tile_hierarchy.levels().rbegin()->second.level;
  auto tiles = tile_hierarchy.levels().rbegin()->second.tiles;
  lock.unlock();

  // Iterate through the tiles in the queue and perform enhancements
  while (true) {

    // Get the next tile Id from the queue and get writeable
    // and readable tile
    lock.lock();
    if (tilequeue.empty()) {
      lock.unlock();
      break;
    }
    GraphId tile_id = tilequeue.front();
    uint32_t id  = tile_id.tileid();
    tilequeue.pop();

    std::unordered_map<uint32_t,multi_polygon_type> polys;
    std::unordered_map<uint32_t,bool> drive_on_right;

    // Get a readable tile.If the tile is empty, skip it. Empty tiles are
    // added where ways go through a tile but no end not is within the tile.
    // This allows creation of connectivity maps using the tile set,
    const GraphTile* tile = reader.GetGraphTile(tile_id);
    if (tile->header()->nodecount() == 0) {
      lock.unlock();
      continue;
    }

    // Tile builder - serialize in existing tile so we can add admin names
    GraphTileBuilder tilebuilder(tile_hierarchy, tile_id, true);
    lock.unlock();

    // Create a dummy admin record at index 0. Used if admin records
    // are not used/created or if none is found.
    tilebuilder.AddAdmin("None","None","","","","");

    // Get the admin polygons. If only one exists for the tile check if the
    // tile is entirely inside the polygon
    bool tile_within_one_admin = false;
    if (db_handle) {
      polys = GetAdminInfo(db_handle, drive_on_right, tiles.TileBounds(id),
                           tilebuilder);
      if (polys.size() == 1) {
        // TODO - check if tile bounding box is entirely inside the polygon...
        tile_within_one_admin = true;
      }
    }

    // First pass - update links (set use to ramp or turn channel) and
    // set opposing local index.
    for (uint32_t i = 0; i < tilebuilder.header()->nodecount(); i++) {
      GraphId startnode(id, local_level, i);
      NodeInfoBuilder& nodeinfo = tilebuilder.node_builder(i);

      const DirectedEdge* edges = tile->directededge(nodeinfo.edge_index());
      for (uint32_t j = 0; j <  nodeinfo.edge_count(); j++) {
        DirectedEdgeBuilder& directededge =
            tilebuilder.directededge_builder(nodeinfo.edge_index() + j);

        // Skip transit lines (don't need opposing local index)
        if (directededge.IsTransitLine()) {
          continue;
        }

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
        if (directededge.link()) {
          UpdateLinkUse(tile, endnodetile, nodeinfo, directededge);
          if (directededge.use() == Use::kTurnChannel) {
            stats.turnchannelcount++;
          } else {
            stats.rampcount++;
          }
        }

        // Set the opposing index on the local level
        directededge.set_opp_local_idx(
               GetOpposingEdgeIndex(endnodetile, startnode, directededge));
      }
    }

    // Second pass - add admin information and edge transition information.
    for (uint32_t i = 0; i < tilebuilder.header()->nodecount(); i++) {
      GraphId startnode(id, local_level, i);
      NodeInfoBuilder& nodeinfo = tilebuilder.node_builder(i);

      // Get relative road density and local density
      uint32_t density = GetDensity(reader, lock, nodeinfo.latlng(),
                                    stats.max_density, tiles, local_level);

      // Set admin index
      uint32_t admin_index = (tile_within_one_admin) ?
                    polys.begin()->first :
                    GetAdminId(polys, nodeinfo.latlng());
      nodeinfo.set_admin_index(admin_index);

      // TODO - timezone

      // Set the country code
      std::string country_code = "";
      if (admin_index != 0)
        country_code = tilebuilder.admins_builder(admin_index).country_iso();
      else stats.no_country_found++;

      nodeinfo.set_density(density);
      stats.density_counts[density]++;

      // Get headings of the edges - set in NodeInfo. Set driveability info
      // on the node as well.
      uint32_t count = nodeinfo.edge_count();
      uint32_t ntrans = std::min(count, kNumberOfEdgeTransitions);
      uint32_t heading[ntrans];
      nodeinfo.set_local_edge_count(ntrans);
      for (uint32_t j = 0; j < ntrans; j++) {
        DirectedEdgeBuilder& directededge =
            tilebuilder.directededge_builder(nodeinfo.edge_index() + j);
        auto shape = tilebuilder.edgeinfo(directededge.edgeinfo_offset())->shape();
        if (!directededge.forward())
          std::reverse(shape.begin(), shape.end());
        heading[j] = std::round(PointLL::HeadingAlongPolyline(shape, kMetersOffsetForHeading));

        // Set heading in NodeInfo. TODO - what if 2 edges have nearly the
        // same heading - should one be "adjusted" so the relative direction
        // is maintained.
        nodeinfo.set_heading(j, heading[j]);

        // Set driveability
        Driveability driveability;
        if (directededge.forwardaccess() & kAutoAccess) {
          driveability = (directededge.reverseaccess() & kAutoAccess) ?
              Driveability::kBoth : Driveability::kForward;
        } else {
          driveability = (directededge.reverseaccess() & kAutoAccess) ?
                        Driveability::kBackward : Driveability::kNone;
        }
        nodeinfo.set_local_driveability(j, driveability);
      }

      // Go through directed edges and "enhance" directed edge attributes
      const DirectedEdge* edges = tile->directededge(nodeinfo.edge_index());
      for (uint32_t j = 0; j <  nodeinfo.edge_count(); j++) {
        DirectedEdgeBuilder& directededge =
            tilebuilder.directededge_builder(nodeinfo.edge_index() + j);

        // Update speed.
        UpdateSpeed(directededge, density);

        // Set drive on right flag
        if (admin_index != 0)
          directededge.set_drive_on_right(drive_on_right[admin_index]);

        // Name continuity - set in NodeInfo
        for (uint32_t k = (j + 1); k < ntrans; k++) {
          if (ConsistentNames(country_code,
              tilebuilder.edgeinfo(directededge.edgeinfo_offset())->GetNames(),
              tilebuilder.edgeinfo(
                  tilebuilder.directededge(nodeinfo.edge_index() + k)
                      .edgeinfo_offset())->GetNames())) {
            nodeinfo.set_name_consistency(j, k, true);
          }
        }

        // No need for edge transitions on transit edges
        if (!directededge.IsTransitLine()) {
          // Edge transitions.
          if (j < kNumberOfEdgeTransitions) {
            ProcessEdgeTransitions(j, directededge, edges, ntrans, heading,
                                   nodeinfo, stats);
          }
        }

        // Set unreachable, not_thru, or internal intersection (except
        // for transit)
        if (directededge.use() < Use::kRail) {
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
        }
      }

      // Set the intersection type
      if (nodeinfo.edge_count() == 1) {
        nodeinfo.set_intersection(IntersectionType::kDeadEnd);
      } else if (nodeinfo.edge_count() == 2) {
        if (nodeinfo.type() == NodeType::kGate ||
            nodeinfo.type() == NodeType::kTollBooth) {
          ; // TODO??
        } else {
          nodeinfo.set_intersection(IntersectionType::kFalse);
        }
      }
    }

    // Write the new file
    lock.lock();
    tilebuilder.StoreTileData(tile_hierarchy, tile_id);
    LOG_TRACE((boost::format("GraphEnhancer completed tile %1%") % tile_id).str());

    // Check if we need to clear the tile cache
    if (reader.OverCommitted()) {
      reader.Clear();
    }
    lock.unlock();
  }

  if (db_handle)
    sqlite3_close (db_handle);

  // Send back the statistics
  result.set_value(stats);
}

}

namespace valhalla {
namespace mjolnir {

// Enhance the local level of the graph
void GraphEnhancer::Enhance(const boost::property_tree::ptree& pt) {
  // A place to hold worker threads and their results, exceptions or otherwise
  std::vector<std::shared_ptr<std::thread> > threads(
    std::max(static_cast<unsigned int>(1),
    pt.get<unsigned int>("concurrency", std::thread::hardware_concurrency())));

  // A place to hold the results of those threads, exceptions or otherwise
  std::list<std::promise<enhancer_stats> > results;

  // Create a randomized queue of tiles to work from
  std::deque<GraphId> tempqueue;
  boost::property_tree::ptree hierarchy_properties = pt.get_child("mjolnir.hierarchy");
  GraphReader reader(hierarchy_properties);
  auto tile_hierarchy = reader.GetTileHierarchy();
  auto local_level = tile_hierarchy.levels().rbegin()->second.level;
  auto tiles = tile_hierarchy.levels().rbegin()->second.tiles;
  for (uint32_t id = 0; id < tiles.TileCount(); id++) {
    // If tile exists add it to the queue
    GraphId tile_id(id, local_level, 0);
    if (GraphReader::DoesTileExist(tile_hierarchy, tile_id)) {
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
                 std::ref(pt.get_child("mjolnir.admin")),
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
