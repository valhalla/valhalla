#include "mjolnir/graphenhancer.h"
#include "mjolnir/idtable.h"
#include "mjolnir/graphtilebuilder.h"

#include <memory>
#include <future>
#include <thread>
#include <mutex>
#include <vector>
#include <list>
#include <unordered_set>
#include <unordered_map>
#include <cinttypes>
#include <limits>

#include <sqlite3.h>
#include <spatialite.h>
#include <boost/filesystem/operations.hpp>

#include <valhalla/midgard/distanceapproximator.h>
#include <valhalla/midgard/pointll.h>
#include <valhalla/baldr/tilehierarchy.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphconstants.h>
#include <valhalla/baldr/graphtile.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/midgard/aabb2.h>
#include <valhalla/midgard/constants.h>
#include <valhalla/midgard/logging.h>

using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::mjolnir;


namespace {

// Number of iterations to try to determine if an edge is unreachable
// by driving. If a search terminates before this without reaching
// a secondary road then the edge is considered unreachable.
constexpr uint32_t kUnreachableIterations = 20;

// Meters offset from start/end of shape for finding heading
constexpr float kMetersOffsetForHeading = 30.0f;

// A little struct to hold stats information during each threads work
struct enhancer_stats {
  float max_density; //(km/km2)
  uint32_t unreachable;
  void operator()(const enhancer_stats& other) {
    if(max_density < other.max_density)
      max_density = other.max_density;
    unreachable += other.unreachable;
  }
};

/**
 * Update directed edge speed based on density and other edge parameters like
 * surface type. TODO - add admin specific logic
 * @param  directededge  Directed edge to update.
 * @param  density       Road density.
 */
void UpdateSpeed(DirectedEdgeBuilder& directededge, const float density) {
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

    // TODO
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

/**
 * Get the road density around the specified lat,lng position. This is a
 * value from 0-15 indicating a relative road density. This can be used
 * in costing methods to help avoid dense, urban areas.
 * @param  reader        Graph reader
 * @param  lock          Mutex for locking while tiles are retrieved
 * @param  ll            Lat,lng position
 * @param  localdensity  (OUT) Local road density (within a smaller radius) -
 *                       might be useful for speed assignment. Units are
 *                       km of road per square kilometer
 * @param  maxdensity    (OUT) max density found
 * @param  tiles         Tiling (for getting list of required tiles)
 * @param  local_level   Level of the local tiles.
 * @return  Returns the relative road density (0-15) - higher values are
 *          more dense.
 */
uint32_t GetDensity(GraphReader& reader, std::mutex& lock, const PointLL& ll,
                    float& localdensity, float& maxdensity, Tiles tiles,
                    uint8_t local_level) {
  // TODO - create constants for the radii we want to use

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
  std::vector<int32_t> tilelist = tiles.TileList(bbox);

  // For all tiles needed to find nodes within the radius...find nodes within
  // the radius and add lengths of directed edges
  float roadlengths = 0.0f;
  float localroadlengths = 0.0f;
  for (auto t : tilelist) {
    // Check all the nodes within the tile
    lock.lock();
    const GraphTile* newtile = reader.GetGraphTile(GraphId(t, local_level, 0));
    lock.unlock();
    if(!newtile)
      continue;
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
  if (density > maxdensity)
     maxdensity = density;
  return static_cast<uint32_t>((density / 32.0f) * 16.0f);
}

uint32_t GetAdminIndex(sqlite3 *db_handle, const std::vector<int32_t>& rowids, const PointLL& ll) {

  uint32_t index = 0;

  if (!rowids.size() || !db_handle)
    return index;

  sqlite3_stmt *stmt = 0;
  uint32_t ret;
  char *err_msg = NULL;

  std::string sql = "SELECT rowid from admins where rowid in (";
  std::string tmp;

  for (auto rowid : rowids) {
    sql += tmp;
    sql += std::to_string(rowid);
    tmp = ", ";
  }

  std::string point = "MakePoint(" + std::to_string(ll.lng());
  point += ", " + std::to_string(ll.lat()) + ")";

  sql += ") and ST_Contains(geom, " + point;
  sql += ") AND ROWID IN (SELECT ROWID FROM SpatialIndex where f_table_name = 'admins' AND search_frame = ";
  sql += point + ") order by admin_level DESC limit 1;";


 // sql += ") and ST_Contains(geom, " + point;
 // sql += ") and rowid in (SELECT rowid FROM idx_admins_geom WHERE xmin <= MbrMaxX(" + point;
 // sql += ") AND xmax >= MbrMinX(" + point + ") AND ymin <= MbrMaxY(" + point;
 // sql += ") AND ymax >= MbrMinY(" + point + ") limit 1) order by admin_level DESC limit 1";

 // std::cout << sql << std::endl;

  ret = sqlite3_prepare_v2(db_handle, sql.c_str(), sql.length(), &stmt, 0);

  if (ret == SQLITE_OK) {
    uint32_t result = 0;
    result = sqlite3_step(stmt);
    while (result == SQLITE_ROW) {
      index = sqlite3_column_int(stmt, 0);
      result = sqlite3_step(stmt);
    }
  }

  if (stmt) {
    sqlite3_finalize(stmt);
    stmt = 0;
  }

  return index;
}

std::vector<int32_t> GetAdmins(sqlite3 *db_handle, const AABB2& aabb, GraphTileBuilder& tilebuilder) {

  std::vector<int32_t> rowids;

  if (!db_handle)
    return rowids;

  sqlite3_stmt *stmt = 0;
  uint32_t ret;
  char *err_msg = NULL;

  std::string sql = "SELECT rowid, name, name_en from admins where ";
  sql += "ST_Intersects(geom, BuildMBR(" + std::to_string(aabb.minx()) + ",";
  sql += std::to_string(aabb.miny()) + ", " + std::to_string(aabb.maxx()) + ",";
  sql += std::to_string(aabb.maxy()) + ")) order by admin_level DESC";

  ret = sqlite3_prepare_v2(db_handle, sql.c_str(), sql.length(), &stmt, 0);

  if (ret == SQLITE_OK) {
    uint32_t result = 0;
    result = sqlite3_step(stmt);
    while (result == SQLITE_ROW) {

      std::vector<std::string> names;

      uint32_t id = sqlite3_column_int(stmt, 0);
      rowids.emplace_back(id);

      if (sqlite3_column_type(stmt, 1) == SQLITE_TEXT)
        names.emplace_back((char*)sqlite3_column_text(stmt, 1));

      if (sqlite3_column_type(stmt, 2) == SQLITE_TEXT)
        names.emplace_back((char*)sqlite3_column_text(stmt, 2));

      tilebuilder.AddAdmin(id,names);

      result = sqlite3_step(stmt);
    }
  }

  if (stmt) {
    sqlite3_finalize(stmt);
    stmt = 0;
  }

  return rowids;

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
 * @param  edges Directed edges outbound from a node.
 * @param  count Number of outbound directed edges to consider.
 * @return  Returns stop impact ranging from 0 (no likely impact) to
 *          7 - large impact.
 */
uint32_t GetStopImpact(const uint32_t from, const uint32_t to,
                       const DirectedEdge* edges, const uint32_t count) {
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

  // TODO:Handle special cases (ramps, turn channels, etc.)

  // TODO: Increase stop impact at large intersections (more edges)

  // TODO:Increase stop level based on classification of edges

  // TODO - clamp to max (create a const)
  return (stop_impact < 8) ? stop_impact : 7;
}

/**
 * Process edge transitions from all other incoming edges onto the
 * specified outbound directed edge.
 * @param  idx            Index of the directed edge - the to edge.
 * @param  directededge   Directed edge builder - set values.
 * @param  edge           Other directed edges at the node.
 * @param  ntrans         Number of transitions (either number of edges or max)
 * @param  start_heading  Headings of directed edges.
 */
void ProcessEdgeTransitions(const uint32_t idx,
          DirectedEdgeBuilder& directededge, const DirectedEdge* edges,
          const uint32_t ntrans, uint32_t* heading) {
  for (uint32_t i = 0; i < ntrans; i++) {
    // Get the turn type (reverse the heading of the from directed edge since
    // it is incoming
    uint32_t from_heading = ((heading[i] + 180) % 360);
    uint32_t turn_degree  = GetTurnDegree(from_heading, heading[idx]);
    directededge.set_turntype(i, Turn::GetType(turn_degree));

    // Get stop impact
    uint32_t stopimpact = GetStopImpact(i, idx, edges, ntrans);
    directededge.set_stopimpact(i, stopimpact);

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
        uint32_t degree = GetTurnDegree(from_heading, heading[j]);
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

// We make sure to lock on reading and writing because we dont want to race
// since difference threads, use for the done map as well
void enhance(GraphReader& reader, IdTable& done_set, std::mutex& lock, std::promise<enhancer_stats>& result) {

  // std::string dir = pt.get<std::string>("admin.admin_dir");
   //std::string db_name = pt.get<std::string>("admin.db_name");

   //std::string database = dir + "/" +  db_name;

 /* std::string database = "/data/valhalla/admin.sqlite";

  sqlite3 *db_handle = nullptr;

  if (boost::filesystem::exists(database)) {

    spatialite_init(0);

    sqlite3_stmt *stmt = 0;
    uint32_t ret;
    char *err_msg = NULL;
    std::string sql;

    ret = sqlite3_open_v2(database.c_str(), &db_handle, SQLITE_OPEN_READONLY, NULL);
    if (ret != SQLITE_OK) {
      LOG_ERROR("cannot open " + database);
      sqlite3_close(db_handle);
      db_handle = NULL;
      return;
    }

    // loading SpatiaLite as an extension
    sqlite3_enable_load_extension(db_handle, 1);
    sql = "SELECT load_extension('libspatialite.so')";
    ret = sqlite3_exec(db_handle, sql.c_str(), NULL, NULL, &err_msg);
    if (ret != SQLITE_OK) {
      LOG_ERROR("load_extension() error: " + std::string(err_msg));
      sqlite3_free(err_msg);
      sqlite3_close(db_handle);
      return;
    }
    LOG_INFO("SpatiaLite loaded as an extension");

  }

  std::vector<int32_t> rowids;
*/
  // Get some things we need throughout
  enhancer_stats stats{std::numeric_limits<float>::min(), 0};
  lock.lock();
  auto tile_hierarchy = reader.GetTileHierarchy();
  auto local_level = tile_hierarchy.levels().rbegin()->second.level;
  auto tiles = tile_hierarchy.levels().rbegin()->second.tiles;
  lock.unlock();

  // Iterate through the tiles and perform enhancements
  for (uint32_t id = 0; id < tiles.TileCount(); id++) {
    GraphId tile_id(id, local_level, 0);

    // If no tile exists skip it
    if(!GraphReader::DoesTileExist(tile_hierarchy, tile_id))
      continue;

    // If someone else is working/worked on this tile we can skip it
    lock.lock();
    if(done_set.IsUsed(id)) {
      lock.unlock();
      continue;
    }
    done_set.set(id);
    lock.unlock();

    // Get writeable and readable tiles
    lock.lock();
    GraphTileBuilder tilebuilder(tile_hierarchy, tile_id);
    const GraphTile* tile = reader.GetGraphTile(tile_id);
    lock.unlock();

    //if (db_handle)
    //  rowids = GetAdmins(db_handle, tiles.TileBounds(id), tilebuilder);

    // Update nodes and directed edges as needed
    std::vector<NodeInfoBuilder> nodes;
    std::vector<DirectedEdgeBuilder> directededges;

    // Iterate through the nodes
    float localdensity = 0.0f;
    uint32_t nodecount = tilebuilder.header()->nodecount();
    for (uint32_t i = 0; i < nodecount; i++) {
      GraphId startnode(id, local_level, i);
      NodeInfoBuilder nodeinfo = tilebuilder.node(i);

      // Get relative road density and local density
      uint32_t density = GetDensity(reader, lock, nodeinfo.latlng(), localdensity, stats.max_density, tiles, local_level);

      // Enhance node attributes (TODO - admin, timezone)

     // uint32_t admin_index = GetAdminIndex(db_handle,rowids, nodeinfo.latlng());

      nodeinfo.set_density(density);

      // Get headings of the edges - set in NodeInfo. Set driveability info
      // on the node as well.
      uint32_t count = nodeinfo.edge_count();
      uint32_t ntrans = std::min(count, kNumberOfEdgeTransitions);
      uint32_t heading[ntrans];
      nodeinfo.set_local_edge_count(ntrans);
      for (uint32_t j = 0; j < ntrans; j++) {
        DirectedEdgeBuilder& directededge = tilebuilder.directededge(nodeinfo.edge_index() + j);
        auto shape = tile->edgeinfo(directededge.edgeinfo_offset())->shape();
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

      // Go through directed edges and update data
      const DirectedEdge* edges = tile->directededge(nodeinfo.edge_index());
      for (uint32_t j = 0, n = nodeinfo.edge_count(); j < n; j++) {
        DirectedEdgeBuilder& directededge = tilebuilder.directededge(nodeinfo.edge_index() + j);
        // Enhance directed edge attributes. TODO - drive_on_right flag
        // and other admin related processing

        // Update speed.
        UpdateSpeed(directededge, localdensity);

        // Edge transitions.
        if (j < kNumberOfEdgeTransitions) {
          ProcessEdgeTransitions(j, directededge, edges, ntrans, heading);
        }

        // TODO - name continuity - set in NodeInfo

        // Set unreachable (driving) flag
        if (IsUnreachable(reader, lock, directededge)) {
          directededge.set_unreachable(true);
          stats.unreachable++;
        }

        // Set the opposing index on the local level
        lock.lock();
        const GraphTile* endnodetile = reader.GetGraphTile(directededge.endnode());
        lock.unlock();
        directededge.set_opp_local_idx(
            GetOpposingEdgeIndex(endnodetile, startnode, directededge));

        // Add the directed edge
        directededges.emplace_back(std::move(directededge));
      }

      // Add the node to the list
      nodes.emplace_back(std::move(nodeinfo));
    }

    // Write the new file
    lock.lock();
    tilebuilder.Update(tile_hierarchy, static_cast<const GraphTileHeaderBuilder&>(*tilebuilder.header()), nodes, directededges);
    lock.unlock();
  }

  // Send back the statistics
  result.set_value(stats);
}

}

namespace valhalla {
namespace mjolnir {

// Enhance the local level of the graph
void GraphEnhancer::Enhance(const boost::property_tree::ptree& pt) {

  // Graphreader
  GraphReader reader(pt);

  // A place to hold worker threads and their results, be they exceptions or otherwise
  std::vector<std::shared_ptr<std::thread> > threads(
    std::max(static_cast<unsigned int>(1), pt.get<unsigned int>("concurrency", std::thread::hardware_concurrency())));
  // A place to hold the results of those threads, be they exceptions or otherwise
  std::list<std::promise<enhancer_stats> > results;
  // A place for the threads to synchronize who is working/worked on what
  IdTable done_set(reader.GetTileHierarchy().levels().rbegin()->second.tiles.TileCount());
  // An atomic object we can use to do the synchronization
  std::mutex lock;

  // Start the threads
  LOG_INFO("Enhancing local graph...");
  for (auto& thread : threads) {
    results.emplace_back();
    thread.reset(new std::thread(enhance, std::ref(reader), std::ref(done_set), std::ref(lock), std::ref(results.back())));
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
}

}
}
