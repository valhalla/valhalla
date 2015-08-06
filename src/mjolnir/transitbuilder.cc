#include "mjolnir/transitbuilder.h"
#include "mjolnir/graphtilebuilder.h"

#include <list>
#include <future>
#include <thread>
#include <mutex>
#include <vector>
#include <queue>
#include <unordered_map>
#include <sqlite3.h>
#include <spatialite.h>
#include <boost/filesystem/operations.hpp>

#include <valhalla/baldr/datetime.h>
#include <valhalla/baldr/graphtile.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/midgard/logging.h>

using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::baldr::DateTime;
using namespace valhalla::mjolnir;

namespace {

struct Stop {
  // Need to add onestop Id, connections (wayid, lat,lon)
  GraphId graphid;
  uint64_t way_id;
  uint32_t key;
  uint32_t type;
  uint32_t parent;
  uint32_t conn_count;     // Number of connections to OSM nodes
  PointLL  ll;
  std::string onestop_id;
  std::string id;
  std::string code;
  std::string name;
  std::string desc;
  std::string zoneid;
};

struct Departure {
  uint32_t key;
  uint32_t orig_stop;
  uint32_t dest_stop;
  uint32_t trip;
  uint32_t route;
  uint32_t blockid;
  uint32_t service;
  uint32_t shapeid;
  uint32_t dep_time;
  uint32_t arr_time;
  uint32_t start_date;
  uint32_t end_date;
  uint32_t dow;
  uint32_t has_subtractions;
  std::string headsign;
};

// Unique route and stop
struct TransitLine {
  uint32_t lineid;
  uint32_t routeid;
  uint32_t stopid;
  uint32_t shapeid;

  TransitLine(const uint32_t d, const uint32_t r, const uint32_t s,
              const uint32_t sh)
      : lineid(d),
        routeid(r),
        stopid(s),
        shapeid(sh) {
  }
};

struct StopEdges {
  uint32_t stop_key;                    // Stop key
  std::vector<uint32_t> intrastation;   // List of intra-station connections
  std::vector<TransitLine> lines;       // Set of unique route/stop pairs
};

struct OSMConnectionEdge {
  GraphId osm_node;
  GraphId stop_node;
  uint32_t stop_key;   // Transit stop key (the to node)
  float length;
  std::vector<PointLL> shape;

  OSMConnectionEdge(const GraphId& f, const GraphId& t, const uint32_t k,
                    const float l, const std::vector<PointLL>& s)
      :  osm_node(f),
         stop_node(t),
         stop_key(k),
         length(l),
         shape(s) {
  }

  // operator < for sorting
  bool operator < (const OSMConnectionEdge& other) const {
    if (osm_node.tileid() == other.osm_node.tileid()) {
      return osm_node.id() < other.osm_node.id();
    } else {
      return osm_node.tileid() < other.osm_node.tileid();
    }
  }
};

// Structure to hold stops from each thread
struct stop_results {
  std::vector<GraphId> tiles;
  std::vector<Stop> stops;

  // Accumulate stops from all threads
  void operator()(const stop_results& other) {
    // Append to the list of tiles that include transit stops
    if (tiles.empty()) {
      tiles = std::move(other.tiles);
    } else {
      tiles.reserve(tiles.size() + other.tiles.size());
      std::move(std::begin(other.tiles), std::end(other.tiles),
                std::back_inserter(tiles));
    }

    // Append to the stop list
    if (stops.empty()) {
      stops = std::move(other.stops);
    } else {
      stops.reserve(stops.size() + other.stops.size());
      std::move(std::begin(other.stops), std::end(other.stops),
                std::back_inserter(stops));
    }
  }
};

// Struct to hold stats information during each threads work
struct builder_stats {
  uint32_t stats;

  // Accumulate stats from all threads
  void operator()(const builder_stats& other) {
    stats += other.stats;
  }
};

// Get stops within a tile's bounding box
std::vector<Stop> GetStops(sqlite3 *db_handle, const AABB2<PointLL>& aabb) {
  // Form query -- for now ignore egress points

  std::string sql = "SELECT stop_key, stop_id, onestop_id, osm_way_id,";
  sql += "stop_code, stop_name,stop_desc, zone_id, location_type, ";
  sql += "parent_station_key,stop_lat, stop_lon from stops where location_type <> 2 and ";
  sql += "ST_Intersects(geom, BuildMBR(" + std::to_string(aabb.minx()) + ",";
  sql += std::to_string(aabb.miny()) + ", " + std::to_string(aabb.maxx()) + ",";
  sql += std::to_string(aabb.maxy()) + ")) ";
  sql += "and rowid IN (SELECT rowid FROM SpatialIndex WHERE f_table_name = ";
  sql += "'stops' AND search_frame = BuildMBR(" + std::to_string(aabb.minx()) + ",";
  sql += std::to_string(aabb.miny()) + ", " + std::to_string(aabb.maxx()) + ",";
  sql += std::to_string(aabb.maxy()) + "));";

  std::vector<Stop> stops;
  sqlite3_stmt* stmt = 0;
  uint32_t ret = sqlite3_prepare_v2(db_handle, sql.c_str(), sql.length(), &stmt, 0);
  if (ret == SQLITE_OK) {
    uint32_t result = sqlite3_step(stmt);
    while (result == SQLITE_ROW) {
      Stop stop;
      stop.key    = sqlite3_column_int(stmt, 0);
      stop.id     = std::string( reinterpret_cast< const char* >((sqlite3_column_text(stmt, 1))));
      stop.onestop_id = std::string( reinterpret_cast< const char* >((sqlite3_column_text(stmt, 2))));
      stop.way_id = sqlite3_column_int64(stmt, 3);
      stop.code   = (sqlite3_column_type(stmt, 4) == SQLITE_TEXT) ?
                    std::string( reinterpret_cast< const char* >(sqlite3_column_text(stmt, 4))) : "";
      stop.name   = (sqlite3_column_type(stmt, 5) == SQLITE_TEXT) ?
                    std::string( reinterpret_cast< const char* >((sqlite3_column_text(stmt, 5)))) : "";
      stop.desc   = (sqlite3_column_type(stmt, 6) == SQLITE_TEXT) ?
                    std::string( reinterpret_cast< const char* >((sqlite3_column_text(stmt, 6)))) : "";
      stop.zoneid = (sqlite3_column_type(stmt, 7) == SQLITE_TEXT) ?
                    std::string( reinterpret_cast< const char* >((sqlite3_column_text(stmt, 7)))) : "";
      stop.type   = sqlite3_column_int(stmt, 8);
      stop.parent = sqlite3_column_int(stmt, 9);
      stop.ll.Set(static_cast<float>(sqlite3_column_double(stmt, 11)),
                  static_cast<float>(sqlite3_column_double(stmt, 10)));

      stops.emplace_back(std::move(stop));

      // TODO - perhaps keep a map of parent/child relations

      result = sqlite3_step(stmt);
    }
  }

  if (stmt) {
    sqlite3_finalize(stmt);
    stmt = 0;
  }
  return stops;
}

// Lock on queue access since we are using it in different threads. No need
// to lock graphreader since no threads are writing tiles yet.
void assign_graphids(const boost::property_tree::ptree& pt,
           boost::property_tree::ptree& hierarchy_properties,
           std::queue<GraphId>& tilequeue, std::mutex& lock,
           std::promise<stop_results>& stop_res) {
  // Construct the transit database
  stop_results stats{};
  std::string dir = pt.get<std::string>("transit_dir");
  std::string db_name = pt.get<std::string>("db_name");
  std::string database = dir + "/" +  db_name;

  // Make sure it exists
  sqlite3* db_handle = nullptr;
  if (boost::filesystem::exists(database)) {
    spatialite_init(0);
    sqlite3_stmt* stmt = 0;
    uint32_t ret;
    char* err_msg = nullptr;
    std::string sql;
    ret = sqlite3_open_v2(database.c_str(), &db_handle,
                          SQLITE_OPEN_READONLY, NULL);
    if (ret != SQLITE_OK) {
      LOG_ERROR("cannot open " + database);
      sqlite3_close(db_handle);
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
  else {
    LOG_INFO("Transit db " + database + " not found.  Transit will not be added.");
    stop_res.set_value(stats);
    return;
  }

  // Local Graphreader. Get tile information so we can find bounding boxes
  GraphReader reader(hierarchy_properties);
  auto tile_hierarchy = reader.GetTileHierarchy();
  auto local_level = tile_hierarchy.levels().rbegin()->second.level;
  auto tiles = tile_hierarchy.levels().rbegin()->second.tiles;

  // Iterate through the tiles in the queue and find any that include stops
  while (true) {
    // Get the next tile Id from the queue
    lock.lock();
    if (tilequeue.empty()) {
      lock.unlock();
      break;
    }
    GraphId tile_id = tilequeue.front();
    uint32_t id  = tile_id.tileid();
    tilequeue.pop();
    lock.unlock();

    // Get stops within the tile. If any exist, assign GraphIds and add to
    // the map/list
    std::vector<Stop> stops = GetStops(db_handle, tiles.TileBounds(id));
    if (stops.size() > 0) {
      LOG_DEBUG("Tile has " + std::to_string(stops.size()) + " stops");

      // Get the number of nodes in the tile so we can assign graph Ids
      uint32_t n = reader.GetGraphTile(tile_id)->header()->nodecount();
      for (auto& stop : stops) {
        stop.graphid = GraphId(tile_id.tileid(), tile_id.level(), n);
        n++;
      }

      // Add all stops and the tile to the results
      if (stats.stops.empty()) {
        stats.stops = std::move(stops);
      } else {
        stats.stops.reserve(stats.stops.size() + stops.size());
        std::move(std::begin(stops), std::end(stops),
                  std::back_inserter(stats.stops));
      }
      stats.tiles.push_back(tile_id);
    }
  }

  if (db_handle)
    sqlite3_close(db_handle);

  // Send back the statistics
  stop_res.set_value(stats);
}

// Get scheduled departures for a stop
std::vector<Departure> GetDepartures(sqlite3* db_handle,
                                     const uint32_t stop_key) {
  // Form query
  // TODO - add shape Id
  std::string sql = "SELECT schedule_key, origin_stop_key, dest_stop_key,";
  sql += "trip_key, route_key, service_key, shape_key, departure_time, arrival_time,";
  sql += "start_date, end_date, dow_mask, has_subtractions, headsign from schedule where ";
  sql += "origin_stop_key = " + std::to_string(stop_key);

  // TODO:  Select wheelchair_accessible and bikes_allowed from the db for the departures
  // TODO:  so that costing can be applied for bike and wheelchair accessibility.
  // TODO:  This will require updates to data structure TransitDeparture.

  std::vector<Departure> departures;
  sqlite3_stmt* stmt = 0;
  uint32_t ret = sqlite3_prepare_v2(db_handle, sql.c_str(), sql.length(), &stmt, 0);
  if (ret == SQLITE_OK) {
    uint32_t result = sqlite3_step(stmt);
    while (result == SQLITE_ROW) {
      Departure dep;
      dep.key       = sqlite3_column_int(stmt, 0);
      dep.orig_stop = sqlite3_column_int(stmt, 1);
      dep.dest_stop = sqlite3_column_int(stmt, 2);
      dep.trip      = sqlite3_column_int(stmt, 3);
      dep.route     = sqlite3_column_int(stmt, 4);
      dep.service   = sqlite3_column_int(stmt, 5);
      dep.shapeid   = sqlite3_column_int(stmt, 6);

      dep.dep_time = DateTime::seconds_from_midnight(std::string( reinterpret_cast< const char* >(sqlite3_column_text(stmt, 7))));
      dep.arr_time = DateTime::seconds_from_midnight(std::string( reinterpret_cast< const char* >(sqlite3_column_text(stmt, 8))));
      dep.start_date = DateTime::days_from_pivot_date(std::string( reinterpret_cast< const char* >(sqlite3_column_text(stmt, 9))));
      dep.end_date = DateTime::days_from_pivot_date(std::string( reinterpret_cast< const char* >(sqlite3_column_text(stmt, 10))));

      dep.dow        = sqlite3_column_int(stmt, 11);
      dep.has_subtractions = sqlite3_column_int(stmt, 12);
      dep.blockid    = sqlite3_column_int(stmt, 13);
      dep.headsign   = (sqlite3_column_type(stmt, 14) == SQLITE_TEXT) ?
                         std::string( reinterpret_cast< const char* >(sqlite3_column_text(stmt, 14))) : "";

      // TODO - configure to reject any where calendar end date is in the past!
      if (dep.end_date > 500) {
        departures.emplace_back(std::move(dep));
      }

      result = sqlite3_step(stmt);
    }
  }
  if (stmt) {
    sqlite3_finalize(stmt);
    stmt = 0;
  }
  return departures;
}

// Add routes to the tile. Return a map of route types vs. id/key.
std::unordered_map<uint32_t, uint32_t> AddRoutes(sqlite3* db_handle,
                   const std::unordered_set<uint32_t>& keys,
                   GraphTileBuilder& tilebuilder) {
  // Map of route keys vs. types
  std::unordered_map<uint32_t, uint32_t> route_types;

  // Iterate through all route keys
  uint32_t n = 0;
  for (const auto& key : keys) {
    // Skip (do not need): route_id, route_color, route_text_color, and
    // route_url
    std::string sql = "SELECT route_key, agency_key, route_short_name,";
    sql += "route_long_name, route_desc, route_type from routes where ";
    sql += "route_key = " + std::to_string(key);

    sqlite3_stmt* stmt = 0;
    uint32_t ret = sqlite3_prepare_v2(db_handle, sql.c_str(), sql.length(), &stmt, 0);
    if (ret == SQLITE_OK) {
      uint32_t result = sqlite3_step(stmt);
      while (result == SQLITE_ROW) {
        uint32_t routeid  = sqlite3_column_int(stmt, 0);
        uint32_t agencyid = sqlite3_column_int(stmt, 1);
        std::string tl_routeid = "";  // TODO
        std::string shortname = (sqlite3_column_type(stmt, 2) == SQLITE_TEXT) ?
                    (char*)sqlite3_column_text(stmt, 2) : "";
        std::string longname = (sqlite3_column_type(stmt, 3) == SQLITE_TEXT) ?
                    (char*)sqlite3_column_text(stmt, 3) : "";
        std::string desc = (sqlite3_column_type(stmt, 4) == SQLITE_TEXT) ?
                    (char*)sqlite3_column_text(stmt, 4) : "";
        uint32_t type = sqlite3_column_int(stmt, 5);

        // Add names and create the transit route
        TransitRoute route(routeid, agencyid, tl_routeid.c_str(),
                           tilebuilder.AddName(shortname),
                           tilebuilder.AddName(longname),
                           tilebuilder.AddName(desc));
        tilebuilder.AddTransitRoute(route);
        n++;

        // Route type - need this to store in edge?
        route_types[routeid] = type;

        result = sqlite3_step(stmt);
      }
    } else {
      LOG_ERROR("Bad query");
    }
    if (stmt) {
      sqlite3_finalize(stmt);
      stmt = 0;
    }
  }
  LOG_DEBUG("Added " + std::to_string(n) + " routes");
  return route_types;
}

// Get trips
std::unordered_map<uint32_t, uint32_t> AddTrips(sqlite3* db_handle,
                   const std::unordered_set<uint32_t>& keys,
                   GraphTileBuilder& tilebuilder) {
  // Map of trip keys vs. shape keys
  std::unordered_map<uint32_t, uint32_t> shape_keys;

  uint32_t n = 0;
  for (const auto& key : keys) {
    // Form query. Skip: service_id, trip_id, direction_id
    std::string sql = "SELECT trip_key, route_key, trip_headsign,";
    sql += "trip_short_name, shape_key from trips where ";
    sql += "trip_key = " + std::to_string(key);

    // TODO:  Select wheelchair_accessible and bikes_allowed from the db for the trips
    // TODO:  so that costing can be applied for bike and wheelchair accessibility.
    // TODO:  This will require updates to data structure TransitTrip.

    sqlite3_stmt* stmt = 0;
    uint32_t ret = sqlite3_prepare_v2(db_handle, sql.c_str(), sql.length(), &stmt, 0);
    if (ret == SQLITE_OK) {
      uint32_t result = sqlite3_step(stmt);
      while (result == SQLITE_ROW) {
        uint32_t tripid  = sqlite3_column_int(stmt, 0);
        uint32_t routeid = sqlite3_column_int(stmt, 1);
        const char* tl_tripid = "";  // TODO - add to table
        std::string headsign = (sqlite3_column_type(stmt, 2) == SQLITE_TEXT) ?
                                    (char*)sqlite3_column_text(stmt, 2) : "";
        std::string shortname = (sqlite3_column_type(stmt, 3) == SQLITE_TEXT) ?
                                    (char*)sqlite3_column_text(stmt, 3) : "";
        uint32_t shapeid = sqlite3_column_int(stmt, 4);

        // Add names and create transit trip
        TransitTrip trip(tripid, routeid, tl_tripid,
                         tilebuilder.AddName(shortname),
                         tilebuilder.AddName(headsign));
        tilebuilder.AddTransitTrip(trip);
        n++;

        shape_keys[tripid] = shapeid;

        result = sqlite3_step(stmt);
      }
    }
    if (stmt) {
      sqlite3_finalize(stmt);
      stmt = 0;
    }
  }
  LOG_DEBUG("Added " + std::to_string(n) + " trips");
  return shape_keys;
}

// Add calendar exceptions
void AddCalendar(sqlite3* db_handle, const std::unordered_set<uint32_t>& keys,
                 GraphTileBuilder& tilebuilder) {

  // Query each unique service Id and add any calendar exceptions
  for (const auto& key : keys) {
    // Skip service_id
    std::string sql = "SELECT service_key, date, exception_type from ";
    sql += "calendar_dates where service_key = " + std::to_string(key);

    sqlite3_stmt* stmt = 0;
    uint32_t ret = sqlite3_prepare_v2(db_handle, sql.c_str(), sql.length(), &stmt, 0);
    if (ret == SQLITE_OK) {
      uint32_t result = sqlite3_step(stmt);
      while (result == SQLITE_ROW) {
        uint32_t serviceid = sqlite3_column_int(stmt, 0);
        uint32_t date = DateTime::days_from_pivot_date(std::string( reinterpret_cast< const char* >(sqlite3_column_text(stmt, 1))));
        uint32_t t = sqlite3_column_int(stmt, 2);

        TransitCalendar calendar(serviceid, date,
                                 static_cast<CalendarExceptionType>(t));
        tilebuilder.AddTransitCalendar(calendar);

        result = sqlite3_step(stmt);
      }
    }
    if (stmt) {
      sqlite3_finalize(stmt);
      stmt = 0;
    }
  }
}

// Add transfers from a stop
void AddTransfers(sqlite3* db_handle, const uint32_t stop_key,
                  GraphTileBuilder& tilebuilder) {
  // Query transfers to see if any exist from the specified stop
  // Skip service_id
  std::string sql = "SELECT from_stop_key, to_stop_key, transfer_type, ";
  sql += "min_transfer_time from transfers where from_stop_key = ";
  sql += std::to_string(stop_key);

  sqlite3_stmt* stmt = 0;
  uint32_t ret = sqlite3_prepare_v2(db_handle, sql.c_str(), sql.length(), &stmt, 0);
  if (ret == SQLITE_OK) {
    uint32_t result = sqlite3_step(stmt);
    while (result == SQLITE_ROW) {
      uint32_t fromstop = sqlite3_column_int(stmt, 0);
      uint32_t tostop   = sqlite3_column_int(stmt, 1);
      uint32_t type     = sqlite3_column_int(stmt, 2);
      uint32_t mintime  = sqlite3_column_int(stmt, 3);
      TransitTransfer transfer(fromstop, tostop,
                 static_cast<TransferType>(type), mintime);
      tilebuilder.AddTransitTransfer(transfer);

      result = sqlite3_step(stmt);
    }
  }
  if (stmt) {
    sqlite3_finalize(stmt);
    stmt = 0;
  }
}

// Get Use given the transit route type
// TODO - add separate Use for different types
Use GetTransitUse(const uint32_t rt) {
  switch (rt) {
    default:
  case 0:       // Tram, streetcar, lightrail
  case 1:       // Subway, metro
  case 2:       // Rail
  case 5:       // Cable car
  case 6:       // Gondola (suspended ferry car)
  case 7:       // Funicular (steep incline)
    return Use::kRail;
  case 3:       // Bus
    return Use::kBus;
  case 4:       // Ferry (boat)
    return Use::kRail;    // TODO - add ferry use
  }
}

std::vector<PointLL> GetShape(sqlite3* db_handle,
                              const PointLL& stop_ll,
                              const PointLL& endstop_ll,
                              const uint32_t shapeid) {
  std::vector<PointLL> shape;

  if (shapeid != 0 && stop_ll != endstop_ll) {
    // Query the shape from the DB based on the shapeid.
    std::string sql = "SELECT shape_pt_lon, shape_pt_lat from shapes ";
    sql += "where shape_key = ";
    sql += std::to_string(shapeid);
    sql += " order by shape_pt_sequence;";

    PointLL  ll;
    std::vector<PointLL> trip_shape;
    sqlite3_stmt* stmt = 0;
    uint32_t ret = sqlite3_prepare_v2(db_handle, sql.c_str(), sql.length(), &stmt, 0);
    if (ret == SQLITE_OK) {
      uint32_t result = sqlite3_step(stmt);
      while (result == SQLITE_ROW) {

        ll.Set(static_cast<float>(sqlite3_column_double(stmt, 0)),
               static_cast<float>(sqlite3_column_double(stmt, 1)));

        trip_shape.push_back(ll);
        result = sqlite3_step(stmt);
      }
    }
    if (stmt) {
      sqlite3_finalize(stmt);
      stmt = 0;
    }

    auto start = stop_ll.ClosestPoint(trip_shape);
    auto end = endstop_ll.ClosestPoint(trip_shape);

    auto start_idx = std::get<2>(start);
    auto end_idx = std::get<2>(end);

    shape.push_back(stop_ll);

    if (start_idx < end_idx) { //forward direction

      if ((trip_shape.at(start_idx)) == stop_ll) // avoid dups
        start_idx++;

      std::copy(trip_shape.begin()+start_idx, trip_shape.begin()+end_idx, back_inserter(shape));

      if ((trip_shape.at(end_idx)) != endstop_ll)
        shape.push_back(endstop_ll);
    }
    else if (start_idx > end_idx) { //backwards

      if ((trip_shape.at(end_idx)) == stop_ll)
        end_idx++;

      std::reverse_copy(trip_shape.begin()+end_idx, trip_shape.begin()+start_idx, back_inserter(shape));

      if ((trip_shape.at(start_idx)) != endstop_ll) // avoid dups
        shape.push_back(endstop_ll);
    }
    else
      shape.push_back(endstop_ll);
  } else {
    shape.push_back(stop_ll);
    shape.push_back(endstop_ll);
  }

  return shape;
}

void AddToGraph(sqlite3* db_handle,
                GraphTileBuilder& tilebuilder,
                std::map<GraphId, StopEdges>& stop_edge_map,
                std::vector<Stop>& stops,
                std::vector<OSMConnectionEdge>& connection_edges,
                std::unordered_map<uint32_t, uint32_t>& stop_indexes,
                std::unordered_map<uint32_t, uint32_t>& route_types) {
  // Copy existing nodes and directed edge builder vectors and clear the lists
  std::vector<NodeInfoBuilder> currentnodes(tilebuilder.nodes());
  tilebuilder.ClearNodes();
  std::vector<DirectedEdgeBuilder> currentedges(tilebuilder.directededges());
  tilebuilder.ClearDirectedEdges();

  LOG_DEBUG("AddToGraph for tileID: " + std::to_string(tilebuilder.header()->graphid().tileid()) +
         " current directed edge count = " + std::to_string(currentedges.size()) +
         " current node count = " + std::to_string(currentnodes.size()));

  // Get the directed edge index of the first sign. If no signs are
  // present in this tile set a value > number of directed edges
  uint32_t nextsignidx = (tilebuilder.header()->signcount() > 0) ?
    tilebuilder.sign(0).edgeindex() : currentedges.size() + 1;

  // Iterate through the nodes - add back any stored edges and insert any
  // connections from a node to a transit stop. Update each nodes edge index.
  uint32_t nodeid = 0;
  uint32_t added_edges = 0;
  uint32_t signidx = 0;
  uint32_t signcount = tilebuilder.header()->signcount();
  for (auto& nb : currentnodes) {
    // Copy existing directed edges from this node and update any signs using
    // the directed edge index
    std::vector<DirectedEdgeBuilder> directededges;
    for (uint32_t i = 0, idx = nb.edge_index(); i < nb.edge_count(); i++, idx++) {
      directededges.emplace_back(std::move(currentedges[idx]));

      // Update any signs that use this idx - increment their index by the
      // number of added edges
      while (idx == nextsignidx && signidx < signcount) {
        if (!currentedges[idx].exitsign()) {
          LOG_ERROR("Signs for this index but directededge says no sign");
        }
        tilebuilder.sign_builder(signidx).set_edgeindex(idx + added_edges);

        // Increment to the next sign and update nextsignidx
        signidx++;
        nextsignidx = (signidx >= signcount) ?
             0 : tilebuilder.sign(signidx).edgeindex();
      }
    }

    // Add directed edges for any connections from the OSM node
    // to a transit stop
    while (added_edges < connection_edges.size() &&
           connection_edges[added_edges].osm_node.id() == nodeid) {
      DirectedEdgeBuilder directededge;
      OSMConnectionEdge& conn = connection_edges[added_edges];
      Stop& stop = stops[stop_indexes[conn.stop_key]];
      directededge.set_endnode(conn.stop_node);
      directededge.set_length(conn.length);
      directededge.set_use(Use::kTransitConnection);
      directededge.set_speed(5);
      directededge.set_classification(RoadClass::kServiceOther);
      directededge.set_localedgeidx(directededges.size());
      directededge.set_pedestrianaccess(true, true);
      directededge.set_pedestrianaccess(false, true);

      // Add edge info to the tile and set the offset in the directed edge
      bool added = false;
      std::vector<std::string> names;
      uint32_t edge_info_offset = tilebuilder.AddEdgeInfo(0, conn.osm_node,
                     conn.stop_node, 0, conn.shape, names, added);
      directededge.set_edgeinfo_offset(edge_info_offset);
      directededge.set_forward(added);
      directededges.emplace_back(std::move(directededge));

      LOG_DEBUG("Add conn from OSM to stop: ei offset = " + std::to_string(edge_info_offset));

      // increment to next connection edge
      added_edges++;
    }

    // Add the node and directed edges
    tilebuilder.AddNodeAndDirectedEdges(nb, directededges);
    nodeid++;
  }

  // Some validation here...
  if (added_edges != connection_edges.size()) {
    LOG_ERROR("Part 1: Added " + std::to_string(added_edges) + " but there are " +
              std::to_string(connection_edges.size()) + " connections");
  }

  // Iterate through the stops and their edges
  uint32_t nadded = 0;
  for (const auto& stop_edges : stop_edge_map) {
    // Get the stop information
    uint32_t stopkey = stop_edges.second.stop_key;
    Stop& stop = stops[stop_indexes[stopkey]];
    if (stop.key != stopkey) {
      LOG_ERROR("Stop key not equal!");
    }

    // Build the node info. Use generic transit stop type
    uint32_t access = kPedestrianAccess;
    bool child  = (stop.parent != 0);  // TODO verify if this is sufficient
    bool parent = (stop.type == 1);    // TODO verify if this is sufficient
    NodeInfoBuilder node(stop.ll, RoadClass::kServiceOther, access,
                        NodeType::kMultiUseTransitStop, false, false);
    node.set_child(child);
    node.set_parent(parent);
    node.set_mode_change(true);
    node.set_stop_id(stop.key);
    LOG_DEBUG("Add node for stop id = " + std::to_string(stop.key));

    // Add connections from the stop to the OSM network
    // TODO - change from linear search for better performance
    std::vector<DirectedEdgeBuilder> directededges;
    for (auto& conn : connection_edges) {
      if (conn.stop_key == stop.key) {
        DirectedEdgeBuilder directededge;
        directededge.set_endnode(conn.osm_node);
        directededge.set_length(conn.length);
        directededge.set_use(Use::kTransitConnection);
        directededge.set_speed(5);
        directededge.set_classification(RoadClass::kServiceOther);
        directededge.set_localedgeidx(directededges.size());
        directededge.set_pedestrianaccess(true, true);
        directededge.set_pedestrianaccess(false, true);

        // Add edge info to the tile and set the offset in the directed edge
        bool added = false;
        std::vector<std::string> names;
        uint32_t edge_info_offset = tilebuilder.AddEdgeInfo(0, conn.stop_node,
                       conn.osm_node, 0, conn.shape, names, added);
        LOG_DEBUG("Add conn from stop to OSM: ei offset = " + std::to_string(edge_info_offset));
        directededge.set_edgeinfo_offset(edge_info_offset);
        directededge.set_forward(added);

        // Add to list of directed edges
        directededges.emplace_back(std::move(directededge));

        nadded++;  // TEMP for error checking
      }
    }

    // Add any intra-station connections
    for (auto endstopkey : stop_edges.second.intrastation) {
      DirectedEdgeBuilder directededge;
      Stop& endstop = stops[stop_indexes[endstopkey]];
      if (endstopkey != endstop.key) {
        LOG_ERROR("End stop key not equal");
      }
      directededge.set_endnode(endstop.graphid);

      // Make sure length is non-zero
      float length = std::max(1.0f, stop.ll.Distance(endstop.ll));
      directededge.set_length(length);
      directededge.set_use(Use::kTransitConnection);
      directededge.set_speed(5);
      directededge.set_classification(RoadClass::kServiceOther);
      directededge.set_localedgeidx(directededges.size());
      directededge.set_pedestrianaccess(true, true);
      directededge.set_pedestrianaccess(false, true);

      LOG_DEBUG("Add parent/child directededge - endnode stop id = " +
               std::to_string(endstop.key) + " GraphId: " +
               std::to_string(endstop.graphid.tileid()) + "," +
               std::to_string(endstop.graphid.id()));

      // Add edge info to the tile and set the offset in the directed edge
      bool added = false;
      std::vector<std::string> names;
      std::vector<PointLL> shape = { stop.ll, endstop.ll };
      uint32_t edge_info_offset = tilebuilder.AddEdgeInfo(0, stop.graphid,
                     endstop.graphid, 0, shape, names, added);
      directededge.set_edgeinfo_offset(edge_info_offset);
      directededge.set_forward(added);

      // Add to list of directed edges
      directededges.emplace_back(std::move(directededge));
    }

    // Add transit lines
    for (auto transitedge : stop_edges.second.lines) {
      // Get the end stop of the connection
      Stop& endstop = stops[stop_indexes[transitedge.stopid]];

      // Set Use based on route type...
      Use use = GetTransitUse(route_types[transitedge.routeid]);
      DirectedEdgeBuilder directededge;
      directededge.set_endnode(endstop.graphid);
      directededge.set_length(stop.ll.Distance(endstop.ll));
      directededge.set_use(use);
      directededge.set_speed(5);
      directededge.set_classification(RoadClass::kServiceOther);
      directededge.set_localedgeidx(directededges.size());
      directededge.set_pedestrianaccess(true, true);
      directededge.set_pedestrianaccess(false, true);
      directededge.set_lineid(transitedge.lineid);

      LOG_DEBUG("Add directededge - lineId = " + std::to_string(transitedge.lineid) +
         " endnode stop id = " + std::to_string(endstop.key) +
         " Route Key = " + std::to_string(transitedge.routeid) +
         " GraphId: " + std::to_string(endstop.graphid.tileid()) + "," +
         std::to_string(endstop.graphid.id()));

      // Add edge info to the tile and set the offset in the directed edge
      // Leave the name empty. Use the trip Id to look up the route Id and
      // route within TripPathBuilder.
      bool added = false;
      std::vector<std::string> names;
      std::vector<PointLL> shape = GetShape(db_handle, stop.ll, endstop.ll, transitedge.shapeid);
      uint32_t edge_info_offset = tilebuilder.AddEdgeInfo(transitedge.routeid,
           stop.graphid, endstop.graphid, 0, shape, names, added);
      directededge.set_edgeinfo_offset(edge_info_offset);
      directededge.set_forward(added);

      // Add to list of directed edges
      directededges.emplace_back(std::move(directededge));
    }
    if (directededges.size() == 0) {
      LOG_ERROR("No directed edges from this node");
    }
    tilebuilder.AddNodeAndDirectedEdges(node, directededges);
  }
  if (nadded != connection_edges.size()) {
    LOG_ERROR("Added " + std::to_string(nadded) + " but there are " +
              std::to_string(connection_edges.size()) + " connections");
  }

  LOG_DEBUG("AddToGraph tileID: " + std::to_string(tilebuilder.header()->graphid().tileid()) +
           " done. New directed edge count = " + std::to_string(tilebuilder.directededges().size()));
}

void AddOSMConnection(Stop& stop, const GraphTile* tile,
                      std::vector<OSMConnectionEdge>& connection_edges) {
  PointLL ll = stop.ll;
  uint64_t wayid = stop.way_id;

  float mindist = 10000000.0f;
  uint32_t edgelength = 0;
  GraphId startnode, endnode;
  std::vector<PointLL> closest_shape;
  std::tuple<PointLL,float,int> closest;
  for (uint32_t i = 0; i < tile->header()->nodecount(); i++) {
    const NodeInfo* node = tile->node(i);
    for (uint32_t j = 0, n = node->edge_count(); j < n; j++) {
      const DirectedEdge* directededge = tile->directededge(node->edge_index() + j);
      auto edgeinfo = tile->edgeinfo(directededge->edgeinfo_offset());
      if (edgeinfo->wayid() == wayid) {
        // Get shape and find closest point
        auto this_shape = edgeinfo->shape();
        auto this_closest = ll.ClosestPoint(this_shape);
        if (std::get<1>(this_closest) < mindist) {
          startnode.Set(tile->header()->graphid().tileid(),
                        tile->header()->graphid().level(), i);
          endnode = directededge->endnode();
          mindist = std::get<1>(this_closest);
          closest = this_closest;
          closest_shape = this_shape;
          edgelength = directededge->length();

          // Reverse the shape if directed edge is not the forward direction
          // along the shape
          if (!directededge->forward()) {
            std::reverse(closest_shape.begin(), closest_shape.end());
          }
        }
      }
    }
  }

  // Check for invalid tile Ids
  if (!startnode.Is_Valid() && !endnode.Is_Valid()) {
    stop.conn_count = 0;
    LOG_ERROR("No closest edge found for this stop: way Id = " +
              std::to_string(wayid));
    return;
  }

  // Check if stop is in same tile as the start node
  stop.conn_count = 0;
  float length = 0.0f;
  if (stop.graphid.Tile_Base() == startnode.Tile_Base()) {
    // Add shape from node along the edge until the closest point, then add
    // the closest point and a straight line to the stop lat,lng
    std::vector<PointLL> shape;
    for (uint32_t i = 0; i <= std::get<2>(closest); i++) {
      shape.push_back(closest_shape[i]);
    }
    shape.push_back(std::get<0>(closest));
    shape.push_back(stop.ll);
    length = std::max(1.0f, PointLL::Length(shape));

    // Add connection to start node
    connection_edges.push_back({startnode, stop.graphid, stop.key, length, shape});
    stop.conn_count++;
  }

  // Check if stop is in same tile as end node
  float length2 = 0.0f;
  if (stop.graphid.Tile_Base() == endnode.Tile_Base()) {
    // Add connection to end node
    if (startnode.tileid() == endnode.tileid()) {
      // Add shape from the end to closest point on edge
      std::vector<PointLL> shape2;
      for (int32_t i = closest_shape.size()-1; i > std::get<2>(closest); i--) {
        shape2.push_back(closest_shape[i]);
      }
      shape2.push_back(std::get<0>(closest));
      shape2.push_back(stop.ll);
      length2 = std::max(1.0f, PointLL::Length(shape2));

      // Add connection to the end node
      connection_edges.push_back({endnode, stop.graphid, stop.key, length2, shape2});
      stop.conn_count++;
    }
  }

  if (length != 0.0f && length2 != 0.0 &&
      (length + length2) < edgelength-1) {
    LOG_ERROR("EdgeLength= " + std::to_string(edgelength) + " < connection lengths: " +
             std::to_string(length) + "," + std::to_string(length2) + " when connecting to stop "
             + std::to_string(stop.key));
  }

  if (stop.conn_count == 0) {
    LOG_ERROR("Stop has no connections to OSM! Stop TileId = " +
              std::to_string(stop.graphid.tileid()) + " Start Node Tile: " +
              std::to_string(startnode.tileid()) + " End Node Tile: " +
              std::to_string(endnode.tileid()));
  }

}

// We make sure to lock on reading and writing since tiles are now being
// written. Also lock on queue access since shared by different threads.
void build(const boost::property_tree::ptree& pt,
           boost::property_tree::ptree& hierarchy_properties,
           std::queue<GraphId>& tilequeue, std::vector<Stop>& stops,
           std::mutex& lock, std::promise<builder_stats>& results) {
  // Construct the transit database
  std::string dir = pt.get<std::string>("transit_dir");
  std::string db_name = pt.get<std::string>("db_name");
  std::string database = dir + "/" +  db_name;

  // Make sure it exists
  sqlite3* db_handle = nullptr;
  if (boost::filesystem::exists(database)) {
    spatialite_init(0);
    sqlite3_stmt* stmt = 0;
    uint32_t ret;
    char* err_msg = nullptr;
    std::string sql;
    ret = sqlite3_open_v2(database.c_str(), &db_handle,
                          SQLITE_OPEN_READONLY, NULL);
    if (ret != SQLITE_OK) {
      LOG_ERROR("cannot open " + database);
      sqlite3_close(db_handle);
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
  else {
    LOG_INFO("Transit db " + database + " not found.  Transit will not be added.");
    return;
  }

  // Get some things we need throughout
  builder_stats stats{};

  // Local Graphreader. Get tile information so we can find bounding boxes
  GraphReader reader(hierarchy_properties);

  lock.lock();
  auto tile_hierarchy = reader.GetTileHierarchy();
  lock.unlock();

  // Create a map of stop key to index in the stop vector
  uint32_t n = 0;
  std::unordered_map<uint32_t, uint32_t> stop_indexes;
  for (auto& stop : stops) {
    stop_indexes[stop.key] = n;
    n++;
  }

  // Iterate through the tiles in the queue and find any that include stops
  while (true) {
    // Get the next tile Id from the queue and get a tile builder
    lock.lock();
    if (tilequeue.empty()) {
      lock.unlock();
      break;
    }
    GraphId tile_id = tilequeue.front();
    uint32_t id  = tile_id.tileid();
    tilequeue.pop();
    const GraphTile* tile = reader.GetGraphTile(tile_id);

    // Read in the existing tile - deserialize it so we can add to it
    GraphTileBuilder tilebuilder(tile_hierarchy, tile_id, true);
    lock.unlock();

    // Iterate through stops and form connections to OSM network. Each
    // stop connects to 1 or 2 OSM nodes along the closest OSM way.
    // TODO - future - how to handle connections that reach nodes
    // outside the tile - may have to move this outside the tile
    // iteration...?
    // TODO - handle a list of connections/egrees points
    // TODO - what if we split the edge and insert a node?
    std::vector<OSMConnectionEdge> connection_edges;
    for (auto& stop : stops) {
      if (stop.graphid.Tile_Base() == tile_id) {
        // Add connections to the OSM network
        if (stop.parent == 0) {
          AddOSMConnection(stop, tile, connection_edges);
        }
      }
    }
    LOG_INFO("Connection Edges: size= " + std::to_string(connection_edges.size()));
    std::sort(connection_edges.begin(), connection_edges.end());

    // Get all scheduled departures from the stops within this tile. Record
    // unique trips, routes, TODO
    std::unordered_set<uint32_t> route_keys;
    std::unordered_set<uint32_t> trip_keys;
    std::unordered_set<uint32_t> service_keys;
    std::map<GraphId, StopEdges> stop_edge_map;
    uint32_t unique_lineid = 1;
    std::vector<TransitDeparture> transit_departures;
    for (auto& stop : stops) {
      if (stop.graphid.Tile_Base() == tile_id) {
        StopEdges stopedges;
        stopedges.stop_key = stop.key;

        // Identify any parent-child edge connections (to add later)
        if (stop.type == 1) {
          // Station - identify any children. TODO - linear search, could
          // improve but there are likely few parent stations
          for (auto& childstop : stops) {
            if (childstop.parent == stop.key) {
              stopedges.intrastation.push_back(childstop.key);
            }
          }
        } else if (stop.parent != 0) {
          stopedges.intrastation.push_back(stop.parent);
        }

        std::map<std::pair<uint32_t, uint32_t>, uint32_t> unique_transit_edges;
        std::vector<Departure> departures = GetDepartures(db_handle, stop.key);

        LOG_INFO("Got " + std::to_string(departures.size()) + " departures for "
          + std::to_string(stop.key) + " location_type = "+ std::to_string(stop.type));

        for (auto& dep : departures) {
          route_keys.insert(dep.route);
          trip_keys.insert(dep.trip);
          service_keys.insert(dep.service);

          // Identify unique route and arrival stop pairs - associate to a
          // unique line Id stored in the directed edge.
          uint32_t lineid;
          auto m = unique_transit_edges.find({dep.route, dep.dest_stop});
          if (m == unique_transit_edges.end()) {
            // Add to the map and update the line id
            lineid = unique_lineid;
            unique_transit_edges[{dep.route, dep.dest_stop}] = unique_lineid;
            unique_lineid++;
            stopedges.lines.emplace_back(lineid, dep.route, dep.dest_stop, dep.shapeid);
          } else {
            lineid = m->second;
          }

          // Form transit departures
          uint32_t headsign_offset = tilebuilder.AddName(dep.headsign);
          uint32_t elapsed_time = dep.arr_time - dep.dep_time;
          TransitDeparture td(lineid, dep.trip, dep.route,
                      dep.blockid, headsign_offset, dep.dep_time, elapsed_time,
                      dep.start_date, dep.end_date, dep.dow, dep.service);

          LOG_INFO("Add departure: " + std::to_string(td.lineid()) +
                       " trip key = " + std::to_string(td.tripid()) +
                       " dep time = " + std::to_string(td.departure_time()) +
                       " start_date = " + std::to_string(td.start_date()) +
                       " end date = " + std::to_string(td.end_date()));

          tilebuilder.AddTransitDeparture(td);
        }

        // Get any transfers from this stop
        AddTransfers(db_handle, stop.key, tilebuilder);

        // Store stop information in TransitStops
        // TODO - tl_stop_id
        uint32_t name_offset = tilebuilder.AddName(stop.name);
        uint32_t desc_offset = tilebuilder.AddName(stop.desc);
        uint32_t farezone = 0;
        TransitStop ts(stop.key, "", name_offset, desc_offset,
                       stop.parent, farezone);
        tilebuilder.AddTransitStop(ts);

        // Add to stop edge map - track edges that need to be added. This is
        // sorted by graph Id so the stop nodes are added in proper order
        stop_edge_map.insert({stop.graphid, stopedges});
      }
    }

    // Add routes to the tile. Get map of route types.
    std::unordered_map<uint32_t, uint32_t> route_types = AddRoutes(db_handle,
                              route_keys, tilebuilder);

    // Add trips to the tile. Get map of shape keys.
    std::unordered_map<uint32_t, uint32_t> shape_keys = AddTrips(db_handle,
                              trip_keys, tilebuilder);

    // Add calendar exceptions (using service Id)
    AddCalendar(db_handle, service_keys, tilebuilder);

    // Add nodes, directededges, and edgeinfo
    AddToGraph(db_handle, tilebuilder, stop_edge_map, stops, connection_edges,
               stop_indexes, route_types);

    // Write the new file
    lock.lock();
    tilebuilder.StoreTileData(tile_hierarchy, tile_id);
    lock.unlock();
  }

  if (db_handle)
    sqlite3_close(db_handle);

  // Send back the statistics
  results.set_value(stats);
  }
}

namespace valhalla {
namespace mjolnir {

// Add transit to the graph
void TransitBuilder::Build(const boost::property_tree::ptree& pt) {
  // A place to hold worker threads and their results
  std::vector<std::shared_ptr<std::thread> > threads(
    std::max(static_cast<uint32_t>(1),
      pt.get<uint32_t>("concurrency", std::thread::hardware_concurrency())));

  // An atomic object we can use to do the synchronization
  std::mutex lock;

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
  LOG_INFO("Done creating queue of tiles: count = " +
           std::to_string(tilequeue.size()));

  // First pass - find all tiles with stops. Create graphids for each stop
  // Start the threads
  LOG_INFO("Assign GraphIds to each stop...");

  // A place to hold the results of those threads (exceptions, stats)
  std::list<std::promise<stop_results> > stop_res;

  for (auto& thread : threads) {
    stop_res.emplace_back();
    thread.reset(new std::thread(assign_graphids,
                     std::ref(pt.get_child("mjolnir.transit")),
                     std::ref(hierarchy_properties), std::ref(tilequeue),
                     std::ref(lock), std::ref(stop_res.back())));
  }

  // Wait for them to finish up their work
  for (auto& thread : threads) {
    thread->join();
  }

  // Accumulate stop results from all the threads
  stop_results all_stops{};
  for (auto& result : stop_res) {
    // If something bad went down this will rethrow it
    try {
      auto thread_stats = result.get_future().get();
      all_stops(thread_stats);
    }
    catch (std::exception& e) {
      //TODO: throw further up the chain?
    }
  }
  LOG_INFO("Done first pass. Total Stops = " +
             std::to_string(all_stops.stops.size()) +
             " tiles: " + std::to_string(all_stops.tiles.size()));

  if (all_stops.tiles.size() == 0) {
    return;
  }

  // TODO - intermediate pass to find any connections that cross into different
  // tile than the stop

  // Second pass - for all tiles with transit stops get all transit information
  // and populate tiles

  // Create transit tile queue
  std::deque<GraphId> tq;
  for (auto& tile : all_stops.tiles) {
    tq.push_back(tile);
  }
  std::queue<GraphId> transit_tiles(tq);

  // A place to hold the results of those threads (exceptions, stats)
  std::list<std::promise<builder_stats> > results;

  // Start the threads
  LOG_INFO("Add transit to the local graph...");
  for (auto& thread : threads) {
    results.emplace_back();
    thread.reset(new std::thread(build, std::ref(pt.get_child("mjolnir.transit")),
                     std::ref(hierarchy_properties), std::ref(transit_tiles),
                     std::ref(all_stops.stops), std::ref(lock),
                     std::ref(results.back())));
  }

  // Wait for them to finish up their work
  for (auto& thread : threads) {
    thread->join();
  }

  // Check all of the outcomes, to see about maximum density (km/km2)
  builder_stats stats{};
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
}

}
}
