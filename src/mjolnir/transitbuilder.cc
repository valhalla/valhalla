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

#include <valhalla/baldr/graphtile.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/midgard/logging.h>

using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::mjolnir;

namespace {

struct Stop {
  // Need to add onestop Id, connections (wayid, lat,lon)
  GraphId graphid;
  uint32_t key;
  uint32_t type;
  uint32_t parent;
  PointLL  ll;
  std::string id;
  std::string code;
  std::string name;
  std::string desc;
  std::string zoneid;
  std::string url;
};

struct Departure {
  uint32_t key;
  uint32_t orig_stop;
  uint32_t dest_stop;
  uint32_t trip;
  uint32_t route;
  uint32_t service;
  uint32_t dep_time;
  uint32_t arr_time;
  uint32_t start_date;
  uint32_t end_date;
  uint32_t dow;
  std::string headsign;
};

// Unique route and stop
struct TransitLine {
  uint32_t diredgeid;
  uint32_t routeid;
  uint32_t stopid;

  TransitLine(const uint32_t d, const uint32_t r, const uint32_t s)
      : diredgeid(d),
        routeid(r),
        stopid(s) {
  }
};

struct StopEdges {
  std::vector<uint32_t> intrastation;   // List of intra-station connections
  std::vector<TransitLine> lines;       // Set of unique route/stop pairs
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
std::vector<Stop> GetStops(sqlite3 *db_handle, const AABB2& aabb) {
  // Form query
  std::string sql = "SELECT stop_key, stop_id, stop_code, stop_name,";
  sql += "stop_desc, zone_id, stop_url, location_type, parent_station_key,";
  sql += "stop_lat, stop_lon from stops where ";
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
      stop.id     = (char*)(sqlite3_column_text(stmt, 1));
      stop.code   = (sqlite3_column_type(stmt, 2) == SQLITE_TEXT) ?
                      (char*)sqlite3_column_text(stmt, 2) : "";
      stop.name   = (sqlite3_column_type(stmt, 3) == SQLITE_TEXT) ?
                      (char*)(sqlite3_column_text(stmt, 3)) : "";
      stop.desc   = (sqlite3_column_type(stmt, 4) == SQLITE_TEXT) ?
                      (char*)(sqlite3_column_text(stmt, 4)) : "";
      stop.zoneid = (sqlite3_column_type(stmt, 5) == SQLITE_TEXT) ?
                      (char*)(sqlite3_column_text(stmt, 5)) : "";
      stop.url    = (sqlite3_column_type(stmt, 5) == SQLITE_TEXT) ?
                      (char*)(sqlite3_column_text(stmt, 6)) : "";
      stop.type   = sqlite3_column_int(stmt, 7);
      stop.parent = sqlite3_column_int(stmt, 8);
      stop.ll.Set(static_cast<float>(sqlite3_column_double(stmt, 9)),
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

  // Local Graphreader. Get tile information so we can find bounding boxes
  GraphReader reader(hierarchy_properties);
  auto tile_hierarchy = reader.GetTileHierarchy();
  auto local_level = tile_hierarchy.levels().rbegin()->second.level;
  auto tiles = tile_hierarchy.levels().rbegin()->second.tiles;

  // Iterate through the tiles in the queue and find any that include stops
  stop_results stats{};
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
      LOG_INFO("Tile has " + std::to_string(stops.size()) + " stops");

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
  std::string sql = "SELECT schedule_key, origin_stop_key, dest_stop_key,";
  sql += "trip_key, route_key, service_key, departure_time, arrival_time,";
  sql += "start_date, end_date, dow_mask, headsign from schedule where ";
  sql += "origin_stop_key = " + std::to_string(stop_key);

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

      // TODO - convert times and dates to int values
      std::string t1 = (sqlite3_column_type(stmt, 6) == SQLITE_TEXT) ?
                         (char*)sqlite3_column_text(stmt, 6) : "";
      std::string t2 = (sqlite3_column_type(stmt, 7) == SQLITE_TEXT) ?
                         (char*)sqlite3_column_text(stmt, 7) : "";
      std::string d1 = (sqlite3_column_type(stmt, 8) == SQLITE_TEXT) ?
                         (char*)sqlite3_column_text(stmt, 8) : "";
      std::string d2 = (sqlite3_column_type(stmt, 9) == SQLITE_TEXT) ?
                         (char*)sqlite3_column_text(stmt, 9) : "";

      dep.dow        = sqlite3_column_int(stmt, 10);
      dep.headsign   = (sqlite3_column_type(stmt, 11) == SQLITE_TEXT) ?
                         (char*)sqlite3_column_text(stmt, 11) : "";
      departures.emplace_back(std::move(dep));

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

        // TODO - add to text list and form offsets
        uint32_t shortname_offset = 0;
        uint32_t longname_offset = 0;
        uint32_t desc_offset = 0;
        TransitRoute route(routeid, agencyid, tl_routeid.c_str(),
                           shortname_offset, longname_offset,
                           desc_offset);
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
  LOG_INFO("Added " + std::to_string(n) + " routes");
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
    sql += "trip_short_name, shape_id from trips where ";
    sql += "trip_key = " + std::to_string(key);

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

        // TODO - add to text list and get offset
        uint32_t shortname_offset = 0;
        uint32_t headsign_offset = 0;
        TransitTrip trip(tripid, routeid, tl_tripid, shortname_offset,
                          headsign_offset);
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
  LOG_INFO("Added " + std::to_string(n) + " trips");
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
        std::string d = (sqlite3_column_type(stmt, 2) == SQLITE_TEXT) ?
            (char*)sqlite3_column_text(stmt, 2) : "";
        uint32_t date = 0;  // TODO
        uint32_t t = sqlite3_column_int(stmt, 3);

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
    stop_indexes[stop.key] = n++;
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
    GraphTileBuilder tilebuilder(tile_hierarchy, tile_id);
    lock.unlock();

    // Get the current number of directed edges
    uint32_t diredgecount = tilebuilder.header()->directededgecount();

    // Get all scheduled departures from the stops within this tile. Record
    // unique trips, routes, TODO
    std::unordered_set<uint32_t> route_keys;
    std::unordered_set<uint32_t> trip_keys;
    std::unordered_set<uint32_t> service_keys;
    std::unordered_map<uint32_t, StopEdges> stop_edge_map;
    std::vector<TransitDeparture> transit_departures;
    for (auto& stop : stops) {
      if (stop.graphid.Tile_Base() == tile_id) {
        StopEdges stopedges;

        // For now skip connections to OSM network - will have to reserve
        // here! TODO- except for child stops?). TODO - what if we split the
        // edge and insert a node?

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
        diredgecount += stopedges.intrastation.size();

        std::map<std::pair<uint32_t, uint32_t>, uint32_t> unique_transit_edges;
        std::vector<Departure> departures = GetDepartures(db_handle, stop.key);

LOG_INFO("Got " + std::to_string(departures.size()) + " departures for "
                     + std::to_string(stop.key) + " location_type = "
                     + std::to_string(stop.type));
        for (auto& dep : departures) {
          route_keys.insert(dep.route);
          trip_keys.insert(dep.trip);
          service_keys.insert(dep.service);

          // Identify unique route and arrival stop pairs - associate to a
          // directed edge id.
          uint32_t diredgeid;
          auto m = unique_transit_edges.find({dep.route, dep.dest_stop});
          if (m == unique_transit_edges.end()) {
            // Add to the map and update the directed edge id
            diredgeid = diredgecount;
            unique_transit_edges[{dep.route, dep.dest_stop}] = diredgeid;
            diredgecount++;
            stopedges.lines.emplace_back(diredgeid, dep.route, dep.orig_stop);
          } else {
            diredgeid = m->second;
          }

          // Form transit departures
          uint32_t edgeid  = diredgeid;
          uint32_t blockid = 0;  // TODO - not part of the schedule?
          uint32_t headsign_offset = 0; // TODO
          uint32_t elapsed_time = dep.arr_time - dep.dep_time;
          transit_departures.emplace_back(edgeid, dep.trip, dep.route,
                          blockid, headsign_offset, dep.dep_time, elapsed_time,
                          dep.start_date, dep.end_date, dep.dow, dep.service);
        }

        // Get any transfers from this stop
        AddTransfers(db_handle, stop.key, tilebuilder);

        // Store stop information in TransitStops
        // TODO - tl_stop_id, name offset, desc offset
        uint32_t name_offset = 0;
        uint32_t desc_offset = 0;
        uint32_t farezone = 0;
        TransitStop ts(stop.key, "", name_offset, desc_offset,
                       stop.parent, farezone);
        tilebuilder.AddTransitStop(ts);

        // Add to stop edge map - track edges that need to be added
        stop_edge_map[stop.key] = std::move(stopedges);
      }
    }

    LOG_INFO("Tile has " + std::to_string(route_keys.size()) + " routes and " +
             std::to_string(trip_keys.size()) + " trips");

    // Add routes to the tile. Get map of route types.
    std::unordered_map<uint32_t, uint32_t> route_types = AddRoutes(db_handle,
                              route_keys, tilebuilder);

    // Add trips to the tile. Get map of shape keys.
    std::unordered_map<uint32_t, uint32_t> shape_keys = AddTrips(db_handle,
                              trip_keys, tilebuilder);

    // Add calendar exceptions (using service Id)
    AddCalendar(db_handle, service_keys, tilebuilder);

    // Add nodes, directededges, and edgeinfo
    // TODO - move to a method
    uint32_t diredgeid = tilebuilder.header()->directededgecount();
    for (const auto& stop_edges : stop_edge_map) {
      // Get the stop information
      uint32_t stopid = stop_edges.first;
      Stop& stop = stops[stop_indexes[stopid]];

      // Build the node info
      // TODO - how to differentiate bus from rail, from multi-use?
      // TODO - do we need any name consistency/heading stuff?
      uint32_t edgeindex = diredgeid;
      uint32_t edgecount = stop_edges.second.intrastation.size() +
                           stop_edges.second.lines.size();
      uint32_t access = 0;  // TODO - walking only!
      bool child = (stop.parent != 0);  // TODO verify if this is sufficient
      bool parent = (stop.type == 1);   // TODO verify if this is sufficient
      NodeInfoBuilder node(stop.ll, edgeindex, edgecount,
                           RoadClass::kServiceOther, access,
                           NodeType::kMultiUseTransitStop, false, false);
      node.set_child(child);
      node.set_parent(parent);
      node.set_mode_change(true);
      node.set_stop_id(stop.key);

      // Add any intra-station connections
      uint32_t idx = 0;
      std::vector<DirectedEdgeBuilder> directededges;
      for (auto endstop : stop_edges.second.intrastation) {
        bool forward = true;          // TBD
        DirectedEdgeBuilder directededge;
        directededge.set_endnode(stops[endstop].graphid);
        directededge.set_length(stop.ll.Distance(stops[endstop].ll));
        directededge.set_use(Use::kTransitConnection);
        directededge.set_speed(5);
        directededge.set_classification(RoadClass::kServiceOther);
        directededge.set_localedgeidx(idx);
        directededge.set_forward(forward);
        directededge.set_pedestrianaccess(forward, true);
        directededge.set_pedestrianaccess(!forward, true);
        // TODO: anything else needed??
        directededges.emplace_back(std::move(directededge));
        idx++;
      }
uint32_t tc = directededges.size();
      // Add any intra-station connections
      for (auto transitedge : stop_edges.second.lines) {
        // Get the end stop of the connection
        Stop& endstop = stops[stop_indexes[transitedge.stopid]];

        // TODO - add EdgeInfo...

        bool forward = true;  // TBD
        // Set Use based on route type...
        Use use = GetTransitUse(route_types[transitedge.routeid]);
        DirectedEdgeBuilder directededge;
        directededge.set_endnode(endstop.graphid);
        directededge.set_length(stop.ll.Distance(endstop.ll));
        directededge.set_use(use);
        directededge.set_speed(5);
        directededge.set_classification(RoadClass::kServiceOther);
        directededge.set_localedgeidx(idx);
        directededge.set_forward(forward);
        directededge.set_pedestrianaccess(forward, true);
        directededge.set_pedestrianaccess(!forward, true);
        // TODO: anything else needed??
        directededges.emplace_back(std::move(directededge));
        idx++;
      }
 LOG_INFO(std::to_string(stopid) + ": add " + std::to_string(idx) + " directed edges. TC= " +
          std::to_string(tc));
      tilebuilder.AddNodeAndDirectedEdges(node, directededges);

      diredgeid += directededges.size();
    }
uint32_t count = diredgeid - tilebuilder.header()->directededgecount();
LOG_INFO("Added " + std::to_string(count) + " directed edges");
    // Write the new file
    lock.lock();
//    tilebuilder.StoreTileData(tile_hierarchy, tile_id);
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
