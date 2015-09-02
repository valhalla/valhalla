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
#include <boost/algorithm/string.hpp>
#include <boost/foreach.hpp>

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
  uint32_t wheelchair_boarding;
  PointLL  ll;
  std::string onestop_id;
  std::string id;
  std::string name;
  std::string desc;
  std::string zoneid;
  std::string timezone;
};

struct Departure {
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
  uint32_t wheelchair_accessible;
  std::string headsign;
  std::string short_name;
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

// Get stops within a tile
std::vector<Stop> GetStops(const std::string& file, const AABB2<PointLL>& aabb) {

  boost::property_tree::ptree pt;
  std::vector<Stop> stops;

  // Make sure it exists
  if (boost::filesystem::exists(file))
    boost::property_tree::read_json(file, pt);
  else
    return stops;

  for (const auto& s : pt.get_child("stops")){

    Stop stop;

    float lon, lat;
    uint32_t index = 0;
    for(auto& coords : s.second.get_child("geometry.coordinates")) {
      if (index) lat = coords.second.get_value<float>();
      else lon = coords.second.get_value<float>();
      index++;
    }

    //Hack for now.  transit land has a BoundBox bug.
    if (aabb.Contains(PointLL(lon,lat))) {

      stop.ll.Set(lon,lat);

      stop.key = s.second.get<uint32_t>("key", 0);
      stop.name = s.second.get<std::string>("name", "");

      if (stop.key == 0) {
        LOG_ERROR("Key missing for stop (" + stop.name + ") in " + file);
        continue;
      }

      stop.way_id = s.second.get<uint64_t>("tags.osm_way_id", 0);
      stop.wheelchair_boarding = s.second.get<bool>("tags.wheelchair_boarding", false);
      stop.onestop_id = s.second.get<std::string>("tags.onestop_id", "");
      stop.desc = s.second.get<std::string>("tags.stop_desc", "");
      stop.zoneid = s.second.get<std::string>("tags.zone_id", "");
      stop.timezone = s.second.get<std::string>("timezone", "");

      //TODO: get these from transitland????
      stop.type = 0;
      stop.parent = 0;

      stops.emplace_back(std::move(stop));
    }
  }

  return stops;

}

// Lock on queue access since we are using it in different threads. No need
// to lock graphreader since no threads are writing tiles yet.
void assign_graphids(const boost::property_tree::ptree& config_pt,
           boost::property_tree::ptree& hierarchy_properties,
           std::queue<GraphId>& tilequeue, std::mutex& lock,
           std::promise<stop_results>& stop_res) {
  // Construct the transit database
  stop_results stats{};

  std::string transit_dir = config_pt.get<std::string>("transit_dir");
  boost::property_tree::ptree pt;

  // Make sure it exists
  if (!boost::filesystem::exists(transit_dir)) {
    LOG_INFO("Transit directory " + transit_dir + " not found.  Transit will not be added.");
    return;
  }
  else transit_dir += "/";

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

    std::string file_name = GraphTile::FileSuffix(GraphId(tile_id.tileid(), tile_id.level(),0), tile_hierarchy);

    boost::algorithm::trim_if(file_name, boost::is_any_of(".gph"));
    file_name += ".json";
    std::string file = transit_dir + file_name;

    // Get stops within the tile. If any exist, assign GraphIds and add to
    // the map/list
    std::vector<Stop> stops = GetStops(file, tiles.TileBounds(id));

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

  // Send back the statistics
  stop_res.set_value(stats);
}

// Get scheduled departures for a stop
std::vector<Departure> ProcessStopPairs(const std::string& file,
                                        std::unordered_map<uint32_t, uint32_t>& trip_routes,
                                        const uint32_t stop_key,
                                        GraphTileBuilder& tilebuilder) {

  boost::property_tree::ptree pt;
  std::vector<Departure> departures;

  // Make sure it exists
  if (boost::filesystem::exists(file))
    boost::property_tree::read_json(file, pt);
  else
    return departures;

  try
  {
    for (const auto& stop_pairs : pt.get_child("schedule_stop_pairs")) {

      // needed?
      const uint32_t origin_key = stop_pairs.second.get<uint32_t>("origin_key", 0);
      if (origin_key != stop_key)
        continue;

      Departure dep;
      dep.orig_stop = origin_key;
      dep.dest_stop = stop_pairs.second.get<uint32_t>("destination_key", 0);
      dep.route = stop_pairs.second.get<uint32_t>("route_key", 0);
      dep.trip = stop_pairs.second.get<uint32_t>("trip_key", 0);

      if (dep.trip == 0) {
        LOG_ERROR("Trip does not exist for route: " +  std::to_string(dep.route) +
                  " file: " + file);
        continue;
      }

      if (dep.route == 0) {
        LOG_ERROR("Route does not exist for trip: " +  std::to_string(dep.trip) +
                  " file: " + file);
        continue;
      }

      //TODO
      dep.shapeid = 0;
      dep.blockid = 0;
      dep.service = 0;
      dep.has_subtractions = false;
      //wheelchair_accessible
      //short_name

      dep.dep_time = DateTime::seconds_from_midnight(stop_pairs.second.get<std::string>("origin_departure_time", ""));
      dep.arr_time = DateTime::seconds_from_midnight(stop_pairs.second.get<std::string>("destination_arrival_time", ""));
      dep.start_date =  DateTime::days_from_pivot_date(stop_pairs.second.get<std::string>("service_start_date", ""));
      dep.end_date =  DateTime::days_from_pivot_date(stop_pairs.second.get<std::string>("service_end_date", ""));

      if (dep.end_date > 500)
        continue;

      uint32_t index = 1;
      uint32_t dow_mask = kDOWNone;

      for(auto& service_days : stop_pairs.second.get_child("service_days_of_week")) {
        bool dow = service_days.second.get_value<bool>();

        if (dow) {
          switch (index) {
            case 1:
              dow_mask = dow_mask | kMonday;
              break;
            case 2:
              dow_mask = dow_mask | kTuesday;
              break;
            case 3:
              dow_mask = dow_mask | kWednesday;
              break;
            case 4:
              dow_mask = dow_mask | kThursday;
              break;
            case 5:
              dow_mask = dow_mask | kFriday;
              break;
            case 6:
              dow_mask = dow_mask | kSaturday;
              break;
            case 7:
              dow_mask = dow_mask | kSunday;
              break;
          }
        }
        index++;
      }

      dep.dow = dow_mask;
      dep.headsign   = stop_pairs.second.get<std::string>("trip_headsign", "");

      auto added_dates = stop_pairs.second.get_child_optional("service_added_dates");

      if (added_dates && added_dates->empty())
        departures.emplace_back(std::move(dep));
      else //do not move, must create additions for the departure. {
        departures.emplace_back(dep);

      //Add the additions.
      if (added_dates && !added_dates->empty()) {
        for(auto& service_added_dates : stop_pairs.second.get_child("service_added_dates")) {

          std::string date = service_added_dates.second.get_value<std::string>();

          uint32_t days = DateTime::days_from_pivot_date(date);

          dep.start_date =  days;
          dep.end_date =  days;
          departures.emplace_back(dep);
        }
      }

      const auto& trip = trip_routes.find(dep.trip);

      //Add the trips.
      if (trip == trip_routes.end()) {

        std::string tl_tripid = stop_pairs.second.get<std::string>("trip", "");
        std::string shortname = stop_pairs.second.get<std::string>("trip_short_name", "");

        // Add names and create transit trip
        TransitTrip trip(dep.trip, dep.route, tl_tripid.c_str(),
                         tilebuilder.AddName(shortname == "null" ? "" : shortname),
                         tilebuilder.AddName(dep.headsign == "null" ? "" : dep.headsign));
        tilebuilder.AddTransitTrip(trip);

        trip_routes[dep.trip] = dep.route;
      }

    }
  }
  catch (std::exception &e) {
    LOG_ERROR("Exception in json file " + file + " for transit.  " + e.what());
  }

  return departures;
}

// Add routes to the tile. Return a map of route types vs. id/key.
std::unordered_map<uint32_t, uint32_t> AddRoutes(const std::string& file,
                   const std::unordered_set<uint32_t>& keys,
                   GraphTileBuilder& tilebuilder) {
  // Map of route keys vs. types
  std::unordered_map<uint32_t, uint32_t> route_types;
  boost::property_tree::ptree pt;

  // Make sure it exists
  if (boost::filesystem::exists(file))
    boost::property_tree::read_json(file, pt);
  else
    return route_types;

  uint32_t n = 0;
  try {
    // Iterate through all route keys
    for (const auto& key : keys) {
      // Skip (do not need): route_id, route_color, route_text_color, and
      // route_url
      for (const auto& routes : pt.get_child("routes")) {

        uint32_t routeid = routes.second.get<uint32_t>("key", 0);
        if (key != routeid)
          continue;

        std::string tl_routeid = routes.second.get<std::string>("onestop_id", "");
        std::string shortname = routes.second.get<std::string>("name", "");
        std::string longname = routes.second.get<std::string>("tags.route_long_name", "");
        std::string desc = routes.second.get<std::string>("tags.route_desc", "");
        std::string vehicle_type = routes.second.get<std::string>("tags.vehicle_type", "");
        uint32_t type = 0;

        if (vehicle_type == "tram")
          type = 0;
        else if (vehicle_type == "metro")
          type = 1;
        else if (vehicle_type == "rail")
          type = 2;
        else if (vehicle_type == "bus")
          type = 3;
        else if (vehicle_type == "ferry")
          type = 4;
        else if (vehicle_type == "cablecar")
          type = 5;
        else if (vehicle_type == "gondola")
          type = 6;
        else if (vehicle_type == "funicular")
          type = 7;
        else {
          LOG_WARN("Unsupported vehicle_type: " + vehicle_type);
          continue;
        }

        // Add names and create the transit route
        // TODO:  Fix short name.
        TransitRoute route(routeid, 0, tl_routeid.c_str(),
                           tilebuilder.AddName(shortname == "null" ? "" : longname),
                           tilebuilder.AddName(longname == "null" ? "" : longname),
                           tilebuilder.AddName(desc == "null" ? "" : desc));
        tilebuilder.AddTransitRoute(route);
        n++;

        // Route type - need this to store in edge?
        route_types[routeid] = type;
        break;
      }
    }
  }
  catch (std::exception &e) {
    LOG_ERROR("Exception in json file " + file + " for transit.  " + e.what());
  }

  LOG_INFO("Added " + std::to_string(n) + " routes");

  return route_types;
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
      //tilebuilder.AddTransitTransfer(transfer);

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

std::vector<PointLL> GetShape(const PointLL& stop_ll,
                              const PointLL& endstop_ll,
                              const uint32_t shapeid) {
  std::vector<PointLL> shape;

  /*
   * TODO: port to use transit land shape.
   *
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
  }*/

  shape.push_back(stop_ll);
  shape.push_back(endstop_ll);

  return shape;
}

void AddToGraph(GraphTileBuilder& tilebuilder,
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
      std::vector<PointLL> shape = GetShape(stop.ll, endstop.ll, transitedge.shapeid);
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
    LOG_ERROR("No closest edge found for this stop: " + stop.name + " way Id = " +
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
void build(const boost::property_tree::ptree& config_pt,
           boost::property_tree::ptree& hierarchy_properties,
           std::queue<GraphId>& tilequeue, std::vector<Stop>& stops,
           std::mutex& lock, std::promise<builder_stats>& results) {

  std::string transit_dir = config_pt.get<std::string>("transit_dir");
  boost::property_tree::ptree pt;

  // Make sure it exists
  if (!boost::filesystem::exists(transit_dir)) {
    LOG_INFO("Transit directory " + transit_dir + " not found.  Transit will not be added.");
    return;
  }
  else transit_dir += "/";

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
  std::unordered_map<uint32_t, uint32_t> trip_routes;
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

    std::string file_name = GraphTile::FileSuffix(GraphId(tile_id.tileid(), tile_id.level(),0), tile_hierarchy);
    boost::algorithm::trim_if(file_name, boost::is_any_of(".gph"));
    file_name += ".json";
    const std::string file = transit_dir + file_name;

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
        std::vector<Departure> departures = ProcessStopPairs(file, trip_routes,
                                                             stop.key, tilebuilder);

        LOG_DEBUG("Got " + std::to_string(departures.size()) + " departures for "
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
          TransitDeparture td(lineid, dep.trip,dep.route,
                      dep.blockid, headsign_offset, dep.dep_time, elapsed_time,
                      dep.start_date, dep.end_date, dep.dow, dep.service);

          LOG_DEBUG("Add departure: " + std::to_string(lineid) +
                       " dep time = " + std::to_string(td.departure_time()) +
                       " arr time = " + std::to_string(dep.arr_time) +
                       " start_date = " + std::to_string(td.start_date()) +
                       " end date = " + std::to_string(td.end_date()));

          tilebuilder.AddTransitDeparture(td);

        }

        // TODO no Transfers exist in transit.land
        // Get any transfers from this stop
        // AddTransfers(db_handle, stop.key, tilebuilder);

        // Store stop information in TransitStops
        uint32_t name_offset = tilebuilder.AddName(stop.name);
        uint32_t desc_offset = tilebuilder.AddName(stop.desc);
        uint32_t farezone = 0;
        TransitStop ts(stop.key, stop.onestop_id.c_str(), name_offset, desc_offset,
                       stop.parent, farezone);
        tilebuilder.AddTransitStop(ts);

        // Add to stop edge map - track edges that need to be added. This is
        // sorted by graph Id so the stop nodes are added in proper order
        stop_edge_map.insert({stop.graphid, stopedges});
      }
    }

    LOG_INFO("Added " + std::to_string(trip_routes.size()) + " trips");

    // Add routes to the tile. Get map of route types.
    std::unordered_map<uint32_t, uint32_t> route_types = AddRoutes(file,
                              route_keys, tilebuilder);

    // TODO no service keys exist in transit.land
    // Add calendar exceptions (using service Id)
    // AddCalendar(db_handle, service_keys, tilebuilder);

    // Add nodes, directededges, and edgeinfo
    AddToGraph(tilebuilder, stop_edge_map, stops, connection_edges,
               stop_indexes, route_types);

    // Write the new file
    lock.lock();
    tilebuilder.StoreTileData(tile_hierarchy, tile_id);
    lock.unlock();
  }

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
  LOG_INFO("Finished with " + std::to_string(all_stops.stops.size()) +
             " stops in " + std::to_string(all_stops.tiles.size()) + " tiles");

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
  LOG_INFO("Finished");
}

}
}
