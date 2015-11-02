#include "mjolnir/transitbuilder.h"
#include "mjolnir/graphtilebuilder.h"
#include "proto/transit.pb.h"

#include <list>
#include <future>
#include <thread>
#include <mutex>
#include <vector>
#include <queue>
#include <unordered_map>
#include <sqlite3.h>
#include <spatialite.h>
#include <fstream>
#include <iostream>
#include <boost/filesystem/operations.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/foreach.hpp>
#include <google/protobuf/io/zero_copy_stream_impl_lite.h>
#include <google/protobuf/io/coded_stream.h>

#include <valhalla/baldr/datetime.h>
#include <valhalla/baldr/graphtile.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/midgard/util.h>
#include <valhalla/midgard/logging.h>
#include <valhalla/midgard/sequence.h>

using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::baldr::DateTime;
using namespace valhalla::mjolnir;

namespace {

struct Stop {
  // Need to add onestop Id, connections (wayid, lat,lon)
  GraphId graphid;
  uint64_t way_id;
  uint32_t type;
  uint32_t parent;
  uint32_t conn_count;     // Number of connections to OSM nodes
  uint32_t wheelchair_boarding;
  uint32_t timezone;
  float lon;
  float lat;
  PointLL ll() const { return {lon, lat}; }
};

struct Departure {
  uint64_t days;
  uint32_t orig_stop;
  uint32_t dest_stop;
  uint32_t trip;
  uint32_t route;
  uint32_t blockid;
  uint32_t shapeid;
  uint32_t dep_time;
  uint32_t arr_time;
  uint32_t start_date;
  uint32_t end_date;
  uint32_t dow;
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
};

struct StopEdges {
  uint32_t stop_key;                     // Stop key
  std::vector<uint32_t> intrastation;   // List of intra-station connections
  std::vector<TransitLine> lines;       // Set of unique route/stop pairs
};

struct OSMConnectionEdge {
  GraphId osm_node;
  GraphId stop_node;
  float length;
  std::list<PointLL> shape;

  OSMConnectionEdge(const GraphId& f, const GraphId& t,
                    const float l, const std::list<PointLL>& s)
      :  osm_node(f),
         stop_node(t),
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

// Struct to hold stats information during each threads work
struct builder_stats {
  uint32_t stats;

  // Accumulate stats from all threads
  void operator()(const builder_stats& other) {
    stats += other.stats;
  }
};

// Write stops within a tile to the sequence
std::vector<Stop> AddStops(const Transit& transit, GraphTileBuilder& tilebuilder,
                           const size_t node_size, const std::vector<std::string>& regions) {

  std::vector<Stop> stops;

  for (uint32_t i = 0; i < transit.stops_size(); i++) {
    const Transit_Stop& s = transit.stops(i);

    // Get the coordinates of the stop as transit land has a BoundBox bug
    Stop stop;
    // Strings to be added to the tile
    const std::string onestop_id = s.onestop_id();
    const std::string name = s.name();

    // Get the rest of the fixed sized data
    stop.lon = s.lon();
    stop.lat = s.lat();
    stop.way_id = s.osm_way_id();
    stop.wheelchair_boarding = s.wheelchair_boarding();
    stop.timezone = s.timezone();

    //TODO: get these from transitland????
    stop.parent = 0;
    stop.type = 0;
    stop.graphid = GraphId(s.graphid()) + node_size;

    // Add the stop to the list
    stops.emplace_back(std::move(stop));

    // Store stop information in TransitStops
    TransitStop ts(stop.graphid.id(), tilebuilder.AddName(onestop_id),
                   tilebuilder.AddName(name));
    tilebuilder.AddTransitStop(ts);
  }

  LOG_INFO("Added " + std::to_string(stops.size()) + " stops");

  return stops;
}

// Get scheduled departures for a stop
std::unordered_multimap<uint32_t, Departure> ProcessStopPairs(const Transit& transit,
                                                              const size_t node_size,
                                                              std::unordered_map<uint32_t,bool>& stop_access) {

  std::unordered_multimap<uint32_t, Departure> departures;

  LOG_INFO("Stop pairs " + std::to_string(transit.stop_pairs_size()) + " size");

  for (uint32_t i = 0; i < transit.stop_pairs_size(); i++) {
    const Transit_StopPair& sp = transit.stop_pairs(i);

    Departure dep;

    GraphId orig = GraphId(sp.origin_graphid()) + node_size;
    GraphId dest = GraphId(sp.destination_graphid()) + node_size;

    dep.orig_stop = orig.id();
    dep.dest_stop = dest.id();
    dep.route = sp.route_index();
    dep.trip = sp.trip_key();

    //TODO
    dep.shapeid = 0;
    dep.blockid = sp.block_id();

    //TODO
    //wheelchair_accessible

    const std::string origin_time = sp.origin_departure_time();
    const std::string dest_time = sp.destination_arrival_time();
    dep.dep_time = DateTime::seconds_from_midnight(origin_time);
    dep.arr_time = DateTime::seconds_from_midnight(dest_time);
    std::string start_date = sp.service_start_date();
    std::string end_date = sp.service_end_date();

    uint32_t dow_mask = kDOWNone;
    for (uint32_t x = 0; x < sp.service_days_of_week_size(); x++) {
      bool dow = sp.service_days_of_week(x);
      if (dow) {
        switch (x) {
          case 0:
            dow_mask |= kMonday;
            break;
          case 1:
            dow_mask |= kTuesday;
            break;
          case 2:
            dow_mask |= kWednesday;
            break;
          case 3:
            dow_mask |= kThursday;
            break;
          case 4:
            dow_mask |= kFriday;
            break;
          case 5:
            dow_mask |= kSaturday;
            break;
          case 6:
            dow_mask |= kSunday;
            break;
        }
      }
    }

    dep.dow = dow_mask;

    std::string tz = sp.origin_timezone();

    //end_date will be updated if greater than 60 days.
    //start_date will be updated to today if the start date is in the past
    //the start date to end date or 60 days, whichever is less.
    //set the bits based on the dow.

    dep.days = DateTime::get_service_days(start_date, end_date, tz, dow_mask);
    dep.start_date =  DateTime::days_from_pivot_date(start_date);
    dep.end_date =  DateTime::days_from_pivot_date(end_date);
    dep.headsign = sp.trip_headsign();

    bool bikes_allowed = sp.bikes_allowed();
    stop_access[dep.orig_stop] = bikes_allowed;
    stop_access[dep.dest_stop] = bikes_allowed;

    //if subtractions are between start and end date then turn off bit.
    for (uint32_t x = 0; x < sp.service_except_dates_size(); x++) {
      std::string date = sp.service_except_dates(x);
      dep.days = DateTime::remove_service_day(dep.days, start_date, end_date, date);
    }

    //if additions are between start and end date then turn on bit.
    for (uint32_t x = 0; x < sp.service_added_dates_size(); x++) {
      std::string date = sp.service_added_dates(x);
      dep.days = DateTime::add_service_day(dep.days, start_date, end_date, date);
    }
    departures.emplace(dep.orig_stop,std::move(dep));
  }

  LOG_INFO("Added " + std::to_string(departures.size()) + " departures");
  return departures;
}

// Add routes to the tile. Return a map of route types vs. id/key.
std::unordered_map<uint32_t, uint32_t> AddRoutes(const Transit& transit,
                   const std::unordered_set<uint32_t>& keys,
                   GraphTileBuilder& tilebuilder) {
  // Map of route keys vs. types
  std::unordered_map<uint32_t, uint32_t> route_types;

  for (uint32_t i = 0; i < transit.routes_size(); i++) {
    const Transit_Route& r = transit.routes(i);
      TransitRoute route(i,
                         tilebuilder.AddName(r.onestop_id()),
                         tilebuilder.AddName(r.operated_by_onestop_id()),
                         tilebuilder.AddName(r.operated_by_name()),
                         r.route_color(),
                         r.route_text_color(),
                         tilebuilder.AddName(r.name()),
                         tilebuilder.AddName(r.route_long_name()),
                         tilebuilder.AddName(r.route_desc()));
      tilebuilder.AddTransitRoute(route);
      // Route type - need this to store in edge?
      route_types[i] = r.vehicle_type();
  }

  LOG_INFO("Added " + std::to_string(route_types.size()) + " routes");

  return route_types;
}

// Remove transfers?
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

std::list<PointLL> GetShape(const PointLL& stop_ll, const PointLL& endstop_ll, const uint32_t shapeid) {

  std::list<PointLL> shape;

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
                const std::map<GraphId, StopEdges>& stop_edge_map,
                const std::vector<Stop>& stops,
                const std::unordered_map<uint32_t, bool>& stop_access,
                const std::vector<OSMConnectionEdge>& connection_edges,
                const std::unordered_map<uint32_t, uint32_t>& stop_indexes,
                const std::unordered_map<uint32_t, uint32_t>& route_types) {
  // Move existing nodes and directed edge builder vectors and clear the lists
  std::vector<NodeInfoBuilder> currentnodes(std::move(tilebuilder.nodes()));
  tilebuilder.nodes().clear();
  std::vector<DirectedEdgeBuilder> currentedges(std::move(tilebuilder.directededges()));
  tilebuilder.directededges().clear();

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
    size_t edge_index = tilebuilder.directededges().size();
    for (uint32_t i = 0, idx = nb.edge_index(); i < nb.edge_count(); i++, idx++) {
      tilebuilder.directededges().emplace_back(std::move(currentedges[idx]));

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
      const OSMConnectionEdge& conn = connection_edges[added_edges];
      directededge.set_endnode(conn.stop_node);
      directededge.set_length(conn.length);
      directededge.set_use(Use::kTransitConnection);
      directededge.set_speed(5);
      directededge.set_classification(RoadClass::kServiceOther);
      directededge.set_localedgeidx(tilebuilder.directededges().size() - edge_index);
      directededge.set_pedestrianaccess(true, true);
      directededge.set_pedestrianaccess(false, true);

      // Add edge info to the tile and set the offset in the directed edge
      bool added = false;
      std::vector<std::string> names;
      uint32_t edge_info_offset = tilebuilder.AddEdgeInfo(0, conn.osm_node,
                     conn.stop_node, 0, conn.shape, names, added);
      directededge.set_edgeinfo_offset(edge_info_offset);
      directededge.set_forward(added);
      tilebuilder.directededges().emplace_back(std::move(directededge));

      LOG_DEBUG("Add conn from OSM to stop: ei offset = " + std::to_string(edge_info_offset));

      // increment to next connection edge
      added_edges++;
    }

    // Add the node and directed edges
    nb.set_edge_index(edge_index);
    nb.set_edge_count(tilebuilder.directededges().size() - edge_index);
    tilebuilder.nodes().emplace_back(std::move(nb));
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
    const Stop& stop = stops[stop_indexes.find(stopkey)->second];
    if (stop.graphid.id() != stopkey) {
      LOG_ERROR("Stop key not equal!");
    }

    // Build the node info. Use generic transit stop type
    uint32_t access = kPedestrianAccess;
    auto s_access = stop_access.find(stop.graphid.id());
    if (s_access != stop_access.end()) {
      if (s_access->second)
        access |= kBicycleAccess;
    }

    bool child  = (stop.parent != 0);  // TODO verify if this is sufficient
    bool parent = (stop.type == 1);    // TODO verify if this is sufficient
    NodeInfoBuilder node(stop.ll(), RoadClass::kServiceOther, access,
                        NodeType::kMultiUseTransitStop, false);
    node.set_child(child);
    node.set_parent(parent);
    node.set_mode_change(true);
    node.set_stop_id(stop.graphid.id());
    node.set_edge_index(tilebuilder.directededges().size());
    node.set_timezone(stop.timezone);
    LOG_DEBUG("Add node for stop id = " + std::to_string(stop.key));

    // Add connections from the stop to the OSM network
    // TODO - change from linear search for better performance
    for (const auto& conn : connection_edges) {
      if (conn.stop_node == stop.graphid) {
        DirectedEdgeBuilder directededge;
        directededge.set_endnode(conn.osm_node);
        directededge.set_length(conn.length);
        directededge.set_use(Use::kTransitConnection);
        directededge.set_speed(5);
        directededge.set_classification(RoadClass::kServiceOther);
        directededge.set_localedgeidx(tilebuilder.directededges().size() - node.edge_index());
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
        tilebuilder.directededges().emplace_back(std::move(directededge));

        nadded++;  // TEMP for error checking
      }
    }

    // Add any intra-station connections
    for (const auto& endstopkey : stop_edges.second.intrastation) {
      DirectedEdgeBuilder directededge;
      const Stop& endstop = stops[stop_indexes.find(endstopkey)->second];
      if (endstopkey != endstop.graphid) {
        LOG_ERROR("End stop key not equal");
      }
      directededge.set_endnode(endstop.graphid);

      // Make sure length is non-zero
      float length = std::max(1.0f, stop.ll().Distance(endstop.ll()));
      directededge.set_length(length);
      directededge.set_use(Use::kTransitConnection);
      directededge.set_speed(5);
      directededge.set_classification(RoadClass::kServiceOther);
      directededge.set_localedgeidx(tilebuilder.directededges().size() - node.edge_index());
      directededge.set_pedestrianaccess(true, true);
      directededge.set_pedestrianaccess(false, true);

      LOG_DEBUG("Add parent/child directededge - endnode stop id = " +
               std::to_string(endstop.key) + " GraphId: " +
               std::to_string(endstop.graphid.tileid()) + "," +
               std::to_string(endstop.graphid.id()));

      // Add edge info to the tile and set the offset in the directed edge
      bool added = false;
      std::vector<std::string> names;
      std::list<PointLL> shape = { stop.ll(), endstop.ll() };
      uint32_t edge_info_offset = tilebuilder.AddEdgeInfo(0, stop.graphid,
                     endstop.graphid, 0, shape, names, added);
      directededge.set_edgeinfo_offset(edge_info_offset);
      directededge.set_forward(added);

      // Add to list of directed edges
      tilebuilder.directededges().emplace_back(std::move(directededge));
    }

    // Add transit lines
    for (const auto& transitedge : stop_edges.second.lines) {
      // Get the end stop of the connection
      const Stop& endstop = stops[stop_indexes.find(transitedge.stopid)->second];

      // Set Use based on route type...
      Use use = GetTransitUse(route_types.find(transitedge.routeid)->second);
      DirectedEdgeBuilder directededge;
      directededge.set_endnode(endstop.graphid);
      directededge.set_length(stop.ll().Distance(endstop.ll()));
      directededge.set_use(use);
      directededge.set_speed(5);
      directededge.set_classification(RoadClass::kServiceOther);
      directededge.set_localedgeidx(tilebuilder.directededges().size() - node.edge_index());
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
      auto shape = GetShape(stop.ll(), endstop.ll(), transitedge.shapeid);

      uint32_t edge_info_offset = tilebuilder.AddEdgeInfo(transitedge.routeid,
           stop.graphid, endstop.graphid, 0, shape, names, added);

      directededge.set_edgeinfo_offset(edge_info_offset);
      directededge.set_forward(added);

      // Add to list of directed edges
      tilebuilder.directededges().emplace_back(std::move(directededge));
    }
    if (tilebuilder.directededges().size() - node.edge_index() == 0) {
      LOG_ERROR("No directed edges from this node");
    }

    // Add the node
    node.set_edge_count(tilebuilder.directededges().size() - node.edge_index());
    tilebuilder.nodes().emplace_back(std::move(node));
  }
  if (nadded != connection_edges.size()) {
    LOG_ERROR("Added " + std::to_string(nadded) + " but there are " +
              std::to_string(connection_edges.size()) + " connections");
  }

  LOG_DEBUG("AddToGraph tileID: " + std::to_string(tilebuilder.header()->graphid().tileid()) +
           " done. New directed edge count = " + std::to_string(tilebuilder.directededges().size()));
}

void AddOSMConnection(Stop& stop, const GraphTile* tile, const TileHierarchy& tilehierarchy,
                      std::vector<OSMConnectionEdge>& connection_edges) {
  PointLL ll = stop.ll();
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
    const AABB2<PointLL>& aabb = tile->BoundingBox(tilehierarchy);
    LOG_ERROR("No closest edge found for this stop: " + std::to_string(stop.graphid) + " way Id = " +
              std::to_string(wayid) + " tile " + std::to_string(aabb.minx()) + ", " + std::to_string(aabb.miny()) + ", " +
              std::to_string(aabb.maxx()) + ", " +  std::to_string(aabb.maxy()));
    return;
  }

  LOG_DEBUG("edge found for this stop: " + std::to_string(stop.graphid.id()) + " way Id = " +
            std::to_string(wayid));

  // Check if stop is in same tile as the start node
  stop.conn_count = 0;
  float length = 0.0f;
  if (stop.graphid.Tile_Base() == startnode.Tile_Base()) {
    // Add shape from node along the edge until the closest point, then add
    // the closest point and a straight line to the stop lat,lng
    std::list<PointLL> shape;
    for (uint32_t i = 0; i <= std::get<2>(closest); i++) {
      shape.push_back(closest_shape[i]);
    }
    shape.push_back(std::get<0>(closest));
    shape.push_back(stop.ll());
    length = std::max(1.0f, valhalla::midgard::length(shape));

    // Add connection to start node
    connection_edges.push_back({startnode, stop.graphid, length, shape});
    stop.conn_count++;
  }

  // Check if stop is in same tile as end node
  float length2 = 0.0f;
  if (stop.graphid.Tile_Base() == endnode.Tile_Base()) {
    // Add connection to end node
    if (startnode.tileid() == endnode.tileid()) {
      // Add shape from the end to closest point on edge
      std::list<PointLL> shape2;
      for (int32_t i = closest_shape.size()-1; i > std::get<2>(closest); i--) {
        shape2.push_back(closest_shape[i]);
      }
      shape2.push_back(std::get<0>(closest));
      shape2.push_back(stop.ll());
      length2 = std::max(1.0f, valhalla::midgard::length(shape2));

      // Add connection to the end node
      connection_edges.push_back({endnode, stop.graphid, length2, shape2});
      stop.conn_count++;
    }
  }

  if (length != 0.0f && length2 != 0.0 &&
      (length + length2) < edgelength-1) {
    LOG_ERROR("EdgeLength= " + std::to_string(edgelength) + " < connection lengths: " +
             std::to_string(length) + "," + std::to_string(length2) + " when connecting to stop "
             + std::to_string(stop.graphid));
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
void build(const std::string& transit_dir,
           const boost::property_tree::ptree& pt, std::mutex& lock,
           std::unordered_map<GraphId, size_t>::const_iterator tile_start,
           std::unordered_map<GraphId, size_t>::const_iterator tile_end,
           std::promise<builder_stats>& results) {

  // Local Graphreader. Get tile information so we can find bounding boxes
  GraphReader reader(pt);
  const TileHierarchy& hierarchy = reader.GetTileHierarchy();

  // Iterate through the tiles in the queue and find any that include stops
  for(; tile_start != tile_end; ++tile_start) {
    // Get the next tile Id from the queue and get a tile builder
    if(reader.OverCommitted())
      reader.Clear();
    GraphId tile_id = tile_start->first.Tile_Base();

    lock.lock();
    const GraphTile* tile = reader.GetGraphTile(tile_id);
    // Read in the existing tile - deserialize it so we can add to it
    GraphTileBuilder tilebuilder(hierarchy, tile_id, true);
    lock.unlock();

    std::string file_name = GraphTile::FileSuffix(GraphId(tile_id.tileid(), tile_id.level(),0), hierarchy);
    boost::algorithm::trim_if(file_name, boost::is_any_of(".gph"));
    file_name += ".pbf";
    const std::string file = transit_dir + file_name;

    // Make sure it exists
    if (!boost::filesystem::exists(file)) {
      LOG_ERROR("File not found.  " + file);
      return;
    }

    Transit transit; {
      std::fstream input(file, std::ios::in | std::ios::binary);
      if (!input) {
        LOG_ERROR("Error opening file:  " + file);
        return;
      }
      std::string buffer((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
      google::protobuf::io::ArrayInputStream as(static_cast<const void*>(buffer.c_str()), buffer.size());
      google::protobuf::io::CodedInputStream cs(static_cast<google::protobuf::io::ZeroCopyInputStream*>(&as));
      cs.SetTotalBytesLimit(buffer.size() * 2, buffer.size() * 2);
      if (!transit.ParseFromCodedStream(&cs)) {
        LOG_ERROR("Failed to parse file: " + file);
        return;
      }
    }

    // Iterate through stops and form connections to OSM network. Each
    // stop connects to 1 or 2 OSM nodes along the closest OSM way.
    // TODO - future - how to handle connections that reach nodes
    // outside the tile - may have to move this outside the tile
    // iteration...?
    // TODO - handle a list of connections/egrees points
    // TODO - what if we split the edge and insert a node?
    std::vector<Stop> stops = AddStops(transit,tilebuilder,tile_start->second,DateTime::get_tz_db().regions);

    std::vector<OSMConnectionEdge> connection_edges;
    Stop stop;
    std::unordered_map<uint32_t, uint32_t> stop_indexes;
    std::unordered_multimap<uint32_t, uint32_t> children;

    // Create a map of stop key to index in the stop vector
    uint32_t n = 0;
    for (auto& stop : stops) {

      if (stop.parent == 0) {
        //TODO: multiple threads writing into the stops at once but never the same one, right?
        AddOSMConnection(stop, tile, hierarchy, connection_edges);
        stop_indexes[stop.graphid.id()] = n;
      }
      // Do we have have a parent
      if (stop.type == 0 && stop.parent != 0) {
        children.emplace(stop.parent, stop.graphid.id());
      }

      n++;
    }

    LOG_INFO("Connection Edges: size= " + std::to_string(connection_edges.size()));
    std::sort(connection_edges.begin(), connection_edges.end());

    // Get all scheduled departures from the stops within this tile. Record
    // unique trips, routes, TODO
    std::unordered_set<uint32_t> route_keys;
    std::unordered_set<uint32_t> trip_keys;
    std::map<GraphId, StopEdges> stop_edge_map;
    uint32_t unique_lineid = 1;
    std::vector<TransitDeparture> transit_departures;

    // Create a map of stop key to index in the stop vector
    std::unordered_map<uint32_t, bool> stop_access;
    std::unordered_multimap<uint32_t, Departure> departures =
        ProcessStopPairs(transit, tile_start->second, stop_access);

    LOG_DEBUG("Got " + std::to_string(departures.size()) + " departures.");

    for (auto& stop : stops) {
      StopEdges stopedges;
      stopedges.stop_key = stop.graphid.id();

      // Identify any parent-child edge connections (to add later)
      if (stop.type == 1) {
        // Station - identify any children.
        auto range = children.equal_range(stop.graphid.id());
        for(auto kv = range.first; kv != range.second; ++kv)
          stopedges.intrastation.push_back(kv->second);
      } else if (stop.parent != 0) {
        stopedges.intrastation.push_back(stop.parent);
      }

      std::map<std::pair<uint32_t, uint32_t>, uint32_t> unique_transit_edges;
      auto range = departures.equal_range(stop.graphid.id());
      for(auto key = range.first; key != range.second; ++key) {
        Departure dep = key->second;
        route_keys.insert(dep.route);
        trip_keys.insert(dep.trip);

        // Identify unique route and arrival stop pairs - associate to a
        // unique line Id stored in the directed edge.
        uint32_t lineid;
        auto m = unique_transit_edges.find({dep.route, dep.dest_stop});
        if (m == unique_transit_edges.end()) {
          // Add to the map and update the line id
          lineid = unique_lineid;
          unique_transit_edges[{dep.route, dep.dest_stop}] = unique_lineid;
          unique_lineid++;
          stopedges.lines.emplace_back(TransitLine{lineid, dep.route, dep.dest_stop, dep.shapeid});
        } else {
          lineid = m->second;
        }

        // Form transit departures
        uint32_t headsign_offset = tilebuilder.AddName(dep.headsign);
        uint32_t elapsed_time = dep.arr_time - dep.dep_time;
        TransitDeparture td(lineid, dep.trip,dep.route,
                    dep.blockid, headsign_offset, dep.dep_time, elapsed_time,
                    dep.start_date, dep.end_date, dep.dow, dep.days);

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

      // Add to stop edge map - track edges that need to be added. This is
      // sorted by graph Id so the stop nodes are added in proper order
      stop_edge_map.insert({stop.graphid, stopedges});
    }

    // Add routes to the tile. Get map of route types.
    const std::unordered_map<uint32_t, uint32_t> route_types = AddRoutes(transit,
                                                                   route_keys,
                                                                   tilebuilder);
    // Add nodes, directededges, and edgeinfo
    AddToGraph(tilebuilder, stop_edge_map, stops, stop_access, connection_edges,
               stop_indexes, route_types);

    // Write the new file
    lock.lock();
    tilebuilder.StoreTileData();
    lock.unlock();
  }

  // Send back the statistics
  results.set_value({});
}

GraphId TransitToTile(const boost::property_tree::ptree& pt, const std::string& transit_tile) {
  auto tile_dir = pt.get<std::string>("mjolnir.hierarchy.tile_dir");
  auto transit_dir = pt.get<std::string>("mjolnir.transit_dir");
  auto graph_tile = tile_dir + transit_tile.substr(transit_dir.size());
  boost::algorithm::trim_if(graph_tile, boost::is_any_of(".pbf"));
  graph_tile += ".gph";
  TileHierarchy hierarchy(pt.get_child("mjolnir.hierarchy"));
  return GraphTile::GetTileId(graph_tile, hierarchy);
}

}

namespace valhalla {
namespace mjolnir {

// Add transit to the graph
void TransitBuilder::Build(const boost::property_tree::ptree& pt) {

  std::unordered_map<GraphId, size_t> tiles;

  // Bail if nothing
  auto transit_dir = pt.get_optional<std::string>("mjolnir.transit_dir");
  if(!transit_dir || !boost::filesystem::exists(*transit_dir) || !boost::filesystem::is_directory(*transit_dir)) {
    LOG_INFO("Transit directory not found. Transit will not be added.");
    return;
  }
  // Also bail if nothing inside
  transit_dir->push_back('/');
  std::map<GraphId, std::string> transit_tiles;
  TileHierarchy hierarchy(pt.get_child("mjolnir.hierarchy"));
  auto local_level = hierarchy.levels().rbegin()->first;
  if(boost::filesystem::is_directory(*transit_dir + std::to_string(local_level) + "/")) {
    boost::filesystem::recursive_directory_iterator transit_file_itr(*transit_dir + std::to_string(local_level) + "/"), end_file_itr;
    for(; transit_file_itr != end_file_itr; ++transit_file_itr) {
      if(boost::filesystem::is_regular(transit_file_itr->path()) && transit_file_itr->path().extension() == ".pbf") {
        auto graph_id = TransitToTile(pt, transit_file_itr->path().string());
        //TODO: this precludes a transit only network, which kind of sucks but
        //right now we are assuming that we have to connect stops to the OSM
        //road network so if that assumption goes away this can too
        if(GraphReader::DoesTileExist(hierarchy, graph_id)) {
          GraphTileBuilder tb(hierarchy, graph_id, true);
          tiles.emplace(graph_id, tb.nodes().size());
          transit_tiles.emplace(graph_id, transit_file_itr->path().string());

        }
      }
    }
  }
  if(!transit_tiles.size()) {
    LOG_INFO("No transit tiles found. Transit will not be added.");
    return;
  }

  // TODO - intermediate pass to find any connections that cross into different
  // tile than the stop

  // Second pass - for all tiles with transit stops get all transit information
  // and populate tiles

  // A place to hold worker threads and their results
  std::vector<std::shared_ptr<std::thread> > threads(
    std::max(static_cast<uint32_t>(1),
      pt.get<uint32_t>("concurrency", std::thread::hardware_concurrency())));

  // An atomic object we can use to do the synchronization
  std::mutex lock;

  // A place to hold the results of those threads (exceptions, stats)
  std::list<std::promise<builder_stats> > results;

  // Start the threads
  LOG_INFO("Adding " + std::to_string(transit_tiles.size()) + " transit tiles to the local graph...");
  // Divvy up the work
  size_t floor = tiles.size() / threads.size();
  size_t at_ceiling = tiles.size() - (threads.size() * floor);
  std::unordered_map<GraphId, size_t>::const_iterator tile_start, tile_end = tiles.begin();

  // Atomically pass around stats info
  for (size_t i = 0; i < threads.size(); ++i) {
    // Figure out how many this thread will work on (either ceiling or floor)
    size_t tile_count = (i < at_ceiling ? floor + 1 : floor);
    // Where the range begins
    tile_start = tile_end;
    // Where the range ends
    std::advance(tile_end, tile_count);
    // Make the thread
    results.emplace_back();
    threads[i].reset(
      new std::thread(build, *transit_dir, std::cref(pt.get_child("mjolnir.hierarchy")),
                      std::ref(lock), tile_start, tile_end, std::ref(results.back()))
    );
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
