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

struct Departure {
  uint64_t days;
  GraphId  orig_pbf_graphid;   // GraphId in pbf tiles
  GraphId  dest_pbf_graphid;   // GraphId in pbf tiles
  uint32_t trip;
  uint32_t route;
  uint32_t blockid;
  uint32_t shapeid;
  uint32_t end_day;
  uint32_t headsign_offset;
  uint32_t dep_time;
  float    orig_dist_traveled;
  float    dest_dist_traveled;
  uint16_t elapsed_time;
  uint8_t  dow;
  bool     wheelchair_accessible;
};

// Unique route and stop
struct TransitLine {
  uint32_t lineid;
  uint32_t routeid;
  GraphId  dest_pbf_graphid;  // GraphId (from pbf) of the destination stop
  uint32_t shapeid;
  float    orig_dist_traveled;
  float    dest_dist_traveled;
};

// Shape
struct Shape {
  uint32_t begins;
  uint32_t ends;
  std::vector<PointLL> shape;
};

struct StopEdges {
  GraphId origin_pbf_graphid;        // GraphId (from pbf) of the origin stop
  std::vector<GraphId> intrastation; // List of intra-station connections
  std::vector<TransitLine> lines;    // Set of unique route/stop pairs
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

// Get scheduled departures for a stop
std::unordered_multimap<GraphId, Departure> ProcessStopPairs(
    GraphTileBuilder& tilebuilder,
    const TileHierarchy& hierarchy,
    const Transit& transit,
    std::unordered_map<GraphId, bool>& stop_access,
    const std::string& file,
    const GraphId& tile_id) {
  // Check if there are no schedule stop pairs in this tile
  std::unordered_multimap<GraphId, Departure> departures;
  uint32_t tile_date = tilebuilder.header()->date_created();

  std::size_t slash_found = file.find_last_of("/\\");
  std::string directory = file.substr(0,slash_found);

  boost::filesystem::recursive_directory_iterator transit_file_itr(directory);
  boost::filesystem::recursive_directory_iterator end_file_itr;

  //for each tile.
  for(; transit_file_itr != end_file_itr; ++transit_file_itr) {
    if(boost::filesystem::is_regular(transit_file_itr->path())) {
      std::string fname = transit_file_itr->path().string();
      std::string ext = transit_file_itr->path().extension().string();
      std::string file_name = fname.substr(0, fname.size() - ext.size());

      // make sure we are looking at a pbf file
      if ((ext == ".pbf" && fname == file) ||
          (file_name.substr(file_name.size()-4) == ".pbf" && file_name == file)) {

        Transit spp; {

          // already loaded
          if (ext == ".pbf")
            spp = transit;
          else {
            std::fstream input(fname, std::ios::in | std::ios::binary);
            if (!input) {
              LOG_ERROR("Error opening file:  " + fname);
              departures.clear();
              return departures;
            }
            std::string buffer((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
            google::protobuf::io::ArrayInputStream as(static_cast<const void*>(buffer.c_str()), buffer.size());
            google::protobuf::io::CodedInputStream cs(static_cast<google::protobuf::io::ZeroCopyInputStream*>(&as));
            cs.SetTotalBytesLimit(buffer.size() * 2, buffer.size() * 2);
            if (!spp.ParseFromCodedStream(&cs)) {
              LOG_ERROR("Failed to parse file: " + fname);
              return departures;
            }
          }
        }

        if (spp.stop_pairs_size() == 0) {
          if (transit.stops_size() > 0) {
            LOG_ERROR("Tile " + fname +
                      " has 0 schedule stop pairs but has " +
                      std::to_string(transit.stops_size()) + " stops");
          }
          departures.clear();
          return departures;
        }

        // Iterate through the stop pairs in this tile and form Valhalla departure
        // records
        for (const auto& sp : spp.stop_pairs()) {
          // We do not know in this step if the end node is in a valid (non-empty)
          // Valhalla tile. So just add the stop pair and we will address this later

          // Use transit PBF graph Ids internally until adding to the graph tiles
          // TODO - wheelchair accessible, shape information
          Departure dep;
          dep.orig_pbf_graphid = GraphId(sp.origin_graphid());
          dep.dest_pbf_graphid = GraphId(sp.destination_graphid());
          dep.route = sp.route_index();
          dep.trip = sp.trip_id();

          // if we have shape data then set everything else shapeid = 0;
          if (sp.has_shape_id() && sp.has_destination_dist_traveled() && sp.has_origin_dist_traveled()) {
            dep.shapeid = sp.shape_id();
            dep.orig_dist_traveled = sp.origin_dist_traveled();
            dep.dest_dist_traveled = sp.destination_dist_traveled();
          } else dep.shapeid = 0;

          dep.blockid = sp.has_block_id() ? sp.block_id() : 0;
          dep.dep_time = sp.origin_departure_time();
          dep.elapsed_time = sp.destination_arrival_time() - dep.dep_time;

          // Set bikes_allowed on the stops
          // TODO - should this be |= ???
          bool bikes_allowed = sp.bikes_allowed();
          stop_access[dep.orig_pbf_graphid] = bikes_allowed;
          stop_access[dep.dest_pbf_graphid] = bikes_allowed;

          // Compute days of week mask
          uint8_t dow_mask = kDOWNone;
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

          // Compute the valid days
          // set the bits based on the dow.
          boost::gregorian::date start_date(boost::gregorian::gregorian_calendar::from_julian_day_number(sp.service_start_date()));
          boost::gregorian::date end_date(boost::gregorian::gregorian_calendar::from_julian_day_number(sp.service_end_date()));
          dep.days = DateTime::get_service_days(start_date, end_date, tile_date, dow_mask);

          // if this is a service addition for one day, delete the dow_mask.
          if (sp.service_start_date() == sp.service_end_date())
            dep.dow = kDOWNone;

          // if dep.days == 0 then feed either starts after the end_date or tile_header_date > end_date
          if (dep.days == 0 && !sp.service_added_dates_size()) {
            LOG_DEBUG("Feed rejected!  Start date: " + to_iso_extended_string(start_date) + " End date: " + to_iso_extended_string(end_date));
            continue;
          }

          dep.headsign_offset = tilebuilder.AddName(sp.trip_headsign());
          dep.end_day = (DateTime::days_from_pivot_date(end_date) - tile_date);

          //if subtractions are between start and end date then turn off bit.
          for (const auto& x : sp.service_except_dates()) {
            boost::gregorian::date d(boost::gregorian::gregorian_calendar::from_julian_day_number(x));
            dep.days = DateTime::remove_service_day(dep.days, start_date, end_date, d);
          }

          //if additions are between start and end date then turn on bit.
          for (const auto& x : sp.service_added_dates()) {
            boost::gregorian::date d(boost::gregorian::gregorian_calendar::from_julian_day_number(x));
            dep.days = DateTime::add_service_day(dep.days, start_date, end_date, d);
          }

          // Add to the departures list
          departures.emplace(dep.orig_pbf_graphid, std::move(dep));
        }
      }
    }
  }
  LOG_INFO("Tile " + std::to_string(tile_id.tileid()) + ": added " +
           std::to_string(departures.size()) + " departures");
  return departures;
}

// Add routes to the tile. Return a vector of route types.
std::vector<uint32_t> AddRoutes(const Transit& transit,
                   GraphTileBuilder& tilebuilder) {
  // Route types vs. index
  std::vector<uint32_t> route_types;

  for (uint32_t i = 0; i < transit.routes_size(); i++) {
    const Transit_Route& r = transit.routes(i);
      TransitRoute route(i,
                         tilebuilder.AddName(r.onestop_id()),
                         tilebuilder.AddName(r.operated_by_onestop_id()),
                         tilebuilder.AddName(r.operated_by_name()),
                         tilebuilder.AddName(r.operated_by_website()),
                         r.route_color(),
                         r.route_text_color(),
                         tilebuilder.AddName(r.name()),
                         tilebuilder.AddName(r.route_long_name()),
                         tilebuilder.AddName(r.route_desc()));
      LOG_DEBUG("Route idx = " + std::to_string(i) + ": " + r.name() + ","
                     + r.route_long_name());
      tilebuilder.AddTransitRoute(route);

      // Route type - need this to store in edge.
      route_types.push_back(r.vehicle_type());
  }
  return route_types;
}

// TODO - Add transfers from a stop - need Transitland support!
void AddTransfers(GraphTileBuilder& tilebuilder) {
  // TransitTransfer transfer(fromstop, tostop,
  //           static_cast<TransferType>(type), mintime);
  //tilebuilder.AddTransitTransfer(transfer);
}

// Get Use given the transit route type
// TODO - add separate Use for different types - when we do this change
// the directed edge IsTransit method
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

std::list<PointLL> GetShape(const PointLL& stop_ll, const PointLL& endstop_ll, uint32_t shapeid,
                            const float orig_dist_traveled, const float dest_dist_traveled,
                            const std::vector<PointLL>& trip_shape, const std::vector<float>& distances) {

  std::list<PointLL> shape;
  if (shapeid != 0 && trip_shape.size() && stop_ll != endstop_ll &&
      orig_dist_traveled < dest_dist_traveled) {

    float distance = 0.0f, d_from_p0_to_x = 0.0f;

    // point x - we are trying to find it on the line segment between p0 and p1
    PointLL x;
    bool found = false;

    // find out where orig_dist_traveled should be in the list.
    auto lower_bound = std::lower_bound(distances.cbegin(), distances.cend(), orig_dist_traveled);
    // find out where dest_dist_traveled should be in the list.
    auto upper_bound = std::upper_bound(distances.cbegin(), distances.cend(), dest_dist_traveled);
    float prev_distance = *(lower_bound);

    // lower_bound returns an iterator pointing to the first element which does not compare less than the dist_traveled;
    // therefore, we need to back up one if it does not equal the lower_bound value.  For example, we could be starting
    // at the beginning of the points list
    if (orig_dist_traveled != (*lower_bound))
      prev_distance = *(--lower_bound);

    // loop through the points.
    for (auto itr = lower_bound; itr != upper_bound; ++itr) {

      /*    |
       *    |
       *    p0
       *    | }--d_from_p0_to_x (distance from p0 to x)
       *    x -- point we are trying to find on the segment (orig_dist_traveled or dest_dist_traveled on this segment)
       *    |
       *    |
       *    |
       *    |
       *    p1
       *    |
       *    |
       */

      // index into our vector of points
      uint32_t index = (itr - distances.cbegin());
      PointLL p0 = trip_shape[index];
      PointLL p1 = trip_shape[index + 1];

      // this is our distance that is beyond x.
      distance = *(itr+1);

      // find point x using the orig_dist_traveled - this is our first point added to shape
      if (itr == lower_bound) {
        // distance from p0 to x using the orig_dist_traveled
        d_from_p0_to_x = (orig_dist_traveled - prev_distance) / (distance - prev_distance);
        x = p0 + (p1 - p0) * d_from_p0_to_x;
        shape.push_back(x);
      }

      // find point x using the dest_dist_traveled - this is our last point added to the shape
      if ((itr+1) == upper_bound) {
        // distance from p0 to x using the dest_dist_traveled
        d_from_p0_to_x = (dest_dist_traveled - prev_distance) / (distance - prev_distance);
        x = p0 + (p1 - p0) * d_from_p0_to_x;
        shape.push_back(x);
        // we are done p1 is too far away
        break;
      }

      // add all the midpoints.
      shape.push_back(p1);

      prev_distance = distance;
    }
  // else no shape exists.
  } else {
    shape.push_back(stop_ll);
    shape.push_back(endstop_ll);
  }
  return shape;
}

// Converts a stop's pbf graph Id to a Valhalla graph Id by adding the
// tile's node count. Returns an Invalid GraphId if the tile is not found
// in the list of Valhalla tiles
GraphId GetGraphId(const GraphId& nodeid,
                   const std::unordered_map<GraphId, size_t>& tile_node_counts) {
  auto t = tile_node_counts.find(nodeid.Tile_Base());
  if (t == tile_node_counts.end()) {
    return GraphId();  // Invalid graph Id
  } else {
    return { nodeid.tileid(), nodeid.level(), nodeid.id() + t->second };
  }
}

// Get PBF transit data given a GraphId / tile
Transit read_pbf(const GraphId& id, const TileHierarchy& hierarchy,
                 const std::string& transit_dir) {
  std::string fname = GraphTile::FileSuffix(id, hierarchy);
  fname = fname.substr(0, fname.size() - 3) + "pbf";
  std::string file_name = transit_dir + '/' + fname;
  std::fstream file(file_name, std::ios::in | std::ios::binary);
  if(!file) {
    throw std::runtime_error("Couldn't load " + file_name);
  }
  std::string buffer((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  google::protobuf::io::ArrayInputStream as(static_cast<const void*>(buffer.c_str()), buffer.size());
  google::protobuf::io::CodedInputStream cs(static_cast<google::protobuf::io::ZeroCopyInputStream*>(&as));
  cs.SetTotalBytesLimit(buffer.size() * 2, buffer.size() * 2);
  Transit transit;
  if(!transit.ParseFromCodedStream(&cs))
    throw std::runtime_error("Couldn't load " + file_name);
  return transit;
}

void AddToGraph(GraphTileBuilder& tilebuilder,
                const TileHierarchy& hierarchy,
                const std::string& transit_dir,
                const std::unordered_map<GraphId, size_t>& tile_node_counts,
                const std::map<GraphId, StopEdges>& stop_edge_map,
                const std::unordered_map<GraphId, bool>& stop_access,
                const std::vector<OSMConnectionEdge>& connection_edges,
                const std::unordered_map<uint32_t, Shape> shape_data,
                const std::vector<float> distances,
                const std::vector<uint32_t>& route_types) {
  auto t1 = std::chrono::high_resolution_clock::now();

  // Move existing nodes and directed edge builder vectors and clear the lists
  std::vector<NodeInfo> currentnodes(std::move(tilebuilder.nodes()));
  uint32_t nodecount = currentnodes.size();
  tilebuilder.nodes().clear();
  std::vector<DirectedEdge> currentedges(std::move(tilebuilder.directededges()));
  uint32_t edgecount = currentedges.size();
  tilebuilder.directededges().clear();

  // Get the directed edge index of the first sign. If no signs are
  // present in this tile set a value > number of directed edges
  uint32_t signidx = 0;
  uint32_t nextsignidx = (tilebuilder.header()->signcount() > 0) ?
    tilebuilder.sign(0).edgeindex() : currentedges.size() + 1;
  uint32_t signcount = tilebuilder.header()->signcount();

  // Get the directed edge index of the first access restriction.
  uint32_t residx = 0;
  uint32_t nextresidx = (tilebuilder.header()->access_restriction_count() > 0) ?
      tilebuilder.accessrestriction(0).edgeindex() : currentedges.size() + 1;;
  uint32_t rescount = tilebuilder.header()->access_restriction_count();

  // Get Transit PBF data for this tile
  GraphId tileid = tilebuilder.header()->graphid().Tile_Base();
  Transit transit = read_pbf(tileid, hierarchy, transit_dir);

  // Iterate through the nodes - add back any stored edges and insert any
  // connections from a node to a transit stop. Update each nodes edge index.
  uint32_t nodeid = 0;
  uint32_t added_edges = 0;
  uint32_t connedges = 0;
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

      // Add any restrictions that use this idx - increment their index by the
      // number of added edges
      while (idx == nextresidx && residx < rescount) {
        if (!currentedges[idx].access_restriction()) {
          LOG_ERROR("Access restrictions for this index but directededge says none");
        }
        tilebuilder.accessrestriction_builder(residx).set_edgeindex(idx + added_edges);

        // Increment to the next restriction and update nextresidx
        residx++;
        nextresidx = (residx >= rescount) ?
              0 : tilebuilder.accessrestriction(residx).edgeindex();
      }
    }

    // Add directed edges for any connections from the OSM node
    // to a transit stop
    while (added_edges < connection_edges.size() &&
           connection_edges[added_edges].osm_node.id() == nodeid) {
      const OSMConnectionEdge& conn = connection_edges[added_edges];

      // Add the tile's node count to the pbf Graph Id
      GraphId endnode = GetGraphId(conn.stop_node, tile_node_counts);
      if (!endnode.Is_Valid()) {
        continue;
      }

      DirectedEdge directededge;
      directededge.set_endnode(endnode);
      directededge.set_length(conn.length);
      directededge.set_use(Use::kTransitConnection);
      directededge.set_speed(5);
      directededge.set_classification(RoadClass::kServiceOther);
      directededge.set_localedgeidx(tilebuilder.directededges().size() - edge_index);
      directededge.set_forwardaccess(kPedestrianAccess);  // TODO - bikes?
      directededge.set_reverseaccess(kPedestrianAccess);  // TODO - bikes?

      // Add edge info to the tile and set the offset in the directed edge
      bool added = false;
      std::vector<std::string> names;
      uint32_t edge_info_offset = tilebuilder.AddEdgeInfo(0, conn.osm_node,
                     endnode, 0, conn.shape, names, added);
      directededge.set_edgeinfo_offset(edge_info_offset);
      directededge.set_forward(added);
      tilebuilder.directededges().emplace_back(std::move(directededge));

      LOG_DEBUG("Add conn from OSM to stop: ei offset = " + std::to_string(edge_info_offset));

      // increment to next connection edge
      connedges++;
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
  uint32_t transitedges = 0;
  for (const auto& stop_edges : stop_edge_map) {
    // Get the stop information
    GraphId stopid = stop_edges.second.origin_pbf_graphid;
    uint32_t stop_index = stopid.id();
    const Transit_Stop& stop = transit.stops(stop_index);
    if (GraphId(stop.graphid()) != stopid) {
      LOG_ERROR("Stop key not equal!");
    }

    LOG_DEBUG("Transit Stop: " + stop.name() + " stop index= " +
              std::to_string(stop_index));

    // Get the Valhalla graphId of the origin node (transit stop)
    GraphId origin_node = GetGraphId(stopid, tile_node_counts);

    // Build the node info. Use generic transit stop type
    uint32_t access = kPedestrianAccess;
    /** TODO bicycle access
    auto s_access = stop_access.find(stop.pbf_graphid);
    if (s_access != stop_access.end()) {
      if (s_access->second)
        access |= kBicycleAccess;
    }
    **/

    // TODO - parent/child flags
    bool child  = false; // (stop.parent.Is_Valid());  // TODO verify if this is sufficient
    bool parent = false; // (stop.type == 1);          // TODO verify if this is sufficient
    PointLL stopll = { stop.lon(), stop.lat() };
    NodeInfo node(stopll, RoadClass::kServiceOther, access,
                        NodeType::kMultiUseTransitStop, false);
    node.set_child(child);
    node.set_parent(parent);
    node.set_mode_change(true);
    node.set_stop_index(stop_index);
    node.set_edge_index(tilebuilder.directededges().size());
    node.set_timezone(stop.timezone());

    // Add connections from the stop to the OSM network
    // TODO - change from linear search for better performance
    for (const auto& conn : connection_edges) {
      if (conn.stop_node == stopid) {
        DirectedEdge directededge;
        directededge.set_endnode(conn.osm_node);
        directededge.set_length(conn.length);
        directededge.set_use(Use::kTransitConnection);
        directededge.set_speed(5);
        directededge.set_classification(RoadClass::kServiceOther);
        directededge.set_localedgeidx(tilebuilder.directededges().size() - node.edge_index());
        directededge.set_forwardaccess(kPedestrianAccess);  // TODO - bikes?
        directededge.set_reverseaccess(kPedestrianAccess);  // TODO - bikes?

        // Add edge info to the tile and set the offset in the directed edge
        bool added = false;
        std::vector<std::string> names;
        uint32_t edge_info_offset = tilebuilder.AddEdgeInfo(0, origin_node,
                       conn.osm_node, 0, conn.shape, names, added);
        LOG_DEBUG("Add conn from stop to OSM: ei offset = " + std::to_string(edge_info_offset));
        directededge.set_edgeinfo_offset(edge_info_offset);
        directededge.set_forward(added);

        // Add to list of directed edges
        tilebuilder.directededges().emplace_back(std::move(directededge));
        connedges++;
        nadded++;  // TEMP for error checking
      }
    }

 /** TODO - future when we get egress, station, platform hierarchy
    // Add any intra-station connections - these are always in the same tile?
    for (const auto& endstopid : stop_edges.second.intrastation) {

      const Stop& endstop = stops[endstopid.id()];
      if (endstopid != endstop.pbf_graphid) {
        LOG_ERROR("End stop key not equal");
      }

      GraphId endnode = GetGraphId(endstop.pbf_graphid);
      if (!endnode.Is_Valid()) {
        continue;
      }

      DirectedEdge directededge;
      directededge.set_endnode(endstop.pbf_graphid);

      // Make sure length is non-zero
      float length = std::max(1.0f, stop.ll().Distance(endstop.ll()));
      directededge.set_length(length);
      directededge.set_use(Use::kTransitConnection);
      directededge.set_speed(5);
      directededge.set_classification(RoadClass::kServiceOther);
      directededge.set_localedgeidx(tilebuilder.directededges().size() - node.edge_index());
      directededge.set_forwardaccess(kPedestrianAccess);  // TODO - bikes?
      directededge.set_reverseaccess(kPedestrianAccess);  // TODO - bikes?

      LOG_DEBUG("Add parent/child directededge - endnode stop id = " +
               std::to_string(endstop.key) + " GraphId: " +
               std::to_string(endstop.graphid.tileid()) + "," +
               std::to_string(endstop.graphid.id()));

      // Add edge info to the tile and set the offset in the directed edge
      bool added = false;
      std::vector<std::string> names;
      std::list<PointLL> shape = { stop.ll(), endstop.ll() };

      // TODO - these need to be valhalla graph Ids
      uint32_t edge_info_offset = tilebuilder.AddEdgeInfo(0, stop.pbf_graphid,
                     endstop.pbf_graphid, 0, shape, names, added);
      directededge.set_edgeinfo_offset(edge_info_offset);
      directededge.set_forward(added);

      // Add to list of directed edges
      tilebuilder.directededges().emplace_back(std::move(directededge));
    }
**/

    // Add transit lines
    for (const auto& transitedge : stop_edges.second.lines) {
      // Get the end node. Skip this directed edge if the Valhalla tile is
      // not valid (or empty)
      GraphId endnode = GetGraphId(transitedge.dest_pbf_graphid, tile_node_counts);
      if (!endnode.Is_Valid()) {
        continue;
      }

      // Find the lat,lng of the end stop
      PointLL endll;
      std::string endstopname;
      GraphId end_stop_graphid = transitedge.dest_pbf_graphid;
      if (end_stop_graphid.Tile_Base() == tileid) {
        // End stop is in the same pbf transit tile
        const Transit_Stop& endstop = transit.stops(end_stop_graphid.id());
        endstopname = endstop.name();
        endll = {endstop.lon(), endstop.lat()};
      } else {
        // Get Transit PBF data for this tile
        Transit endtransit = read_pbf(end_stop_graphid.Tile_Base(), hierarchy, transit_dir);
        const Transit_Stop& endstop = endtransit.stops(end_stop_graphid.id());
        endstopname = endstop.name();
        endll = {endstop.lon(), endstop.lat()};
      }

      // Add the directed edge
      DirectedEdge directededge;
      directededge.set_endnode(endnode);
      directededge.set_length(stopll.Distance(endll));
      Use use = GetTransitUse(route_types[transitedge.routeid]);
      directededge.set_use(use);
      directededge.set_speed(5);
      directededge.set_classification(RoadClass::kServiceOther);
      directededge.set_localedgeidx(tilebuilder.directededges().size() - node.edge_index());
      directededge.set_forwardaccess(kPedestrianAccess);  // TODO - bikes?
      directededge.set_reverseaccess(kPedestrianAccess);  // TODO - bikes?
      directededge.set_lineid(transitedge.lineid);

      LOG_DEBUG("Add transit directededge - lineId = " + std::to_string(transitedge.lineid) +
         " Route Key = " + std::to_string(transitedge.routeid) +
         " EndStop " + endstopname);

      // Add edge info to the tile and set the offset in the directed edge
      // Leave the name empty. Use the trip Id to look up the route Id and
      // route within TripPathBuilder.
      bool added = false;
      std::vector<std::string> names;

      std::vector<PointLL> points;
      std::vector<float> distance;
      // get the indexes and vector of points for this shape id
      const auto& found = shape_data.find(transitedge.shapeid);
      if (transitedge.shapeid != 0 && found != shape_data.cend()) {
        const auto& shape_d = found->second;
        points = shape_d.shape;
        // copy only the distances that we care about.
        std::copy((distances.cbegin()+shape_d.begins),
                  (distances.cbegin()+shape_d.ends),back_inserter(distance));
      }
      else if (transitedge.shapeid != 0)
        LOG_WARN("Shape Id not found: " + std::to_string(transitedge.shapeid));

      auto shape = GetShape(stopll, endll, transitedge.shapeid, transitedge.orig_dist_traveled,
                            transitedge.dest_dist_traveled, points, distance);
      uint32_t edge_info_offset = tilebuilder.AddEdgeInfo(transitedge.routeid,
           origin_node, endnode, 0, shape, names, added);

      directededge.set_edgeinfo_offset(edge_info_offset);
      directededge.set_forward(added);

      // Add to list of directed edges
      tilebuilder.directededges().emplace_back(std::move(directededge));
      transitedges++;
    }

    // Get the directed edge count, log an error if no directed edges are added
    uint32_t edge_count = tilebuilder.directededges().size() - node.edge_index();
    if (edge_count == 0) {
      // Set the edge index to 0
      node.set_edge_index(0);
      LOG_ERROR("No directed edges from this node");
    }

    // Add the node
    node.set_edge_count(edge_count);
    tilebuilder.nodes().emplace_back(std::move(node));
  }
  if (nadded != connection_edges.size()) {
    LOG_ERROR("Added " + std::to_string(nadded) + " but there are " +
              std::to_string(connection_edges.size()) + " connections");
  }

  // Log the number of added nodes and edges
  uint32_t addednodes = tilebuilder.nodes().size() - nodecount;
  auto t2 = std::chrono::high_resolution_clock::now();
  uint32_t msecs = std::chrono::duration_cast<std::chrono::milliseconds>(
                  t2 - t1).count();
  LOG_INFO("Tile " + std::to_string(tilebuilder.header()->graphid().tileid())
          + ": added " + std::to_string(connedges) + " connection edges, "
          + std::to_string(transitedges) + " transit edges, and "
          + std::to_string(addednodes) + " nodes. time = "
          + std::to_string(msecs) + " ms");
}

// Add connection edges from the transit stop to an OSM edge
void AddOSMConnection(const Transit_Stop& stop, const GraphTile* tile,
                      const TileHierarchy& tilehierarchy,
                      GraphReader& reader,
                      std::mutex& lock,
                      std::vector<OSMConnectionEdge>& connection_edges) {
  PointLL stop_ll = {stop.lon(), stop.lat() };
  uint64_t wayid = stop.osm_way_id();

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

        // this is a temp hack until we have some time to have loki return multiple, ranked results.
        // if the assigned wayid is a ferry, try to find a pedestrian edge instead.
        // use the distance to the nodes to determine who is closer and where to look for pedestrian edges
        if (directededge->use() == Use::kFerry || directededge->use() == Use::kRailFerry) {
          float start_distance = node->latlng().Distance(stop_ll);

          const DirectedEdge* opp_de = tile->directededge(node->edge_index() + directededge->opp_index());
          GraphId end_node = opp_de->endnode();
          float end_distance = 0.0f;

          const GraphTile* endnode_tile = tile;
          if ( directededge->endnode().Tile_Base() != end_node.Tile_Base()) {
              // Get the end node tile
              lock.lock();
              endnode_tile = reader.GetGraphTile(end_node);
              lock.unlock();
          }

          const NodeInfo* closest_node = endnode_tile->node(end_node.id());
          end_distance = closest_node->latlng().Distance(stop_ll);
          if (start_distance <= end_distance)
            closest_node = node;

          // loop until we hopefully find an edge with pedestrian access.
          for (uint32_t x = 0, count = closest_node->edge_count(); x < count; x++) {
            const DirectedEdge* de = endnode_tile->directededge(closest_node->edge_index() + x);
            auto edge_info = endnode_tile->edgeinfo(de->edgeinfo_offset());

            if (edge_info->wayid() != wayid && (de->forwardaccess() & kPedestrianAccess) &&
                de->use() != Use::kFerry && de->use() != Use::kRailFerry) {
              // update the wayid
              wayid = edge_info->wayid();
              break;
            }
          }
          if (wayid != edgeinfo->wayid()) {
            //restart the search process with the updated wayid.
            i = 0;
            mindist = 10000000.0f;
            edgelength = 0;
            startnode = GraphId{}, endnode = GraphId{};
            closest_shape.clear();
            break;
          }
        }

        // Get shape and find closest point
        auto this_shape = edgeinfo->shape();
        auto this_closest = stop_ll.ClosestPoint(this_shape);

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
    const AABB2<PointLL>& aabb = tile->BoundingBox(tilehierarchy);
    LOG_ERROR("No closest edge found for this stop: " + stop.name() + " way Id = " +
              std::to_string(wayid) + " tile " + std::to_string(aabb.minx()) + ", " + std::to_string(aabb.miny()) + ", " +
              std::to_string(aabb.maxx()) + ", " +  std::to_string(aabb.maxy()));
    return;
  }

  LOG_DEBUG("edge found for this stop: " + stop.name() + " way Id = " +
            std::to_string(wayid));

  // Check if stop is in same tile as the start node
  uint32_t conn_count = 0;
  float length = 0.0f;
  GraphId stop_pbf_graphid = GraphId(stop.graphid());
  if (stop_pbf_graphid.Tile_Base() == startnode.Tile_Base()) {
    // Add shape from node along the edge until the closest point, then add
    // the closest point and a straight line to the stop lat,lng
    std::list<PointLL> shape;
    for (uint32_t i = 0; i <= std::get<2>(closest); i++) {
      shape.push_back(closest_shape[i]);
    }
    shape.push_back(std::get<0>(closest));
    shape.push_back(stop_ll);
    length = std::max(1.0f, valhalla::midgard::length(shape));

    // Add connection to start node
    connection_edges.push_back({startnode, stop_pbf_graphid, length, shape});
    conn_count++;
  }

  // Check if stop is in same tile as end node
  float length2 = 0.0f;
  if (stop_pbf_graphid.Tile_Base() == endnode.Tile_Base()) {
    // Add connection to end node
    if (startnode.tileid() == endnode.tileid()) {
      // Add shape from the end to closest point on edge
      std::list<PointLL> shape2;
      for (int32_t i = closest_shape.size()-1; i > std::get<2>(closest); i--) {
        shape2.push_back(closest_shape[i]);
      }
      shape2.push_back(std::get<0>(closest));
      shape2.push_back(stop_ll);
      length2 = std::max(1.0f, valhalla::midgard::length(shape2));

      // Add connection to the end node
      connection_edges.push_back({endnode, stop_pbf_graphid, length2, shape2});
      conn_count++;
    }
  }

  // Check for errors
  if (length != 0.0f && length2 != 0.0 && (length + length2) < edgelength-1) {
    LOG_ERROR("EdgeLength= " + std::to_string(edgelength) + " < connection lengths: " +
             std::to_string(length) + "," + std::to_string(length2) + " when connecting to stop "
             + stop.name());
  }
  if (conn_count == 0) {
    LOG_ERROR("Stop " + stop.name() + " has no connections to OSM! Stop TileId = " +
              std::to_string(stop_pbf_graphid.tileid()) + " Start Node Tile: " +
              std::to_string(startnode.tileid()) + " End Node Tile: " +
              std::to_string(endnode.tileid()));
  }
}

// We make sure to lock on reading and writing since tiles are now being
// written. Also lock on queue access since shared by different threads.
void build(const std::string& transit_dir,
           const boost::property_tree::ptree& pt, std::mutex& lock,
           const std::unordered_map<GraphId, size_t>& tiles,
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

    // Get transit pbf tile
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

    // Get Valhalla tile - get a read only instance for reference and
    // a writeable instance (deserialize it so we can add to it)
    lock.lock();
    const GraphTile* tile = reader.GetGraphTile(tile_id);
    GraphTileBuilder tilebuilder(hierarchy, tile_id, true);
    lock.unlock();

    // Iterate through stops and form connections to OSM network. Each
    // stop connects to 1 or 2 OSM nodes along the closest OSM way.
    // TODO - future - how to handle connections that reach nodes
    // outside the tile - may have to move this outside the tile
    // iteration...?
    // TODO - handle a list of connections/egrees points
    // TODO - what if we split the edge and insert a node?
    std::vector<OSMConnectionEdge> connection_edges;
    std::unordered_multimap<GraphId, GraphId> children;
    for (uint32_t i = 0; i < transit.stops_size(); i++) {
      const Transit_Stop& stop = transit.stops(i);

      // Form connections to the stop
      // TODO - deal with hierarchy (only connect egress locations)
      AddOSMConnection(stop, tile, hierarchy, reader, lock, connection_edges);

      // Store stop information in TransitStops
      tilebuilder.AddTransitStop( { tilebuilder.AddName(stop.onestop_id()),
                                    tilebuilder.AddName(stop.name()) } );

      /** TODO - parent/child relationships
      if (stop.type == 0 && stop.parent.Is_Valid()) {
        children.emplace(stop.parent, stop.pbf_graphid);
      }       **/
    }

    //Get all the shapes for this tile and calculate the distances
    std::unordered_map<uint32_t, Shape> shapes;
    std::vector<float> distances;
    for (uint32_t i = 0; i < transit.shapes_size(); i++) {
      const Transit_Shape& shape = transit.shapes(i);
      const std::vector<PointLL> trip_shape = decode<std::vector<PointLL> >(shape.encoded_shape());

      float distance = 0.0f;
      Shape shape_data;
      //first is always 0.0f.
      distances.push_back(distance);
      shape_data.begins = distances.size()-1;

      // loop through the points getting the distances.
      for (size_t index = 0; index < trip_shape.size() - 1; ++index) {
        PointLL p0 = trip_shape[index];
        PointLL p1 = trip_shape[index + 1];
        distance += p0.Distance(p1);
        distances.push_back(distance);
      }
      //must be distances.size for the end index as we use std::copy later on and want
      //to include the last element in the vector we wish to copy.
      shape_data.ends = distances.size();
      shape_data.shape = trip_shape;
      //shape id --> begin and end indexes in the distance vector and vector of points.
      shapes[shape.shape_id()] = shape_data;
    }

    // Sort the connection edges
    std::sort(connection_edges.begin(), connection_edges.end());

    LOG_INFO("Tile " + std::to_string(tile_id.tileid()) + ": added " +
             std::to_string(transit.stops_size()) + " stops and " +
             std::to_string(transit.shapes_size()) + " shapes and " +
             std::to_string(connection_edges.size()) + " connection edges");

    // Get all scheduled departures from the stops within this tile.
    std::map<GraphId, StopEdges> stop_edge_map;
    uint32_t unique_lineid = 1;
    std::vector<TransitDeparture> transit_departures;

    // Create a map of stop key to index in the stop vector

    // Process schedule stop pairs (departures)
    std::unordered_map<GraphId, bool> stop_access;
    std::unordered_multimap<GraphId, Departure> departures =
                ProcessStopPairs(tilebuilder, hierarchy,
                                 transit, stop_access,
                                 file, tile_id);

    // Form departures and egress/station/platform hierarchy
    for (uint32_t i = 0; i < transit.stops_size(); i++) {
      const Transit_Stop& stop = transit.stops(i);
      GraphId stop_pbf_graphid = GraphId(stop.graphid());
      StopEdges stopedges;
      stopedges.origin_pbf_graphid = stop_pbf_graphid;

      /** TODO - Identify any parent-child edge connections
      if (stop.type == 1) {
        // Station - identify any children.
        auto range = children.equal_range(stop.pbf_graphid);
        for(auto kv = range.first; kv != range.second; ++kv)
          stopedges.intrastation.push_back(kv->second);
      } else if (stop.parent != 0) {
        stopedges.intrastation.push_back(stop.parent);
      } **/

      // Find unique transit graph edges.
      std::map<std::pair<uint32_t, GraphId>, uint32_t> unique_transit_edges;
      auto range = departures.equal_range(stop_pbf_graphid);
      for(auto key = range.first; key != range.second; ++key) {
        Departure dep = key->second;

        // Identify unique route and arrival stop pairs - associate to a
        // unique line Id stored in the directed edge.
        uint32_t lineid;
        auto m = unique_transit_edges.find({dep.route, dep.dest_pbf_graphid});
        if (m == unique_transit_edges.end()) {
          // Add to the map and update the line id
          lineid = unique_lineid;
          unique_transit_edges[{dep.route, dep.dest_pbf_graphid}] = unique_lineid;
          unique_lineid++;
          stopedges.lines.emplace_back(TransitLine{lineid, dep.route,
                      dep.dest_pbf_graphid, dep.shapeid, dep.orig_dist_traveled, dep.dest_dist_traveled});
        } else {
          lineid = m->second;
        }

        // Form transit departures
        TransitDeparture td(lineid, dep.trip, dep.route,
                    dep.blockid, dep.headsign_offset, dep.dep_time,
                    dep.elapsed_time, dep.end_day, dep.dow, dep.days);

/*        LOG_DEBUG("Add departure: " + std::to_string(lineid) +
                     " dep time = " + std::to_string(td.departure_time()) +
                     " arr time = " + std::to_string(dep.arr_time));*/

        tilebuilder.AddTransitDeparture(std::move(td));
      }

      // TODO Get any transfers from this stop (no transfers currently
      // available from Transitland)
      // AddTransfers(tilebuilder);

      // Add to stop edge map - track edges that need to be added. This is
      // sorted by graph Id so the stop nodes are added in proper order
      stop_edge_map.insert({stop_pbf_graphid, stopedges});
    }

    // Add routes to the tile. Get vector of route types.
    std::vector<uint32_t> route_types = AddRoutes(transit, tilebuilder);
    LOG_INFO("Tile " + std::to_string(tile_id.tileid()) +
             ": added " + std::to_string(route_types.size()) + " routes");

    // Add nodes, directededges, and edgeinfo
    AddToGraph(tilebuilder, hierarchy, transit_dir, tiles, stop_edge_map,
               stop_access, connection_edges, shapes, distances, route_types);

    // Write the new file
    lock.lock();
    tilebuilder.StoreTileData();
    lock.unlock();
  }

  // Send back the statistics
  results.set_value({});
}

GraphId TransitToTile(const boost::property_tree::ptree& pt, const std::string& transit_tile) {
  auto tile_dir = pt.get<std::string>("mjolnir.tile_dir");
  auto transit_dir = pt.get<std::string>("mjolnir.transit_dir");
  auto graph_tile = tile_dir + transit_tile.substr(transit_dir.size());
  boost::algorithm::trim_if(graph_tile, boost::is_any_of(".pbf"));
  graph_tile += ".gph";
  TileHierarchy hierarchy(tile_dir);
  return GraphTile::GetTileId(graph_tile, hierarchy);
}

}

namespace valhalla {
namespace mjolnir {

// Add transit to the graph
void TransitBuilder::Build(const boost::property_tree::ptree& pt) {

  auto t1 = std::chrono::high_resolution_clock::now();
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
  GraphReader reader(pt.get_child("mjolnir"));
  const auto& hierarchy = reader.GetTileHierarchy();
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
          const GraphTile* tile = reader.GetGraphTile(graph_id);
          tiles.emplace(graph_id, tile->header()->nodecount());
          transit_tiles.emplace(graph_id, transit_file_itr->path().string());
        }
      }
    }
  }
  if (!transit_tiles.size()) {
    LOG_INFO("No transit tiles found. Transit will not be added.");
    return;
  }

  // TODO - intermediate pass to find any connections that cross into different
  // tile than the stop

  // Second pass - for all tiles with transit stops get all transit information
  // and populate tiles

  // A place to hold worker threads and their results
  // (Change threads to 1 if running DEBUG to get more info)
  //std::vector<std::shared_ptr<std::thread> > threads(1);
  std::vector<std::shared_ptr<std::thread> > threads(
     std::max(static_cast<uint32_t>(1),
       pt.get<uint32_t>("mjolnir.concurrency",
       std::thread::hardware_concurrency())));

  // An atomic object we can use to do the synchronization
  std::mutex lock;

  // A place to hold the results of those threads (exceptions, stats)
  std::list<std::promise<builder_stats> > results;

  // Start the threads, divvy up the work
  LOG_INFO("Adding " + std::to_string(transit_tiles.size()) + " transit tiles to the local graph...");
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
      new std::thread(build, *transit_dir, std::cref(pt.get_child("mjolnir")),
                      std::ref(lock), std::cref(tiles), tile_start, tile_end,
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

  auto t2 = std::chrono::high_resolution_clock::now();
  uint32_t secs = std::chrono::duration_cast<std::chrono::seconds>(t2 - t1).count();
  LOG_INFO("Finished - TransitBuilder took " + std::to_string(secs) + " secs");
}

}
}
