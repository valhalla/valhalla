#include <cmath>
#include <cstdint>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <unordered_set>

#include "baldr/rapidjson_utils.h"
#include <boost/algorithm/string.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/tokenizer.hpp>

#include "baldr/datetime.h"
#include "baldr/graphconstants.h"
#include "baldr/graphid.h"
#include "baldr/graphreader.h"
#include "baldr/graphtile.h"
#include "baldr/tilehierarchy.h"
#include "filesystem.h"
#include "midgard/encoded.h"
#include "midgard/logging.h"
#include "midgard/sequence.h"
#include "midgard/vector2.h"

#include "mjolnir/admin.h"
#include "mjolnir/convert_transit.h"
#include "mjolnir/graphtilebuilder.h"
#include "mjolnir/ingest_transit.h"
#include "mjolnir/servicedays.h"
#include "mjolnir/util.h"

#include "proto/transit.pb.h"

using namespace boost::property_tree;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::mjolnir;

namespace {

// Struct to hold stats information during each threads work
struct builder_stats {
  uint32_t no_dir_edge_count = 0;
  uint32_t dep_count = 0;
  uint32_t midnight_dep_count = 0;
  uint32_t invalid_service_dates = 0;
  // Accumulate stats from all threads
  void operator()(const builder_stats& other) {
    no_dir_edge_count += other.no_dir_edge_count;
    dep_count += other.dep_count;
    midnight_dep_count += other.midnight_dep_count;
    invalid_service_dates += other.invalid_service_dates;
  }
};

// Shape
struct Shape {
  uint32_t begins;
  uint32_t ends;
  std::vector<valhalla::midgard::PointLL> shape;
};

struct Departure {
  valhalla::baldr::GraphId orig_pbf_graphid; // GraphId in pbf tiles
  valhalla::baldr::GraphId dest_pbf_graphid; // GraphId in pbf tiles
  uint32_t trip;
  uint32_t route;
  uint32_t blockid;
  uint32_t shapeid;
  uint32_t headsign_offset;
  uint32_t dep_time;
  uint32_t schedule_index;
  uint32_t frequency_end_time;
  uint16_t elapsed_time;
  uint16_t frequency;
  float orig_dist_traveled;
  float dest_dist_traveled;
  bool wheelchair_accessible;
  bool bicycle_accessible;
};

// Unique route and stop
struct TransitLine {
  uint32_t lineid;
  uint32_t routeid;
  valhalla::baldr::GraphId dest_pbf_graphid; // GraphId (from pbf) of the destination stop
  uint32_t shapeid;
  float orig_dist_traveled;
  float dest_dist_traveled;
};

struct StopEdges {
  valhalla::baldr::GraphId origin_pbf_graphid;        // GraphId (from pbf) of the origin stop
  std::vector<valhalla::baldr::GraphId> intrastation; // List of intra-station connections
  std::vector<TransitLine> lines;                     // Set of unique route/stop pairs
};

// Get scheduled departures for a stop; here we also look at the .pbf.x files,
// as there can be only stop pairs in the extended ones
std::unordered_multimap<GraphId, Departure>
ProcessStopPairs(GraphTileBuilder& transit_tilebuilder,
                 const uint32_t tile_date,
                 const Transit& tile_pbf,
                 std::unordered_map<GraphId, uint16_t>& stop_no_access,
                 const std::string& pbf_fp,
                 std::mutex& lock,
                 builder_stats& stats) {
  // Check if there are no schedule stop pairs in this tile
  std::unordered_multimap<GraphId, Departure> departures;

  // Map of unique schedules (validity) in this tile
  uint32_t schedule_index = 0;
  std::map<TransitSchedule, uint32_t> schedules;

  std::size_t slash_found = pbf_fp.find_last_of("/\\");
  std::string directory = pbf_fp.substr(0, slash_found);

  filesystem::recursive_directory_iterator transit_file_itr(directory);
  filesystem::recursive_directory_iterator end_file_itr;

  // lambda to add a schedule
  auto add_schedule = [&schedule_index, &schedules,
                       &transit_tilebuilder](Departure& dep, const uint64_t days, const uint32_t dow,
                                             const uint32_t end_day) {
    TransitSchedule sched(days, dow, end_day);
    auto sched_itr = schedules.find(sched);
    if (sched_itr == schedules.end()) {
      transit_tilebuilder.AddTransitSchedule(sched);
      // Add to the map and increment the index
      schedules[sched] = schedule_index;
      dep.schedule_index = schedule_index;
      schedule_index++;
    } else {
      dep.schedule_index = sched_itr->second;
    }
  };

  // for each tile.
  for (; transit_file_itr != end_file_itr; ++transit_file_itr) {
    if (filesystem::is_regular_file(transit_file_itr->path())) {
      std::string fname = transit_file_itr->path().string();
      std::string ext = transit_file_itr->path().extension().string();
      std::string file_name = fname.substr(0, fname.size() - ext.size());

      // make sure we are looking at a pbf file
      if ((ext == ".pbf" && fname == pbf_fp) ||
          (file_name.substr(file_name.size() - 4) == ".pbf" && file_name == pbf_fp)) {

        Transit curr_tile_pbf;
        {
          // already loaded or it's with a xxx.pbf.y extension
          if (ext == ".pbf") {
            curr_tile_pbf = tile_pbf;
          } else {
            curr_tile_pbf = read_pbf(fname, lock);
          }
        }

        if (curr_tile_pbf.stop_pairs_size() == 0) {
          if (tile_pbf.nodes_size() > 0) {
            LOG_ERROR("Tile " + fname + " has 0 schedule stop pairs but has " +
                      std::to_string(tile_pbf.nodes_size()) + " stops");
          }
          departures.clear();
          return departures;
        }

        // Iterate through the stop pairs in this tile and form Valhalla departure
        // records
        for (const auto& stop_pair : curr_tile_pbf.stop_pairs()) {
          // We do not know in this step if the end node is in a valid (non-empty)
          // Valhalla tile. So just add the stop pair and we will address this later

          // Use transit PBF graph Ids internally until adding to the graph tiles
          // TODO - wheelchair accessible, shape information
          Departure dep;
          dep.orig_pbf_graphid = GraphId(stop_pair.origin_graphid());
          dep.dest_pbf_graphid = GraphId(stop_pair.destination_graphid());
          dep.route = stop_pair.route_index();
          dep.trip = stop_pair.trip_id();

          // Compute the valid days
          // set the bits based on the dow.// Compute days of week mask
          uint32_t dow_mask = kDOWNone;
          for (uint32_t x = 0; x < (uint32_t)stop_pair.service_days_of_week_size(); x++) {
            bool dow = stop_pair.service_days_of_week(x);
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

          // service_start_date are relative to our pivot date
          auto d = date::floor<date::days>(DateTime::pivot_date_);
          date::sys_days start_date = date::sys_days(date::year_month_day(
              d +
              date::floor<date::days>(date::days(stop_pair.service_start_date() / kSecondsPerDay))));
          date::sys_days end_date = date::sys_days(date::year_month_day(
              d + date::ceil<date::days>(date::days(stop_pair.service_end_date() / kSecondsPerDay))));

          uint64_t days = get_service_days(start_date, end_date, tile_date, dow_mask);

          // if this is a service addition for one day, delete the dow_mask.
          // TODO(nils): why? it's still valid for this one day right? So rather find out that dow?
          if (stop_pair.service_start_date() == stop_pair.service_end_date()) {
            dow_mask = kDOWNone;
          }

          date::sys_days t_d = date::sys_days(date::year_month_day(d + date::days(tile_date)));
          uint32_t end_day = static_cast<uint32_t>((end_date - t_d).count());

          if (end_day > kScheduleEndDay) {
            end_day = kScheduleEndDay;
          }

          // if subtractions are between start and end date then turn off bit.
          for (const auto& x : stop_pair.service_except_dates()) {
            date::sys_days rm_date =
                date::sys_days(date::year_month_day(d + date::days(x / kSecondsPerDay)));
            days = remove_service_day(days, end_date, tile_date, rm_date);
          }

          // if additions are between start and end date then turn on bit.
          for (const auto& x : stop_pair.service_added_dates()) {
            date::sys_days add_date =
                date::sys_days(date::year_month_day(d + date::days(x / kSecondsPerDay)));
            days = add_service_day(days, end_date, tile_date, add_date);
          }

          // skip if no days are valid; this can happen a lot when there's a calendar.txt entry with
          // no valid days but some calendar_dates.txt entries which add a date lying in the past
          if (!days && !dow_mask) {
            stats.invalid_service_dates++;
            continue;
          }

          // if we have shape data then set everything, else shapeid = 0;
          if (stop_pair.has_shape_id() && stop_pair.has_destination_dist_traveled() &&
              stop_pair.has_origin_dist_traveled()) {
            dep.shapeid = stop_pair.shape_id();
            dep.orig_dist_traveled = stop_pair.origin_dist_traveled();
            dep.dest_dist_traveled = stop_pair.destination_dist_traveled();
          } else {
            dep.shapeid = 0;
          }

          dep.blockid = stop_pair.has_block_id() ? stop_pair.block_id() : 0;
          dep.dep_time = stop_pair.origin_departure_time();
          dep.elapsed_time = stop_pair.destination_arrival_time() - dep.dep_time;
          dep.headsign_offset = transit_tilebuilder.AddName(stop_pair.trip_headsign());

          dep.frequency_end_time =
              stop_pair.has_frequency_end_time() ? stop_pair.frequency_end_time() : 0;
          dep.frequency =
              stop_pair.has_frequency_headway_seconds() ? stop_pair.frequency_headway_seconds() : 0;

          if (!stop_pair.bikes_allowed()) {
            stop_no_access[dep.orig_pbf_graphid] |= kBicycleAccess;
            stop_no_access[dep.dest_pbf_graphid] |= kBicycleAccess;
          }

          if (!stop_pair.wheelchair_accessible()) {
            stop_no_access[dep.orig_pbf_graphid] |= kWheelchairAccess;
            stop_no_access[dep.dest_pbf_graphid] |= kWheelchairAccess;
          }

          dep.bicycle_accessible = stop_pair.bikes_allowed();
          dep.wheelchair_accessible = stop_pair.wheelchair_accessible();

          add_schedule(dep, days, dow_mask, end_day);

          // is this past midnight?
          // create a departure for before midnight and one after
          uint32_t origin_seconds = stop_pair.origin_departure_time();
          if (origin_seconds >= kSecondsPerDay) {

            // Add the current dep to the departures list
            // and then update it with new dep time.  This
            // dep will be used when the start time is after
            // midnight.
            stats.midnight_dep_count++;
            departures.emplace(dep.orig_pbf_graphid, dep);
            while (origin_seconds >= kSecondsPerDay) {
              origin_seconds -= kSecondsPerDay;
              // Then we need to fix the dow mask and dates
              // The departure that was initially for every Friday   26h
              // needs to be for                      every Saturday 02h
              // If there was an exception on the Friday 11th of January,
              // then we need an exception on the Saturday 12th of January instead
              days = shift_service_day(days);
              dow_mask =
                  ((dow_mask << 1) & kAllDaysOfWeek) | (dow_mask & kSaturday ? kSunday : kDOWNone);

              add_schedule(dep, days, dow_mask, end_day);
            }

            dep.dep_time = origin_seconds;
            // if there used to be a frequency, there'll be one now
            if (dep.frequency_end_time && dep.frequency) {
              uint32_t frequency_end_time = stop_pair.frequency_end_time();
              // adjust the end time if it is after midnight.
              while (frequency_end_time >= kSecondsPerDay) {
                frequency_end_time -= kSecondsPerDay;
              }

              dep.frequency_end_time = frequency_end_time;
              dep.frequency = stop_pair.frequency_headway_seconds();
            }
          }
          // Add to the departures list
          departures.emplace(dep.orig_pbf_graphid, std::move(dep));
          stats.dep_count++;
        }
      }
    }
  }
  return departures;
}

// Add routes to the tile. Return a vector of route types.
std::vector<uint32_t> AddRoutes(const Transit& pbf_tile, GraphTileBuilder& tilebuilder) {
  // Route types vs. index
  std::vector<uint32_t> route_types;

  for (uint32_t i = 0; i < (uint32_t)pbf_tile.routes_size(); i++) {
    const Transit_Route& r = pbf_tile.routes(i);

    // These should all be correctly set in the fetcher as it tosses types that we
    // don't support.  However, let's report an error if we encounter one.
    TransitType route_type = static_cast<TransitType>(r.vehicle_type());
    switch (route_type) {
      case TransitType::kTram:      // Tram, streetcar, lightrail
      case TransitType::kMetro:     // Subway, metro
      case TransitType::kRail:      // Rail
      case TransitType::kBus:       // Bus
      case TransitType::kFerry:     // Ferry
      case TransitType::kCableCar:  // Cable car
      case TransitType::kGondola:   // Gondola (suspended cable car)
      case TransitType::kFunicular: // Funicular (steep incline)
        break;
      default:
        // Log an unsupported vehicle type, set to bus for now
        LOG_ERROR("Unsupported vehicle type!");
        route_type = TransitType::kBus;
        break;
    }

    TransitRoute route(route_type, tilebuilder.AddName(r.onestop_id()),
                       tilebuilder.AddName(r.operated_by_onestop_id()),
                       tilebuilder.AddName(r.operated_by_name()),
                       tilebuilder.AddName(r.operated_by_website()), r.route_color(),
                       r.route_text_color(), tilebuilder.AddName(r.name()),
                       tilebuilder.AddName(r.route_long_name()), tilebuilder.AddName(r.route_desc()));
    LOG_DEBUG("Route idx = " + std::to_string(i) + ": " + r.name() + "," + r.route_long_name());
    tilebuilder.AddTransitRoute(route);

    // Route type - need this to store in edge.
    route_types.push_back(r.vehicle_type());
  }
  return route_types;
}

// Get Use given the transit route type
// TODO - add separate Use for different types - when we do this change
// the directed edge IsTransit method
Use GetTransitUse(const uint32_t rt) {
  switch (static_cast<TransitType>(rt)) {
    default:
    case TransitType::kTram:      // Tram, streetcar, lightrail
    case TransitType::kMetro:     // Subway, metro
    case TransitType::kRail:      // Rail
    case TransitType::kCableCar:  // Cable car
    case TransitType::kGondola:   // Gondola (suspended cable car)
    case TransitType::kFunicular: // Funicular (steep incline)
      return Use::kRail;
    case TransitType::kBus: // Bus
      return Use::kBus;
    case TransitType::kFerry: // Ferry (boat)
      return Use::kRail;      // TODO - add ferry use
  }
}

std::list<PointLL> GetShape(const PointLL& stop_ll,
                            const PointLL& endstop_ll,
                            uint32_t shapeid,
                            const float orig_dist_traveled,
                            const float dest_dist_traveled,
                            const std::vector<PointLL>& trip_shape,
                            const std::vector<float>& distances) {

  std::list<PointLL> shape;
  if (shapeid != 0 && trip_shape.size() && stop_ll != endstop_ll &&
      orig_dist_traveled < dest_dist_traveled) {

    float distance = 0.0f, d_from_p0_to_x = 0.0f;

    // point x - we are trying to find it on the line segment between p0 and p1
    PointLL x;
    // find out where orig_dist_traveled should be in the list.
    auto lower_bound = std::lower_bound(distances.cbegin(), distances.cend(), orig_dist_traveled);
    // find out where dest_dist_traveled should be in the list.
    auto upper_bound = std::upper_bound(distances.cbegin(), distances.cend(), dest_dist_traveled);
    float prev_distance = *(lower_bound);

    // distance calculations can be off just a bit (i.e., 9372.224609 < 9372.500000) so set it to
    // the last element.
    if (distances.back() < dest_dist_traveled) {
      upper_bound = distances.cend() - 1;
    }

    // lower_bound returns an iterator pointing to the first element which does not compare less
    // than the dist_traveled; therefore, we need to back up one if it does not equal the
    // lower_bound value.  For example, we could be starting at the beginning of the points list
    if (orig_dist_traveled != (*lower_bound)) {
      prev_distance = *(--lower_bound);
    }

    // loop through the points.
    for (auto itr = lower_bound; itr != upper_bound; ++itr) {

      /*    |
       *    |
       *    p0
       *    | }--d_from_p0_to_x (distance from p0 to x)
       *    x -- point we are trying to find on the segment (orig_dist_traveled or
       * dest_dist_traveled on this segment)
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
      distance = *(itr + 1);

      // find point x using the orig_dist_traveled - this is our first point added to shape
      if (itr == lower_bound) {
        if (orig_dist_traveled == *itr) { // just add p0
          shape.push_back(p0);
        } else {
          // distance from p0 to x using the orig_dist_traveled
          d_from_p0_to_x = (orig_dist_traveled - prev_distance) / (distance - prev_distance);
          x = p0 + (p1 - p0) * d_from_p0_to_x;
          shape.push_back(x);
        }
      }

      // find point x using the dest_dist_traveled - this is our last point added to the shape
      if ((itr + 1) == upper_bound) {
        if (dest_dist_traveled == *itr) { // just add p0
          if (shape.back() != p0) {       // avoid dups
            shape.push_back(p0);
          }
        } else {
          // distance from p0 to x using the dest_dist_traveled
          d_from_p0_to_x = (dest_dist_traveled - prev_distance) / (distance - prev_distance);
          x = p0 + (p1 - p0) * d_from_p0_to_x;

          if (shape.back() != x) { // avoid dups
            shape.push_back(x);
          }
          // we are done p1 is too far away
        }
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

  if (shape.size() == 0) {
    LOG_ERROR("Invalid shape between " + stop_ll.to_string() + " and " + endstop_ll.to_string());
    shape.push_back(stop_ll);
    shape.push_back(endstop_ll);
  }

  return shape;
}

// TODO: we cannot leave this function like this! the code in its current state is completely
//  illegible due to copy pasting. there are multiple nested shadowings of variables which encourages
//  mistakes. at the very least we should break out the 3 different types of connections into separate
//  functions. it would be great if it could be a single function with different arguments to get all
//  3 done but if not at least separating them will remove the shadowing. we cannot call transit done
//  until this is rectified
void AddToGraph(GraphTileBuilder& tilebuilder_transit,
                const GraphId& tileid,
                const Transit& tile_pbf,
                const std::string& transit_dir,
                std::mutex& lock,
                const std::map<GraphId, StopEdges>& stop_edge_map,
                const std::unordered_map<GraphId, uint16_t>& stop_no_access,
                const std::unordered_map<uint32_t, Shape>& shape_data,
                const std::vector<float>& distances,
                const std::vector<uint32_t>& route_types,
                bool tile_within_one_tz,
                const std::multimap<uint32_t, multi_polygon_type>& tz_polys,
                uint32_t& no_dir_edge_count) {
  auto t1 = std::chrono::high_resolution_clock::now();

  std::set<uint64_t> added_stations;
  std::set<uint64_t> added_egress;

  // Data looks like the following.stop_index(
  // Egress1_for_Station_A
  // Egress2_for_Station_A
  // Station_A
  // Platform1_for_Station_A
  // Platform2_for_Station_A
  // Egress_for_Station_B
  // Station_B
  // Platform_for_Station_B
  // . . . and so on

  //  tiles will look like the following with N egresses and N platforms.
  //  osm--------->egress--------->station--------->platform
  //  node<---------node<-----------node<-------------node

  // osm and egress nodes are connected by transitconnections.
  // egress and stations are connected by egressconnections.
  // stations and platforms are connected by platformconnections

  // Iterate through the platform and their edges
  uint32_t transitedges = 0;
  for (const auto& stop_edges : stop_edge_map) {
    // Get the platform information
    GraphId platform_graphid = stop_edges.second.origin_pbf_graphid;
    const Transit_Node& platform = tile_pbf.nodes(platform_graphid.id());
    if (GraphId(platform.graphid()) != platform_graphid) {
      LOG_ERROR("Platform key not equal!");
    }

    LOG_DEBUG("Transit Platform: " + platform.name() +
              " index= " + std::to_string(platform_graphid.id()));
    PointLL platform_ll = {platform.lon(), platform.lat()};

    // the prev_type_graphid is actually the station or parent in
    // platforms
    GraphId parent(platform.prev_type_graphid());
    const Transit_Node& station = tile_pbf.nodes(parent.id());

    // Get the Valhalla graphId of the station node
    GraphId station_graphid(station.graphid());

    PointLL station_ll = {station.lon(), station.lat()};
    // Build the station node if it has not already been added.
    if (added_stations.find(platform.prev_type_graphid()) == added_stations.end()) {

      // Build the station node
      uint32_t n_access = (kPedestrianAccess | kWheelchairAccess | kBicycleAccess);
      auto s_access = stop_no_access.find(station_graphid);
      if (s_access != stop_no_access.end()) {
        n_access &= ~s_access->second;
      }

      // Set the station lat,lon using the tile base LL
      PointLL base_ll = tilebuilder_transit.header_builder().base_ll();
      NodeInfo station_node(base_ll, station_ll, n_access, NodeType::kTransitStation, false, true,
                            false, false);
      station_node.set_stop_index(station_graphid.id());

      const std::string& tz = station.has_timezone() ? station.timezone() : "";
      uint32_t timezone = 0;
      if (!tz.empty()) {
        timezone = DateTime::get_tz_db().to_index(tz);
      }

      if (timezone == 0) {
        // fallback to tz database.
        timezone =
            (tile_within_one_tz) ? tz_polys.begin()->first : GetMultiPolyId(tz_polys, station_ll);

        if (timezone == 0) {
          LOG_WARN("Timezone not found for station " + station.name());
        }
      }
      station_node.set_timezone(timezone);

      LOG_DEBUG("Transit Platform: " + platform.name() +
                " index= " + std::to_string(platform_graphid.id()));

      // set the index to the first egress.
      // loop over egresses add the DE to the station from the egress
      // there is always at least one egress and they are before the stations in the pbf
      GraphId eg = GraphId(station.prev_type_graphid());
      uint32_t index = eg.id();

      while (true) {
        const Transit_Node& egress = tile_pbf.nodes(index);
        if (static_cast<NodeType>(egress.type()) != NodeType::kTransitEgress) {
          break;
        }

        // Get the Valhalla graphId of the origin node (transit stop)
        GraphId egress_graphid(egress.graphid());
        DirectedEdge directededge;
        directededge.set_endnode(station_graphid);
        PointLL egress_ll = {egress.lon(), egress.lat()};

        // Build the egress node
        uint32_t n_access = (kPedestrianAccess | kWheelchairAccess | kBicycleAccess);
        auto s_access = stop_no_access.find(egress_graphid);
        if (s_access != stop_no_access.end()) {
          n_access &= ~s_access->second;
        }

        const std::string& tz = egress.has_timezone() ? egress.timezone() : "";
        uint32_t timezone = 0;
        if (!tz.empty()) {
          timezone = DateTime::get_tz_db().to_index(tz);
        }

        if (timezone == 0) {
          // fallback to tz database.
          timezone =
              (tile_within_one_tz) ? tz_polys.begin()->first : GetMultiPolyId(tz_polys, egress_ll);
          if (timezone == 0) {
            LOG_WARN("Timezone not found for egress " + egress.name());
          }
        }

        // Set the egress lat,lon using the tile base LL
        PointLL base_ll = tilebuilder_transit.header_builder().base_ll();
        NodeInfo egress_node(base_ll, egress_ll, n_access, NodeType::kTransitEgress, false, true,
                             false, false);
        egress_node.set_stop_index(index);
        egress_node.set_timezone(timezone);
        egress_node.set_edge_index(tilebuilder_transit.directededges().size());
        if (egress.has_osm_connecting_lat() && egress.has_osm_connecting_lon()) {
          egress_node.set_connecting_point(
              PointLL(egress.osm_connecting_lon(), egress.osm_connecting_lat()));
        } else if (egress.has_osm_connecting_way_id()) {
          egress_node.set_connecting_wayid(egress.osm_connecting_way_id());
        }

        // add the egress connection
        // Make sure length is non-zero
        double length = std::max(1.0, egress_ll.Distance(station_ll));
        directededge.set_length(length);
        directededge.set_use(Use::kEgressConnection);
        directededge.set_speed(5);
        directededge.set_classification(RoadClass::kServiceOther);
        directededge.set_localedgeidx(tilebuilder_transit.directededges().size() -
                                      egress_node.edge_index());
        directededge.set_forwardaccess((kPedestrianAccess | kWheelchairAccess | kBicycleAccess));
        directededge.set_reverseaccess((kPedestrianAccess | kWheelchairAccess | kBicycleAccess));
        directededge.set_named(false);

        // Add edge info to the tile and set the offset in the directed edge
        bool added = false;
        std::vector<std::string> names, tagged_values, linguistics;

        std::list<PointLL> shape = {egress_ll, station_ll};

        uint32_t edge_info_offset =
            tilebuilder_transit.AddEdgeInfo(0, egress_graphid, station_graphid, 0, 0, 0, 0, shape,
                                            names, tagged_values, linguistics, 0, added);
        directededge.set_edgeinfo_offset(edge_info_offset);
        directededge.set_forward(true);

        // Add to list of directed edges
        tilebuilder_transit.directededges().emplace_back(std::move(directededge));

        // set the count to 1 DE
        // osm connections will be added later.
        egress_node.set_edge_count(1);
        // Add the egress node
        tilebuilder_transit.nodes().emplace_back(std::move(egress_node));
        index++;
      }

      station_node.set_edge_index(tilebuilder_transit.directededges().size());
      // now add the DE to the egress from the station.
      // index now points to the station.
      for (uint32_t j = eg.id(); j < index; j++) {

        const Transit_Node& egress = tile_pbf.nodes(j);
        PointLL egress_ll = {egress.lon(), egress.lat()};

        // Get the Valhalla graphId of the origin node (transit stop)
        GraphId egress_graphid(egress.graphid());
        DirectedEdge directededge;
        directededge.set_endnode(egress_graphid);

        // add the platform connection
        // Make sure length is non-zero
        double length = std::max(1.0, station_ll.Distance(egress_ll));
        directededge.set_length(length);
        directededge.set_use(Use::kEgressConnection);
        directededge.set_speed(5);
        directededge.set_classification(RoadClass::kServiceOther);
        directededge.set_localedgeidx(tilebuilder_transit.directededges().size() -
                                      station_node.edge_index());
        directededge.set_forwardaccess((kPedestrianAccess | kWheelchairAccess | kBicycleAccess));
        directededge.set_reverseaccess((kPedestrianAccess | kWheelchairAccess | kBicycleAccess));
        directededge.set_named(false);
        // Add edge info to the tile and set the offset in the directed edge
        bool added = false;
        std::vector<std::string> names, tagged_values, linguistics;
        std::list<PointLL> shape = {station_ll, egress_ll};

        // TODO - these need to be valhalla graph Ids
        uint32_t edge_info_offset =
            tilebuilder_transit.AddEdgeInfo(0, station_graphid, egress_graphid, 0, 0, 0, 0, shape,
                                            names, tagged_values, linguistics, 0, added);
        directededge.set_edgeinfo_offset(edge_info_offset);
        directededge.set_forward(false);

        // Add to list of directed edges
        tilebuilder_transit.directededges().emplace_back(std::move(directededge));
      }

      // advance the index to skip the station and point to the first platform
      index++;
      while (true) {

        if (index == (uint32_t)tile_pbf.nodes_size()) {
          break;
        }

        const Transit_Node& platform = tile_pbf.nodes(index);
        if (static_cast<NodeType>(platform.type()) != NodeType::kMultiUseTransitPlatform) {
          break;
        }

        // Get the Valhalla graphId of the origin node (transit stop)
        GraphId platform_graphid(platform.graphid());
        DirectedEdge directededge;
        directededge.set_endnode(platform_graphid);
        PointLL platform_ll = {platform.lon(), platform.lat()};

        // add the platform connection
        // Make sure length is non-zero
        double length = std::max(1.0, station_ll.Distance(platform_ll));
        directededge.set_length(length);
        directededge.set_use(Use::kPlatformConnection);
        directededge.set_speed(5);
        directededge.set_classification(RoadClass::kServiceOther);
        directededge.set_localedgeidx(tilebuilder_transit.directededges().size() -
                                      station_node.edge_index());
        directededge.set_forwardaccess((kPedestrianAccess | kWheelchairAccess | kBicycleAccess));
        directededge.set_reverseaccess((kPedestrianAccess | kWheelchairAccess | kBicycleAccess));
        directededge.set_named(false);

        // Add edge info to the tile and set the offset in the directed edge
        bool added = false;
        std::vector<std::string> names, tagged_values, linguistics;
        std::list<PointLL> shape = {station_ll, platform_ll};

        // TODO - these need to be valhalla graph Ids
        uint32_t edge_info_offset =
            tilebuilder_transit.AddEdgeInfo(0, station_graphid, platform_graphid, 0, 0, 0, 0, shape,
                                            names, tagged_values, linguistics, 0, added);
        directededge.set_edgeinfo_offset(edge_info_offset);
        directededge.set_forward(true);

        // Add to list of directed edges
        tilebuilder_transit.directededges().emplace_back(std::move(directededge));
        index++;
      }

      // Get the directed edge count, log an error if no directed edges are added
      uint32_t edge_count = tilebuilder_transit.directededges().size() - station_node.edge_index();
      if (edge_count == 0) {
        // Set the edge index to 0
        // TODO: add ERROR log for no directed edges out of the station
        station_node.set_edge_index(0);
        no_dir_edge_count++;
      }

      // Add the node
      station_node.set_edge_count(edge_count);
      tilebuilder_transit.nodes().emplace_back(std::move(station_node));
      added_stations.emplace(platform.prev_type_graphid());
    }

    // Build the platform node
    uint32_t n_access = (kPedestrianAccess | kWheelchairAccess | kBicycleAccess);
    auto s_access = stop_no_access.find(platform_graphid);
    if (s_access != stop_no_access.end()) {
      n_access &= ~s_access->second;
    }

    const std::string& tz = platform.has_timezone() ? platform.timezone() : "";
    uint32_t timezone = 0;
    if (!tz.empty()) {
      timezone = DateTime::get_tz_db().to_index(tz);
    }

    if (timezone == 0) {
      // fallback to tz database.
      timezone =
          (tile_within_one_tz) ? tz_polys.begin()->first : GetMultiPolyId(tz_polys, platform_ll);
      if (timezone == 0) {
        LOG_WARN("Timezone not found for platform " + platform.name());
      }
    }

    // Set the platform lat,lon using the tile base LL
    PointLL base_ll = tilebuilder_transit.header_builder().base_ll();
    NodeInfo platform_node(base_ll, platform_ll, n_access, NodeType::kMultiUseTransitPlatform, false,
                           true, false, false);
    platform_node.set_mode_change(true);
    platform_node.set_stop_index(platform_graphid.id());
    platform_node.set_timezone(timezone);
    platform_node.set_edge_index(tilebuilder_transit.directededges().size());

    // Add DE to the station from the platform
    DirectedEdge directededge;
    directededge.set_endnode(station_graphid);

    // add the platform connection
    // Make sure length is non-zero
    double length = std::max(1.0, platform_ll.Distance(station_ll));
    directededge.set_length(length);
    directededge.set_use(Use::kPlatformConnection);
    directededge.set_speed(5);
    directededge.set_classification(RoadClass::kServiceOther);
    directededge.set_localedgeidx(tilebuilder_transit.directededges().size() -
                                  platform_node.edge_index());
    directededge.set_forwardaccess((kPedestrianAccess | kWheelchairAccess | kBicycleAccess));
    directededge.set_reverseaccess((kPedestrianAccess | kWheelchairAccess | kBicycleAccess));
    directededge.set_named(false);
    // Add edge info to the tile and set the offset in the directed edge
    bool added = false;
    std::vector<std::string> names, tagged_values, linguistics;
    std::list<PointLL> shape = {platform_ll, station_ll};

    // TODO - these need to be valhalla graph Ids
    uint32_t edge_info_offset =
        tilebuilder_transit.AddEdgeInfo(0, platform_graphid, station_graphid, 0, 0, 0, 0, shape,
                                        names, tagged_values, linguistics, 0, added);

    directededge.set_edgeinfo_offset(edge_info_offset);
    directededge.set_forward(false);

    // Add to list of directed edges
    tilebuilder_transit.directededges().emplace_back(std::move(directededge));

    // Add transit lines
    // level 3
    for (const auto& transitedge : stop_edges.second.lines) {
      // Get the end node. Skip this directed edge if the Valhalla tile is
      // not valid (or empty)
      GraphId end_platform_graphid(transitedge.dest_pbf_graphid);
      if (!end_platform_graphid.Is_Valid()) {
        LOG_ERROR("Unstitched stop pair detected with origin near " +
                  std::to_string(platform_ll.lat()) + ',' + std::to_string(platform_ll.lng()));
        continue;
      }

      // Find the lat,lng of the end stop
      PointLL endll;
      std::string endstopname;
      if (end_platform_graphid.Tile_Base() == tileid) {
        // End stop is in the same pbf transit tile
        const Transit_Node& endplatform = tile_pbf.nodes(end_platform_graphid.id());
        endstopname = endplatform.name();
        endll = {endplatform.lon(), endplatform.lat()};
      } else {
        // Get Transit PBF data for this tile
        // Get transit pbf tile
        std::string file_name = GraphTile::FileSuffix(
            GraphId(end_platform_graphid.tileid(), end_platform_graphid.level(), 0));
        boost::algorithm::trim_if(file_name, boost::is_any_of(".gph"));
        file_name += ".pbf";
        const std::string file = transit_dir + filesystem::path::preferred_separator + file_name;
        Transit endtransit = read_pbf(file, lock);
        const Transit_Node& endplatform = endtransit.nodes(end_platform_graphid.id());
        endstopname = endplatform.name();
        endll = {endplatform.lon(), endplatform.lat()};
      }

      // Add the directed edge
      DirectedEdge directededge;
      directededge.set_endnode(end_platform_graphid);
      directededge.set_length(platform_ll.Distance(endll));
      Use use = GetTransitUse(route_types[transitedge.routeid]);
      directededge.set_use(use);
      directededge.set_speed(5);
      directededge.set_classification(RoadClass::kServiceOther);
      directededge.set_localedgeidx(tilebuilder_transit.directededges().size() -
                                    platform_node.edge_index());
      directededge.set_forwardaccess((kPedestrianAccess | kWheelchairAccess | kBicycleAccess));
      directededge.set_reverseaccess((kPedestrianAccess | kWheelchairAccess | kBicycleAccess));
      directededge.set_lineid(transitedge.lineid);

      LOG_DEBUG("Add transit directededge - lineId = " + std::to_string(transitedge.lineid) +
                " Route Key = " + std::to_string(transitedge.routeid) + " EndStop " + endstopname);

      // Add edge info to the tile and set the offset in the directed edge
      // Leave the name empty. Use the trip Id to look up the route Id and
      // route within TripLegBuilder.
      bool added = false;
      std::vector<std::string> names, tagged_values, linguistics;

      std::vector<PointLL> points;
      std::vector<float> distance;
      // get the indexes and vector of points for this shape id
      const auto& found = shape_data.find(transitedge.shapeid);
      if (transitedge.shapeid != 0 && found != shape_data.cend()) {
        const auto& shape_d = found->second;
        points = shape_d.shape;
        // copy only the distances that we care about.
        std::copy((distances.cbegin() + shape_d.begins), (distances.cbegin() + shape_d.ends),
                  back_inserter(distance));
      } else if (transitedge.shapeid != 0) {
        LOG_WARN("Shape Id not found: " + std::to_string(transitedge.shapeid));
      }

      // TODO - if we separate transit edges based on more than just routeindex
      // we will need to do something to differentiate edges (maybe use
      // lineid) so the shape doesn't get messed up.
      auto shape = GetShape(platform_ll, endll, transitedge.shapeid, transitedge.orig_dist_traveled,
                            transitedge.dest_dist_traveled, points, distance);

      uint32_t edge_info_offset =
          tilebuilder_transit.AddEdgeInfo(transitedge.routeid, platform_graphid, end_platform_graphid,
                                          0, 0, 0, 0, shape, names, tagged_values, linguistics, 0,
                                          added);

      directededge.set_edgeinfo_offset(edge_info_offset);
      directededge.set_forward(added);

      // Add to list of directed edges
      tilebuilder_transit.directededges().emplace_back(std::move(directededge));
      transitedges++;
    }

    // Get the directed edge count, log an error if no directed edges are added
    // TODO: log error
    uint32_t edge_count = tilebuilder_transit.directededges().size() - platform_node.edge_index();
    if (edge_count == 0) {
      // Set the edge index to 0
      platform_node.set_edge_index(0);
      no_dir_edge_count++;
    }

    // Add the node
    platform_node.set_edge_count(edge_count);
    tilebuilder_transit.nodes().emplace_back(std::move(platform_node));
  }

  // Log the number of added nodes and edges
  auto t2 = std::chrono::high_resolution_clock::now();
  [[maybe_unused]] uint32_t msecs =
      std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
  LOG_INFO("Tile " + std::to_string(tileid.tileid()) + ": added " + std::to_string(transitedges) +
           " transit edges, and " + std::to_string(tilebuilder_transit.nodes().size()) +
           " nodes. time = " + std::to_string(msecs) + " ms");
}

// We make sure to lock on reading and writing since tiles are now being
// written. Also lock on queue access since shared by different threads.
void build_tiles(const boost::property_tree::ptree& pt,
                 std::mutex& lock,
                 std::unordered_set<GraphId>::const_iterator tile_start,
                 std::unordered_set<GraphId>::const_iterator tile_end,
                 std::promise<builder_stats>& results) {

  builder_stats stats;

  GraphReader reader(pt);
  auto database = pt.get_optional<std::string>("timezone");
  // Initialize the tz DB (if it exists)
  sqlite3* tz_db_handle = GetDBHandle(*database);
  if (!tz_db_handle) {
    LOG_WARN("Time zone db " + *database + " not found.  Not saving time zone information from db.");
  }
  auto tz_conn = make_spatialite_cache(tz_db_handle);

  const auto& tiles = TileHierarchy::levels().back().tiles;
  // Iterate through the tiles in the queue and find any that include stops
  for (; tile_start != tile_end; ++tile_start) {
    // Get the next tile Id from the queue and get a tile builder
    if (reader.OverCommitted()) {
      reader.Trim();
    }
    GraphId tile_id = tile_start->Tile_Base();

    // Get transit pbf tile
    const std::string transit_dir = pt.get<std::string>("transit_dir");
    std::string file_name = GraphTile::FileSuffix(GraphId(tile_id.tileid(), tile_id.level(), 0));
    boost::algorithm::trim_if(file_name, boost::is_any_of(".gph"));
    file_name += ".pbf";
    const std::string pbf_fp = transit_dir + filesystem::path::preferred_separator + file_name;

    // Make sure it exists
    if (!filesystem::exists(pbf_fp)) {
      LOG_ERROR("File not found.  " + pbf_fp);
      return;
    }

    Transit tile_pbf = read_pbf(pbf_fp, lock);
    // Get Valhalla tile - get a read only instance for reference and
    // a writeable instance (deserialize it so we can add to it)
    lock.lock();

    GraphId transit_tile_id = GraphId(tile_id.tileid(), tile_id.level(), tile_id.id());
    graph_tile_ptr transit_tile = reader.GetGraphTile(transit_tile_id);
    GraphTileBuilder tilebuilder_transit(transit_dir, transit_tile_id, false);

    // normalize tile creation date on America/New_York timezone
    auto tz = DateTime::get_tz_db().from_index(DateTime::get_tz_db().to_index("America/New_York"));
    uint32_t tile_creation_date =
        DateTime::days_from_pivot_date(DateTime::get_formatted_date(DateTime::iso_date_time(tz)));
    tilebuilder_transit.AddTileCreationDate(tile_creation_date);

    // Set the tile base LL
    PointLL base_ll = TileHierarchy::GetTransitLevel().tiles.Base(tile_id.tileid());
    tilebuilder_transit.header_builder().set_base_ll(base_ll);

    lock.unlock();

    std::unordered_map<GraphId, uint16_t> stop_no_access;
    // add Transit nodes in order.
    for (const auto& transit_node : tile_pbf.nodes()) {

      if (!transit_node.wheelchair_boarding()) {
        stop_no_access[GraphId(transit_node.graphid())] |= kWheelchairAccess;
      }

      // Store stop information in TransitStops
      tilebuilder_transit.AddTransitStop({tilebuilder_transit.AddName(transit_node.onestop_id()),
                                          tilebuilder_transit.AddName(transit_node.name()),
                                          transit_node.generated(), transit_node.traversability()});
    }

    // Get all the shapes for this tile and calculate the distances
    std::unordered_map<uint32_t, Shape> shapes;
    std::vector<float> distances;
    for (const auto& shape : tile_pbf.shapes()) {
      const std::vector<PointLL> trip_shape = decode7<std::vector<PointLL>>(shape.encoded_shape());

      float distance = 0.0f;
      Shape shape_data;
      // first is always 0.0f.
      distances.push_back(distance);
      shape_data.begins = distances.size() - 1;

      // loop through the points getting the distances.
      for (size_t index = 0; index < trip_shape.size() - 1; ++index) {
        PointLL p0 = trip_shape[index];
        PointLL p1 = trip_shape[index + 1];
        distance += p0.Distance(p1);
        distances.push_back(distance);
      }
      // must be distances.size for the end index as we use std::copy later on and want
      // to include the last element in the vector we wish to copy.
      shape_data.ends = distances.size();
      shape_data.shape = trip_shape;
      // shape id --> begin and end indexes in the distance vector and vector of points.
      shapes[shape.shape_id()] = shape_data;
    }

    // Get all scheduled departures from the stops within this tile.
    std::map<GraphId, StopEdges> stop_edge_map;
    uint32_t unique_lineid = 1;
    std::vector<TransitDeparture> transit_departures;

    // Process schedule stop pairs (departures)
    std::unordered_multimap<GraphId, Departure> departures =
        ProcessStopPairs(tilebuilder_transit, tile_creation_date, tile_pbf, stop_no_access, pbf_fp,
                         lock, stats);

    // Form departures and egress/station/platform hierarchy
    // TODO: get pathways.txt or some other means to better correlate the intra-station edges
    for (const auto& platform : tile_pbf.nodes()) {
      if (static_cast<NodeType>(platform.type()) != NodeType::kMultiUseTransitPlatform) {
        continue;
      }

      GraphId platform_pbf_graphid = GraphId(platform.graphid());
      StopEdges stopedges;
      stopedges.origin_pbf_graphid = platform_pbf_graphid;

      // TODO - perhaps replace this code with use of headsign below
      // to solve problem of a trip that doesn't go the whole way to
      // the end of the route line
      std::map<std::pair<uint32_t, GraphId>, uint32_t> unique_transit_edges;
      auto range = departures.equal_range(platform_pbf_graphid);

      for (auto key = range.first; key != range.second; ++key) {
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
          stopedges.lines.emplace_back(TransitLine{lineid, dep.route, dep.dest_pbf_graphid,
                                                   dep.shapeid, dep.orig_dist_traveled,
                                                   dep.dest_dist_traveled});
        } else {
          lineid = m->second;
        }

        // we prefer the frequency-based schedule if it was set
        try {
          if (dep.frequency == 0) {
            // Form transit departures -- fixed departure time
            TransitDeparture td(lineid, dep.trip, dep.route, dep.blockid, dep.headsign_offset,
                                dep.dep_time, dep.elapsed_time, dep.schedule_index,
                                dep.wheelchair_accessible, dep.bicycle_accessible);
            tilebuilder_transit.AddTransitDeparture(std::move(td));
          } else {

            // Form transit departures -- frequency departure time
            // TODO(nils): this doesn't differentiate between frequencies.txt entries with
            // exact_times true/false; it's a bit random right now what departure_time represents:
            // it's the time set by the stop_time's departure_time which might just be an example,
            // but not represent an actual departure_time. can't we just take the service's start_time
            // if it's exact_times=true or start_time + 0.5 * headway for exact_times=false?
            TransitDeparture td(lineid, dep.trip, dep.route, dep.blockid, dep.headsign_offset,
                                dep.dep_time, dep.frequency_end_time, dep.frequency, dep.elapsed_time,
                                dep.schedule_index, dep.wheelchair_accessible,
                                dep.bicycle_accessible);
            tilebuilder_transit.AddTransitDeparture(std::move(td));
          }
        } catch (const std::exception& e) { LOG_ERROR(e.what()); }
      }

      // TODO Get any transfers from this stop
      // AddTransfers(tilebuilder);

      // Add to stop edge map - track edges that need to be added. This is
      // sorted by graph Id so the stop nodes are added in proper order
      stop_edge_map.insert({platform_pbf_graphid, stopedges});
    }

    // Add routes to the tile. Get vector of route types.
    std::vector<uint32_t> route_types = AddRoutes(tile_pbf, tilebuilder_transit);
    auto tile_bounds = tiles.TileBounds(tile_id.tileid());
    bool tile_within_one_tz = false;
    std::multimap<uint32_t, multi_polygon_type> tz_polys;
    if (tz_db_handle) {
      tz_polys = GetTimeZones(tz_db_handle, tile_bounds);
      if (tz_polys.size() < 2) {
        tile_within_one_tz = true;
      }
    }

    // Add nodes, directededges, and edgeinfo
    AddToGraph(tilebuilder_transit, tile_id, tile_pbf, transit_dir, lock, stop_edge_map,
               stop_no_access, shapes, distances, route_types, tile_within_one_tz, tz_polys,
               stats.no_dir_edge_count);

    LOG_INFO("Tile " + std::to_string(tile_id.tileid()) + ": added " +
             std::to_string(tile_pbf.nodes_size()) + " stops, " +
             std::to_string(tile_pbf.shapes_size()) + " shapes, " +
             std::to_string(route_types.size()) + " routes, and " +
             std::to_string(departures.size()) + " departures");

    // Write the new file
    lock.lock();
    tilebuilder_transit.StoreTileData();
    lock.unlock();
  }

  if (tz_db_handle) {
    sqlite3_close(tz_db_handle);
  }

  // Send back the statistics
  results.set_value(stats);
}

} // namespace

namespace valhalla {
namespace mjolnir {

std::unordered_set<GraphId> convert_transit(const ptree& pt) {

  // figure out which transit tiles even exist
  filesystem::recursive_directory_iterator transit_file_itr(
      pt.get<std::string>("mjolnir.transit_dir") + filesystem::path::preferred_separator +
      std::to_string(TileHierarchy::GetTransitLevel().level));
  filesystem::recursive_directory_iterator end_file_itr;
  std::unordered_set<GraphId> all_tiles;
  for (; transit_file_itr != end_file_itr; ++transit_file_itr) {
    auto tile_path = transit_file_itr->path();
    if (filesystem::is_regular_file(transit_file_itr->path()) &&
        (tile_path.extension() == ".pbf" || std::isdigit(tile_path.string().back()))) {
      all_tiles.emplace(GraphTile::GetTileId(tile_path.string()));
    }
  }

  auto thread_count =
      pt.get<unsigned int>("mjolnir.concurrency", std::max(static_cast<unsigned int>(1),
                                                           std::thread::hardware_concurrency()));
  LOG_INFO("Building transit network.");

  auto t1 = std::chrono::high_resolution_clock::now();
  if (!all_tiles.size()) {
    LOG_INFO("No transit tiles found. Transit will not be added.");
    return all_tiles;
  }

  // TODO - intermediate pass to find any connections that cross into different
  // tile than the stop

  // Second pass - for all tiles with transit stops get all transit information
  // and populate tiles

  // A place to hold worker threads and their results
  std::vector<std::shared_ptr<std::thread>> threads(thread_count);

  // An atomic object we can use to do the synchronization
  std::mutex lock;

  // A place to hold the results of those threads (exceptions, stats)
  std::list<std::promise<builder_stats>> results;

  // Start the threads, divvy up the work
  LOG_INFO("Creating " + std::to_string(all_tiles.size()) + " transit graph tiles...");
  size_t floor = all_tiles.size() / threads.size();
  size_t at_ceiling = all_tiles.size() - (threads.size() * floor);
  std::unordered_set<GraphId>::const_iterator tile_start, tile_end = all_tiles.begin();

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
    threads[i].reset(new std::thread(build_tiles, std::cref(pt.get_child("mjolnir")), std::ref(lock),
                                     tile_start, tile_end, std::ref(results.back())));
  }

  // Wait for them to finish up their work
  for (auto& thread : threads) {
    thread->join();
  }

  // Check all of the outcomes, to see about maximum density (km/km2)
  builder_stats stats{};
  uint32_t total_no_dir_edge_count = 0;
  uint32_t total_dep_count = 0;
  uint32_t total_midnight_dep_count = 0;
  uint32_t total_invalid_service_dates = 0;

  for (auto& result : results) {
    // If something bad went down this will rethrow it
    try {
      auto thread_stats = result.get_future().get();
      stats(thread_stats);
      total_no_dir_edge_count += stats.no_dir_edge_count;
      total_dep_count += stats.dep_count;
      total_midnight_dep_count += stats.midnight_dep_count;
      total_invalid_service_dates += stats.invalid_service_dates;
    } catch (std::exception& e) {
      // TODO: throw further up the chain?
    }
  }

  if (total_invalid_service_dates) {
    LOG_ERROR("There were " + std::to_string(total_invalid_service_dates) +
              " stop pairs with invalid service dates");
  }

  if (total_no_dir_edge_count) {
    LOG_ERROR("There were " + std::to_string(total_no_dir_edge_count) +
              " nodes with no directed edges");
  }

  if (total_dep_count) {
    float percent =
        static_cast<float>(total_midnight_dep_count) / static_cast<float>(total_dep_count);
    percent *= 100;

    LOG_INFO("There were " + std::to_string(total_dep_count) + " departures and " +
             std::to_string(total_midnight_dep_count) +
             " midnight departures were added: " + std::to_string(percent) + "% increase.");
  }

  auto t2 = std::chrono::high_resolution_clock::now();
  [[maybe_unused]] uint32_t secs = std::chrono::duration_cast<std::chrono::seconds>(t2 - t1).count();
  LOG_INFO("Finished building transit network - took " + std::to_string(secs) + " secs");

  return all_tiles;
}

} // namespace mjolnir
} // namespace valhalla
