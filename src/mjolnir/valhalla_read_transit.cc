#include <cmath>
#include <cstdint>
#include <fstream>
#include <future>
#include <iostream>
#include <memory>
#include <queue>
#include <random>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>

#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/tokenizer.hpp>
#include <curl/curl.h>

#include "baldr/graphconstants.h"
#include "baldr/graphid.h"
#include "baldr/graphtile.h"
#include "baldr/rapidjson_utils.h"
#include "baldr/tilehierarchy.h"
#include "midgard/encoded.h"
#include "midgard/logging.h"
#include "midgard/sequence.h"
#include "midgard/tiles.h"

#include "filesystem.h"
#include "just_gtfs/just_gtfs.h"
#include "midgard/util.h"
#include "mjolnir/admin.h"
#include "mjolnir/servicedays.h"
#include "mjolnir/transitpbf.h"
#include "mjolnir/util.h"
#include "valhalla/proto/transit.pb.h"

using namespace boost::property_tree;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::mjolnir;

// information referred by tileBuilder saved spatially
typedef struct tileTransitInfo {
  GraphId graphId;
  std::set<gtfs::Id> tile_stops;
  std::set<gtfs::Id> tile_trips;
  std::set<gtfs::Id> tile_routes;
  std::set<gtfs::Id> tile_services;
  std::set<gtfs::Id> tile_agencies;
  std::set<gtfs::Id> tile_shapes;
} tileTransitInfo;

typedef struct shapeInfo {
  gtfs::Id shape_id;
  float sq_distance;
} shapeInfo;

struct CompareNumStops {
  bool operator()(tileTransitInfo const& t1, tileTransitInfo const& t2) {
    // sort tileTransitInfo by size
    return t1.tile_stops.size() < t2.tile_stops.size();
  }
};

std::priority_queue<tileTransitInfo, std::vector<tileTransitInfo>, CompareNumStops>
select_tiles(const std::string& path) {
  gtfs::Feed feed(path);
  // TODO: CREATE LOOP THROUGH ALL FEEDS
  feed.read_feed();
  const auto& stops = feed.get_stops();

  std::set<GraphId> tiles;
  const auto& tile_level = TileHierarchy::levels().back();
  std::unordered_map<GraphId, tileTransitInfo> tile_map;
  // std::priority_queue<tileTransitInfo, std::vector<tileTransitInfo>, CompareNumStops> prioritized;

  for (const auto& stop : stops) {
    gtfs::Id currStopId = stop.stop_id;
    float x = stop.stop_lon;
    float y = stop.stop_lat;

    // add unique GraphId and stop
    // might have to typecast these doubles into uint32_t
    GraphId currId = GraphId(tile_level.tiles.TileId(y, x), 3, 0);
    auto currTile = tile_map[currId];
    currTile.graphId = currId;
    currTile.tile_stops.insert(currStopId);
    gtfs::StopTimes currStopTimes = feed.get_stop_times_for_stop(currStopId);

    for (auto stopTime : currStopTimes) {
      // add trip, route, agency and service_id from stop_time
      // could use std::set to not check if it already exists

      // use a set instead to keep track of these points
      gtfs::Trip currTrip = *(feed.get_trip(stopTime.trip_id));
      currTile.tile_services.insert(currTrip.service_id);
      currTile.tile_trips.insert(currTrip.trip_id);

      currTile.tile_shapes.insert(currTrip.shape_id);
      // remember the shape id such that we can access it later
      gtfs::Route currRoute = *(feed.get_route(currTrip.route_id));
      currTile.tile_routes.insert(currRoute.route_id);
      currTile.tile_agencies.insert(currRoute.agency_id);
      // add to the queue of tiles
      // prioritized.push(currTile);
    }
  }
  std::priority_queue<tileTransitInfo, std::vector<tileTransitInfo>, CompareNumStops> prioritized;
  for (auto it = tile_map.begin(); it != tile_map.end(); it++) {
    prioritized.push(it->second);
  }
  return prioritized;
}

std::unordered_map<gtfs::Id, GraphId>
write_stops(Transit& tile, const tileTransitInfo& tile_info, const std::string& path) {
  gtfs::Feed feed(path);
  feed.read_feed();
  auto tile_stopIds = tile_info.tile_stops;
  auto node_id = tile_info.graphId;

  std::unordered_map<gtfs::Id, GraphId> stop_graphIds;
  // loop through all stops inside the tile
  for (const gtfs::Id& tile_stopId : tile_stopIds) {
    auto* node = tile.add_nodes();
    gtfs::Stop tile_stop = *(feed.get_stop(tile_stopId));
    node->set_lon(tile_stop.stop_lon);
    node->set_lat(tile_stop.stop_lat);
    node->set_type((int)tile_stop.location_type);
    node->set_graphid(node_id);
    node_id++;
    // node->set_prev_graphid(); what is prev?
    node->set_name(tile_stop.stop_name);
    // node->set_osm_way_id(); where is this?
    node->set_timezone(tile_stop.stop_timezone);
    // 0 is No Info ; 1 is True ; 2 is False
    bool wheelchair_accessible = (tile_stop.wheelchair_boarding == "1");
    node->set_wheelchair_boarding(wheelchair_accessible);
    // node->set_generated(feed_stop.generated); ?

    stop_graphIds[tile_stopId] = node_id;
  }
  return stop_graphIds;
}

// read per stop, given shape
float add_stop_pair_shapes(const gtfs::Stop& stop_connect, const gtfs::Shape& trip_shape) {
  // check which segment would belong
  // projector_t project(PointLL(lon, lat)); building a path to geodesic .?
  float dist_traveled = 0;
  float min_sq_distance = INFINITY;
  for (int segment = 0; segment < trip_shape.size() - 1; segment++) {
    // change to if shape point size >= 2 and for until 1 before last
    auto currOrigin = trip_shape[segment];
    auto currDest = trip_shape[segment + 1];
    PointLL stopPoint = PointLL(stop_connect.stop_lon, stop_connect.stop_lat);
    PointLL originPoint = PointLL(currOrigin.shape_pt_lon, currOrigin.shape_pt_lat);
    PointLL destPoint = PointLL(currDest.shape_pt_lon, currDest.shape_pt_lat);
    // why am i getting a re-declaration error? is this not how i use operator()?
    projector_t project(stopPoint);
    PointLL minPoint = project(originPoint, destPoint);
    float sq_distance = project.approx.DistanceSquared(minPoint);
    if (sq_distance < min_sq_distance) {
      min_sq_distance = dist_traveled + originPoint.Distance(minPoint);
    }
    dist_traveled += originPoint.Distance(destPoint);
  }
  return dist_traveled;
}
// return dangling stop_pairs
bool write_stop_pairs(Transit& tile,
                      const tileTransitInfo& tile_info,
                      const std::string& path,
                      std::unordered_map<gtfs::Id, GraphId> stop_graphIds) {
  gtfs::Feed feed(path);
  feed.read_feed();
  auto& tile_tripIds = tile_info.tile_trips;
  bool dangles = false;
  for (const gtfs::Id& tile_tripId : tile_tripIds) {

    gtfs::Trip currTrip = *(feed.get_trip(tile_tripId));

    // already sorted by stop_sequence
    auto tile_stopTimes = feed.get_stop_times_for_trip(tile_tripId);

    for (int stop_sequence = 0; stop_sequence < tile_stopTimes.size() - 1; stop_sequence++) {
      gtfs::StopTime origin_stopTime = tile_stopTimes[stop_sequence];
      gtfs::Id origin_stopId = origin_stopTime.stop_id;
      gtfs::StopTime dest_stopTime = tile_stopTimes[stop_sequence + 1];
      gtfs::Id dest_stopId = dest_stopTime.stop_id;
      const bool origin_is_in_tile =
          tile_info.tile_stops.find(origin_stopId) != tile_info.tile_stops.end();
      const bool dest_is_in_tile =
          tile_info.tile_stops.find(dest_stopId) != tile_info.tile_stops.end();
      // check if this stop_pair (the origin of the pair) is inside the current tile
      if (origin_is_in_tile || dest_is_in_tile) {
        auto* stop_pair = tile.add_stop_pairs();
        stop_pair->set_bikes_allowed(currTrip.bikes_allowed == gtfs::TripAccess::Yes);
        gtfs::Shape currShape = feed.get_shape(currTrip.shape_id);
        // where is blockid??
        // convert Time to uint32 (note: hh, mm, ss are all uint16s)
        if (!(origin_is_in_tile && dest_is_in_tile)) {
          dangles = true;
        }
        if (origin_is_in_tile) {
          stop_pair->set_origin_departure_time(stoi(origin_stopTime.departure_time.get_raw_time()));
          stop_pair->set_origin_graphid(stop_graphIds[origin_stopId]);
          stop_pair->set_origin_onestop_id(origin_stopId);
          // call function to set shape
          float dist = add_stop_pair_shapes(*(feed.get_stop(origin_stopId)), currShape);
          stop_pair->set_origin_dist_traveled(dist);
        }

        if (dest_is_in_tile) {
          stop_pair->set_destination_arrival_time(stoi(dest_stopTime.arrival_time.get_raw_time()));
          stop_pair->set_destination_graphid(stop_graphIds[dest_stopId]);
          stop_pair->set_destination_onestop_id(dest_stopTime.stop_id);
          float dist = add_stop_pair_shapes(*(feed.get_stop(dest_stopId)), currShape);
          stop_pair->set_destination_dist_traveled(dist);
        }

        stop_pair->set_route_index(stoi(currTrip.route_id));
        // add information from calendar_dates.txt
        gtfs::CalendarDates trip_calDates = feed.get_calendar_dates(currTrip.service_id);
        for (const auto& cal_date_item : trip_calDates) {

          auto d = (cal_date_item.date).get_raw_date();
          if (cal_date_item.exception_type == gtfs::CalendarDateException::Added)
            stop_pair->add_service_added_dates(stoi(d));
          else
            stop_pair->add_service_except_dates(stoi(d));
        }
        gtfs::CalendarItem trip_calendar = *(feed.get_calendar(currTrip.service_id));
        std::vector<bool> service_days = {(bool)trip_calendar.monday,    (bool)trip_calendar.tuesday,
                                          (bool)trip_calendar.wednesday, (bool)trip_calendar.thursday,
                                          (bool)trip_calendar.friday,    (bool)trip_calendar.saturday,
                                          (bool)trip_calendar.sunday};
        for (int i = 0; i < 7; i++) {
          stop_pair->set_service_days_of_week(i, service_days[i]);
        }
        stop_pair->set_service_start_date(trip_calendar.start_date);
        stop_pair->set_service_end_date(trip_calendar.end_date);
        stop_pair->set_trip_headsign(currTrip.trip_headsign);
        stop_pair->set_trip_id(stoi(currTrip.trip_id));
        stop_pair->set_wheelchair_accessible(currTrip.wheelchair_accessible == 1);

        // insert shape info

        // get frequency info
        auto currFrequencies = feed.get_frequencies(currTrip.trip_id);
        // what to do if there are multiple frequencies?
        if (currFrequencies.size() > 0) {
          stop_pair->set_frequency_end_time(
              DateTime::seconds_from_midnight(stoi(currFrequencies[0].end_time.get_raw_time()));
          stop_pair->set_frequency_headway_seconds(currFrequencies[0].headway_secs);
        }
      }
    }
  }
  return dangles;
}

void write_routes(Transit& tile, const tileTransitInfo& tile_info, const std::string& path) {
  gtfs::Feed feed(path);
  feed.read_feed();

  for (const auto& routeId : tile_info.tile_routes) {
    auto* route = tile.add_routes();
    gtfs::Route currRoute = *(feed.get_route(routeId));

    route->set_name(currRoute.route_short_name);
    route->set_onestop_id(currRoute.route_id);
    route->set_operated_by_onestop_id(currRoute.agency_id);
    gtfs::Agency currAgency = *(feed.get_agency(currRoute.agency_id));
    route->set_operated_by_name(currAgency.agency_name);
    route->set_operated_by_website(currAgency.agency_url);

    route->set_route_color(strtol(currRoute.route_color.c_str(), nullptr, 16));
    route->set_route_desc(currRoute.route_desc);
    route->set_route_long_name(currRoute.route_long_name);
    route->set_route_text_color(strtol(currRoute.route_text_color.c_str(), nullptr, 16));
    route->set_vehicle_type((valhalla::mjolnir::Transit_Vehicletype)((int)currRoute.route_type));
  }
}

void write_shapes(Transit& tile, const tileTransitInfo& tile_info, const std::string& path) {
  // it's all very straightforward after implementing the matching stops -> shape_pts .
  // just include all shape_pts (in this tile)
  gtfs::Feed feed(path);
  feed.read_feed();
  for (const auto& tile_shape : tile_info.tile_shapes) {
    auto* shape = tile.add_shapes();
    gtfs::Shape currShape = feed.get_shape(tile_shape, true);
    // We use currShape[0] because 'Shape' type is a vector of ShapePoints
    shape->set_shape_id(stoi(currShape[0].shape_id));
    std::vector<PointLL> trip_shape;
    for (const auto& shape_pt : currShape) {
      trip_shape.emplace_back(PointLL(shape_pt.shape_pt_lon, shape_pt.shape_pt_lat));
    }
    shape->set_encoded_shape(encode7(trip_shape));
  }
}

// Must decide what I should change this to.
struct unique_transit_t {
  std::mutex lock;
  std::unordered_map<std::string, size_t> trips;
  std::unordered_map<std::string, size_t> block_ids;
  std::unordered_set<std::string> missing_routes;
  std::unordered_map<std::string, size_t> lines;
};

// note: so basically fetch_tiles is the function and
// fetch_ only does the threading
void build_tiles(std::priority_queue<tileTransitInfo>& queue,
                 const std::string& path,
                 unique_transit_t uniques,
                 std::promise<std::list<GraphId>>& promise) {
  // write some stuff here like all the
  // write_stops / stop_pairs / routes / shapes belong here
  // refer to fetch_tiles when trying to write this f^n
  std::list<GraphId> dangling;
  while (true) {
    tileTransitInfo current;
    uniques.lock.lock();
    if (queue.empty()) {
      uniques.lock.unlock();
      break;
    }
    current = queue.top();
    queue.pop();
    uniques.lock.unlock();

    Transit tile;

    std::unordered_map<gtfs::Id, GraphId> stop_graphIds = write_stops(tile, current, path);
    bool dangles = write_stop_pairs(tile, current, path, stop_graphIds);
    write_routes(tile, current, path);
    write_shapes(tile, current, path);

    // list of dangling tiles
    if (dangles) {
      dangling.emplace_back(current.graphId);
    }

    if (tile.stop_pairs_size()) {
      // what is the pt?
      auto file_name = GraphTile::FileSuffix(current.graphId, SUFFIX_NON_COMPRESSED);
      filesystem::path transit_tile = pt.get<std::string>("mjolnir.transit_dir") +
                                      filesystem::path::preferred_separator + file_name;
      write_pbf(tile, transit_tile.string());
    }
  }
  promise.set_value(dangling);
}

std::list<GraphId> fetch(const ptree& pt,
                         std::priority_queue<weighted_tile_t>& tiles,
                         unsigned int thread_count = std::max(static_cast<unsigned int>(1),
                                                              std::thread::hardware_concurrency())) {
  LOG_INFO("Fetching " + std::to_string(tiles.size()) + " transit tiles with " +
           std::to_string(thread_count) + " threads...");

  // schedule some work
  unique_transit_t uniques;
  std::vector<std::shared_ptr<std::thread>> threads(thread_count);
  std::vector<std::promise<std::list<GraphId>>> promises(threads.size());
  for (size_t i = 0; i < threads.size(); ++i) {
    threads[i].reset(new std::thread(fetch_tiles, std::cref(pt), std::ref(tiles), std::ref(uniques),
                                     std::ref(promises[i])));
  }

  // let the threads finish and get the dangling list
  for (auto& thread : threads) {
    thread->join();
  }
  std::list<GraphId> dangling;
  for (auto& promise : promises) {
    try {
      dangling.splice(dangling.end(), promise.get_future().get());
    } catch (std::exception& e) {
      // TODO: throw further up the chain?
    }
  }

  LOG_INFO("Finished");
  return dangling;
}