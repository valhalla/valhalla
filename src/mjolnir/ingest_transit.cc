#include <cmath>
#include <cstdint>
#include <fstream>
#include <functional>
#include <future>
#include <iostream>
#include <memory>
#include <mutex>
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
#include "mjolnir/ingest_transit.h"
#include "mjolnir/servicedays.h"
#include "mjolnir/util.h"
#include "proto/transit.pb.h"

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl_lite.h>

using namespace boost::property_tree;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::mjolnir;

namespace {

struct feedObject {
  gtfs::Id id;
  std::string feed;
};

} // namespace

// information referred by tileBuilder saved spatially
namespace std {
template <> struct hash<feedObject> {
  inline size_t operator()(const feedObject& o) const {
    return hash<string>()(o.id + o.feed);
  }
};
template <> struct equal_to<feedObject> {
  inline bool operator()(const feedObject& a, const feedObject& b) const {
    return a.id == b.id && a.feed == b.feed;
  }
};
} // namespace std

namespace {

struct tileTransitInfo {
  GraphId graphId;
  std::unordered_set<feedObject> tile_stops;
  std::unordered_set<feedObject> tile_trips;
  std::unordered_set<feedObject> tile_routes;
  std::unordered_set<feedObject> tile_services;
  std::unordered_set<feedObject> tile_agencies;
  std::unordered_set<feedObject> tile_shapes;

  bool operator<(const tileTransitInfo& t1) const {
    // sort tileTransitInfo by size
    return tile_stops.size() < t1.tile_stops.size();
  }
};

struct shapeInfo {
  gtfs::Id shape_id;
  float sq_distance;
};

struct feedCache {
  std::unordered_map<std::string, gtfs::Feed> cache;

  const gtfs::Feed& operator()(const feedObject& feed_object) {
    auto found = cache.find(feed_object.feed);
    if (found != cache.end()) {
      return found->second;
    }

    auto inserted = cache.insert({feed_object.feed, gtfs::Feed(feed_object.feed)});
    inserted.first->second.read_feed();
    return inserted.first->second;
  }
};

struct dist_sort_t {
  PointLL center;
  Tiles<PointLL> grid;
  dist_sort_t(const GraphId& center, const Tiles<PointLL>& grid) : grid(grid) {
    this->center = grid.TileBounds(center.tileid()).Center();
  }
  bool operator()(const GraphId& a, const GraphId& b) const {
    auto a_dist = center.Distance(grid.TileBounds(a.tileid()).Center());
    auto b_dist = center.Distance(grid.TileBounds(b.tileid()).Center());
    if (a_dist == b_dist) {
      return a.tileid() < b.tileid();
    }
    return a_dist < b_dist;
  }
};

struct unique_transit_t {
  std::mutex lock;
  std::unordered_map<std::string, size_t> trips;
  std::unordered_map<std::string, size_t> block_ids;
  std::unordered_map<std::string, size_t> lines;
};

// Read from GTFS feed, sort data into the tiles they belong to
std::priority_queue<tileTransitInfo> select_transit_tiles(const boost::property_tree::ptree& pt) {

  auto path = pt.get<std::string>("mjolnir.transit_feeds_dir");

  std::set<GraphId> tiles;
  const auto& tile_level = TileHierarchy::levels().back();
  std::unordered_map<GraphId, tileTransitInfo> tile_map;

  filesystem::recursive_directory_iterator gtfs_feed_itr(path);
  filesystem::recursive_directory_iterator end_file_itr;
  for (; gtfs_feed_itr != end_file_itr; ++gtfs_feed_itr) {
    if (filesystem::is_directory(gtfs_feed_itr->path())) {
      auto feed_path = gtfs_feed_itr->path().string();
      gtfs::Feed feed(feed_path);
      feed.read_feed();
      const auto& stops = feed.get_stops();

      for (const auto& stop : stops) {
        if (stop.stop_id.empty()) {
          continue;
        }
        gtfs::Id currStopId = stop.stop_id;
        float x = stop.stop_lon;
        float y = stop.stop_lat;

        // add unique GraphId and stop
        // might have to typecast these doubles into uint32_t
        GraphId currId = GraphId(tile_level.tiles.TileId(y, x), 3, 0);

        tileTransitInfo currTile;
        if (tile_map.find(currId) != tile_map.end()) {
          currTile = tile_map[currId];
        } else {
          currTile.graphId = currId;
        }
        currTile.tile_stops.insert({currStopId, feed_path});
        gtfs::StopTimes currStopTimes = feed.get_stop_times_for_stop(currStopId);

        for (const auto& stopTime : currStopTimes) {
          // add trip, route, agency and service_id from stop_time
          // could use std::set to not check if it already exists

          // use a set instead to keep track of these points
          if (stopTime.trip_id.empty() || !feed.get_trip(stopTime.trip_id)) {
            continue;
          }
          gtfs::Trip currTrip = *(feed.get_trip(stopTime.trip_id));
          if (currTrip.service_id.empty() || currTrip.route_id.empty() ||
              !feed.get_route(currTrip.route_id)) {
            continue;
          }
          currTile.tile_services.insert({currTrip.service_id, feed_path});
          currTile.tile_trips.insert({currTrip.trip_id, feed_path});

          currTile.tile_shapes.insert({currTrip.shape_id, feed_path});
          // remember the shape id such that we can access it later
          gtfs::Route currRoute = *(feed.get_route(currTrip.route_id));
          currTile.tile_routes.insert({currRoute.route_id, feed_path});
          currTile.tile_agencies.insert({currRoute.agency_id, feed_path});
          // add to the queue of tiles
        }
        tile_map[currId] = currTile;
      }
    }
  }
  std::priority_queue<tileTransitInfo> prioritized;
  for (auto it = tile_map.begin(); it != tile_map.end(); it++) {
    prioritized.push(it->second);
  }
  return prioritized;
}

std::unordered_map<gtfs::Id, GraphId> write_stops(Transit& tile, const tileTransitInfo& tile_info) {

  const auto& tile_stopIds = tile_info.tile_stops;
  auto node_id = tile_info.graphId;
  feedCache stopFeeds;

  std::unordered_map<gtfs::Id, GraphId> stop_graphIds;
  // loop through all stops inside the tile
  for (const feedObject& feed_stop : tile_stopIds) {
    const auto& tile_stopId = feed_stop.id;
    const auto& feed = stopFeeds(feed_stop);
    auto* node = tile.add_nodes();
    gtfs::Stop tile_stop = *(feed.get_stop(tile_stopId));
    node->set_lon(tile_stop.stop_lon);
    node->set_lat(tile_stop.stop_lat);
    node->set_type(static_cast<int>(tile_stop.location_type));
    node->set_graphid(node_id);
    node_id++;
    // TODO: look at how prev_graphid() is used in convert transit (and if it is necessary)
    node->set_name(tile_stop.stop_name);
    // TODO: Determine how to .set_osm_way_id().
    // can't be set directly because it used to be processed by transitland but it is not part of GTFS
    // data
    node->set_timezone(tile_stop.stop_timezone);
    // 0 is No Info ; 1 is True ; 2 is False
    bool wheelchair_accessible = (tile_stop.wheelchair_boarding == "1");
    node->set_wheelchair_boarding(wheelchair_accessible);
    // TODO: look at how generated() is used in convert transit (and if it is necessary)
    node->set_onestop_id(tile_stop.stop_id);
    stop_graphIds[tile_stopId] = node_id;
  }
  return stop_graphIds;
}

// read feed data per stop, given shape
float add_stop_pair_shapes(const gtfs::Stop& stop_connect,
                           const gtfs::Shape& trip_shape,
                           const gtfs::StopTime& pointStopTime) {
  // check which segment would belong to which tile
  if (pointStopTime.shape_dist_traveled > 0) {
    return pointStopTime.shape_dist_traveled;
  }
  float dist_traveled = 0;
  float min_sq_distance = INFINITY;
  PointLL stopPoint = PointLL(stop_connect.stop_lon, stop_connect.stop_lat);
  projector_t project(stopPoint);
  for (int segment = 0; segment < static_cast<int>(trip_shape.size()) - 1; segment++) {
    auto currOrigin = trip_shape[segment];
    auto currDest = trip_shape[segment + 1];
    PointLL originPoint = PointLL(currOrigin.shape_pt_lon, currOrigin.shape_pt_lat);
    PointLL destPoint = PointLL(currDest.shape_pt_lon, currDest.shape_pt_lat);

    PointLL minPoint = project(originPoint, destPoint);
    float sq_distance = project.approx.DistanceSquared(minPoint);
    if (sq_distance < min_sq_distance) {
      min_sq_distance = dist_traveled + originPoint.Distance(minPoint);
    }
    dist_traveled += originPoint.Distance(destPoint);
  }
  return dist_traveled;
}

// return dangling stop_pairs, write stop data from feed
bool write_stop_pairs(Transit& tile,
                      const tileTransitInfo& tile_info,
                      std::unordered_map<gtfs::Id, GraphId> stop_graphIds,
                      unique_transit_t& uniques) {

  const auto& tile_tripIds = tile_info.tile_trips;
  feedCache tripFeeds;
  bool dangles = false;

  for (const feedObject& feed_trip : tile_tripIds) {
    const auto& tile_tripId = feed_trip.id;
    const std::string currFeedPath = feed_trip.feed;
    const auto& feed = tripFeeds(feed_trip);

    gtfs::Trip currTrip = *(feed.get_trip(tile_tripId));

    // already sorted by stop_sequence
    auto tile_stopTimes = feed.get_stop_times_for_trip(tile_tripId);

    for (int stop_sequence = 0; stop_sequence < static_cast<int>(tile_stopTimes.size()) - 1;
         stop_sequence++) {
      gtfs::StopTime origin_stopTime = tile_stopTimes[stop_sequence];
      gtfs::Id origin_stopId = origin_stopTime.stop_id;
      gtfs::StopTime dest_stopTime = tile_stopTimes[stop_sequence + 1];
      gtfs::Id dest_stopId = dest_stopTime.stop_id;
      const bool origin_is_in_tile =
          tile_info.tile_stops.find({origin_stopId, currFeedPath}) != tile_info.tile_stops.end();
      const bool dest_is_in_tile =
          tile_info.tile_stops.find({dest_stopId, currFeedPath}) != tile_info.tile_stops.end();
      // check if this stop_pair (the origin of the pair) is inside the current tile
      if (origin_is_in_tile || dest_is_in_tile) {
        auto* stop_pair = tile.add_stop_pairs();
        stop_pair->set_bikes_allowed(currTrip.bikes_allowed == gtfs::TripAccess::Yes);
        gtfs::Shape currShape = feed.get_shape(currTrip.shape_id);

        if (currTrip.block_id != "") {
          uniques.lock.lock();
          stop_pair->set_block_id(stoi(currTrip.block_id));
          uniques.block_ids.insert({currTrip.block_id, uniques.block_ids.size() + 1});
          uniques.lock.unlock();
        }
        // convert Time to uint32 (note: hh, mm, ss are all uint16s)
        if (!(origin_is_in_tile && dest_is_in_tile)) {
          dangles = true;
        }
        stop_pair->set_origin_onestop_id(origin_stopId);
        stop_pair->set_destination_onestop_id(dest_stopId);
        if (origin_is_in_tile) {
          stop_pair->set_origin_departure_time(stoi(origin_stopTime.departure_time.get_raw_time()));
          stop_pair->set_origin_graphid(stop_graphIds[origin_stopId]);

          // call function to set shape
          float dist =
              add_stop_pair_shapes(*(feed.get_stop(origin_stopId)), currShape, origin_stopTime);
          stop_pair->set_origin_dist_traveled(dist);
        }

        if (dest_is_in_tile) {
          stop_pair->set_destination_arrival_time(stoi(dest_stopTime.arrival_time.get_raw_time()));
          stop_pair->set_destination_graphid(stop_graphIds[dest_stopId]);
          // call function to set shape
          float dist = add_stop_pair_shapes(*(feed.get_stop(dest_stopId)), currShape, dest_stopTime);
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

        stop_pair->set_service_start_date(stoi(trip_calendar.start_date.get_raw_date()));
        stop_pair->set_service_end_date(stoi(trip_calendar.end_date.get_raw_date()));
        stop_pair->set_trip_headsign(currTrip.trip_headsign);

        uniques.lock.lock();
        uniques.trips.insert({currTrip.trip_id, uniques.trips.size()});
        stop_pair->set_trip_id(stoi(currTrip.trip_id));
        uniques.lock.unlock();

        stop_pair->set_wheelchair_accessible(currTrip.wheelchair_accessible == gtfs::TripAccess::Yes);

        // get frequency info
        auto currFrequencies = feed.get_frequencies(currTrip.trip_id);
        auto freq_start_time = (currFrequencies[0].start_time.get_raw_time());
        auto freq_end_time = (currFrequencies[0].start_time.get_raw_time());
        auto freq_time = freq_start_time + freq_end_time;
        // TODO: Determine what to do if there are multiple frequencies.
        // currently, choose fist one
        if (currFrequencies.size() > 0) {
          stop_pair->set_frequency_end_time(DateTime::seconds_from_midnight(freq_end_time));
          stop_pair->set_frequency_headway_seconds(currFrequencies[0].headway_secs);
        }

        auto line_id = stop_pair->origin_onestop_id() < stop_pair->destination_onestop_id()
                           ? stop_pair->origin_onestop_id() + stop_pair->destination_onestop_id() +
                                 currTrip.route_id + freq_time
                           : stop_pair->destination_onestop_id() + stop_pair->origin_onestop_id() +
                                 currTrip.route_id + freq_time;
        uniques.lock.lock();
        uniques.lines.insert({line_id, uniques.lines.size()});
        uniques.lock.unlock();
      }
    }
  }
  return dangles;
}

// read routes data from feed
void write_routes(Transit& tile, const tileTransitInfo& tile_info) {

  const auto& tile_routeIds = tile_info.tile_routes;
  feedCache feedRoutes;

  // loop through all stops inside the tile

  for (const feedObject& feed_route : tile_routeIds) {
    const auto& tile_routeId = feed_route.id;
    const auto& feed = feedRoutes(feed_route);
    auto* route = tile.add_routes();
    gtfs::Route currRoute = *(feed.get_route(tile_routeId));

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
    route->set_vehicle_type(
        (valhalla::mjolnir::Transit_VehicleType)(static_cast<int>(currRoute.route_type)));
  }
}

// grab feed data from feed
void write_shapes(Transit& tile, const tileTransitInfo& tile_info) {

  feedCache feedShapes;

  // loop through all shapes inside the tile
  for (const feedObject& feed_shape : tile_info.tile_shapes) {
    const auto& tile_shape = feed_shape.id;
    const auto& feed = feedShapes(feed_shape);
    auto* shape = tile.add_shapes();
    gtfs::Shape currShape = feed.get_shape(tile_shape, true);
    // We use currShape[0] because 'Shape' type is a vector of ShapePoints, but they all have the same
    // shape_id
    shape->set_shape_id(stoi(currShape[0].shape_id));
    std::vector<PointLL> trip_shape;
    for (const auto& shape_pt : currShape) {
      trip_shape.emplace_back(PointLL(shape_pt.shape_pt_lon, shape_pt.shape_pt_lat));
    }
    shape->set_encoded_shape(encode7(trip_shape));
  }
}

// pre-processes feed data and writes to the pbfs (calls the 'write' functions)
void ingest_tiles(const boost::property_tree::ptree& pt,
                  std::priority_queue<tileTransitInfo>& queue,
                  unique_transit_t& uniques,
                  std::promise<std::list<GraphId>>& promise) {
  // write some stuff here like all the
  // write_stops / stop_pairs / routes / shapes belong here
  // refer to fetch_tiles when trying to write this f^n
  std::list<GraphId> dangling;
  auto now = time(nullptr);
  auto* utc = gmtime(&now);
  utc->tm_year += 1900;
  ++utc->tm_mon; // TODO: use timezone code?

  auto database = pt.get_optional<std::string>("mjolnir.timezone");
  auto path = pt.get<std::string>("mjolnir.transit_feeds_dir");
  // Initialize the tz DB (if it exists)
  sqlite3* tz_db_handle = database ? GetDBHandle(*database) : nullptr;
  if (!database) {
    LOG_WARN("Time zone db not found.  Not saving time zone information from db.");
  } else if (!tz_db_handle) {
    LOG_WARN("Time zone db " + *database + " not found.  Not saving time zone information from db.");
  }
  auto tz_conn = valhalla::mjolnir::make_spatialite_cache(tz_db_handle);

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
    auto file_name = GraphTile::FileSuffix(current.graphId, SUFFIX_NON_COMPRESSED);
    file_name = file_name.substr(0, file_name.size() - 3) + "pbf";
    filesystem::path transit_tile = pt.get<std::string>("mjolnir.transit_dir") +
                                    filesystem::path::preferred_separator + file_name;

    // tiles are wrote out with .pbf or .pbf.n ext
    const std::string& prefix = transit_tile.string();
    LOG_INFO("Fetching " + prefix);

    std::unordered_map<gtfs::Id, GraphId> stop_graphIds = write_stops(tile, current);
    bool dangles = write_stop_pairs(tile, current, stop_graphIds, uniques);
    write_routes(tile, current);
    write_shapes(tile, current);

    // list of dangling tiles
    if (dangles) {
      dangling.emplace_back(current.graphId);
    }

    if (tile.stop_pairs_size()) {
      write_pbf(tile, transit_tile.string());
    }
  }
  promise.set_value(dangling);
}

// connect the stop_pairs that span multiple tiles by processing dangling tiles
void stitch_tiles(const boost::property_tree::ptree& pt,
                  const std::unordered_set<GraphId>& all_tiles,
                  std::list<GraphId>& tiles,
                  std::mutex& lock) {
  auto grid = TileHierarchy::levels().back().tiles;
  auto tile_name = [&pt](const GraphId& id) {
    auto file_name = GraphTile::FileSuffix(id);
    file_name = file_name.substr(0, file_name.size() - 3) + "pbf";
    return pt.get<std::string>("mjolnir.transit_dir") + filesystem::path::preferred_separator +
           file_name;
  };

  // for each tile
  while (true) {
    GraphId current;
    lock.lock();
    if (tiles.empty()) {
      lock.unlock();
      break;
    }
    current = tiles.front();
    tiles.pop_front();
    lock.unlock();

    auto prefix = tile_name(current);
    auto file_name = prefix;
    int ext = 0;

    do {

      // open tile make a hash of missing stop to invalid graphid
      auto tile = read_pbf(file_name, lock);
      std::unordered_map<std::string, GraphId> needed;
      for (const auto& stop_pair : tile.stop_pairs()) {
        if (!stop_pair.has_origin_graphid()) {
          needed.emplace(stop_pair.origin_onestop_id(), GraphId{});
        }
        if (!stop_pair.has_destination_graphid()) {
          needed.emplace(stop_pair.destination_onestop_id(), GraphId{});
        }
      }

      // do while we have more to find and arent sick of searching
      std::set<GraphId, dist_sort_t> unchecked(all_tiles.cbegin(), all_tiles.cend(),
                                               dist_sort_t(current, grid));
      size_t found = 0;
      while (found < needed.size() && unchecked.size()) {
        // crack it open to see if it has what we want
        auto neighbor_id = *unchecked.cbegin();
        unchecked.erase(unchecked.begin());
        if (neighbor_id != current) {
          auto neighbor_file_name = tile_name(neighbor_id);
          auto neighbor = read_pbf(neighbor_file_name, lock);
          for (const auto& node : neighbor.nodes()) {
            auto platform_itr = needed.find(node.onestop_id());
            if (platform_itr != needed.cend()) {
              platform_itr->second.value = node.graphid();
              ++found;
            }
          }
        }
      }

      // get the ids fixed up and write pbf to file
      std::unordered_set<std::string> not_found;
      for (auto& stop_pair : *tile.mutable_stop_pairs()) {
        if (!stop_pair.has_origin_graphid()) {
          auto found = needed.find(stop_pair.origin_onestop_id())->second;
          if (found.Is_Valid()) {
            stop_pair.set_origin_graphid(found);
          } else if (not_found.find(stop_pair.origin_onestop_id()) == not_found.cend()) {
            LOG_ERROR("Stop not found: " + stop_pair.origin_onestop_id());
            not_found.emplace(stop_pair.origin_onestop_id());
          }
          // else{ TODO: we could delete this stop pair }
        }
        if (!stop_pair.has_destination_graphid()) {
          auto found = needed.find(stop_pair.destination_onestop_id())->second;
          if (found.Is_Valid()) {
            stop_pair.set_destination_graphid(found);
          } else if (not_found.find(stop_pair.destination_onestop_id()) == not_found.cend()) {
            LOG_ERROR("Stop not found: " + stop_pair.destination_onestop_id());
            not_found.emplace(stop_pair.destination_onestop_id());
          }
          // else{ TODO: we could delete this stop pair }
        }
      }
      lock.lock();
#if GOOGLE_PROTOBUF_VERSION >= 3001000
      auto size = tile.ByteSizeLong();
#else
      auto size = tile.ByteSize();
#endif
      valhalla::midgard::mem_map<char> buffer;
      // this is giving an error in the header file
      buffer.create(file_name, size);
      tile.SerializeToArray(buffer.get(), size);
      lock.unlock();
      LOG_INFO(file_name + " stitched " + std::to_string(found) + " of " +
               std::to_string(needed.size()) + " stops");

      file_name = prefix + "." + std::to_string(ext++);
    } while (filesystem::exists(file_name));
  }
}
} // namespace

namespace valhalla {
namespace mjolnir {

// thread and call ingest_tiles
std::list<GraphId> ingest_transit(const boost::property_tree::ptree& pt) {

  auto thread_count =
      pt.get<unsigned int>("mjolnir.concurrency", std::max(static_cast<unsigned int>(1),
                                                           std::thread::hardware_concurrency()));
  // go get information about what transit tiles we should be fetching
  LOG_INFO("Tiling GTFS Feeds");
  auto tiles = select_transit_tiles(pt);

  LOG_INFO("Fetching " + std::to_string(tiles.size()) + " transit tiles with " +
           std::to_string(thread_count) + " threads...");

  // schedule some work
  unique_transit_t uniques;
  std::vector<std::shared_ptr<std::thread>> threads(thread_count);
  std::vector<std::promise<std::list<GraphId>>> promises(threads.size());

  // ingest_tiles(pt, tiles, uniques, promises[0]);
  for (size_t i = 0; i < threads.size(); ++i) {
    threads[i].reset(new std::thread(ingest_tiles, std::cref(pt), std::ref(tiles), std::ref(uniques),
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

// thread and call stitch_tiles
void stitch_transit(const boost::property_tree::ptree& pt, std::list<GraphId>& dangling_tiles) {

  auto thread_count =
      pt.get<unsigned int>("mjolnir.concurrency", std::max(static_cast<unsigned int>(1),
                                                           std::thread::hardware_concurrency()));
  // figure out which transit tiles even exist
  filesystem::recursive_directory_iterator transit_file_itr(
      pt.get<std::string>("mjolnir.transit_dir") + filesystem::path::preferred_separator +
      std::to_string(TileHierarchy::GetTransitLevel().level));
  filesystem::recursive_directory_iterator end_file_itr;
  std::unordered_set<GraphId> all_tiles;
  for (; transit_file_itr != end_file_itr; ++transit_file_itr) {
    if (filesystem::is_regular_file(transit_file_itr->path()) &&
        transit_file_itr->path().extension() == ".pbf") {
      all_tiles.emplace(GraphTile::GetTileId(transit_file_itr->path().string()));
    }
  }

  LOG_INFO("Stitching " + std::to_string(dangling_tiles.size()) + " transit tiles with " +
           std::to_string(thread_count) + " threads...");

  // figure out where the work should go
  std::vector<std::shared_ptr<std::thread>> threads(thread_count);
  std::mutex lock;

  // stitch_tiles(pt, all_tiles, dangling_tiles, lock);
  // make let them rip
  for (size_t i = 0; i < threads.size(); ++i) {
    threads[i].reset(new std::thread(stitch_tiles, std::cref(pt), std::cref(all_tiles),
                                     std::ref(dangling_tiles), std::ref(lock)));
  }

  // wait for them to finish
  for (auto& thread : threads) {
    thread->join();
  }

  LOG_INFO("Finished");
}

Transit read_pbf(const std::string& file_name, std::mutex& lock) {
  lock.lock();
  std::fstream file(file_name, std::ios::in | std::ios::binary);
  if (!file) {
    throw std::runtime_error("Couldn't load " + file_name);
    lock.unlock();
  }
  std::string buffer((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  lock.unlock();
  google::protobuf::io::ArrayInputStream as(static_cast<const void*>(buffer.c_str()), buffer.size());
  google::protobuf::io::CodedInputStream cs(
      static_cast<google::protobuf::io::ZeroCopyInputStream*>(&as));
  auto limit = std::max(static_cast<size_t>(1), buffer.size() * 2);
#if GOOGLE_PROTOBUF_VERSION >= 3006000
  cs.SetTotalBytesLimit(limit);
#else
  cs.SetTotalBytesLimit(limit, limit);
#endif
  Transit transit;
  if (!transit.ParseFromCodedStream(&cs)) {
    throw std::runtime_error("Couldn't load " + file_name);
  }
  return transit;
}

Transit read_pbf(const std::string& file_name) {
  std::fstream file(file_name, std::ios::in | std::ios::binary);
  if (!file) {
    throw std::runtime_error("Couldn't load " + file_name);
  }
  std::string buffer((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  google::protobuf::io::ArrayInputStream as(static_cast<const void*>(buffer.c_str()), buffer.size());
  google::protobuf::io::CodedInputStream cs(
      static_cast<google::protobuf::io::ZeroCopyInputStream*>(&as));
  auto limit = std::max(static_cast<size_t>(1), buffer.size() * 2);
#if GOOGLE_PROTOBUF_VERSION >= 3006000
  cs.SetTotalBytesLimit(limit);
#else
  cs.SetTotalBytesLimit(limit, limit);
#endif
  Transit transit;
  if (!transit.ParseFromCodedStream(&cs)) {
    throw std::runtime_error("Couldn't load " + file_name);
  }
  return transit;
}

// Get PBF transit data given a GraphId / tile
Transit read_pbf(const GraphId& id, const std::string& transit_dir, std::string& file_name) {
  std::string fname = GraphTile::FileSuffix(id);
  fname = fname.substr(0, fname.size() - 3) + "pbf";
  file_name = transit_dir + '/' + fname;
  Transit transit;
  transit = read_pbf(file_name);
  return transit;
}

void write_pbf(const Transit& tile, const filesystem::path& transit_tile) {
  // check for empty stop pairs and routes.
  if (tile.stop_pairs_size() == 0 && tile.routes_size() == 0 && tile.shapes_size() == 0) {
    LOG_WARN(transit_tile.string() + " had no data and will not be stored");
    return;
  }

  // write pbf to file
  if (!filesystem::exists(transit_tile.parent_path())) {
    filesystem::create_directories(transit_tile.parent_path());
  }
#if GOOGLE_PROTOBUF_VERSION >= 3001000
  auto size = tile.ByteSizeLong();
#else
  auto size = tile.ByteSize();
#endif
  valhalla::midgard::mem_map<char> buffer;
  buffer.create(transit_tile.string(), size);
  if (!tile.SerializeToArray(buffer.get(), size)) {
    LOG_ERROR("Couldn't write: " + transit_tile.string() + " it would have been " +
              std::to_string(size));
  }

  if (tile.routes_size() && tile.nodes_size() && tile.stop_pairs_size() && tile.shapes_size()) {
    LOG_INFO(transit_tile.string() + " had " + std::to_string(tile.nodes_size()) + " nodes " +
             std::to_string(tile.routes_size()) + " routes " + std::to_string(tile.shapes_size()) +
             " shapes " + std::to_string(tile.stop_pairs_size()) + " stop pairs");
  } else {
    LOG_INFO(transit_tile.string() + " had " + std::to_string(tile.stop_pairs_size()) +
             " stop pairs");
  }
}

// Converts a stop's pbf graph Id to a Valhalla graph Id by adding the
// tile's node count. Returns an Invalid GraphId if the tile is not found
// in the list of Valhalla tiles
GraphId GetGraphId(const GraphId& nodeid, const std::unordered_set<GraphId>& all_tiles) {
  auto t = all_tiles.find(nodeid.Tile_Base());
  if (t == all_tiles.end()) {
    return GraphId(); // Invalid graph Id
  } else {
    return {nodeid.tileid(), nodeid.level() + 1, nodeid.id()};
  }
}

} // namespace mjolnir
} // namespace valhalla