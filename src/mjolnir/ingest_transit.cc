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

struct feed_object_t {
  gtfs::Id id;
  std::string feed;
};

} // namespace

// information referred by tileBuilder saved spatially
namespace std {
template <> struct hash<feed_object_t> {
  inline size_t operator()(const feed_object_t& o) const {
    return hash<string>()(o.id + o.feed);
  }
};
template <> struct equal_to<feed_object_t> {
  inline bool operator()(const feed_object_t& a, const feed_object_t& b) const {
    return a.id == b.id && a.feed == b.feed;
  }
};
} // namespace std

namespace {

struct tile_transit_info_t {
  GraphId graphid;
  // TODO: unordered multimap-string to a pair of strings that maps from station (parent_id) ->
  //  platform/egress (child) (Max 2 kinds / many could exists)
  std::unordered_multimap<feed_object_t, gtfs::Id> station_children;
  std::unordered_set<feed_object_t> stations;
  std::unordered_set<feed_object_t> trips;
  std::unordered_map<feed_object_t, size_t> routes;
  std::unordered_map<feed_object_t, size_t> shapes;

  bool operator<(const tile_transit_info_t& t1) const {
    // the queue needs to be able to sort
    return stations.size() < t1.stations.size();
  }
};

struct feed_cache_t {
  std::unordered_map<std::string, gtfs::Feed> cache;
  std::string gtfs_dir;

  feed_cache_t(const std::string& gtfs_dir) : gtfs_dir(gtfs_dir) {
  }

  const gtfs::Feed& operator()(const feed_object_t& feed_object) {
    auto found = cache.find(feed_object.feed);
    if (found != cache.end()) {
      return found->second;
    }

    auto inserted = cache.insert({feed_object.feed, gtfs::Feed(gtfs_dir + feed_object.feed)});
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

std::string get_tile_path(const std::string& tile_dir, const GraphId& tile_id) {
  auto file_name = GraphTile::FileSuffix(tile_id);
  file_name = file_name.substr(0, file_name.size() - 3) + "pbf";
  return tile_dir + file_name;
};

// converts service start/end dates of the form (yyyymmdd) into epoch seconds
uint32_t to_local_pivot_sec(const std::string& dt, bool end_of_day = false) {
  date::local_seconds tp;
  std::istringstream in{dt};
  in >> date::parse("%Y%m%d", tp);
  auto epoch = static_cast<uint32_t>((tp - DateTime::pivot_date_).count());
  epoch += end_of_day ? kSecondsPerDay - 1 : 0;
  return epoch;
};

std::string get_onestop_id_base(const std::string& stop_id, const std::string& feed_name) {
  return feed_name + "_" + stop_id;
}

// Read from GTFS feed, sort data into the unique tiles they belong to
std::priority_queue<tile_transit_info_t> select_transit_tiles(const std::string& gtfs_path) {

  std::set<GraphId> tiles;
  const auto& local_tiles = TileHierarchy::levels().back().tiles;
  std::unordered_map<GraphId, tile_transit_info_t> tile_map;

  // adds a tile_info for the tile_id to tile_map if there is none yet, and returns the tile_info
  // object
  auto get_tile_info = [&tile_map, &local_tiles](const gtfs::Stop& stop) -> tile_transit_info_t& {
    auto graphid = GraphId(local_tiles.TileId(stop.stop_lat, stop.stop_lon),
                           TileHierarchy::GetTransitLevel().level, 0);

    return tile_map.insert({graphid, tile_transit_info_t{graphid}}).first->second;
  };

  filesystem::recursive_directory_iterator gtfs_feed_itr(gtfs_path);
  filesystem::recursive_directory_iterator end_file_itr;
  for (; gtfs_feed_itr != end_file_itr; ++gtfs_feed_itr) {
    const auto& feed_path = gtfs_feed_itr->path();
    if (filesystem::is_directory(feed_path)) {
      // feed_path has a trailing separator
      const auto feed_name = feed_path.filename().string();

      LOG_INFO("Loading " + feed_name);
      gtfs::Feed feed(feed_path.string());
      feed.read_feed();
      LOG_INFO("Done loading, now parsing " + feed_name);

      const auto& stops = feed.get_stops();
      // 1st pass to add all the stations, so we can add stops to its children in a 2nd pass
      for (const auto& stop : stops) {
        if (stop.location_type == gtfs::StopLocationType::Station) {
          auto& tile_info = get_tile_info(stop);
          tile_info.stations.insert({stop.stop_id, feed_name});
        }
      }

      // 2nd pass to add the platforms/stops
      for (const auto& stop : stops) {
        // TODO: GenericNode & BoardingArea could be useful at some point
        if (!(stop.location_type == gtfs::StopLocationType::StopOrPlatform) &&
            !(stop.location_type == gtfs::StopLocationType::EntranceExit)) {
          continue;
        }

        auto& tile_info = get_tile_info(stop);

        // if this station doesn't exist, we need to create it: we use the fact that this entry is
        // not a station type to fake a station object in write_stops()
        auto station_in_tile = tile_info.stations.find({stop.parent_station, feed_name});
        if (station_in_tile != tile_info.stations.end()) {
          tile_info.station_children.insert({{stop.parent_station, feed_name}, stop.stop_id});
        } else {
          // we don't have the parent station, if
          // 1) this stop has none or 2) its parent station is in another tile
          // TODO: need to handle the 2nd case somehow! Fow now, log it as ERROR
          auto parent_station = feed.get_stop(stop.parent_station);
          if (parent_station &&
              tile_info.graphid !=
                  GraphId(local_tiles.TileId(parent_station->stop_lat, parent_station->stop_lon),
                          TileHierarchy::GetTransitLevel().level, 0)) {
            LOG_WARN("Station ID " + stop.parent_station + " is not in stop's " + stop.stop_id +
                     " tile: " + std::to_string(tile_info.graphid));
          }
          tile_info.stations.insert({stop.stop_id, feed_name});
          tile_info.station_children.insert({{stop.stop_id, feed_name}, stop.stop_id});
        }

        for (const auto& stopTime : feed.get_stop_times_for_stop(stop.stop_id)) {
          // add trip, route, agency and service_id from stop_time, it's the only place with that info
          // TODO: should we throw here?
          auto trip = feed.get_trip(stopTime.trip_id);
          auto route = feed.get_route(trip->route_id);
          if (!trip || !route || trip->service_id.empty()) {
            LOG_ERROR("Missing trip or route or service_id for trip");
            continue;
          }

          tile_info.trips.insert({trip->trip_id, feed_name});
          tile_info.routes.insert({{route->route_id, feed_name}, tile_info.routes.size()});

          // shapes are optional, don't keep non-existing shapes around
          if (!trip->shape_id.empty()) {
            tile_info.shapes.insert({{trip->shape_id, feed_name}, tile_info.shapes.size()});
          }
        }
      }

      LOG_INFO("Done parsing " + std::to_string(tile_map.size()) + " transit tiles for GTFS feed " +
               feed_name);
    }
  }
  std::priority_queue<tile_transit_info_t> queue;
  for (auto it = tile_map.begin(); it != tile_map.end(); it++) {
    queue.push(it->second);
  }
  return queue;
}

void setup_stops(Transit& tile,
                 const gtfs::Stop& tile_stop,
                 GraphId& node_id,
                 std::unordered_map<feed_object_t, GraphId>& platform_node_ids,
                 const std::string& feed_name,
                 NodeType node_type,
                 bool isGenerated,
                 GraphId prev_id = {}) {
  auto* node = tile.mutable_nodes()->Add();
  node->set_lon(tile_stop.stop_lon);
  node->set_lat(tile_stop.stop_lat);

  // change set_type to match the child / parent type.
  node->set_type(static_cast<uint32_t>(node_type));
  node->set_graphid(node_id);

  if (node_type != NodeType::kTransitEgress) {
    node->set_prev_type_graphid(prev_id.Is_Valid() ? prev_id : node_id - 1);
  } // in/egresses need accessibility set so that transit connect edges inherit that access
  else {
    // TODO: its unclear how to determine unidirectionality (entrance-only vs exit-only) from the gtfs
    //  spec, perhaps one should use pathways.txt::is_bidirectional to differentiate?
    node->set_traversability(static_cast<uint32_t>(Traversability::kBoth));
  }
  node->set_name(tile_stop.stop_name);
  node->set_timezone(tile_stop.stop_timezone);
  bool wheelchair_accessible = (tile_stop.wheelchair_boarding == "1");
  node->set_wheelchair_boarding(wheelchair_accessible);
  node->set_generated(isGenerated);
  const auto& onestop_base = get_onestop_id_base(tile_stop.stop_id, feed_name);
  node->set_onestop_id(isGenerated ? onestop_base + "_" + to_string(node_type) : onestop_base);
  if (node_type == NodeType::kMultiUseTransitPlatform) {
    // when this platform is not generated, we can use its actual given id verbatim because thats how
    // other gtfs entities will refer to it. however when it is generated we must use its parents id
    // and not the generated one because the references in the feed have no idea that we are doing
    // the generation of new ideas for non-existant platforms
    platform_node_ids[{tile_stop.stop_id, feed_name}] = node_id;
  }
  node_id++;
}

/**
 * Writes all of the nodes of the graph in pbf tile
 * @param tile       the transit pbf tile to modify
 * @param tile_info  ephemeral info from the transit feeds which we write into the tile
 * @return a map of string gtfs id to graph id (so node id) for every platform in the tile
 *         later on we use these ids to connect platforms that reference each other in the schedule
 */
std::unordered_map<feed_object_t, GraphId>
write_stops(Transit& tile, const tile_transit_info_t& tile_info, feed_cache_t& feeds) {
  const auto& tile_children = tile_info.station_children;
  auto node_id = tile_info.graphid;

  // loop through all stations inside the tile, and write PBF nodes in the order that is expected
  // anything can be in tile_info.stations, but we can distinguish and write the proper order
  std::unordered_map<feed_object_t, GraphId> platform_node_ids;
  for (const feed_object_t& station : tile_info.stations) {
    const auto& feed = feeds(station);
    auto station_as_stop = feed.get_stop(station.id);

    // Add the Egress
    int node_count = tile.nodes_size();
    for (const auto& child : tile_children) {
      auto child_stop = feed.get_stop(child.second);
      if (child.first.id == station.id && child.first.feed == station.feed &&
          child_stop->location_type == gtfs::StopLocationType::EntranceExit) {
        setup_stops(tile, *child_stop, node_id, platform_node_ids, station.feed,
                    NodeType::kTransitEgress, false);
      }
    }
    // We require an in/egress so if we didnt add one we need to fake one
    if (tile.nodes_size() == node_count) {
      setup_stops(tile, *station_as_stop, node_id, platform_node_ids, station.feed,
                  NodeType::kTransitEgress, true);
    }

    // Add the Station
    GraphId prev_id(tile.nodes(node_count).graphid());
    if (station_as_stop->location_type == gtfs::StopLocationType::Station) {
      // TODO(nils): what happens to the station if it was actually in another tile but still recorded
      // here (see above)?!
      setup_stops(tile, *station_as_stop, node_id, platform_node_ids, station.feed,
                  NodeType::kTransitStation, false, prev_id);
    } else {
      // if there was a platform/egress with no parent station, we add one
      setup_stops(tile, *station_as_stop, node_id, platform_node_ids, station.feed,
                  NodeType::kTransitStation, true, prev_id);
    }

    // Add the Platform
    node_count = tile.nodes_size();
    prev_id = GraphId(tile.nodes().rbegin()->graphid());
    for (const auto& child : tile_children) {
      auto child_stop = feed.get_stop(child.second);

      if (child.first.id == station.id && child.first.feed == station.feed &&
          child_stop->location_type == gtfs::StopLocationType::StopOrPlatform) {
        setup_stops(tile, *child_stop, node_id, platform_node_ids, station.feed,
                    NodeType::kMultiUseTransitPlatform, false, prev_id);
      }
    }
    // This really shouldn't happen, as platform IDs are referenced by the
    // stop_times.txt and the whole downstream logic would get messed up
    // TODO: might want to completely skip this stop if this happens, be careful
    //   to properly remove its associated station/egress IF they're not referenced
    //   by other platforms.
    if (tile.nodes_size() == node_count) {
      LOG_ERROR("Generated platform for station " + station_as_stop->stop_id);
      setup_stops(tile, *station_as_stop, node_id, platform_node_ids, station.feed,
                  NodeType::kMultiUseTransitPlatform, true, prev_id);
    }
  }
  return platform_node_ids;
}

// read feed data per stop, given shape
float get_stop_pair_dist(const gtfs::Stop& stop_connect,
                         const gtfs::Shape& trip_shape,
                         const gtfs::StopTime& pointStopTime) {
  // check which segment would belong to which tile
  if (pointStopTime.shape_dist_traveled > 0) {
    return pointStopTime.shape_dist_traveled;
  } else if (!trip_shape.size()) {
    return 0.f;
  }

  // collect all distances and return the index with the lowest
  float dist_traveled = 0;
  float final_dist_traveled = 0;
  float min_sq_distance = INFINITY;

  PointLL stopPoint = PointLL(stop_connect.stop_lon, stop_connect.stop_lat);
  projector_t project(stopPoint);
  for (size_t segment = 0; segment < trip_shape.size() - 1; segment++) {
    auto currOrigin = trip_shape[segment];
    auto currDest = trip_shape[segment + 1];
    // TODO: we can use the trip_shape.shape_dist_traveled here too and early exit if it's there
    PointLL originPoint = PointLL(currOrigin.shape_pt_lon, currOrigin.shape_pt_lat);
    PointLL destPoint = PointLL(currDest.shape_pt_lon, currDest.shape_pt_lat);

    PointLL proj_point = project(originPoint, destPoint);
    float sq_distance = project.approx.DistanceSquared(proj_point);

    float segment_length = originPoint.Distance(destPoint);
    dist_traveled += segment_length;

    DistanceApproximator<PointLL> proj_approx(proj_point);
    if (sq_distance < min_sq_distance) {
      min_sq_distance = sq_distance;
      final_dist_traveled =
          proj_approx.DistanceSquared(originPoint) > proj_approx.DistanceSquared(destPoint)
              ? dist_traveled
              : dist_traveled - segment_length;
    }
  }
  return final_dist_traveled;
}

// return dangling stop_pairs, write stop data from feed
bool write_stop_pair(
    Transit& tile,
    const tile_transit_info_t& tile_info,
    const feed_object_t& feed_trip,
    const gtfs::Feed& feed,
    const std::unordered_map<feed_object_t, GraphId>& platform_node_ids,
    unique_transit_t& uniques,
    const google::protobuf::RepeatedPtrField<valhalla::mjolnir::Transit_Node>& tile_nodes,
    const std::unordered_map<feed_object_t, size_t>& routes_ids) {
  bool dangles = false;

  const auto& tile_tripId = feed_trip.id;
  const std::string currFeedPath = feed_trip.feed;

  const auto& currTrip = feed.get_trip(tile_tripId);
  const auto& trip_calendar = feed.get_calendar(currTrip->service_id);
  const gtfs::CalendarDates& trip_calDates = feed.get_calendar_dates(currTrip->service_id);
  if (!currTrip || !trip_calendar) {
    LOG_ERROR("Feed " + feed_trip.feed + ", trip ID" + tile_tripId +
              " can't be found or has no calendar.txt entry, skipping...");
    return false;
  }

  // get the gtfs shape and our pbf shape_id if present
  const auto& currShape = feed.get_shape(currTrip->shape_id);
  auto pbf_shape_it = tile_info.shapes.find({currTrip->shape_id, feed_trip.feed});

  // already sorted by stop_sequence
  const auto tile_stopTimes = feed.get_stop_times_for_trip(tile_tripId);

  for (size_t stop_sequence = 0; stop_sequence < tile_stopTimes.size() - 1; stop_sequence++) {
    const auto& origin_stopTime = tile_stopTimes[stop_sequence];
    const auto& origin_stopId = origin_stopTime.stop_id;
    const auto& origin_stop = feed.get_stop(origin_stopId);
    assert(origin_stop);
    const auto& dest_stopTime = tile_stopTimes[stop_sequence + 1];
    const auto& dest_stopId = dest_stopTime.stop_id;
    const auto& dest_stop = feed.get_stop(dest_stopId);
    assert(dest_stop);
    const auto origin_graphid_it = platform_node_ids.find({origin_stopId, currFeedPath});
    const auto dest_graphid_it = platform_node_ids.find({dest_stopId, currFeedPath});
    const bool origin_is_in_tile = origin_graphid_it != platform_node_ids.end();
    const bool dest_is_in_tile = dest_graphid_it != platform_node_ids.end();

    // if it's not in the tile, we can't do anything else than take it's gtfs stop_id
    // we check further down when stitching and adjust if it's was generated
    std::string origin_onestop_id = origin_is_in_tile
                                        ? tile_nodes.Get(origin_graphid_it->second.id()).onestop_id()
                                        : get_onestop_id_base(origin_stopId, feed_trip.feed);
    std::string dest_onestop_id = dest_is_in_tile
                                      ? tile_nodes.Get(dest_graphid_it->second.id()).onestop_id()
                                      : get_onestop_id_base(dest_stopId, feed_trip.feed);

    // we don't use this value unless the origin is in the tile, so it's fine to set it false
    bool origin_is_generated =
        origin_is_in_tile ? tile_nodes.Get(origin_graphid_it->second.id()).generated() : false;
    bool dest_is_generated =
        dest_is_in_tile ? tile_nodes.Get(dest_graphid_it->second.id()).generated() : false;

    // check if this stop_pair (the origin of the pair) is inside the current tile
    if ((origin_is_in_tile || dest_is_in_tile) && origin_stopTime.trip_id == dest_stopTime.trip_id) {
      auto* stop_pair = tile.add_stop_pairs();

      // add information from calendar.txt and calendar_dates.txt
      auto* service_dow = stop_pair->mutable_service_days_of_week();
      service_dow->Add(trip_calendar->monday == gtfs::CalendarAvailability::Available);
      service_dow->Add(trip_calendar->tuesday == gtfs::CalendarAvailability::Available);
      service_dow->Add(trip_calendar->wednesday == gtfs::CalendarAvailability::Available);
      service_dow->Add(trip_calendar->thursday == gtfs::CalendarAvailability::Available);
      service_dow->Add(trip_calendar->friday == gtfs::CalendarAvailability::Available);
      service_dow->Add(trip_calendar->saturday == gtfs::CalendarAvailability::Available);
      service_dow->Add(trip_calendar->sunday == gtfs::CalendarAvailability::Available);

      bool had_added_date = false;
      for (const auto& cal_date_item : trip_calDates) {
        auto d = to_local_pivot_sec(cal_date_item.date.get_raw_date());
        if (cal_date_item.exception_type == gtfs::CalendarDateException::Added) {
          stop_pair->add_service_added_dates(d);
          had_added_date = true;
        } else
          stop_pair->add_service_except_dates(d);
      }

      // this shouldn't happen, but let's make sure it doesn't
      // in convert_transit we'll check if there was a valid date for this service and skip if not
      if (!service_dow->size() && !had_added_date) {
        LOG_WARN("Service ID " + currTrip->service_id +
                 " has no valid calendar or calendar_dates entry, skipping...");
        tile.mutable_stop_pairs()->RemoveLast();
        continue;
      }

      // test this, but careful, we might have to adjust the test's dist_shape_traveled or whatever
      // for the test shapes to be a bit more realistic with the actual map where it travels much
      // further than the GTFS objects indicate
      if (!currShape.empty()) {
        stop_pair->set_shape_id(pbf_shape_it->second);
      }

      stop_pair->set_service_start_date(to_local_pivot_sec(trip_calendar->start_date.get_raw_date()));
      // TODO: add a day worth of seconds - 1 to get the last second of that day
      stop_pair->set_service_end_date(
          to_local_pivot_sec(trip_calendar->end_date.get_raw_date(), true));

      dangles = dangles || !origin_is_in_tile || !dest_is_in_tile;
      stop_pair->set_bikes_allowed(currTrip->bikes_allowed == gtfs::TripAccess::Yes);

      if (currTrip->block_id != "") {
        uniques.lock.lock();
        auto inserted = uniques.block_ids.insert({currTrip->block_id, uniques.block_ids.size() + 1});
        stop_pair->set_block_id(inserted.first->second);
        uniques.lock.unlock();
      }

      stop_pair->set_origin_onestop_id(origin_onestop_id);
      stop_pair->set_destination_onestop_id(dest_onestop_id);

      stop_pair->set_destination_arrival_time(dest_stopTime.arrival_time.get_total_seconds());
      stop_pair->set_origin_departure_time(origin_stopTime.departure_time.get_total_seconds());

      // maybe set the dist_traveled
      if (const auto dist = get_stop_pair_dist(*origin_stop, currShape, origin_stopTime)) {
        stop_pair->set_origin_dist_traveled(dist);
      }
      if (const auto dist = get_stop_pair_dist(*dest_stop, currShape, dest_stopTime)) {
        stop_pair->set_destination_dist_traveled(dist);
      }

      if (origin_is_in_tile) {
        // So we looked up the node graphid by name, the name is either the actual name of the
        // platform (track 5 or something) OR its just the name of the station (in the case that the
        // platform is generated). So when its generated we will have gotten back the graphid for
        // the parent station, not for the platform, and the generated platform in that case will be
        // the next node after the station (we did this in write_stops); this also means that every
        // platform which is generated has one individual station
        stop_pair->set_origin_graphid(origin_graphid_it->second +
                                      static_cast<uint64_t>(origin_is_generated));
      }
      if (dest_is_in_tile) {
        // Same as above wrt to named and unnamed (generated) platforms
        stop_pair->set_destination_graphid(dest_graphid_it->second +
                                           static_cast<uint64_t>(dest_is_generated));
      }

      // set the proper route_index which will be referred to later in convert_transit
      stop_pair->set_route_index(routes_ids.at({currTrip->route_id, currFeedPath}));

      // grab the headsign
      stop_pair->set_trip_headsign(currTrip->trip_headsign);

      uniques.lock.lock();
      // trips should never have ID=0, it messes up the triplegbuilder logic
      auto inserted = uniques.trips.insert({currTrip->trip_id, uniques.trips.size() + 1});
      stop_pair->set_trip_id(inserted.first->second);
      uniques.lock.unlock();

      stop_pair->set_wheelchair_accessible(currTrip->wheelchair_accessible == gtfs::TripAccess::Yes);

      // get frequency info
      if (!feed.get_frequencies(currTrip->trip_id).empty()) {
        const auto& currFrequencies = feed.get_frequencies(currTrip->trip_id);
        if (currFrequencies.size() > 1) {
          // TODO(nils): this should be properly handled as 1 trip id can have
          // multiple frequencies, e.g. the example Google feed does
          LOG_WARN("More than one frequencies based schedule for " + currTrip->trip_id);
        }

        auto freq_start_time = (currFrequencies[0].start_time.get_raw_time());
        auto freq_end_time = (currFrequencies[0].end_time.get_raw_time());
        auto freq_time = freq_start_time + freq_end_time;

        // TODO: check which type of frequency it is, could be exact_time = true (meaning schedule
        // starts exactly
        //  at start_time and ends before end_time, one headway after the other) or schedule based
        //  (start_time is approximate, i.e. we don't know when the departure really is, only how long
        //  it'll take)

        if (currFrequencies.size() > 0) {
          stop_pair->set_frequency_end_time(DateTime::seconds_from_midnight(freq_end_time));
          stop_pair->set_frequency_headway_seconds(currFrequencies[0].headway_secs);
        }

        auto line_id = stop_pair->origin_onestop_id() < stop_pair->destination_onestop_id()
                           ? stop_pair->origin_onestop_id() + stop_pair->destination_onestop_id() +
                                 currTrip->route_id + freq_time
                           : stop_pair->destination_onestop_id() + stop_pair->origin_onestop_id() +
                                 currTrip->route_id + freq_time;
        uniques.lock.lock();
        uniques.lines.insert({line_id, uniques.lines.size()});
        uniques.lock.unlock();
      }
    }
  }

  return dangles;
}

// read routes data from feed
std::unordered_map<feed_object_t, size_t>
write_routes(Transit& tile, const tile_transit_info_t& tile_info, feed_cache_t& feeds) {

  const auto& tile_routeIds = tile_info.routes;

  std::unordered_map<feed_object_t, size_t> routes_ids;

  // loop through all stops inside the tile
  size_t idx = 0;
  for (const auto& feed_route : tile_routeIds) {
    const auto& tile_routeId = feed_route.first.id;
    const auto& feed = feeds(feed_route.first);
    auto* route = tile.add_routes();
    auto currRoute = feed.get_route(tile_routeId);

    route->set_name(currRoute->route_short_name);
    route->set_onestop_id(get_onestop_id_base(currRoute->route_id, feed_route.first.feed));
    route->set_operated_by_onestop_id(
        get_onestop_id_base(currRoute->agency_id, feed_route.first.feed));

    auto currAgency = feed.get_agency(currRoute->agency_id);
    route->set_operated_by_name(currAgency->agency_name);
    route->set_operated_by_website(currAgency->agency_url);
    // TODO(nils): add operated_by_onestop_id to the route, convert transit sets it and it's
    // used for filtering

    route->set_route_color(strtol(currRoute->route_color.c_str(), nullptr, 16));
    route->set_route_desc(currRoute->route_desc);
    route->set_route_long_name(currRoute->route_long_name);
    route->set_route_text_color(strtol(currRoute->route_text_color.c_str(), nullptr, 16));
    route->set_vehicle_type(
        (valhalla::mjolnir::Transit_VehicleType)(static_cast<int>(currRoute->route_type)));

    routes_ids.emplace(feed_route.first, idx);
    idx++;
  }

  return routes_ids;
}

// grab feed data from feed
void write_shapes(Transit& tile, const tile_transit_info_t& tile_info, feed_cache_t& feeds) {

  // loop through all shapes inside the tile
  for (const auto& feed_shape : tile_info.shapes) {
    const auto& tile_shape = feed_shape.first.id;
    const auto& feed = feeds(feed_shape.first);
    auto* shape = tile.add_shapes();
    const gtfs::Shape& currShape = feed.get_shape(tile_shape, true);
    shape->set_shape_id(feed_shape.second);
    std::vector<PointLL> trip_shape;
    for (const auto& shape_pt : currShape) {
      trip_shape.emplace_back(PointLL(shape_pt.shape_pt_lon, shape_pt.shape_pt_lat));
    }
    shape->set_encoded_shape(encode7(trip_shape));
  }
}

// pre-processes feed data and writes to the pbfs (calls the 'write' functions)
void ingest_tiles(const std::string& gtfs_dir,
                  const std::string& transit_dir,
                  const uint32_t pbf_trip_limit,
                  std::priority_queue<tile_transit_info_t>& queue,
                  unique_transit_t& uniques,
                  std::promise<std::list<GraphId>>& promise) {

  std::list<GraphId> dangling;

  while (true) {
    tile_transit_info_t current;
    uniques.lock.lock();
    if (queue.empty()) {
      uniques.lock.unlock();
      break;
    }
    current = queue.top();
    queue.pop();
    uniques.lock.unlock();

    Transit tile;
    bool dangles = false;
    uint16_t ext = 0;

    const auto tile_path = get_tile_path(transit_dir, current.graphid);
    auto current_path = tile_path;

    // collect all the feeds in this tile
    feed_cache_t feeds(gtfs_dir);
    for (const auto& route : current.routes) {
      feeds(route.first);
    }

    // keep track of the PBF insertion order for the routes to set route_index on the stop_pairs
    std::unordered_map<feed_object_t, size_t> routes_ids = write_routes(tile, current, feeds);
    write_shapes(tile, current, feeds);
    std::unordered_map<feed_object_t, GraphId> platform_node_ids = write_stops(tile, current, feeds);

    // keep the tile's nodes, they'll be cleared if we exceed the config's trip_limit
    const auto tile_nodes = tile.nodes();
    // we have to be careful with writing stop_pairs to not exceed PBF's stupid 2 GB limit
    size_t trip_count = 0;
    for (const auto& trip : current.trips) {
      trip_count++;

      dangles = write_stop_pair(tile, current, trip, feeds(trip), platform_node_ids, uniques,
                                tile_nodes, routes_ids) ||
                dangles;

      if (trip_count >= pbf_trip_limit) {
        LOG_INFO("Writing " + current_path);
        write_pbf(tile, current_path);
        tile.Clear();
        trip_count = 0;
        current_path = tile_path + "." + std::to_string(ext++);
      }
    }

    if (dangles) {
      dangling.emplace_back(current.graphid);
    }

    // write the last tile
    if (tile.stop_pairs_size()) {
      LOG_INFO("Writing " + current_path);
      write_pbf(tile, current_path);
    }
  }
  promise.set_value(dangling);
}

// connect the stop_pairs that span multiple tiles by processing dangling tiles
void stitch_tiles(const std::string& transit_dir,
                  const std::unordered_set<GraphId>& all_tiles,
                  std::list<GraphId>& tiles,
                  std::mutex& lock) {
  auto grid = TileHierarchy::GetTransitLevel().tiles;

  // for a missing stop_pair member's onestop_id we had to take the stop's gtfs stop_id
  // but generated nodes actually have their node_type appended on the onestop_id, so here we
  // need to remove the node_type before matching a candidate tile's nodes with the missing ones
  const auto stop_id_from_onestop_id = [](const auto& n_oid, NodeType node_type, bool is_generated) {
    return is_generated ? n_oid.substr(0, n_oid.length() - to_string(node_type).size()) : n_oid;
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

    auto tile_path = get_tile_path(transit_dir, current);
    auto current_path = tile_path;
    int ext = 0;

    do {

      // open tile make a hash of missing stop to invalid graphid

      // NOTE, that for missing origin/dest, we don't have any information right
      // now if it was generated or not. If it's missing, it's just the "base"
      // onestop ID and we also don't patch it here, since (for now) nodes' onestop
      // IDs aren't used anywhere after this code block
      auto tile = read_pbf(current_path);
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
      // the first one will have to be the same tile as current, so remove it
      unchecked.erase(unchecked.begin());
      size_t found = 0;
      while (found < needed.size() && unchecked.size()) {
        // crack it open to see if it has what we want
        auto neighbor_id = *unchecked.cbegin();
        unchecked.erase(unchecked.begin());
        auto neighbor_file_name = get_tile_path(transit_dir, neighbor_id);
        auto neighbor = read_pbf(neighbor_file_name, lock);
        for (const auto& node : neighbor.nodes()) {
          const auto node_type = static_cast<NodeType>(node.type());
          if (node_type != NodeType::kMultiUseTransitPlatform) {
            continue;
          }
          auto platform_itr =
              needed.find(stop_id_from_onestop_id(node.onestop_id(), node_type, node.generated()));
          if (platform_itr != needed.cend()) {
            platform_itr->second.value = node.graphid();
            ++found;
          }
        }
      }

      // get the ids fixed up and write pbf to file
      std::unordered_set<std::string> not_found;
      for (auto& stop_pair : *tile.mutable_stop_pairs()) {
        if (!stop_pair.has_origin_graphid()) {
          auto found_stop = needed.find(stop_pair.origin_onestop_id())->second;
          if (found_stop.Is_Valid()) {
            stop_pair.set_origin_graphid(found_stop);
          } else if (not_found.find(stop_pair.origin_onestop_id()) == not_found.cend()) {
            LOG_ERROR("Stop not found: " + stop_pair.origin_onestop_id());
            not_found.emplace(stop_pair.origin_onestop_id());
          }
          // else{ TODO: we could delete this stop pair }
        }
        if (!stop_pair.has_destination_graphid()) {
          auto found_stop = needed.find(stop_pair.destination_onestop_id())->second;
          if (found_stop.Is_Valid()) {
            stop_pair.set_destination_graphid(found_stop);
          } else if (not_found.find(stop_pair.destination_onestop_id()) == not_found.cend()) {
            LOG_ERROR("Stop not found: " + stop_pair.destination_onestop_id());
            not_found.emplace(stop_pair.destination_onestop_id());
          }
          // else{ TODO: we could delete this stop pair }
        }
      }
      lock.lock();
      write_pbf(tile, current_path);
      lock.unlock();
      LOG_INFO(current_path + " stitched " + std::to_string(found) + " of " +
               std::to_string(needed.size()) + " stops");

      current_path = tile_path + "." + std::to_string(ext++);
    } while (filesystem::exists(current_path));
  }
}
} // namespace

namespace valhalla {
namespace mjolnir {

// thread and call ingest_tiles
std::list<GraphId> ingest_transit(const boost::property_tree::ptree& pt) {
  // remove transit directory if it exists
  std::string transit_dir = pt.get<std::string>("mjolnir.transit_dir");
  if (transit_dir.back() != filesystem::path::preferred_separator) {
    transit_dir.push_back(filesystem::path::preferred_separator);
  }
  if (filesystem::exists(transit_dir) && !filesystem::is_empty(transit_dir)) {
    LOG_WARN("Non-empty " + transit_dir + " will be purged of tiles");
    filesystem::remove_all(transit_dir);
  }

  std::string gtfs_dir = pt.get<std::string>("mjolnir.transit_feeds_dir");
  if (gtfs_dir.back() != filesystem::path::preferred_separator) {
    gtfs_dir.push_back(filesystem::path::preferred_separator);
  }

  auto thread_count =
      pt.get<unsigned int>("mjolnir.concurrency", std::max(static_cast<unsigned int>(1),
                                                           std::thread::hardware_concurrency()));
  // go get information about what transit tiles we should be fetching
  LOG_INFO("Tiling GTFS Feeds");
  auto tiles = select_transit_tiles(gtfs_dir);

  LOG_INFO("Writing " + std::to_string(tiles.size()) + " transit pbf tiles with " +
           std::to_string(thread_count) + " threads...");

  // schedule some work
  unique_transit_t uniques;
  std::vector<std::shared_ptr<std::thread>> threads(thread_count);
  std::vector<std::promise<std::list<GraphId>>> promises(threads.size());

  auto pbf_trip_limit = pt.get<uint32_t>("mjolnir.transit_pbf_limit");

  for (size_t i = 0; i < threads.size(); ++i) {
    threads[i].reset(new std::thread(ingest_tiles, std::cref(gtfs_dir), std::cref(transit_dir),
                                     pbf_trip_limit, std::ref(tiles), std::ref(uniques),
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
  auto transit_dir = pt.get<std::string>("mjolnir.transit_dir");
  if (transit_dir.back() != filesystem::path::preferred_separator) {
    transit_dir.push_back(filesystem::path::preferred_separator);
  }
  filesystem::recursive_directory_iterator transit_file_itr(
      transit_dir + std::to_string(TileHierarchy::GetTransitLevel().level));
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

  // make let them rip
  std::mutex lock;
  for (size_t i = 0; i < threads.size(); ++i) {
    threads[i].reset(new std::thread(stitch_tiles, std::cref(transit_dir), std::cref(all_tiles),
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
    lock.unlock();
    throw std::runtime_error("Couldn't load " + file_name);
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

} // namespace mjolnir
} // namespace valhalla