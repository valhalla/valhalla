#include "gurka.h"
#include "just_gtfs/just_gtfs.h"

#include "mjolnir/convert_transit.h"
#include "mjolnir/ingest_transit.h"
#include "proto/transit.pb.h"
#include "test.h"
#include <gtest/gtest.h>

using namespace gtfs;
using namespace valhalla;

// static ids of different feed items, shared amongst tests, we could do more
const std::string tripOneID = "trip_one";
const std::string tripTwoID = "trip_two";
const std::string stopOneID = "stop_one";
const std::string stopTwoID = "stop_two";
const std::string stopThreeID = "stop_three";
const std::string shapeOneID = "shape_one";
const std::string serviceOneID = "service_one";
const std::string routeID = "route_one";
const std::string blockID = "block_one";
const int headwaySec = 1800;

// the transit shape uses * as separators rather than -/\|

// each station is a number, 1 2 or 3. inside of each station located at the same spot we have
// an in/egress which connections to a station which connects to a platform
// from the platform we should have a connection to another platform at the next station
// it should look like:
//
// street node --> transit connect edge --> egress node --> egress connect edge --> station node -->
// platform connection edge --> platform node --> rail/bus --> platform node (at another station) -->
//
// from here down to station node and out through egress node to the street OR
// keep going on rail/bus/ferry edges to the next platform OR
// if you need to make a transfer you might go down to the station node and back up to an adjacent
// platform at the same station. we should test all of this stuff eventually

// loki::Search won't find osm <-> transit connection edges, so pad with extra edge on each end

// the map should be big enough so that there's one stop_pair with stops in different tiles.
const std::string ascii_map = R"(
        a******************************b
        *                              *
 A---B--1---------------------------C--2---D
                                       *   |
                                       *   |
                                       *   |
                                       *   |
                                       *   |
                                       *   |
                                       *   |
                                       *   |
                                       c***3
                                           |
                                           E
                                           |
                                           F
    )";

// TODO: cant get higher road classes to allow egress/ingress connections, no ped access?
const gurka::ways ways = {
    {"AB", {{"highway", "residential"}}}, {"BC", {{"highway", "residential"}}},
    {"CD", {{"highway", "residential"}}}, {"DE", {{"highway", "residential"}}},
    {"EF", {{"highway", "residential"}}},
};

boost::property_tree::ptree get_config() {

  return test::make_config(VALHALLA_BUILD_DIR "test/data/transit_tests",
                           {{"mjolnir.transit_feeds_dir",
                             VALHALLA_BUILD_DIR "test/data/transit_tests/gtfs_feeds"},
                            {"mjolnir.transit_dir",
                             VALHALLA_BUILD_DIR "test/data/transit_tests/transit_tiles"},
                            {"mjolnir.transit_pbf_limit",
                             "1"}, // so we create more than one file per tile
                            {"mjolnir.timezone", VALHALLA_BUILD_DIR "test/data/tz.sqlite"},
                            {"mjolnir.tile_dir", VALHALLA_BUILD_DIR "test/data/transit_tests/tiles"},
                            // TODO: fix hierarchy builder transit support
                            {"mjolnir.hierarchy", "false"},
                            {"service_limits.pedestrian.max_transit_walking_distance", "100000"}});
}

// put the base in toronto for timezone stuff to work and put it on the edge of a level 2/3 tile
valhalla::gurka::nodelayout create_layout() {
  int gridsize_metres = 1000;
  auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {-79.249, 43.749});

  return layout;
}
gurka::map map;

// test to write gtfs files
TEST(GtfsExample, WriteGtfs) {
  auto pt = get_config();
  filesystem::remove_all(pt.get<std::string>("mjolnir.tile_dir"));
  filesystem::remove_all(pt.get<std::string>("mjolnir.transit_feeds_dir"));
  filesystem::remove_all(pt.get<std::string>("mjolnir.transit_dir"));
  filesystem::create_directories(pt.get<std::string>("mjolnir.tile_dir"));
  filesystem::create_directories(pt.get<std::string>("mjolnir.transit_feeds_dir"));
  filesystem::create_directories(pt.get<std::string>("mjolnir.transit_dir"));

  auto layout = create_layout();
  auto station_one_ll = layout.find("1");
  auto station_two_ll = layout.find("2");
  auto station_three_ll = layout.find("3");

  Feed feed;

  std::string path_directory = pt.get<std::string>("mjolnir.transit_feeds_dir") +
                               filesystem::path::preferred_separator + "toronto";
  filesystem::create_directories(path_directory);

  // write agency.txt
  struct Agency ttc {
    .agency_id = "TTC", .agency_name = "Toronto Commission", .agency_url = "http://www.ttc.ca",
    .agency_timezone = "America/Toronto"
  };
  feed.add_agency(ttc);

  feed.write_agencies(path_directory);

  // write stops.txt:
  // 1st has all stop objects, egress/station/platform
  // 2nd has only platform, station
  // 3rd has only platform
  struct gtfs::Stop first_stop_egress {
    .stop_id = stopOneID + "_rotating_door_eh", .stop_name = gtfs::Text("POINT NEMO"),
    .coordinates_present = true, .stop_lat = station_one_ll->second.second,
    .stop_lon = station_one_ll->second.first, .parent_station = stopOneID,
    .location_type = gtfs::StopLocationType::EntranceExit, .wheelchair_boarding = "1",
  };
  feed.add_stop(first_stop_egress);
  struct gtfs::Stop first_stop_station {
    .stop_id = stopOneID, .stop_name = gtfs::Text("POINT NEMO"), .coordinates_present = true,
    .stop_lat = station_one_ll->second.second, .stop_lon = station_one_ll->second.first,
    .parent_station = "", .location_type = gtfs::StopLocationType::Station,
    .wheelchair_boarding = "1",
  };
  feed.add_stop(first_stop_station);
  struct gtfs::Stop first_stop_platform {
    .stop_id = stopOneID + "_ledge_to_the_train_bucko", .stop_name = gtfs::Text("POINT NEMO"),
    .coordinates_present = true, .stop_lat = station_one_ll->second.second,
    .stop_lon = station_one_ll->second.first, .parent_station = stopOneID,
    .location_type = gtfs::StopLocationType::StopOrPlatform, .wheelchair_boarding = "1",
  };
  feed.add_stop(first_stop_platform);

  struct gtfs::Stop second_stop_station {
    .stop_id = stopTwoID, .stop_name = gtfs::Text("SECOND STATION"), .coordinates_present = true,
    .stop_lat = station_two_ll->second.second, .stop_lon = station_two_ll->second.first,
    .parent_station = "", .location_type = gtfs::StopLocationType::Station,
    .stop_timezone = "America/Toronto", .wheelchair_boarding = "1",
  };
  feed.add_stop(second_stop_station);
  struct gtfs::Stop second_stop_platform {
    .stop_id = stopTwoID + "_platform", .stop_name = gtfs::Text("SECOND STOP"),
    .coordinates_present = true, .stop_lat = station_two_ll->second.second,
    .stop_lon = station_two_ll->second.first, .parent_station = stopTwoID,
    .location_type = gtfs::StopLocationType::StopOrPlatform, .stop_timezone = "America/Toronto",
    .wheelchair_boarding = "1",
  };
  feed.add_stop(second_stop_platform);

  struct gtfs::Stop third_stop_platform {
    .stop_id = stopThreeID, .stop_name = gtfs::Text("THIRD STOP"), .coordinates_present = true,
    .stop_lat = station_three_ll->second.second, .stop_lon = station_three_ll->second.first,
    .parent_station = "", .location_type = gtfs::StopLocationType::StopOrPlatform,
    .stop_timezone = "America/Toronto", .wheelchair_boarding = "1",
  };
  feed.add_stop(third_stop_platform);

  feed.write_stops(path_directory);

  // write routes.txt
  struct Route lineOne {
    .route_id = routeID, .route_type = RouteType::Subway, .agency_id = "TTC",
    .route_short_name = "ba", .route_long_name = "bababa", .route_desc = "this is the first route",
    .route_color = "ff0000", .route_text_color = "00ff00"
  };
  feed.add_route(lineOne);

  feed.write_routes(path_directory);

  // write trips.txt
  struct gtfs::Trip tripOne {
    .route_id = routeID, .service_id = serviceOneID, .trip_id = tripOneID, .trip_headsign = "hello",
    .block_id = blockID, .shape_id = shapeOneID, .wheelchair_accessible = gtfs::TripAccess::Yes,
    .bikes_allowed = gtfs::TripAccess::No,
  };
  feed.add_trip(tripOne);

  struct gtfs::Trip tripTwo {
    .route_id = routeID, .service_id = serviceOneID, .trip_id = tripTwoID, .trip_headsign = "bonjour",
    .block_id = blockID, .wheelchair_accessible = gtfs::TripAccess::Yes,
    .bikes_allowed = gtfs::TripAccess::No,
  };
  feed.add_trip(tripTwo);
  feed.write_trips(path_directory);

  // write stop_times.txt
  struct StopTime trip_one_stop_one {
    .trip_id = tripOneID, .stop_id = stopOneID + "_ledge_to_the_train_bucko", .stop_sequence = 0,
    .arrival_time = Time("6:00:00"), .departure_time = Time("6:00:00"), .stop_headsign = "head",
    .shape_dist_traveled = 0.0, .timepoint = gtfs::StopTimePoint::Exact,
  };
  feed.add_stop_time(trip_one_stop_one);

  struct StopTime trip_one_stop_two {
    .trip_id = tripOneID, .stop_id = stopTwoID + "_platform", .stop_sequence = 1,
    .arrival_time = Time("6:03:00"), .departure_time = Time("6:03:00"), .stop_headsign = "head",
    .shape_dist_traveled = 3.0, .timepoint = gtfs::StopTimePoint::Exact,
  };
  feed.add_stop_time(trip_one_stop_two);

  struct StopTime trip_one_stop_three {
    .trip_id = tripOneID, .stop_id = stopThreeID, .stop_sequence = 2, .arrival_time = Time("6:06:00"),
    .departure_time = Time("6:06:00"), .stop_headsign = "head", .shape_dist_traveled = 6.0,
    .timepoint = gtfs::StopTimePoint::Exact,
  };
  feed.add_stop_time(trip_one_stop_three);

  struct StopTime trip_two_stop_one {
    .trip_id = tripTwoID, .stop_id = stopOneID + "_ledge_to_the_train_bucko", .stop_sequence = 0,
    .arrival_time = Time("10:00:00"), .departure_time = Time("10:00:00"), .stop_headsign = "head",
    .timepoint = gtfs::StopTimePoint::Exact,
  };
  feed.add_stop_time(trip_two_stop_one);

  struct StopTime trip_two_stop_two {
    .trip_id = tripTwoID, .stop_id = stopTwoID + "_platform", .stop_sequence = 1,
    .arrival_time = Time("10:03:00"), .departure_time = Time("10:03:00"), .stop_headsign = "head",
    .timepoint = gtfs::StopTimePoint::Exact,
  };
  feed.add_stop_time(trip_two_stop_two);

  struct StopTime trip_two_stop_three {
    .trip_id = tripTwoID, .stop_id = stopThreeID, .stop_sequence = 2,
    .arrival_time = Time("10:06:00"), .departure_time = Time("10:06:00"), .stop_headsign = "head",
    .timepoint = gtfs::StopTimePoint::Exact,
  };
  feed.add_stop_time(trip_two_stop_three);

  feed.write_stop_times(path_directory);

  // write calendar.txt
  struct CalendarItem calendarOne {
    .service_id = serviceOneID, .monday = CalendarAvailability::Available,
    .tuesday = CalendarAvailability::Available, .wednesday = CalendarAvailability::Available,
    .thursday = CalendarAvailability::Available, .friday = CalendarAvailability::Available,
    .saturday = CalendarAvailability::Available, .sunday = CalendarAvailability::Available,
    .start_date = Date(2023, 1, 31), .end_date = Date(2024, 1, 31),
  };
  feed.add_calendar_item(calendarOne);

  feed.write_calendar(path_directory);

  // write calendar_dates.txt
  struct CalendarDate servAdded {
    .service_id = serviceOneID, .date = Date(2023, 2, 2),
    .exception_type = gtfs::CalendarDateException::Added,
  };
  struct CalendarDate servRemoved {
    .service_id = serviceOneID, .date = Date(2023, 2, 3),
    .exception_type = gtfs::CalendarDateException::Removed,
  };

  feed.add_calendar_date(servAdded);
  feed.add_calendar_date(servRemoved);

  feed.write_calendar_dates(path_directory);

  // write shapes.txt
  // TODO: write shapes before stop_times so that we can determine the correct travelled distance
  for (const auto& shape_ll : std::vector<PointLL>{layout["1"], layout["a"], layout["b"], layout["2"],
                                                   layout["c"], layout["3"]}) {
    feed.add_shape(ShapePoint{.shape_id = shapeOneID,
                              .shape_pt_lat = shape_ll.second,
                              .shape_pt_lon = shape_ll.first,
                              .shape_pt_sequence = feed.get_shapes().size()});
  }
  feed.write_shapes(path_directory);

  // write frequencies.txt
  struct Frequency freqBased {
    .trip_id = tripOneID, .start_time = Time(6, 0, 0), .end_time = Time(22, 0, 0),
    .headway_secs = headwaySec, .exact_times = gtfs::FrequencyTripService::FrequencyBased,
  };
  struct Frequency schedBased {
    .trip_id = tripTwoID, .start_time = Time(10, 0, 0), .end_time = Time(22, 0, 0),
    .headway_secs = headwaySec, .exact_times = gtfs::FrequencyTripService::ScheduleBased,
  };

  feed.add_frequency(freqBased);
  feed.add_frequency(schedBased);
  feed.write_frequencies(path_directory);

  Feed feed_reader(path_directory);
  feed_reader.read_feed();

  // make sure files are actually written

  const auto& trips = feed_reader.get_trips();
  EXPECT_EQ(trips.size(), 2);
  EXPECT_EQ(trips[0].trip_id, tripOneID);

  const auto& stops = feed_reader.get_stops();
  EXPECT_EQ(stops.size(), 6);
  EXPECT_EQ(stops[0].stop_id, stopOneID + "_rotating_door_eh");

  const auto& shapes = feed_reader.get_shapes();
  EXPECT_EQ(shapes.size(), 6);
  EXPECT_EQ(shapes[0].shape_id, shapeOneID);

  const auto& calendarGTFS = feed_reader.get_calendar();
  EXPECT_EQ(calendarGTFS.size(), 1);
  EXPECT_EQ(calendarGTFS[0].service_id, serviceOneID);

  const auto& calendarExceptions = feed_reader.get_calendar_dates();
  EXPECT_EQ(calendarExceptions.size(), 2);
  EXPECT_EQ(calendarExceptions[0].exception_type,
            gtfs::CalendarDateException::Added); // service added
  EXPECT_EQ(calendarExceptions[1].exception_type,
            gtfs::CalendarDateException::Removed); // service added

  const auto& frequencies = feed_reader.get_frequencies();
  EXPECT_EQ(frequencies.size(), 2);
  EXPECT_EQ(frequencies[0].exact_times, gtfs::FrequencyTripService::FrequencyBased);
  EXPECT_EQ(frequencies[1].exact_times, gtfs::FrequencyTripService::ScheduleBased);
}

TEST(GtfsExample, MakeProto) {
  auto pt = get_config();

  // constants written in the last function
  auto serviceStartDate =
      baldr::DateTime::get_formatted_date("2023-01-31").time_since_epoch().count();
  auto serviceEndDate = baldr::DateTime::get_formatted_date("2024-01-31").time_since_epoch().count();
  auto addedDate = baldr::DateTime::get_formatted_date("2023-02-02").time_since_epoch().count();
  auto removedDate = baldr::DateTime::get_formatted_date("2023-02-03").time_since_epoch().count();

  // spawn threads to download all the tiles returning a list of
  // tiles that ended up having dangling stop pairs
  auto dangling_tiles = valhalla::mjolnir::ingest_transit(pt);

  // spawn threads to connect dangling stop pairs to adjacent tiles' stops
  valhalla::mjolnir::stitch_transit(pt, dangling_tiles);
  // call the two functions, in main valhalla_ingest-transit it's gonna write protobufs
  filesystem::recursive_directory_iterator transit_file_itr(
      pt.get<std::string>("mjolnir.transit_dir"));
  filesystem::recursive_directory_iterator end_file_itr;

  std::unordered_set<std::string> stops;
  std::unordered_set<std::string> stop_pairs;
  std::unordered_set<std::string> routes;
  std::unordered_set<uint32_t> service_start_dates;
  std::unordered_set<uint32_t> service_end_dates;
  std::unordered_set<uint32_t> service_added_dates;
  std::unordered_set<uint32_t> service_except_dates;
  std::unordered_set<uint32_t> headway_seconds;

  std::vector<float> first_origin_dist_traveled;
  std::vector<float> last_dest_dist_traveled;

  // get the full shape length
  float shape_length = 0;

  auto layout = create_layout();
  const std::vector<PointLL> transit_lls{layout["1"], layout["a"], layout["b"],
                                         layout["2"], layout["c"], layout["3"]};
  for (uint32_t segment = 0; segment < transit_lls.size() - 1; segment++) {
    auto currOrigin = transit_lls[segment];
    auto currDest = transit_lls[segment + 1];
    shape_length += currOrigin.Distance(currDest);
  }

  size_t shapes = 0;
  // for each pbf.
  for (; transit_file_itr != end_file_itr; ++transit_file_itr) {
    if (filesystem::is_regular_file(transit_file_itr->path())) {
      std::string fname = transit_file_itr->path().string();
      mjolnir::Transit transit = mjolnir::read_pbf(fname);

      if (std::isdigit(fname.back())) {
        // we produce 2 pbf tiles on purpose, where the last one (xx.pbf.0) only has a bunch of stop
        // pairs
        EXPECT_NE(transit.stop_pairs_size(), 0);
      }

      // make sure we are looking at a pbf file

      // make sure that the data written in the previous test is readable through pbfs
      // shape info
      // becomes 1 shape from 4 shape_points last test

      shapes += transit.shapes().size();
      // stop(node) info
      for (const auto& node : transit.nodes()) {
        stops.insert(node.onestop_id());
        if (node.type() == static_cast<uint32_t>(NodeType::kTransitEgress)) {
          EXPECT_NE(node.traversability(), static_cast<uint32_t>(Traversability::kNone));
        }
      }

      // routes info
      for (const auto& route : transit.routes()) {
        routes.insert(route.onestop_id());
      }

      // stop_pair info
      for (int i = 0; i < transit.stop_pairs_size(); i++) {
        EXPECT_TRUE(transit.stop_pairs(i).has_origin_graphid());
        EXPECT_TRUE(transit.stop_pairs(i).has_destination_graphid());
      }

      // calendar information
      for (const auto& stop_pair : transit.stop_pairs()) {
        EXPECT_TRUE(stop_pair.has_origin_graphid());
        EXPECT_TRUE(stop_pair.has_destination_graphid());
        stop_pairs.insert(stop_pair.origin_onestop_id());
        stop_pairs.insert(stop_pair.destination_onestop_id());
        service_start_dates.insert(stop_pair.service_start_date());
        service_end_dates.insert(stop_pair.service_end_date());
        headway_seconds.insert(stop_pair.frequency_headway_seconds());
        for (const auto& added_date : stop_pair.service_added_dates()) {
          service_added_dates.insert(added_date);
        }
        for (const auto& except_date : stop_pair.service_except_dates()) {
          service_except_dates.insert(except_date);
        }

        // make sure:
        //   - the first stop pair has 0 as origin_dist_traveled
        //   - the last stop pair of tripOne has exactly 6.0f and tripTwo has the full shape length
        if (stop_pair.origin_onestop_id() == "toronto_" + stopOneID + "_ledge_to_the_train_bucko") {
          first_origin_dist_traveled.push_back(stop_pair.origin_dist_traveled());
        } else if (stop_pair.destination_onestop_id() == "toronto_" + stopThreeID) {
          last_dest_dist_traveled.push_back(stop_pair.destination_dist_traveled());
        }
      }
    }
  }

  EXPECT_EQ(shapes, 2);

  // routes
  EXPECT_TRUE(routes.find("toronto_" + routeID) != routes.end());
  EXPECT_EQ(routes.size(), 1);

  // stops
  std::string stopIds[3] = {stopOneID, stopTwoID, stopThreeID};
  EXPECT_EQ(stops.size(), 9);
  for (const auto& stopID : stopIds) {
    EXPECT_TRUE(stops.find("toronto_" + stopID) != stops.end());
  }

  // stop_pairs, we have 3 in total since one stop_pair is across 2 tiles
  std::string stop_pair_ids[3] = {
      stopOneID + "_ledge_to_the_train_bucko",
      stopTwoID,
      stopThreeID,
  };
  EXPECT_EQ(stop_pairs.size(), 3);
  for (const auto& stop_pair_id : stop_pair_ids) {
    EXPECT_TRUE(stops.find("toronto_" + stop_pair_id) != stops.end());
  }
  // we have 4 here since we have the first stop_pair in two tiles on two trips
  EXPECT_EQ(first_origin_dist_traveled.size(), 4);
  for (const auto& dist : first_origin_dist_traveled) {
    EXPECT_EQ(dist, 0.f);
  }
  EXPECT_EQ(last_dest_dist_traveled.size(), 2);
  // Only one shape was present, for the other one we have to generate
  // the edge's shapes from the the stop locations in convert_transit,
  // in the pbf the dist_traveled will be empty for that one though
  EXPECT_TRUE(std::find(last_dest_dist_traveled.begin(), last_dest_dist_traveled.end(), 6.0f) !=
              last_dest_dist_traveled.end());
  EXPECT_TRUE(std::find(last_dest_dist_traveled.begin(), last_dest_dist_traveled.end(), 0.0f) !=
              last_dest_dist_traveled.end());

  // service
  EXPECT_EQ(service_start_dates.size(), 1);
  EXPECT_EQ(*service_start_dates.begin(), serviceStartDate);

  EXPECT_EQ(service_end_dates.size(), 1);
  EXPECT_EQ(*service_end_dates.begin(), serviceEndDate);

  EXPECT_EQ(service_added_dates.size(), 1);
  EXPECT_EQ(*service_added_dates.begin(), addedDate);

  EXPECT_EQ(service_except_dates.size(), 1);
  EXPECT_EQ(*service_except_dates.begin(), removedDate);

  EXPECT_EQ(headway_seconds.size(), 1);
  EXPECT_EQ(*headway_seconds.begin(), headwaySec);
}

TEST(GtfsExample, MakeTile) {
  boost::property_tree::ptree pt = get_config();

  auto layout = create_layout();
  auto station_one_ll = layout.find("1");
  auto station_two_ll = layout.find("2");
  auto station_three_ll = layout.find("3");

  auto dt = "2023-03-27T05:50";
  auto dt_date = DateTime::get_formatted_date(dt);
  auto dt_date_days = DateTime::days_from_pivot_date(dt_date);
  auto dt_dow = DateTime::day_of_week_mask(dt);
  bool date_before_tile = false;

  // this creates routable transit tiles but doesnt connect them to the rest of the graph
  auto all_tiles = valhalla::mjolnir::convert_transit(pt);

  // now we have to build the tiles again to get the transit tiles connected to the regular graph
  map = gurka::buildtiles(layout, ways, {}, {}, pt);

  // files are already going to be written from
  filesystem::recursive_directory_iterator transit_file_itr(pt.get<std::string>("mjolnir.tile_dir"));
  filesystem::recursive_directory_iterator end_file_itr;

  GraphReader reader(pt.get_child("mjolnir"));
  auto tileids = reader.GetTileSet();
  size_t transit_nodes = 0;
  std::unordered_map<baldr::Use, size_t> uses;
  for (auto tileid : tileids) {
    LOG_INFO("Working on : " + std::to_string(tileid));
    graph_tile_ptr tile = reader.GetGraphTile(tileid);

    uint32_t date_created = tile->header()->date_created();
    uint32_t dt_day;
    if (dt_date_days < date_created) {
      date_before_tile = true;
    } else {
      dt_day = dt_date_days - date_created;
    }
    for (const auto& edge : tile->GetDirectedEdges()) {
      uses[edge.use()]++;
      if (edge.use() == Use::kTransitConnection) {
        EXPECT_TRUE((edge.forwardaccess() & kPedestrianAccess) ||
                    (edge.reverseaccess() & kPedestrianAccess));
      } else if (edge.use() == Use::kRail || edge.use() == Use::kBus) {
        const valhalla::baldr::TransitDeparture* dep =
            tile->GetNextDeparture(edge.lineid(), 21600, // 06:00 am
                                   dt_day, dt_dow, date_before_tile, false, false);
        EXPECT_NE(dep->elapsed_time(), 0);
        const auto shape = tile->edgeinfo(&edge).encoded_shape();
        EXPECT_FALSE(shape.empty());
      }
    }

    if (tileid.level() != TileHierarchy::GetTransitLevel().level) {
      continue;
    }

    if (tile->GetStopOneStops().empty()) {
      LOG_ERROR("NO one stops found");
    }
    auto tileStops = tile->GetStopOneStops();
    if (tile->GetNodes().size() == 0) {
      LOG_WARN("There are no nodes inside tile " + std::to_string(tileid));
    } else {
      LOG_INFO("There are " + std::to_string(tile->GetNodes().size()) + " nodes inside tile " +
               std::to_string(tileid));
    }
    transit_nodes += tile->header()->nodecount();
    std::unordered_set<NodeType> node_types = {NodeType::kMultiUseTransitPlatform,
                                               NodeType::kTransitStation, NodeType::kTransitEgress};
    std::vector<PointLL> station_coords = {station_one_ll->second, station_two_ll->second,
                                           station_three_ll->second};
    for (const auto& node : tile->GetNodes()) {
      auto currNode_type = node_types.find(node.type());
      EXPECT_NE(node_types.end(), currNode_type);
      auto node_ll = node.latlng(tile->header()->base_ll());
      // TODO: check which tile, match station coords accordingly
      uint64_t node_way_id = node.connecting_wayid();
      LOG_INFO("Current Way Id: " + std::to_string(node_way_id));
      bool coordinates_found = false;
      for (const auto& station_coord : station_coords) {
        coordinates_found = coordinates_found || node_ll.ApproximatelyEqual(station_coord);
      }
      EXPECT_TRUE(coordinates_found);
      // some GTFS stops don't define a timezone, but we fall back to our own
      EXPECT_EQ(node.timezone(), 145); // America/Toronto
    }

    std::unordered_set<std::string> stopIds = {stopOneID, stopTwoID, stopThreeID};

    auto currRoute = tile->GetTransitRoute(0);
    EXPECT_EQ(tile->GetName(currRoute->one_stop_offset()), "toronto_" + routeID);
    EXPECT_EQ(currRoute->route_text_color(), strtol("00ff00", nullptr, 16));
    EXPECT_EQ(currRoute->route_color(), strtol("ff0000", nullptr, 16));
    EXPECT_EQ(tile->GetName(currRoute->op_by_onestop_id_offset()), "toronto_TTC");

    std::unordered_set<std::string> tripIDs = {tripOneID, tripTwoID};

    for (const auto& departure : tile->GetTransitDepartures()) {
      TransitDeparture* currDeparture = departure.second;
      auto transit_route = tile->GetTransitRoute(currDeparture->routeindex());
      EXPECT_EQ(tile->GetName(transit_route->one_stop_offset()), "toronto_" + routeID);
    }

    for (uint32_t schedule_it = 0; schedule_it < tile->header()->schedulecount(); schedule_it++) {
      auto* tileSchedule = tile->GetTransitSchedule(schedule_it);
      EXPECT_EQ(tileSchedule->days(), (static_cast<int64_t>(1) << 60) - 1);
      EXPECT_EQ(tileSchedule->end_day(), 59);
    }
  }

  EXPECT_EQ(transit_nodes, 9);
  EXPECT_EQ(uses[Use::kRoad], 10);
  // NOTE: there are 4 for every station (3 of those) because we connect to both ends of the closest
  // edge to the station and the connections are bidirectional (as per usual), plus some more because
  // the second platform has no parent
  EXPECT_EQ(uses[Use::kTransitConnection], 10);
  EXPECT_EQ(uses[Use::kPlatformConnection], 6);
  EXPECT_EQ(uses[Use::kEgressConnection], 6);
  // TODO: this is the only time in the graph that we dont have opposing directed edges (should fix)
  EXPECT_EQ(uses[Use::kRail], 2);
}

TEST(GtfsExample, route) {
  valhalla::Api result0 =
      gurka::do_action(valhalla::Options::route, map, {"A", "F"}, "multimodal",
                       {{"/date_time/type", "1"},
                        {"/date_time/value", "2023-02-27T05:50"},
                        {"/costing_options/pedestrian/transit_start_end_max_distance", "20000"}});
  EXPECT_EQ(result0.trip().routes_size(), 1);
}

TEST(GtfsExample, isochrones) {
  std::string res_string;
  valhalla::Api res =
      gurka::do_action(valhalla::Options::isochrone, map, {"C"}, "multimodal",
                       {{"/date_time/type", "1"},
                        {"/date_time/value", "2023-02-27T05:58"},
                        {"/contours/0/time", "20"},
                        {"/costing_options/pedestrian/transit_start_end_max_distance", "20000"}},
                       {}, &res_string);

  rapidjson::Document doc;
  doc.Parse(res_string.c_str());
  // TODO: some more testing of this similar to the isochrone.cc test: dump the polygon
  // to geos and check if at least the stops are inside. Mid-future: play a bit more
  // with schedules to see it's doing the right thing
  EXPECT_TRUE(doc.HasMember("features"));
}
