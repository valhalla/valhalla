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
const std::string ascii_map = R"(
        a**************b
        *              *
     A--1--------------2--B
    )";

// TODO: cant get higher road classes to allow egress/ingress connections, no ped access?
const gurka::ways ways = {{"AB", {{"highway", "residential"}}}};

boost::property_tree::ptree get_config() {

  return test::make_config(VALHALLA_BUILD_DIR "test/data/transit_tests",
                           {
                               {"mjolnir.transit_feeds_dir",
                                VALHALLA_BUILD_DIR "test/data/transit_tests_temp/gtfs_feeds"},
                               {"mjolnir.transit_dir",
                                VALHALLA_BUILD_DIR "test/data/transit_tests_temp/transit_tiles"},
                               {"mjolnir.transit_pbf_limit", "1"},
                               {"mjolnir.timezone", VALHALLA_BUILD_DIR "test/data/tz.sqlite"},
                               {"mjolnir.tile_dir",
                                VALHALLA_BUILD_DIR "test/data/transit_tests_temp/tiles"},
                               // TODO: fix hierarchy builder transit support
                               {"mjolnir.hierarchy", "false"},
                           });
}

// put the base somewhere in toronto for timezone stuff to work
valhalla::gurka::nodelayout create_layout() {
  int gridsize_metres = 100;
  auto layout =
      gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {-79.401798, 43.679187});

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

  // write stops.txt
  struct gtfs::Stop first_stop_egress {
    .stop_id = stopOneID + "_rotating_door_eh", .stop_name = gtfs::Text("POINT NEMO"),
    .coordinates_present = true, .stop_lat = station_one_ll->second.second,
    .stop_lon = station_one_ll->second.first, .parent_station = stopOneID,
    .location_type = gtfs::StopLocationType::EntranceExit, .stop_timezone = "America/Toronto",
    .wheelchair_boarding = "1",
  };
  feed.add_stop(first_stop_egress);
  struct gtfs::Stop first_stop_platform {
    .stop_id = stopOneID + "_ledge_to_the_train_bucko", .stop_name = gtfs::Text("POINT NEMO"),
    .coordinates_present = true, .stop_lat = station_one_ll->second.second,
    .stop_lon = station_one_ll->second.first, .parent_station = stopOneID,
    .location_type = gtfs::StopLocationType::StopOrPlatform, .stop_timezone = "America/Toronto",
    .wheelchair_boarding = "1",
  };
  feed.add_stop(first_stop_platform);
  struct gtfs::Stop first_stop_station {
    .stop_id = stopOneID, .stop_name = gtfs::Text("POINT NEMO"), .coordinates_present = true,
    .stop_lat = station_one_ll->second.second, .stop_lon = station_one_ll->second.first,
    .parent_station = "", .location_type = gtfs::StopLocationType::Station,
    .stop_timezone = "America/Toronto", .wheelchair_boarding = "1",
  };
  feed.add_stop(first_stop_station);
  struct gtfs::Stop second_stop_station {
    .stop_id = stopTwoID, .stop_name = gtfs::Text("SECOND STOP"), .coordinates_present = true,
    .stop_lat = station_two_ll->second.second, .stop_lon = station_two_ll->second.first,
    .parent_station = "", .location_type = gtfs::StopLocationType::Station,
    .stop_timezone = "America/Toronto", .wheelchair_boarding = "1",
  };
  feed.add_stop(second_stop_station);
  feed.write_stops(path_directory);

  // write routes.txt
  struct Route lineOne {
    .route_id = routeID, .route_type = RouteType::Rail, .agency_id = "TTC", .route_short_name = "ba",
    .route_long_name = "bababa", .route_desc = "this is the first route", .route_color = "ff0000",
    .route_text_color = "00ff00"
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
  feed.write_trips(path_directory);

  struct StopTime trip_one_stop_one {
    .trip_id = tripOneID, .stop_id = stopOneID + "_ledge_to_the_train_bucko", .stop_sequence = 0,
    .arrival_time = Time("6:00:00"), .departure_time = Time("6:00:00"), .stop_headsign = "head",
    .shape_dist_traveled = 0.0, .timepoint = gtfs::StopTimePoint::Exact,
  };
  feed.add_stop_time(trip_one_stop_one);

  struct StopTime trip_one_stop_two {
    .trip_id = tripOneID, .stop_id = stopTwoID, .stop_sequence = 1, .arrival_time = Time("6:03:00"),
    .departure_time = Time("6:03:00"), .stop_headsign = "head", .shape_dist_traveled = 3.0,
    .timepoint = gtfs::StopTimePoint::Exact,
  };
  feed.add_stop_time(trip_one_stop_two);

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

  // write shapes.txt
  // TODO: write shapes before stop_times so that we can determine the correct travelled distance
  for (const auto& shape_ll :
       std::vector<PointLL>{layout["1"], layout["a"], layout["b"], layout["2"]}) {
    feed.add_shape(ShapePoint{.shape_id = shapeOneID,
                              .shape_pt_lat = shape_ll.second,
                              .shape_pt_lon = shape_ll.first,
                              .shape_pt_sequence = feed.get_shapes().size()});
  }
  feed.write_shapes(path_directory);
  feed.write_frequencies(path_directory);

  Feed feed_reader(path_directory);
  feed_reader.read_feed();
}

TEST(GtfsExample, MakeProto) {
  auto pt = get_config();

  // constants written in the last function
  auto serviceStartDate =
      baldr::DateTime::get_formatted_date("2023-01-31").time_since_epoch().count();
  auto serviceEndDate = baldr::DateTime::get_formatted_date("2024-01-31").time_since_epoch().count();
  auto addedDate = baldr::DateTime::get_formatted_date("2023-02-02").time_since_epoch().count();
  auto removedDate = baldr::DateTime::get_formatted_date("2023-02-03").time_since_epoch().count();
  const int headwaySec = 1800;

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
  // for each pbf.
  for (; transit_file_itr != end_file_itr; ++transit_file_itr) {
    if (filesystem::is_regular_file(transit_file_itr->path())) {
      std::string fname = transit_file_itr->path().string();
      mjolnir::Transit transit = mjolnir::read_pbf(fname);

      if (std::isdigit(fname.back())) {
        // we produce 2 pbf tiles on purpose, where the last one (xx.pbf.0) only has a bunch of stop
        // pairs
        EXPECT_NE(transit.stop_pairs_size(), 0);
        continue;
      }

      // make sure we are looking at a pbf file

      // make sure that the data written in the previous test is readable through pbfs
      // shape info
      // becomes 1 shape from 4 shape_points last test
      EXPECT_EQ(transit.shapes_size(), 1);
      // stop(node) info
      for (int i = 0; i < transit.nodes_size(); i++) {
        const auto& tn = transit.nodes(i);
        stops.insert(tn.onestop_id());
        if (tn.type() == static_cast<uint32_t>(NodeType::kTransitEgress)) {
          EXPECT_NE(tn.traversability(), static_cast<uint32_t>(Traversability::kNone));
        }
      }

      // routes info
      EXPECT_EQ(transit.routes_size(), 1);
      EXPECT_EQ(transit.routes(0).onestop_id(), routeID);

      // stop_pair info
      for (int i = 0; i < transit.stop_pairs_size(); i++) {
        EXPECT_TRUE(transit.stop_pairs(i).has_origin_graphid());
        EXPECT_TRUE(transit.stop_pairs(i).has_destination_graphid());
        stop_pairs.insert(transit.stop_pairs(i).origin_onestop_id());
        stop_pairs.insert(transit.stop_pairs(i).destination_onestop_id());
      }

      // calendar information
      EXPECT_EQ(transit.stop_pairs(0).service_start_date(), serviceStartDate);
      EXPECT_EQ(transit.stop_pairs(0).service_end_date(), serviceEndDate);

      // frequency information
      EXPECT_EQ(transit.stop_pairs(0).frequency_headway_seconds(), headwaySec);
    }
  }
}

TEST(GtfsExample, MakeTile) {
  boost::property_tree::ptree pt = get_config();

  auto layout = create_layout();
  auto station_one_ll = layout.find("1");
  auto station_two_ll = layout.find("2");

  auto dt = "2023-02-27T05:50";
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
    std::vector<PointLL> station_coords = {station_one_ll->second, station_two_ll->second};
    for (const auto& node : tile->GetNodes()) {
      // I notice that the coordinates in the tiles are 'rounded' to [x] decimal places, what is a way
      // to round when testing?
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
    }

    auto currRoute = tile->GetTransitRoute(0);
    EXPECT_EQ(tile->GetName(currRoute->one_stop_offset()), routeID);
    EXPECT_EQ(currRoute->route_text_color(), strtol("00ff00", nullptr, 16));
    EXPECT_EQ(currRoute->route_color(), strtol("ff0000", nullptr, 16));

    for (const auto& departure : tile->GetTransitDepartures()) {
      TransitDeparture* currDeparture = departure.second;
      auto transit_route = tile->GetTransitRoute(currDeparture->routeindex());
      EXPECT_EQ(tile->GetName(transit_route->one_stop_offset()), routeID);
    }

    for (uint32_t schedule_it = 0; schedule_it < tile->header()->schedulecount(); schedule_it++) {
      auto* tileSchedule = tile->GetTransitSchedule(schedule_it);
      // TODO: get the right value here, should be first 60 bits set
      // EXPECT_EQ(tileSchedule->days(), );
      // TODO: why not 60 days? Why only 60 days schedule validity at all?
      EXPECT_EQ(tileSchedule->end_day(), 59);
    }
  }
}

// TODO: TEST THAT TRANSIT ROUTING IS FUNCTIONAL BY MULTIMOTDAL ROUTING THROUGH WAYPOINTS, CHECK THAT
// THE TRIP TYPE IS A TRANSIT TYPE

TEST(GtfsExample, test_routing) {

  // TODO: create a simpler map: currently somehow at edge label 8 or so, it connects back to edge
  // label 3, which has the origin as predecessor.. smth is wrong with some directed edges.. possibly
  // with kRail not having opposing edges?

  // TODO: now the test only fails to find a path
  valhalla::Api result0 =
      gurka::do_action(valhalla::Options::route, map, {"A", "B"}, "multimodal",
                       {{"/date_time/type", "1"}, {"/date_time/value", "2023-02-27T05:45"}});
  EXPECT_EQ(result0.trip().routes_size(), 1);
  EXPECT_EQ(result0.trip().routes(0).legs(0).node_size(), 4);
  EXPECT_EQ(result0.trip().routes()[0].legs()[0].node(1).edge().travel_mode(), kTransit);
}