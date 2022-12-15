#include "gurka.h"
#include "just_gtfs/just_gtfs.h"

#include "mjolnir/convert_transit.h"
#include "mjolnir/ingest_transit.h"
#include "proto/transit.pb.h"
#include "test.h"
#include <gtest/gtest.h>

using namespace gtfs;
using namespace valhalla;

// the transit shape uses * as separators rather than -/\|
const std::string ascii_map = R"(
        a**************b
        *              *
     A--1-----------B--2---C
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
                           F
    )";
// TODO: cant get higher road classes to allow egress/ingress connections, no ped access?
const gurka::ways ways = {{"AB", {{"highway", "residential"}}},
                          {"BC", {{"highway", "residential"}}},
                          {"CF", {{"highway", "residential"}}}};

boost::property_tree::ptree get_config() {

  return test::make_config(VALHALLA_BUILD_DIR "test/data/transit_tests",
                           {{"mjolnir.transit_feeds_dir",
                             VALHALLA_BUILD_DIR "test/data/transit_tests/gtfs_feeds"},
                            {"mjolnir.transit_dir",
                             VALHALLA_BUILD_DIR "test/data/transit_tests/transit_tiles"},
                            {"mjolnir.timezone", VALHALLA_BUILD_DIR "test/data/tz.sqlite"},
                            {"mjolnir.tile_dir",
                             VALHALLA_BUILD_DIR "test/data/transit_tests/tiles"}});
}

valhalla::gurka::nodelayout create_layout() {
  int gridsize_metres = 100;
  auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {-0.000001, -0.000001});

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

  const std::string tripOneID = "10";
  const std::string tripTwoID = "11";
  const std::string stopOneID = "7";
  const std::string stopTwoID = "8";
  const std::string stopThreeID = "19";
  const std::string shapeOneID = "5";
  const std::string serviceOneID = "9";
  const int headwaySec = 1800;
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
  struct gtfs::Stop nemo {
    .stop_id = stopOneID, .stop_name = gtfs::Text("POINT NEMO"), .coordinates_present = true,
    .stop_lat = station_one_ll->second.second, .stop_lon = station_one_ll->second.first,
    .parent_station = "", .location_type = gtfs::StopLocationType::Station,
    .stop_timezone = "America/Toronto", .wheelchair_boarding = "1",
  };
  feed.add_stop(nemo);
  struct gtfs::Stop nemo_egress {
    .stop_id = stopOneID + "_" + to_string(NodeType::kTransitEgress),
    .stop_name = gtfs::Text("POINT NEMO"), .coordinates_present = true,
    .stop_lat = station_one_ll->second.second, .stop_lon = station_one_ll->second.first,
    .parent_station = stopOneID, .location_type = gtfs::StopLocationType::EntranceExit,
    .stop_timezone = "America/Toronto", .wheelchair_boarding = "1",
  };
  feed.add_stop(nemo_egress);
  struct gtfs::Stop nemo_platform {
    .stop_id = stopOneID + "_" + to_string(NodeType::kMultiUseTransitPlatform),
    .stop_name = gtfs::Text("POINT NEMO"), .coordinates_present = true,
    .stop_lat = station_one_ll->second.second, .stop_lon = station_one_ll->second.first,
    .parent_station = stopOneID, .location_type = gtfs::StopLocationType::StopOrPlatform,
    .stop_timezone = "America/Toronto", .wheelchair_boarding = "1",
  };
  feed.add_stop(nemo_platform);

  struct gtfs::Stop secondStop {
    .stop_id = stopTwoID, .stop_name = gtfs::Text("SECOND STOP"), .coordinates_present = true,
    .stop_lat = station_two_ll->second.second, .stop_lon = station_two_ll->second.first,
    .parent_station = "", .location_type = gtfs::StopLocationType::Station,
    .stop_timezone = "America/Toronto", .wheelchair_boarding = "1",
  };
  feed.add_stop(secondStop);

  struct gtfs::Stop thirdStop {
    .stop_id = stopThreeID, .stop_name = gtfs::Text("THIRD STOP"), .coordinates_present = true,
    .stop_lat = station_three_ll->second.second, .stop_lon = station_three_ll->second.first,
    .parent_station = "", .location_type = gtfs::StopLocationType::Station,
    .stop_timezone = "America/Toronto", .wheelchair_boarding = "1",
  };
  feed.add_stop(thirdStop);
  feed.write_stops(path_directory);

  // write routes.txt
  struct Route lineOne {
    .route_id = "2", .route_type = RouteType::Subway, .route_short_name = "ba",
    .route_long_name = "bababa", .route_desc = "this is the first route", .route_color = "ff0000",
    .route_text_color = "00ff00",
  };
  feed.add_route(lineOne);

  feed.write_routes(path_directory);

  // write trips.txt
  struct gtfs::Trip tripOne {
    .route_id = "2", .service_id = serviceOneID, .trip_id = tripOneID, .trip_headsign = "hello",
    .block_id = "3", .shape_id = shapeOneID, .wheelchair_accessible = gtfs::TripAccess::Yes,
    .bikes_allowed = gtfs::TripAccess::No,
  };
  feed.add_trip(tripOne);

  struct gtfs::Trip tripTwo {
    .route_id = "2", .service_id = serviceOneID, .trip_id = tripTwoID, .trip_headsign = "bonjour",
    .block_id = "3", .shape_id = shapeOneID, .wheelchair_accessible = gtfs::TripAccess::Yes,
    .bikes_allowed = gtfs::TripAccess::No,
  };
  feed.add_trip(tripTwo);
  feed.write_trips(path_directory);

  // write stop_times.txt
  std::string stopIds[3] = {stopOneID + "_" + to_string(NodeType::kMultiUseTransitPlatform),
                            stopTwoID, stopThreeID};
  for (int i = 0; i < 6; i++) {
    struct StopTime stopTime {
      .trip_id = "", .stop_id = "", .stop_sequence = 0, .arrival_time = Time("6:00:00"),
      .departure_time = Time("6:00:00"), .stop_headsign = "head", .shape_dist_traveled = 0.0,
      .timepoint = gtfs::StopTimePoint::Exact,
    };
    stopTime.stop_id = stopIds[i % 3];
    stopTime.trip_id = (i < 3) ? tripOneID : tripTwoID;
    feed.add_stop_time(stopTime);
  }

  feed.write_stop_times(path_directory);

  // write calendar.txt
  struct CalendarItem calendarOne {
    .service_id = serviceOneID, .monday = CalendarAvailability::Available,
    .tuesday = CalendarAvailability::Available, .wednesday = CalendarAvailability::Available,
    .thursday = CalendarAvailability::Available, .friday = CalendarAvailability::Available,
    .saturday = CalendarAvailability::Available, .sunday = CalendarAvailability::Available,
    .start_date = Date(2022, 1, 31), .end_date = Date(2023, 1, 31),
  };
  feed.add_calendar_item(calendarOne);

  feed.write_calendar(path_directory);

  // write calendar_dates.txt
  struct CalendarDate servAdded {
    .service_id = serviceOneID, .date = Date(2022, 2, 2),
    .exception_type = gtfs::CalendarDateException::Added,
  };
  struct CalendarDate servRemoved {
    .service_id = serviceOneID, .date = Date(2022, 2, 3),
    .exception_type = gtfs::CalendarDateException::Removed,
  };

  feed.add_calendar_date(servAdded);
  feed.add_calendar_date(servRemoved);

  feed.write_calendar_dates(path_directory);

  // write shapes.txt
  for (const auto& shape_ll :
       {layout["1"], layout["a"], layout["b"], layout["2"], layout["c"], layout["3"]}) {
    feed.add_shape({.shape_id = shapeOneID,
                    .shape_pt_lat = shape_ll.second,
                    .shape_pt_lon = shape_ll.first,
                    .shape_pt_sequence = feed.get_shapes().size()});
  }
  feed.write_shapes(path_directory);

  // write frequencies.txt
  struct Frequency freqBased {
    .trip_id = tripOneID, .start_time = Time(0, 0, 0), .end_time = Time(2, 0, 0),
    .headway_secs = headwaySec, .exact_times = gtfs::FrequencyTripService::FrequencyBased,
  };
  struct Frequency schedBased {
    .trip_id = tripTwoID, .start_time = Time(0, 0, 0), .end_time = Time(2, 0, 0),
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
  EXPECT_EQ(stops.size(), 5);
  EXPECT_EQ(stops[0].stop_id, stopOneID);

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
  const std::string tripOneID = "10";
  const std::string tripTwoID = "11";
  const std::string stopOneID = "7";
  const std::string stopTwoID = "8";
  const std::string stopThreeID = "19";
  const std::string shapeOneID = "5";
  const std::string serviceOneID = "9";
  auto serviceStartDate =
      baldr::DateTime::get_formatted_date("2022-01-31").time_since_epoch().count();
  auto serviceEndDate = baldr::DateTime::get_formatted_date("2023-01-31").time_since_epoch().count();
  auto addedDate = baldr::DateTime::get_formatted_date("2022-02-02").time_since_epoch().count();
  auto removedDate = baldr::DateTime::get_formatted_date("2022-02-03").time_since_epoch().count();
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

      // make sure we are looking at a pbf file

      // make sure that the data written in the previous test is readable through pbfs
      // shape info
      // becomes 1 shape from 4 shape_points last test
      EXPECT_EQ(transit.shapes_size(), 1);
      // stop(node) info
      for (int i = 0; i < transit.nodes_size(); i++) {
        stops.insert(transit.nodes(i).onestop_id());
      }

      // routes info
      EXPECT_EQ(transit.routes_size(), 1);
      EXPECT_EQ(transit.routes(0).onestop_id(), "2");

      // stop_pair info
      for (int i = 0; i < transit.stop_pairs_size(); i++) {
        stop_pairs.insert(transit.stop_pairs(i).origin_onestop_id());
        stop_pairs.insert(transit.stop_pairs(i).destination_onestop_id());
      }

      // calendar information
      EXPECT_EQ(transit.stop_pairs(0).service_start_date(), serviceStartDate);
      EXPECT_EQ(transit.stop_pairs(0).service_end_date(), serviceEndDate);
      //   .start_date = Date(2022, 1, 31), .end_date = Date(2023, 1, 31) is written to gtfs

      // calendar date information
      EXPECT_EQ(transit.stop_pairs(0).service_except_dates_size(), 1);
      EXPECT_EQ(transit.stop_pairs(0).service_except_dates(0), removedDate);
      EXPECT_EQ(transit.stop_pairs(0).service_added_dates(0), addedDate);

      // frequency information
      EXPECT_EQ(transit.stop_pairs(0).frequency_headway_seconds(), headwaySec);
      EXPECT_EQ(transit.stop_pairs(1).frequency_headway_seconds(), headwaySec);
    }
  }
  EXPECT_EQ(stops.size(), 9);
  std::string stopIds[3] = {stopOneID, stopTwoID, stopThreeID};
  std::string stop_pair_ids[3] = {stopOneID + "_" + to_string(NodeType::kMultiUseTransitPlatform),
                                  stopTwoID + "_" + to_string(NodeType::kMultiUseTransitPlatform),
                                  stopThreeID + "_" + to_string(NodeType::kMultiUseTransitPlatform)};

  for (const auto& stopID : stopIds) {
    EXPECT_EQ(*stops.find((stopID)), stopID);
  }
  for (const auto& stop_pair_id : stop_pair_ids) {
    EXPECT_EQ(*stops.find((stop_pair_id)), stop_pair_id);
  }
  // TODO: MAKE SURE ALL THE GENEREATED STOPS ARE ALSO TESTED FOR
}

TEST(GtfsExample, MakeTile) {
  const std::string tripOneID = "10";
  const std::string tripTwoID = "11";
  const std::string stopOneID = "7";
  const std::string stopTwoID = "8";
  const std::string stopThreeID = "19";
  const std::string shapeOneID = "5";
  const std::string serviceOneID = "9";
  const std::string routeID = "2";

  boost::property_tree::ptree pt = get_config();

  auto layout = create_layout();
  auto station_one_ll = layout.find("1");
  auto station_two_ll = layout.find("2");
  auto station_three_ll = layout.find("3");

  // we build the graph so we can find edges (way ids) where we can connect the transit subgraph
  auto tile_dir = pt.get<std::string>("mjolnir.tile_dir");
  map = gurka::buildtiles(layout, ways, {}, {}, tile_dir);

  // this creates routable transit tiles but doesnt connect them to the rest of the graph
  auto all_tiles = valhalla::mjolnir::convert_transit(pt);

  // now we have to build the tiles again to get the transit tiles connected to the regular graph
  map = gurka::buildtiles(layout, ways, {}, {}, pt);

  // files are already going to be written from
  filesystem::recursive_directory_iterator transit_file_itr(tile_dir);
  filesystem::recursive_directory_iterator end_file_itr;

  GraphReader reader(pt.get_child("mjolnir"));
  auto graphids = reader.GetTileSet();
  int totalNodes = 0;
  for (auto graphid : graphids) {
    LOG_INFO("Working on : " + std::to_string(graphid));
    graph_tile_ptr tile = reader.GetGraphTile(graphid);

    if (graphid.level() != TileHierarchy::GetTransitLevel().level) {
      if (graphid.level() == TileHierarchy::levels().back().level) {
        bool foundTransitConnect = false;
        for (const auto& edge : tile->GetDirectedEdges()) {
          foundTransitConnect =
              foundTransitConnect || edge.use() == valhalla::baldr::Use::kTransitConnection;
        }
        // TODO: this is the failing test and the next step that we have to fix
        EXPECT_TRUE(foundTransitConnect);
      }
      continue;
    }

    if (tile->GetStopOneStops().empty()) {
      LOG_ERROR("NO one stops found");
    }
    auto tileStops = tile->GetStopOneStops();
    if (tile->GetNodes().size() == 0) {
      LOG_WARN("There are no nodes inside tile " + std::to_string(graphid));
    } else {
      LOG_INFO("There are " + std::to_string(tile->GetNodes().size()) + " nodes inside tile " +
               std::to_string(graphid));
    }
    totalNodes += tile->header()->nodecount();
    std::unordered_set<NodeType> node_types = {NodeType::kMultiUseTransitPlatform,
                                               NodeType::kTransitStation, NodeType::kTransitEgress};
    std::vector<PointLL> station_coords = {station_one_ll->second, station_two_ll->second,
                                           station_three_ll->second};
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

    std::unordered_set<std::string> stopIds = {stopOneID, stopTwoID, stopThreeID};

    auto currRoute = tile->GetTransitRoute(0);
    EXPECT_EQ(tile->GetName(currRoute->one_stop_offset()), routeID);
    EXPECT_EQ(currRoute->route_text_color(), strtol("00ff00", nullptr, 16));
    EXPECT_EQ(currRoute->route_color(), strtol("ff0000", nullptr, 16));

    std::unordered_set<int> tripIDs = {stoi(tripOneID), stoi(tripTwoID)};

    for (const auto& departure : tile->GetTransitDepartures()) {
      TransitDeparture* currDeparture = departure.second;
      auto transit_route = tile->GetTransitRoute(currDeparture->routeindex());
      EXPECT_EQ(tile->GetName(transit_route->one_stop_offset()), routeID);
    }

    for (uint32_t schedule_it = 0; schedule_it < tile->header()->schedulecount(); schedule_it++) {
      auto* tileSchedule = tile->GetTransitSchedule(schedule_it);
      EXPECT_EQ(tileSchedule->days(), 0);
      EXPECT_EQ(tileSchedule->end_day(), 60);
    }
  }
  EXPECT_EQ(totalNodes, 9);
}

// TODO: TEST THAT TRANSIT ROUTING IS FUNCTIONAL BY MULTIMOTDAL ROUTING THROUGH WAYPOINTS, CHECK THAT
// THE TRIP TYPE IS A TRANSIT TYPE

TEST(GtfsExample, DISABLED_testRouting) {

  boost::property_tree::ptree pt = get_config();

  auto layout = create_layout();

  valhalla::Api result0 =
      gurka::do_action(valhalla::Options::route, map, {"A", "F"}, "multimodal",
                       {{"/date_time/type", "1"}, {"/date_time/value", "2022-02-17T05:45"}});
  EXPECT_EQ(result0.trip().routes_size(), 1);
}