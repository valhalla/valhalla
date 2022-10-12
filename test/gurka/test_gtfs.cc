#include "gurka.h"
#include "just_gtfs/just_gtfs.h"

#include "mjolnir/ingest_transit.h"
#include "proto/transit.pb.h"
#include "test.h"
#include <gtest/gtest.h>

using namespace gtfs;
using namespace valhalla;

// test to write gtfs files
TEST(GtfsExample, WriteGtfs) {

  const std::string tripOneID = "10";
  const std::string tripTwoID = "11";
  const std::string stopOneID = "7";
  const std::string stopTwoID = "8";
  const std::string stopThreeID = "19";
  const std::string shapeOneID = "5";
  const std::string serviceOneID = "9";
  const int serviceStartDate = 20220131;
  const int serviceEndDate = 20230131;
  const int addedDate = 20220202;
  const int removedDate = 20220203;
  const int headwaySec = 1800;
  Feed feed;
  filesystem::create_directories("test/data/gtfs_feeds/toronto");

  // write agency.txt
  struct Agency ttc {
    .agency_id = "TTC", .agency_name = "Toronto Commission", .agency_url = "http://www.ttc.ca",
    .agency_timezone = "America/Toronto"
  };
  feed.add_agency(ttc);
  feed.write_agencies(("test/data/gtfs_feeds/toronto"));

  // write stops.txt
  struct gtfs::Stop nemo {
    .stop_id = stopOneID, .stop_name = gtfs::Text("POINT NEMO"), .coordinates_present = true,
    .stop_lat = 0.1, .stop_lon = 0.1, .parent_station = "0",
    .location_type = gtfs::StopLocationType::StopOrPlatform, .stop_timezone = "America/Toronto",
    .wheelchair_boarding = "1",
  };
  feed.add_stop(nemo);

  struct gtfs::Stop secondStop {
    .stop_id = stopTwoID, .stop_name = gtfs::Text("SECOND STOP"), .coordinates_present = true,
    .stop_lat = 0.2, .stop_lon = 0.2, .parent_station = "1",
    .location_type = gtfs::StopLocationType::StopOrPlatform, .stop_timezone = "America/Toronto",
    .wheelchair_boarding = "1",
  };
  feed.add_stop(secondStop);

  struct gtfs::Stop thirdStop {
    .stop_id = stopThreeID, .stop_name = gtfs::Text("THIRD STOP"), .coordinates_present = true,
    .stop_lat = 5, .stop_lon = 5, .parent_station = "1",
    .location_type = gtfs::StopLocationType::StopOrPlatform, .stop_timezone = "America/Toronto",
    .wheelchair_boarding = "1",
  };
  feed.add_stop(thirdStop);
  feed.write_stops(("test/data/gtfs_feeds/toronto"));

  // write routes.txt
  struct Route lineOne {
    .route_id = "2", .route_type = RouteType::Subway, .route_short_name = "ba",
    .route_long_name = "bababa", .route_desc = "this is the first route", .route_color = "blue",
    .route_text_color = "black",
  };
  feed.add_route(lineOne);
  feed.write_routes(("test/data/gtfs_feeds/toronto"));

  // write trips.txt
  struct gtfs::Trip tripOne {
    .route_id = "2", .service_id = serviceOneID, .trip_id = tripOneID, .trip_headsign = "haha",
    .block_id = "3", .shape_id = shapeOneID, .wheelchair_accessible = gtfs::TripAccess::Yes,
    .bikes_allowed = gtfs::TripAccess::No,
  };
  feed.add_trip(tripOne);

  struct gtfs::Trip tripTwo {
    .route_id = "2", .service_id = serviceOneID, .trip_id = tripTwoID, .trip_headsign = "bonjour",
    .block_id = "4", .shape_id = shapeOneID, .wheelchair_accessible = gtfs::TripAccess::Yes,
    .bikes_allowed = gtfs::TripAccess::No,
  };
  feed.add_trip(tripTwo);
  feed.write_trips("test/data/gtfs_feeds/toronto");

  // write stop_times.txt
  for (int i = 0; i < 6; i++) {
    std::string stopIds[3] = {stopOneID, stopTwoID, stopThreeID};
    struct StopTime stopTime {
      .trip_id = "", .stop_id = "", .stop_sequence = 0, .arrival_time = Time("6:00:00"),
      .departure_time = Time("6:00:00"), .stop_headsign = "head", .shape_dist_traveled = 0.0,
      .timepoint = gtfs::StopTimePoint::Exact,
    };
    stopTime.stop_id = stopIds[i % 3];
    stopTime.trip_id = (i < 3) ? tripOneID : tripTwoID;
    feed.add_stop_time(stopTime);
  }

  feed.write_stop_times("test/data/gtfs_feeds/toronto");

  // write calendar.txt
  struct CalendarItem calendarOne {
    .service_id = serviceOneID, .monday = CalendarAvailability::Available,
    .tuesday = CalendarAvailability::Available, .wednesday = CalendarAvailability::Available,
    .thursday = CalendarAvailability::Available, .friday = CalendarAvailability::Available,
    .saturday = CalendarAvailability::Available, .sunday = CalendarAvailability::Available,
    .start_date = Date(2022, 1, 31), .end_date = Date(2023, 1, 31),
  };
  feed.add_calendar_item(calendarOne);
  feed.write_calendar("test/data/gtfs_feeds/toronto");

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
  feed.write_calendar_dates(("test/data/gtfs_feeds/toronto"));

  // write shapes.txt
  for (size_t i = 0; i < 4; i++) {
    struct ShapePoint shapePointTest {
      .shape_id = shapeOneID, .shape_pt_lat = i + 0.2, .shape_pt_lon = i + 0.2,
      .shape_pt_sequence = (i + 1),
    };
    feed.add_shape(shapePointTest);
  }

  feed.write_shapes(("test/data/gtfs_feeds/toronto"));

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
  feed.write_frequencies(("test/data/gtfs_feeds/toronto"));

  Feed feed_reader(("test/data/gtfs_feeds/toronto"));
  feed_reader.read_feed();

  // make sure files are actually written

  const auto& trips = feed_reader.get_trips();
  EXPECT_EQ(trips.size(), 2);
  EXPECT_EQ(trips[0].trip_id, tripOneID);

  const auto& stops = feed_reader.get_stops();
  EXPECT_EQ(stops.size(), 3);
  EXPECT_EQ(stops[1].stop_id, stopTwoID);

  const auto& shapes = feed_reader.get_shapes();
  EXPECT_EQ(shapes.size(), 4);
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

TEST(GtfsExample, MakeTiles) {
  filesystem::create_directories("test/data/transit_test");
  filesystem::create_directories("test/data/transit_tiles");
  auto pt = test::make_config("test/data/transit_test",
                              {{"mjolnir.transit_feeds_dir", "test/data/gtfs_feeds/"},
                               {"mjolnir.transit_dir", "test/data/transit_tiles"}});

  // constants written in the last function
  const std::string tripOneID = "10";
  const std::string tripTwoID = "11";
  const std::string stopOneID = "7";
  const std::string stopTwoID = "8";
  const std::string stopThreeID = "19";
  const std::string shapeOneID = "5";
  const std::string serviceOneID = "9";
  const int serviceStartDate = 20220131;
  const int serviceEndDate = 20230131;
  const int addedDate = 20220202;
  const int removedDate = 20220203;
  const int headwaySec = 1800;

  // spawn threads to download all the tiles returning a list of
  // tiles that ended up having dangling stop pairs
  auto dangling_tiles = valhalla::mjolnir::ingest_transit(pt);

  // spawn threads to connect dangling stop pairs to adjacent tiles' stops
  valhalla::mjolnir::stitch_transit(pt, dangling_tiles);
  // call the two functions, in main valhalla_ingest-transit
  // it's gonna write protobufs
  filesystem::recursive_directory_iterator transit_file_itr("test/data/transit_tiles");
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
      EXPECT_EQ(transit.shapes(0).shape_id(), stoi(shapeOneID));
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
  EXPECT_EQ(stops.size(), 3);
  std::string stopIds[3] = {stopOneID, stopTwoID, stopThreeID};

  for (const auto& stopID : stopIds) {
    EXPECT_EQ(*stops.find((stopID)), stopID);
    EXPECT_EQ(*stop_pairs.find((stopID)), stopID);
  }
}

// make sure all the data is inside -> which we can test using the same tests as above
