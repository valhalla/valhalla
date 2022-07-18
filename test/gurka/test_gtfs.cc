#include "gurka.h"
#include "just_gtfs/just_gtfs.h"

#include "mjolnir/ingest_transit.h"
#include "string.h"
#include <gtest/gtest.h>

using namespace gtfs;
using namespace valhalla;

// test to write gtfs files
TEST(GtfsExample, WriteGtfs) {

  const std::string tripOneID = "bar";
  const std::string tripTwoID = "barbar";
  const std::string stopOneID = "foo";
  const std::string stopTwoID = "foo2";
  const std::string shapeOneID = "square";
  const std::string serviceOneID = "bon";
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
  feed.write_stops(("test/data/gtfs_feeds/toronto"));

  // write routes.txt
  struct Route lineOne {
    .route_id = "baz", .route_type = RouteType::Subway, .route_short_name = "ba",
    .route_long_name = "bababa", .route_desc = "this is the first route", .route_color = "blue",
    .route_text_color = "black",
  };
  feed.add_route(lineOne);
  feed.write_routes(("test/data/gtfs_feeds/toronto"));

  // write trips.txt
  struct gtfs::Trip tripOne {
    .route_id = "baz", .service_id = serviceOneID, .trip_id = tripOneID, .trip_headsign = "haha",
    .block_id = "block", .shape_id = shapeOneID, .wheelchair_accessible = gtfs::TripAccess::Yes,
    .bikes_allowed = gtfs::TripAccess::No,
  };
  feed.add_trip(tripOne);
  feed.write_trips("test/data/gtfs_feeds/toronto");

  // write stop_times.txt
  for (int i = 0; i < 4; i++) {
    struct StopTime stopTime {
      .trip_id = "", .stop_id = "", .stop_sequence = 0, .arrival_time = Time("6:00:00"),
      .departure_time = Time("6:00:00"), .stop_headsign = "head", .shape_dist_traveled = 0.0,
      .timepoint = gtfs::StopTimePoint::Exact,
    };
    stopTime.stop_id = (i % 2 == 0) ? stopOneID : stopTwoID;
    stopTime.trip_id = (i < 2) ? tripOneID : tripTwoID;
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
      .shape_id = shapeOneID, .shape_pt_lat = 0.2, .shape_pt_lon = 0.2, .shape_pt_sequence = (i + 1),
    };
    feed.add_shape(shapePointTest);
  }

  feed.write_shapes(("test/data/gtfs_feeds/toronto"));

  // write frequencies.txt
  struct Frequency freqBased {
    .trip_id = tripOneID, .start_time = Time(0, 0, 0), .end_time = Time(2, 0, 0),
    .headway_secs = 1800, .exact_times = gtfs::FrequencyTripService::FrequencyBased,
  };
  struct Frequency schedBased {
    .trip_id = tripTwoID, .start_time = Time(0, 0, 0), .end_time = Time(2, 0, 0),
    .headway_secs = 1800, .exact_times = gtfs::FrequencyTripService::ScheduleBased,
  };

  feed.add_frequency(freqBased);
  feed.add_frequency(schedBased);
  feed.write_frequencies(("test/data/gtfs_feeds/toronto"));

  Feed feed_reader(("test/data/gtfs_feeds/toronto"));
  feed_reader.read_feed();

  // make sure files are actually written

  const auto& trips = feed_reader.get_trips();
  EXPECT_EQ(trips.size(), 1);
  EXPECT_EQ(trips[0].trip_id, tripOneID);

  const auto& stops = feed_reader.get_stops();
  EXPECT_EQ(stops.size(), 2);
  EXPECT_EQ(stops[1].stop_id, stopTwoID);

  const auto& shapes = feed_reader.get_shapes();
  EXPECT_EQ(shapes.size(), 4);
  EXPECT_EQ(shapes[0].shape_id, shapeOneID);

  const auto& calendarGTFS = feed_reader.get_calendar();
  EXPECT_EQ(calendarGTFS.size(), 1);
  EXPECT_EQ(calendarGTFS[0].service_id, serviceOneID);
}

TEST(GtfsExample, MakeTiles) {
  // create a config in memory make_config
  // make_config("")

  // call the two functions, in main valhalla_ingest-transit
  // it's gonna write protobufs
  // make sure all the data is inside -> which we can test using the same tests as above
}
