#include "gurka.h"
#include "just_gtfs/just_gtfs.h"

#include "baldr/datetime.h"
#include "mjolnir/convert_transit.h"
#include "mjolnir/ingest_transit.h"
#include "proto/common.pb.h"
#include "proto/transit.pb.h"
#include "test.h"
#include <gtest/gtest.h>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

using point_type = boost::geometry::model::d2::point_xy<double>;
using polygon_type = boost::geometry::model::polygon<point_type>;
using boost::geometry::within;

using namespace gtfs;
using namespace valhalla;
using namespace std::chrono;
using rp = rapidjson::Pointer;

// since writing GTFS feeds with C++ is sooo annoying, we'll have some var templates, e.g.
// sv1_id: service 1, ID
//
// a: agency
// b: block
// c: calendar
// e: egress
// f: feed
// p: platform
// sh: shape
// st: station
// sv: service
// t: trip

std::vector<PointLL> polygon_from_geojson(const std::string& geojson) {
  rapidjson::Document response;
  response.Parse(geojson);

  auto feature_count = rp("/features").Get(response)->GetArray().Size();
  for (size_t i = 0; i < feature_count; ++i) {
    std::string type =
        rp("/features/" + std::to_string(i) + "/geometry/type").Get(response)->GetString();

    if (type != "Point") {
      auto geom = rp("/features/" + std::to_string(i) + "/geometry/coordinates" +
                     (type == "Polygon" ? "/0" : ""))
                      .Get(response)
                      ->GetArray();
      std::vector<PointLL> res;
      res.reserve(geom.Size());
      for (size_t j = 0; j < geom.Size(); ++j) {
        auto coord = geom[j].GetArray();
        res.emplace_back(coord[0].GetDouble(), coord[1].GetDouble());
      }
      return res;
    }
  }
  return {};
}

const auto NOW = system_clock::now();
// dynamically get today's date so we don't fail the test at some point
std::string get_date_time_formatted(int offset_days = 0) {
  std::ostringstream iso_date_time;
  NOW.time_since_epoch().count();
  iso_date_time << date::format("%Y%m%d", NOW + hours(offset_days * 24));

  return iso_date_time.str();
}
// dynamically get today's date so we don't fail the test at some point
date::local_seconds parse_date_time_string(const std::string& date_time) {
  std::istringstream in{date_time};
  date::local_seconds tp;
  in >> date::parse("%Y%m%d", tp);

  return tp;
}

// static ids of different feed items, shared amongst tests, we could do more
const std::string a1_id = "TTC";
const std::string a2_id = "TTC2";
const std::string f1_name = "toronto_1";
const std::string f2_name = "toronto_2";
const std::string t1_id = "t1_id";
const std::string t2_id = "t2_id";
const std::string t3_id = "t3_id";
const std::string t4_id = "t4_id";
const std::string st1_id = "st1_id";
const std::string st2_id = "st2_id";
const std::string st3_id = "st3_id";
const std::string st4_id = "st4_id";
const std::string sh1_id = "sh1_id";
const std::string sv1_id = "sv1_id";
const std::string sv2_id = "sv2_id";
const std::string sv3_id = "sv2_id";
const std::string r1_id = "r1_id";
const std::string r2_id = "r2_id";
const std::string b1_id = "b1_id";
const std::string b2_id = "b2_id";
const uint32_t t1_headsecs = 1800; // 30 mins headway
const uint32_t t2_headsecs = 300;  // 5 mins headway
const std::string sv_start = get_date_time_formatted();
const std::string sv_end = get_date_time_formatted(80);
const std::string added_date = get_date_time_formatted(10);
const std::string removed_date = get_date_time_formatted(20);

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
        a******************************b********d
        *                              *        *
 A---B--1-------------C----------g--D--2---E    *
                                       *   |    *
                                       *   |    *
                                       *   |    *
                                       *   |    *
                                       *   4****e
                                       *   |
                                       c***3
                                           |
                                           F
                                           |
                                           h
                                           |
                                           G
    )";

// TODO: cant get higher road classes to allow egress/ingress connections, no ped access?
const gurka::ways ways = {{"AB", {{"highway", "primary"}}}, {"BC", {{"highway", "primary"}}},
                          {"CD", {{"highway", "primary"}}}, {"DE", {{"highway", "primary"}}},
                          {"EF", {{"highway", "primary"}}}, {"FG", {{"highway", "primary"}}}};

boost::property_tree::ptree get_config() {

  return test::make_config(VALHALLA_BUILD_DIR "test/data/transit_tests",
                           {{"mjolnir.transit_feeds_dir",
                             VALHALLA_BUILD_DIR "test/data/transit_tests/gtfs_feeds"},
                            {"mjolnir.transit_dir",
                             VALHALLA_BUILD_DIR "test/data/transit_tests/transit_tiles"},
                            {"mjolnir.transit_pbf_limit",
                             "1"}, // so we create more than one file per tile
                            {"mjolnir.hierarchy", "1"},
                            {"mjolnir.timezone", VALHALLA_BUILD_DIR "test/data/tz.sqlite"},
                            {"mjolnir.tile_dir", VALHALLA_BUILD_DIR "test/data/transit_tests/tiles"},
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
  const std::string gtfs_dir =
      pt.get<std::string>("mjolnir.transit_feeds_dir") + filesystem::path::preferred_separator;
  filesystem::remove_all(gtfs_dir);
  filesystem::remove_all(pt.get<std::string>("mjolnir.tile_dir"));
  filesystem::remove_all(pt.get<std::string>("mjolnir.transit_dir"));
  filesystem::create_directories(pt.get<std::string>("mjolnir.tile_dir"));
  filesystem::create_directories(pt.get<std::string>("mjolnir.transit_feeds_dir"));
  filesystem::create_directories(pt.get<std::string>("mjolnir.transit_dir"));

  auto layout = create_layout();
  auto st1_ll = layout.find("1");
  auto st2_ll = layout.find("2");
  auto st3_ll = layout.find("3");
  auto st4_ll = layout.find("4");

  Feed f1;
  Feed f2;
  std::string f1_path = gtfs_dir + f1_name;
  std::string f2_path = gtfs_dir + f2_name;
  for (const auto& f : {"toronto_1", "toronto_2"}) {
    filesystem::create_directories(pt.get<std::string>("mjolnir.transit_feeds_dir") +
                                   filesystem::path::preferred_separator + f);
  }

  // write agency.txt
  struct Agency ttc {
    .agency_id = a1_id, .agency_name = "Toronto Commission", .agency_url = "http://www.ttc.ca",
    .agency_timezone = "America/Toronto"
  };
  f1.add_agency(ttc);

  struct Agency ttc2 {
    .agency_id = a2_id, .agency_name = "Toronto Commission Next Gen",
    .agency_url = "http://www.ttc-next.ca", .agency_timezone = "America/Toronto"
  };
  f2.add_agency(ttc2);

  f1.write_agencies(f1_path);
  f2.write_agencies(f2_path);

  // write stops.txt:
  // 1st has all stop objects, egress/station/platform
  // 2nd has only platform, station
  // 3rd has only platform
  struct gtfs::Stop e1 {
    .stop_id = st1_id + "_egress", .stop_name = gtfs::Text("FIRST EGRESS"),
    .coordinates_present = true, .stop_lat = st1_ll->second.second, .stop_lon = st1_ll->second.first,
    .parent_station = st1_id, .location_type = gtfs::StopLocationType::EntranceExit,
    .wheelchair_boarding = "1",
  };
  f1.add_stop(e1);
  struct gtfs::Stop st1 {
    .stop_id = st1_id, .stop_name = gtfs::Text("FIRST STATION"), .coordinates_present = true,
    .stop_lat = st1_ll->second.second, .stop_lon = st1_ll->second.first, .parent_station = "",
    .location_type = gtfs::StopLocationType::Station, .wheelchair_boarding = "1",
  };
  f1.add_stop(st1);
  struct gtfs::Stop p1 {
    .stop_id = st1_id + "_platform", .stop_name = gtfs::Text("FIRST STOP"),
    .coordinates_present = true, .stop_lat = st1_ll->second.second, .stop_lon = st1_ll->second.first,
    .parent_station = st1_id, .location_type = gtfs::StopLocationType::StopOrPlatform,
    .wheelchair_boarding = "1",
  };
  f1.add_stop(p1);

  struct gtfs::Stop st2 {
    .stop_id = st2_id, .stop_name = gtfs::Text("SECOND STATION"), .coordinates_present = true,
    .stop_lat = st2_ll->second.second, .stop_lon = st2_ll->second.first, .parent_station = "",
    .location_type = gtfs::StopLocationType::Station, .stop_timezone = "America/Toronto",
    .wheelchair_boarding = "1",
  };
  f1.add_stop(st2);
  f2.add_stop(st2);
  struct gtfs::Stop p2 {
    .stop_id = st2_id + "_platform", .stop_name = gtfs::Text("SECOND STOP"),
    .coordinates_present = true, .stop_lat = st2_ll->second.second, .stop_lon = st2_ll->second.first,
    .parent_station = st2_id, .location_type = gtfs::StopLocationType::StopOrPlatform,
    .stop_timezone = "America/Toronto", .wheelchair_boarding = "1",
  };
  f1.add_stop(p2);
  f2.add_stop(p2);

  struct gtfs::Stop p3 {
    .stop_id = st3_id, .stop_name = gtfs::Text("THIRD STOP"), .coordinates_present = true,
    .stop_lat = st3_ll->second.second, .stop_lon = st3_ll->second.first, .parent_station = "",
    .location_type = gtfs::StopLocationType::StopOrPlatform, .stop_timezone = "America/Toronto",
    .wheelchair_boarding = "1",
  };
  f1.add_stop(p3);

  struct gtfs::Stop p4 {
    .stop_id = st4_id, .stop_name = gtfs::Text("FOURTH STOP"), .coordinates_present = true,
    .stop_lat = st4_ll->second.second, .stop_lon = st4_ll->second.first, .parent_station = "",
    .location_type = gtfs::StopLocationType::StopOrPlatform, .stop_timezone = "America/Toronto",
    .wheelchair_boarding = "1",
  };
  f2.add_stop(p4);

  f1.write_stops(f1_path);
  f2.write_stops(f2_path);

  // write routes.txt
  struct Route r1 {
    .route_id = r1_id, .route_type = RouteType::Subway, .agency_id = a1_id, .route_short_name = "ba",
    .route_long_name = "bababa", .route_desc = "this is the first route for TTC",
    .route_color = "ff0000", .route_text_color = "00ff00"
  };
  f1.add_route(r1);

  struct Route r2 {
    .route_id = r2_id, .route_type = RouteType::Subway, .agency_id = a2_id, .route_short_name = "ba2",
    .route_long_name = "bababa2", .route_desc = "this is the first route for TTC2",
    .route_color = "0000ff", .route_text_color = "001100"
  };
  f2.add_route(r2);

  f1.write_routes(f1_path);
  f2.write_routes(f2_path);

  // write trips.txt
  struct gtfs::Trip t1 {
    .route_id = r1_id, .service_id = sv1_id, .trip_id = t1_id, .trip_headsign = "hello",
    .block_id = b1_id, .shape_id = sh1_id, .wheelchair_accessible = gtfs::TripAccess::Yes,
    .bikes_allowed = gtfs::TripAccess::No,
  };
  f1.add_trip(t1);

  struct gtfs::Trip t2 {
    .route_id = r1_id, .service_id = sv2_id, .trip_id = t2_id, .trip_headsign = "bonjour",
    .block_id = b2_id, .wheelchair_accessible = gtfs::TripAccess::Yes,
    .bikes_allowed = gtfs::TripAccess::No,
  };
  f1.add_trip(t2);

  struct gtfs::Trip t3 {
    .route_id = r2_id, .service_id = sv3_id, .trip_id = t3_id, .trip_headsign = "grüß gott!",
    .wheelchair_accessible = gtfs::TripAccess::Yes, .bikes_allowed = gtfs::TripAccess::No
  };
  f2.add_trip(t3);

  struct gtfs::Trip t4 {
    .route_id = r2_id, .service_id = sv3_id, .trip_id = t4_id, .trip_headsign = "grüß gott!",
    .wheelchair_accessible = gtfs::TripAccess::Yes, .bikes_allowed = gtfs::TripAccess::No
  };
  f2.add_trip(t4);

  f1.write_trips(f1_path);
  f2.write_trips(f2_path);

  // write stop_times.txt
  struct StopTime t1p1 {
    .trip_id = t1_id, .stop_id = st1_id + "_platform", .stop_sequence = 0,
    .arrival_time = Time("6:00:00"), .departure_time = Time("6:00:00"), .stop_headsign = "t1p1",
    .shape_dist_traveled = 0.0, .timepoint = gtfs::StopTimePoint::Exact,
  };
  f1.add_stop_time(t1p1);

  struct StopTime t1p2 {
    .trip_id = t1_id, .stop_id = st2_id + "_platform", .stop_sequence = 1,
    .arrival_time = Time("6:03:00"), .departure_time = Time("6:03:00"), .stop_headsign = "t1p2",
    .shape_dist_traveled = 3.0, .timepoint = gtfs::StopTimePoint::Exact,
  };
  f1.add_stop_time(t1p2);

  struct StopTime t1p3 {
    .trip_id = t1_id, .stop_id = st3_id, .stop_sequence = 2, .arrival_time = Time("6:06:00"),
    .departure_time = Time("6:06:00"), .stop_headsign = "t1p3", .shape_dist_traveled = 6.0,
    .timepoint = gtfs::StopTimePoint::Exact,
  };
  f1.add_stop_time(t1p3);

  struct StopTime t2p1 {
    .trip_id = t2_id, .stop_id = st1_id + "_platform", .stop_sequence = 0,
    .arrival_time = Time("10:00:00"), .departure_time = Time("10:00:00"), .stop_headsign = "t2p1",
    .timepoint = gtfs::StopTimePoint::Exact,
  };
  f1.add_stop_time(t2p1);

  struct StopTime t2p2 {
    .trip_id = t2_id, .stop_id = st2_id + "_platform", .stop_sequence = 1,
    .arrival_time = Time("10:03:00"), .departure_time = Time("10:03:00"), .stop_headsign = "t2p2",
    .timepoint = gtfs::StopTimePoint::Exact,
  };
  f1.add_stop_time(t2p2);

  struct StopTime t2p3 {
    .trip_id = t2_id, .stop_id = st3_id, .stop_sequence = 2, .arrival_time = Time("10:06:00"),
    .departure_time = Time("10:06:00"), .stop_headsign = "t2p3",
    .timepoint = gtfs::StopTimePoint::Exact,
  };
  f1.add_stop_time(t2p3);

  // add one stop_pair which has the same
  struct StopTime t3p1 {
    .trip_id = t3_id, .stop_id = st2_id + "_platform", .stop_sequence = 0,
    .arrival_time = Time("10:03:00"), .departure_time = Time("10:03:00"), .stop_headsign = "t3p1",
    .timepoint = gtfs::StopTimePoint::Exact,
  };
  f2.add_stop_time(t3p1);

  struct StopTime t3p2 {
    .trip_id = t3_id, .stop_id = st4_id, .stop_sequence = 1, .arrival_time = Time("10:06:00"),
    .departure_time = Time("10:06:00"), .stop_headsign = "t3p2",
    .timepoint = gtfs::StopTimePoint::Exact,
  };
  f2.add_stop_time(t3p2);

  // TODO: smth doesn't work here, we fail some departures' elapsed_time test and there's 2 more
  // transit edges all of a sudden
  struct StopTime t3p3 {
    .trip_id = t4_id, .stop_id = st2_id + "_platform", .stop_sequence = 0,
    .arrival_time = Time("23:58:00"), .departure_time = Time("23:58:00"), .stop_headsign = "t3p3",
    .timepoint = gtfs::StopTimePoint::Exact,
  };
  f2.add_stop_time(t3p3);

  struct StopTime t3p4 {
    .trip_id = t4_id, .stop_id = st4_id, .stop_sequence = 1, .arrival_time = Time("24:02:00"),
    .departure_time = Time("24:02:00"), .stop_headsign = "t3p4",
    .timepoint = gtfs::StopTimePoint::Exact,
  };
  f2.add_stop_time(t3p4);

  f1.write_stop_times(f1_path);
  f2.write_stop_times(f2_path);

  // write calendar.txt
  struct CalendarItem c1 {
    .service_id = sv1_id, .monday = CalendarAvailability::Available,
    .tuesday = CalendarAvailability::Available, .wednesday = CalendarAvailability::Available,
    .thursday = CalendarAvailability::Available, .friday = CalendarAvailability::Available,
    .saturday = CalendarAvailability::Available, .sunday = CalendarAvailability::Available,
    .start_date = Date(sv_start), .end_date = Date(sv_end),
  };
  f1.add_calendar_item(c1);

  // only week days are available, no service on weekends, add to both feeds
  struct CalendarItem c2 {
    .service_id = sv2_id, .monday = CalendarAvailability::Available,
    .tuesday = CalendarAvailability::Available, .wednesday = CalendarAvailability::Available,
    .thursday = CalendarAvailability::Available, .friday = CalendarAvailability::Available,
    .saturday = CalendarAvailability::NotAvailable, .sunday = CalendarAvailability::NotAvailable,
    .start_date = Date(sv_start), .end_date = Date(sv_end),
  };
  f1.add_calendar_item(c2);

  // only week days are available, no service on weekends, add to both feeds
  struct CalendarItem c3 {
    .service_id = sv3_id, .monday = CalendarAvailability::Available,
    .tuesday = CalendarAvailability::Available, .wednesday = CalendarAvailability::Available,
    .thursday = CalendarAvailability::Available, .friday = CalendarAvailability::Available,
    .saturday = CalendarAvailability::Available, .sunday = CalendarAvailability::NotAvailable,
    .start_date = Date(sv_start), .end_date = Date(sv_end),
  };
  f2.add_calendar_item(c3);

  f1.write_calendar(f1_path);
  f2.write_calendar(f2_path);

  // write calendar_dates.txt
  struct CalendarDate servAdded {
    .service_id = sv1_id, .date = Date(added_date),
    .exception_type = gtfs::CalendarDateException::Added,
  };
  struct CalendarDate servRemoved {
    .service_id = sv1_id, .date = Date(removed_date),
    .exception_type = gtfs::CalendarDateException::Removed,
  };

  f1.add_calendar_date(servAdded);
  f1.add_calendar_date(servRemoved);
  f1.write_calendar_dates(f1_path);

  // write shapes.txt
  // TODO: write shapes before stop_times so that we can determine the correct travelled distance
  for (const auto& shape_ll : std::vector<PointLL>{layout["1"], layout["a"], layout["b"], layout["2"],
                                                   layout["c"], layout["3"]}) {
    f1.add_shape(ShapePoint{.shape_id = sh1_id,
                            .shape_pt_lat = shape_ll.second,
                            .shape_pt_lon = shape_ll.first,
                            .shape_pt_sequence = f1.get_shapes().size()});
  }
  f1.write_shapes(f1_path);

  // write frequencies.txt
  struct Frequency freqBased {
    .trip_id = t1_id, .start_time = Time(6, 0, 0), .end_time = Time(22, 0, 0),
    .headway_secs = t1_headsecs, .exact_times = gtfs::FrequencyTripService::FrequencyBased,
  };
  struct Frequency schedBased {
    .trip_id = t2_id, .start_time = Time(10, 0, 0), .end_time = Time(22, 0, 0),
    .headway_secs = t2_headsecs, .exact_times = gtfs::FrequencyTripService::FrequencyBased,
  };

  f1.add_frequency(freqBased);
  f1.add_frequency(schedBased);
  f1.write_frequencies(f1_path);

  Feed f1_reader(f1_path);
  Feed f2_reader(f2_path);
  f1_reader.read_feed();
  f2_reader.read_feed();

  // make sure files are actually written

  // feed1
  auto routes = f1_reader.get_routes();
  EXPECT_EQ(routes.size(), 1);
  EXPECT_EQ(routes[0].route_id, r1_id);

  auto trips = f1_reader.get_trips();
  EXPECT_EQ(trips.size(), 2);
  EXPECT_EQ(trips[0].trip_id, t1_id);

  auto stops = f1_reader.get_stops();
  EXPECT_EQ(stops.size(), 6);
  EXPECT_EQ(stops[0].stop_id, st1_id + "_egress");

  auto shapes = f1_reader.get_shapes();
  EXPECT_EQ(shapes.size(), 6);
  EXPECT_EQ(shapes[0].shape_id, sh1_id);

  auto calendarGTFS = f1_reader.get_calendar();
  EXPECT_EQ(calendarGTFS.size(), 2);
  EXPECT_EQ(calendarGTFS[0].service_id, sv1_id);

  auto calendarExceptions = f1_reader.get_calendar_dates();
  EXPECT_EQ(calendarExceptions.size(), 2);
  EXPECT_EQ(calendarExceptions[0].exception_type,
            gtfs::CalendarDateException::Added); // service added
  EXPECT_EQ(calendarExceptions[1].exception_type,
            gtfs::CalendarDateException::Removed); // service added

  auto frequencies = f1_reader.get_frequencies();
  EXPECT_EQ(frequencies.size(), 2);
  EXPECT_EQ(frequencies[0].exact_times, gtfs::FrequencyTripService::FrequencyBased);
  EXPECT_EQ(frequencies[1].exact_times, gtfs::FrequencyTripService::FrequencyBased);

  // feed2
  routes = f2_reader.get_routes();
  EXPECT_EQ(routes.size(), 1);
  EXPECT_EQ(routes[0].route_id, r2_id);

  trips = f2_reader.get_trips();
  EXPECT_EQ(trips.size(), 2);
  EXPECT_EQ(trips[0].trip_id, t3_id);

  stops = f2_reader.get_stops();
  EXPECT_EQ(stops.size(), 3);
  EXPECT_EQ(stops[0].stop_id, st2_id);

  shapes = f2_reader.get_shapes();
  EXPECT_EQ(shapes.size(), 0);

  calendarGTFS = f2_reader.get_calendar();
  EXPECT_EQ(calendarGTFS.size(), 1);
  EXPECT_EQ(calendarGTFS[0].service_id, sv2_id);

  calendarExceptions = f2_reader.get_calendar_dates();
  EXPECT_EQ(calendarExceptions.size(), 0);

  frequencies = f2_reader.get_frequencies();
  EXPECT_EQ(frequencies.size(), 0);
}

TEST(GtfsExample, MakeProto) {
  auto pt = get_config();

  // constants written in the last function
  auto serviceStartDate = (parse_date_time_string(sv_start) - DateTime::pivot_date_).count();
  auto serviceEndDate =
      (parse_date_time_string(sv_end) - DateTime::pivot_date_).count() + kSecondsPerDay - 1;
  auto addedDate = (parse_date_time_string(added_date) - DateTime::pivot_date_).count();
  auto removedDate = (parse_date_time_string(removed_date) - DateTime::pivot_date_).count();

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
        if (stop_pair.has_frequency_headway_seconds()) {
          headway_seconds.insert(stop_pair.frequency_headway_seconds());
        }
        for (const auto& added_date : stop_pair.service_added_dates()) {
          service_added_dates.insert(added_date);
        }
        for (const auto& except_date : stop_pair.service_except_dates()) {
          service_except_dates.insert(except_date);
        }

        // make sure:
        //   - the first stop pair has 0 as origin_dist_traveled
        //   - the last stop pair of tripOne has exactly 6.0f and tripTwo has the full shape length
        if (stop_pair.origin_onestop_id() == f1_name + "_" + st1_id + "_platform") {
          first_origin_dist_traveled.push_back(stop_pair.origin_dist_traveled());
        } else if (stop_pair.destination_onestop_id() == f1_name + "_" + st3_id) {
          last_dest_dist_traveled.push_back(stop_pair.destination_dist_traveled());
        }
      }
    }
  }

  EXPECT_EQ(shapes, 2);

  // routes
  EXPECT_TRUE(routes.find(f1_name + "_" + r1_id) != routes.end());
  EXPECT_TRUE(routes.find(f2_name + "_" + r2_id) != routes.end());
  EXPECT_EQ(routes.size(), 2);

  // stops
  std::string stopIds[4] = {f1_name + "_" + st1_id, f1_name + "_" + st2_id, f1_name + "_" + st3_id,
                            f2_name + "_" + st4_id};
  EXPECT_EQ(stops.size(), 15);
  for (const auto& stopID : stopIds) {
    EXPECT_TRUE(stops.find(stopID) != stops.end());
  }

  // stop_pairs, we have 5 in total since one stop_pair is across 2 tiles
  std::string stop_pair_ids[4] = {f1_name + "_" + st1_id + "_platform",
                                  f1_name + "_" + st2_id + "_platform", f1_name + "_" + st3_id,
                                  f2_name + "_" + st4_id};
  EXPECT_EQ(stop_pairs.size(), 5);
  for (const auto& stop_pair_id : stop_pair_ids) {
    EXPECT_TRUE(stops.find(stop_pair_id) != stops.end());
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

  EXPECT_EQ(headway_seconds.size(), 2);
  EXPECT_TRUE(headway_seconds.find(t1_headsecs) != headway_seconds.end());
  EXPECT_TRUE(headway_seconds.find(t2_headsecs) != headway_seconds.end());
}

TEST(GtfsExample, MakeTile) {
  boost::property_tree::ptree pt = get_config();

  auto layout = create_layout();
  auto station_one_ll = layout.find("1");
  auto station_two_ll = layout.find("2");
  auto station_three_ll = layout.find("3");
  auto station_four_ll = layout.find("4");

  // Tuesday, only to get the dow for getting the departures from tiles
  // it's fine to hard-code, would be more hassle to dynamically allocate
  // as we'd need to make sure we're not falling on a weekend (no service2)
  auto dt = "2023-03-28";
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
  std::unordered_set<uint32_t> routes_colors;
  std::unordered_set<std::string> routes_onestopids;
  std::unordered_set<std::string> deps_route_oid;
  // std::unordered_map<GraphId, baldr::TransitSchedule> schedules;

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
        EXPECT_EQ(dep->elapsed_time(), 180);
        const auto shape = tile->edgeinfo(&edge).encoded_shape();
        EXPECT_FALSE(shape.empty());
        dep->routeindex();
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
                                           station_three_ll->second, station_four_ll->second};
    for (const auto& node : tile->GetNodes()) {
      auto currNode_type = node_types.find(node.type());
      EXPECT_NE(node_types.end(), currNode_type);
      auto node_ll = node.latlng(tile->header()->base_ll());
      bool coordinates_found = false;
      for (const auto& station_coord : station_coords) {
        coordinates_found = coordinates_found || node_ll.ApproximatelyEqual(station_coord);
      }
      EXPECT_TRUE(coordinates_found);
      // some GTFS stops don't define a timezone, but we fall back to our own
      EXPECT_EQ(node.timezone(), 145); // America/Toronto
    }

    for (const auto& route_idx : {0, 1}) {
      // swallow exception for out-of-bounds
      try {
        const auto route = tile->GetTransitRoute(route_idx);
        if (!route) {
          break;
        }
        routes_colors.insert(route->route_color());
        routes_onestopids.insert(tile->GetName(route->one_stop_offset()));
      } catch (...) {};
    }

    for (const auto& departure : tile->GetTransitDepartures()) {
      TransitDeparture* currDeparture = departure.second;
      auto transit_route = tile->GetTransitRoute(currDeparture->routeindex());
      deps_route_oid.insert(tile->GetName(transit_route->one_stop_offset()));
    }

    for (uint32_t schedule_it = 0; schedule_it < tile->header()->schedulecount(); schedule_it++) {
      auto* tileSchedule = tile->GetTransitSchedule(schedule_it);
      // we either remove a date or we don't allow weekends, so it's neither all days, nor no days
      EXPECT_NE(tileSchedule->days(), (static_cast<int64_t>(1) << 60) - 1);
      EXPECT_NE(tileSchedule->days(), 0);
      // TODO: why is 59 and not 60? see service_days.cc::get_service_days: we add 59 days, not 60?
      EXPECT_EQ(tileSchedule->end_day(), 59);
    }
  }

  // test route onestop ids, also in the departures
  const auto onestopids = {f1_name + "_" + r1_id, f2_name + "_" + r2_id};
  for (const auto& oid : onestopids) {
    EXPECT_TRUE(routes_onestopids.find(oid) != routes_onestopids.end());
    EXPECT_TRUE(deps_route_oid.find(oid) != deps_route_oid.end());
  }
  // test route colors
  const auto colors = {"0000ff", "ff0000"};
  for (const auto& c : colors) {
    EXPECT_TRUE(routes_colors.find(strtol(c, nullptr, 16)) != routes_colors.end());
  }

  EXPECT_EQ(transit_nodes, 15);
  EXPECT_EQ(uses[Use::kRoad], 12);
  EXPECT_EQ(uses[Use::kTransitConnection], 20);
  EXPECT_EQ(uses[Use::kPlatformConnection], 10);
  EXPECT_EQ(uses[Use::kEgressConnection], 10);
  // TODO: this is the only time in the graph that we dont have opposing directed edges (should fix)
  EXPECT_EQ(uses[Use::kRail], 3);
}

TEST(GtfsExample, route_trip1) {
  // here we request with the relative current time for tmrw 05:50 am
  auto req_time = DateTime::iso_date_time(DateTime::get_tz_db().from_index(145));
  size_t tmrw_int = std::stoul(std::string(&req_time[8], &req_time[10])) + 1;
  // wrap around the next month if it's getting critical
  if (tmrw_int > 27) {
    auto new_month = std::to_string(std::stoul(std::string(&req_time[5], &req_time[7])) + 1);
    new_month = std::string(2 - std::min(2UL, new_month.length()), '0') + new_month;
    req_time.replace(req_time.find('T') - 5, 2, new_month);
    tmrw_int -= 20;
  }
  auto tmrw_str = std::to_string(tmrw_int);
  tmrw_str = std::string(2 - std::min(2UL, tmrw_str.length()), '0') + tmrw_str;
  // replace the day and the time
  req_time.replace(req_time.find('T') - 2, 2, tmrw_str);
  req_time.replace(req_time.find('T') + 1, 5, "05:50");

  std::string res_json;
  valhalla::Api res =
      gurka::do_action(valhalla::Options::route, map, {"A", "G"}, "multimodal",
                       {{"/date_time/type", "1"},
                        {"/date_time/value", req_time},
                        {"/costing_options/pedestrian/transit_start_end_max_distance", "20000"}},
                       {}, &res_json);

  // test the PBF output
  EXPECT_EQ(res.directions().routes().size(), 1);
  EXPECT_EQ(res.directions().routes(0).legs().size(), 1);

  const auto& leg = res.directions().routes(0).legs(0);
  EXPECT_NEAR(leg.summary().length(), 41.033, 0.001);
  EXPECT_EQ(leg.maneuver(0).type(), DirectionsLeg_Maneuver_Type_kStart);
  EXPECT_EQ(leg.maneuver(1).type(), DirectionsLeg_Maneuver_Type_kTransitConnectionStart);
  EXPECT_EQ(leg.maneuver(2).type(), DirectionsLeg_Maneuver_Type_kTransit);
  EXPECT_EQ(leg.maneuver(2).transit_type(), valhalla::TransitType::kMetro);

  const auto& transit_info = leg.maneuver(2).transit_info();
  EXPECT_EQ(transit_info.onestop_id(), f1_name + "_" + r1_id);
  EXPECT_EQ(transit_info.headsign(), "hello");
  EXPECT_EQ(transit_info.transit_stops().size(), 3);
  EXPECT_EQ(transit_info.transit_stops(0).type(), TransitPlatformInfo_Type_kStation);
  // TODO: JSON & PBF disagree: see TODO at beginning of triplegbuilder_util.h::AddTransitInfo
  // EXPECT_EQ(transit_info.transit_stops(0).onestop_id(), f1_name + "_" + st1_id);
  // EXPECT_EQ(transit_info.transit_stops(2).onestop_id(), f1_name + "_" + st3_id +
  // "_transit_station");

  EXPECT_EQ(transit_info.transit_stops(0).arrival_date_time(), "");
  EXPECT_EQ(transit_info.transit_stops(2).departure_date_time(), "");

  // test the JSON output
  rapidjson::Document doc;
  auto& res_doc = doc.Parse(res_json);
  EXPECT_EQ(res_doc["trip"]["legs"].GetArray().Size(), 1);

  const auto& leg_json = res_doc["trip"]["legs"].GetArray()[0];
  EXPECT_NEAR(leg_json["summary"]["time"].GetDouble(), 8769.058, 0.001);
  EXPECT_NEAR(leg_json["summary"]["length"].GetDouble(), 41.033, 0.001);
  EXPECT_EQ(leg_json["maneuvers"][0]["type"].GetUint(),
            static_cast<uint32_t>(DirectionsLeg::Maneuver::kStart));
  EXPECT_EQ(leg_json["maneuvers"][1]["type"].GetUint(),
            static_cast<uint32_t>(DirectionsLeg::Maneuver::kTransitConnectionStart));
  EXPECT_EQ(leg_json["maneuvers"][2]["type"].GetUint(),
            static_cast<uint32_t>(DirectionsLeg::Maneuver::kTransit));
  EXPECT_EQ(leg_json["maneuvers"][2]["travel_type"], "metro");

  const auto& ti_json = leg_json["maneuvers"][2]["transit_info"];
  EXPECT_EQ(ti_json["onestop_id"].GetString(), f1_name + "_" + r1_id);
  EXPECT_EQ(ti_json["headsign"], "hello");
  EXPECT_EQ(ti_json["transit_stops"].GetArray().Size(), 3);
  EXPECT_EQ(ti_json["transit_stops"][0]["type"], "station");
  EXPECT_FALSE(ti_json["transit_stops"][0].HasMember("arrival_date_time"));
  EXPECT_FALSE(ti_json["transit_stops"][2].HasMember("departure_date_time"));

  // determine the right day
  req_time.append("-04:00"); // TODO: why -04:00, not -05:00??
  req_time.replace(req_time.find('T') + 1, 5, "07:00");
  EXPECT_EQ(transit_info.transit_stops(0).departure_date_time(), req_time);
  EXPECT_EQ(ti_json["transit_stops"][0]["departure_date_time"].GetString(), req_time);

  req_time.replace(req_time.find('T') + 1, 5, "07:06");
  EXPECT_EQ(transit_info.transit_stops(2).arrival_date_time(), req_time);
  EXPECT_EQ(ti_json["transit_stops"][2]["arrival_date_time"].GetString(), req_time);
}

TEST(GtfsExample, route_trip4) {
  std::string res_json;
  valhalla::Api res =
      gurka::do_action(valhalla::Options::route, map, {"g", "h"}, "multimodal",
                       {{"/date_time/type", "1"},
                        {"/date_time/value", "2023-02-27T22:50"},
                        {"/costing_options/pedestrian/transit_start_end_max_distance", "20000"}},
                       {}, &res_json);

  // test the PBF output
  const auto& leg = res.directions().routes(0).legs(0);
  EXPECT_NEAR(leg.summary().time(), 8529.033, 0.001);
  EXPECT_NEAR(leg.summary().length(), 16.112, 0.001);

  const auto& transit_info = leg.maneuver(2).transit_info();
  EXPECT_EQ(transit_info.transit_stops(0).departure_date_time(), "2023-02-27T23:58-05:00");
  EXPECT_EQ(transit_info.transit_stops(1).arrival_date_time(), "2023-02-28T00:02-05:00");
  EXPECT_EQ(transit_info.headsign(), "grüß gott!");
  EXPECT_EQ(transit_info.onestop_id(), f2_name + "_" + r2_id);
}

TEST(GtfsExample, isochrones) {

  auto WaypointToBoostPoint = [&](std::string waypoint) {
    auto point = map.nodes[waypoint];
    return point_type(point.x(), point.y());
  };

  std::string res_string;
  valhalla::Api res =
      gurka::do_action(valhalla::Options::isochrone, map, {"g"}, "multimodal",
                       {{"/date_time/type", "1"},
                        {"/date_time/value", "2023-02-27T04:58"},
                        {"/contours/0/time", "120"},
                        {"/costing_options/pedestrian/transit_start_end_max_distance", "20000"}},
                       {}, &res_string);

  std::vector<PointLL> iso_polygon = polygon_from_geojson(res_string);
  polygon_type polygon;
  for (const auto& p : iso_polygon) {
    boost::geometry::append(polygon.outer(), point_type(p.x(), p.y()));
  }

  EXPECT_EQ(within(WaypointToBoostPoint("D"), polygon), true);
  EXPECT_EQ(within(WaypointToBoostPoint("2"), polygon), true);
  EXPECT_EQ(within(WaypointToBoostPoint("E"), polygon), true);
  EXPECT_EQ(within(WaypointToBoostPoint("4"), polygon), true);
  EXPECT_EQ(within(WaypointToBoostPoint("F"), polygon), true);
  EXPECT_EQ(within(WaypointToBoostPoint("1"), polygon), false);
}

TEST(GtfsExample, status) {
  std::string req = R"({"verbose": true})";
  std::string res_string;
  valhalla::Api res = gurka::do_action(valhalla::Options::status, map, req, {}, &res_string);
  EXPECT_NE(res_string.find(R"("has_transit_tiles":true)"), std::string::npos);
}
