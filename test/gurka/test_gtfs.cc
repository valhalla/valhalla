#include "just_gtfs/just_gtfs.h"
#include "gurka.h"

#include <gtest/gtest.h>
#include "string.h"

using namespace gtfs; 
using namespace valhalla; 


TEST(GtfsExample, ReadGtfs){
  Feed feed("../test/data/gtfs_sample"); 
  EXPECT_EQ(feed.read_agencies().code, ResultCode::OK);
  const auto & agencies = feed.get_agencies(); 
  EXPECT_EQ(agencies.size(), 1);
  EXPECT_EQ(agencies[0].agency_id, "DTA");
}

TEST(GtfsExample, WriteGtfs) {

  Feed feed; 

  struct Agency ttc {
    "TTC","Toronto Commission","http://www.ttc.ca","America/Toronto"
    //agency_id, agency_name, agency_url, agency_timezone
  }; 
  feed.add_agency(ttc);
  feed.write_agencies("../test/data/gtfs_test/");

  struct gtfs::Stop nemo{
    .stop_id="foo",
    .coordinates_present=true,.stop_lat=0.1,.stop_lon=0.1,
    .parent_station="0", .location_type= gtfs::StopLocationType::StopOrPlatform,
    .stop_name=gtfs::Text("POINT NEMO"), .stop_timezone="America/Toronto", 
    .wheelchair_boarding = "1", 
    
  };
  feed.add_stop(nemo);
  feed.write_stops("../test/data/gtfs_test/");

  struct Route lineOne{
    .route_id="baz", .route_type=RouteType::Subway, .route_short_name="ba", .route_long_name="bababa",
    .route_desc="this is the first route", .route_color="blue", .route_text_color="black",
  };
  feed.add_route(lineOne); 
  feed.write_routes("../test/data/gtfs_test/");

  struct gtfs::Trip tripOne{
    .route_id = "baz", .service_id= "bon", .trip_id="bar",
    .trip_headsign="haha", .block_id="block", .shape_id="square", 
    .wheelchair_accessible=gtfs::TripAccess::Yes, .bikes_allowed=gtfs::TripAccess::No, 
  }; 
  feed.add_trip(tripOne); 
  feed.write_trips("../test/data/gtfs_test/");

  struct StopTime stopTimeOne{
    .trip_id="bar", .arrival_time=Time("6:00:00"), .departure_time=Time("6:00:00"), .stop_id="foo", .stop_sequence=0,
    .stop_headsign="head", .shape_dist_traveled=0.0, .timepoint=gtfs::StopTimePoint::Exact, 
  }; 
  feed.add_stop_time(stopTimeOne);
  feed.write_stop_times("../test/data/gtfs_test/");

  struct CalendarItem calendarOne{
    .service_id= "bon",// connected to trip
    .monday = CalendarAvailability::Available, .tuesday=CalendarAvailability::Available, 
    .wednesday=CalendarAvailability::Available, .thursday=CalendarAvailability::Available, 
    .friday=CalendarAvailability::Available, .saturday=CalendarAvailability::Available, .sunday=CalendarAvailability::Available, 
    .start_date=Date(2022,1,31), .end_date=Date(2022,2,1), 

  }; 
  feed.add_calendar_item(calendarOne); 
  feed.write_calendar("../test/data/gtfs_test/"); 

  struct CalendarDate dateOne{
    .service_id= "bon", //connected to trip
    .date=Date(2022,2,2), 
    .exception_type=gtfs::CalendarDateException::Added, 
  };
  feed.add_calendar_date(dateOne); 
  feed.write_calendar_dates("../test/data/gtfs_test/"); 

  struct ShapePoint shapeOne{
    .shape_id="square", .shape_pt_lat=0.2,.shape_pt_lon=0.2, .shape_pt_sequence=1,
    .shape_id="square", .shape_pt_lat=0.21,.shape_pt_lon=0.21, .shape_pt_sequence=2,
    .shape_id="square", .shape_pt_lat=0.22,.shape_pt_lon=0.22, .shape_pt_sequence=3,
    .shape_id="square", .shape_pt_lat=0.23,.shape_pt_lon=0.23, .shape_pt_sequence=4,
  };
  feed.add_shape(shapeOne); 
  feed.write_shapes("../test/data/gtfs_test/"); 

  struct Frequency freqOne{
    .trip_id="bar", .start_time=Time(0,0,0), .end_time=Time(2,0,0),.headway_secs=1800,
    .exact_times=gtfs::FrequencyTripService::FrequencyBased, 
  };
  feed.add_frequency(freqOne); 
  feed.write_frequencies("../test/data/gtfs_test/"); 

  EXPECT_EQ(stopTimeOne.trip_id,"bar");
}

