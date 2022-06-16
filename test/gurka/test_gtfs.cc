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
    .stop_id="foo",.stop_name=gtfs::Text("POINT NEMO"),
    .coordinates_present=true,.stop_lat=0.1,.stop_lon=0.1,
    .parent_station="0", .location_type=
  };
  feed.add_stop(nemo);
  feed.write_stops("../test/data/gtfs_test/");

  struct Route lineOne{
    .route_id="baz", .route_type=RouteType::Subway
  };
  feed.add_route(lineOne); 
  feed.write_routes("../test/data/gtfs_test/");

  struct gtfs::Trip tripOne{
    .route_id = "baz", .service_id= "bon", .trip_id="bar"
  }; 
  feed.add_trip(tripOne); 
  feed.write_trips("../test/data/gtfs_test/");

  struct StopTime stopTimeOne{
    .trip_id="bar", .departure_time=Time("6:00:00"), .stop_id="foo", .stop_sequence=0
  }; 
  feed.add_stop_time(stopTimeOne);
  feed.write_stop_times("../test/data/gtfs_test/");

  EXPECT_EQ(stopTimeOne.trip_id,"bar");
}

