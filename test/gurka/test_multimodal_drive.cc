#include "gurka.h"
#include "test.h"
#include "tyr/actor.h"
#include "valhalla/baldr/graphconstants.h"

#include <gtest/gtest.h>

using namespace valhalla;
using namespace baldr;

TEST(Standalone, ParseParking) {
  const std::string ascii_map = R"(
      A--------------B-----------C-----------D
  )";

  const gurka::ways ways = {
      {"AB", {{"highway", "residential"}}},
      {"BC", {{"highway", "residential"}}},
      {"CD", {{"highway", "residential"}}},
  };

  const gurka::nodes nodes = {{"B", {{"amenity", "parking"}}}};

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, nodes, {}, "test/data/gurka_parking");

  auto reader = test::make_clean_graphreader(map.config.get_child("mjolnir"));
  auto node = gurka::findNode(*reader, layout, "B");
  auto nodeinfo = reader->nodeinfo(node);

  EXPECT_EQ(nodeinfo->type(), baldr::NodeType::kParking)
      << "Expected parking, got " << baldr::to_string(nodeinfo->type());
  EXPECT_EQ(nodeinfo->access(), 2047)
      << "Expected vehicular and pedestrian access , got " << nodeinfo->access();

  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "D"}, "multimodal_drive", {});

  EXPECT_EQ(result.directions().routes(0).legs(0).maneuver_size(), 3);
  EXPECT_EQ(result.directions().routes(0).legs(0).maneuver(1).type(),
            DirectionsLeg_Maneuver::kParkVehicle);

  // travel mode changes along the route
  EXPECT_EQ(result.directions().routes(0).legs(0).maneuver(0).travel_mode(), TravelMode::kDrive);
  EXPECT_EQ(result.directions().routes(0).legs(0).maneuver(2).travel_mode(), TravelMode::kPedestrian);
}

TEST(Standalone, Utrecht) {
  const auto utrecht_conf = test::make_config(VALHALLA_BUILD_DIR "test/data/utrecht_tiles");
  auto reader = test::make_clean_graphreader(utrecht_conf.get_child("mjolnir"));
  tyr::actor_t actor(utrecht_conf);

  // Request the same tile without shortcuts (default)
  std::string request =
      R"({"locations": [{"lat": 52.105031, "lon": 5.077844}, {"lat": 52.0942903, "lon": 5.1300778}], "costing": "multimodal_drive", "format": "pbf"})";
  auto r = actor.route(request);
  valhalla::Api response;

  response.ParseFromString(r);
  ASSERT_EQ(response.directions().routes_size(), 1);
  EXPECT_EQ(response.directions().routes(0).legs(0).maneuver(0).travel_mode(), TravelMode::kDrive);
  EXPECT_EQ(response.directions().routes(0).legs(0).maneuver(20).travel_mode(),
            TravelMode::kPedestrian);
  actor.cleanup();
}
