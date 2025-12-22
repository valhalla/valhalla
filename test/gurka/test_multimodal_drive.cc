#include "gurka.h"
#include "test.h"
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
  auto map = gurka::buildtiles(layout, ways, nodes, {}, VALHALLA_BUILD_DIR "test/data/gurka_parking");

  auto reader = test::make_clean_graphreader(map.config.get_child("mjolnir"));
  auto node = gurka::findNode(*reader, layout, "B");
  auto nodeinfo = reader->nodeinfo(node);

  EXPECT_EQ(nodeinfo->type(), baldr::NodeType::kParking)
      << "Expected parking, got " << baldr::to_string(nodeinfo->type());
  EXPECT_EQ(nodeinfo->access(), 2047)
      << "Expected vehicular and pedestrian access , got " << nodeinfo->access();

  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "D"}, "auto_walk", {});

  EXPECT_EQ(result.directions().routes(0).legs(0).maneuver_size(), 4);
  EXPECT_EQ(result.directions().routes(0).legs(0).maneuver(1).type(),
            DirectionsLeg_Maneuver::kParkVehicle);

  // travel mode changes along the route
  EXPECT_EQ(result.directions().routes(0).legs(0).maneuver(0).travel_mode(), TravelMode::kDrive);
  EXPECT_EQ(result.directions().routes(0).legs(0).maneuver(2).travel_mode(), TravelMode::kPedestrian);
}
