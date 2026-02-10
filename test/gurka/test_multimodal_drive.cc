#include "gurka.h"
#include "test.h"
#include "valhalla/baldr/graphconstants.h"

#include <gtest/gtest.h>

using namespace valhalla;
using namespace baldr;

TEST(Standalone, SimpleRoute) {
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

  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "D"}, "auto_pedestrian", {});

  EXPECT_EQ(result.directions().routes(0).legs(0).maneuver_size(), 4);
  EXPECT_EQ(result.directions().routes(0).legs(0).maneuver(1).type(),
            DirectionsLeg_Maneuver::kParkVehicle);

  // travel mode changes along the route
  EXPECT_EQ(result.directions().routes(0).legs(0).maneuver(0).travel_mode(), TravelMode::kDrive);
  EXPECT_EQ(result.directions().routes(0).legs(0).maneuver(2).travel_mode(), TravelMode::kPedestrian);
}

TEST(Standalone, ChooseOptimalPath) {
  // 1 and 2 are parking nodes, A->D should
  // yield a parking maneuver at 2, which is the
  // closer parking spot
  const std::string ascii_map = R"(
      A--------------B-----------C-----------D
                     |           |
                     |           |
                     |           |
                     |           |
                     E-1------2--F
  )";

  const gurka::ways ways = {
      {"AB", {{"highway", "residential"}}},   {"BC", {{"highway", "residential"}}},
      {"CD", {{"highway", "residential"}}},   {"BE", {{"highway", "residential"}}},
      {"E12F", {{"highway", "residential"}}}, {"FC", {{"highway", "residential"}}},
  };

  // create two parking nodes
  const gurka::nodes nodes = {
      {"1", {{"amenity", "parking"}}},
      {"2", {{"amenity", "parking"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, nodes, {}, VALHALLA_BUILD_DIR "test/data/gurka_parking");

  auto reader = test::make_clean_graphreader(map.config.get_child("mjolnir"));
  for (const auto& n : {"1", "2"}) {
    auto node = gurka::findNode(*reader, layout, n);
    auto nodeinfo = reader->nodeinfo(node);

    EXPECT_EQ(nodeinfo->type(), baldr::NodeType::kParking)
        << "Expected parking, got " << baldr::to_string(nodeinfo->type());
    EXPECT_EQ(nodeinfo->access(), 2047)
        << "Expected vehicular and pedestrian access , got " << nodeinfo->access();
  }

  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "D"}, "auto_pedestrian", {});

  auto& leg = result.directions().routes(0).legs(0);
  EXPECT_EQ(leg.maneuver_size(), 6);
  bool has_parking_maneuver = false;
  for (const auto& man : leg.maneuver()) {
    if (man.type() == valhalla::DirectionsLeg_Maneuver_Type_kParkVehicle) {
      has_parking_maneuver = true;
      break;
    } else {
      std::cerr << "Man Type: " << man.type();
    }
  }
  EXPECT_TRUE(has_parking_maneuver);

  // travel mode changes along the route
  EXPECT_EQ(leg.maneuver(0).travel_mode(), TravelMode::kDrive);
  EXPECT_EQ(leg.maneuver(leg.maneuver_size() - 1).travel_mode(), TravelMode::kPedestrian);
  gurka::assert::raw::expect_path(result, {"AB", "BE", "E12F", "E12F", "E12F", "FC", "CD"});
}
