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

  gurka::assert::raw::
      expect_maneuvers(result,
                       {
                           DirectionsLeg_Maneuver_Type::DirectionsLeg_Maneuver_Type_kStart,
                           DirectionsLeg_Maneuver_Type::DirectionsLeg_Maneuver_Type_kParkVehicle,
                           DirectionsLeg_Maneuver_Type::DirectionsLeg_Maneuver_Type_kDestination,
                       });

  // travel mode changes along the route
  auto& leg = result.directions().routes(0).legs(0);
  EXPECT_EQ(leg.maneuver(0).travel_mode(), TravelMode::kDrive);
  EXPECT_EQ(leg.maneuver(leg.maneuver_size() - 1).travel_mode(), TravelMode::kPedestrian);
}

TEST(Standalone, ChooseOptimalPath) {
  // 1 and 2 are parking nodes, A->D should
  // yield a parking maneuver at 2, which is the
  // closer parking spot
  const std::string ascii_map = R"(
      A--------------B-----------C-----------D
                     |           |
                     |           |
                     |     Z     |
                     |     |     |
                     E-1---X     |
                           |     |
                           Y--2--F
                           |
                           U
  )";

  const gurka::ways ways = {
      {"AB", {{"highway", "residential"}}},     {"BC", {{"highway", "residential"}}},
      {"CD", {{"highway", "residential"}}},     {"BE", {{"highway", "residential"}}},
      {"E1XY2F", {{"highway", "residential"}}}, {"FC", {{"highway", "residential"}}},
      {"XZ", {{"highway", "residential"}}},     {"YU", {{"highway", "residential"}}},
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

  gurka::assert::raw::
      expect_maneuvers(result,
                       {
                           DirectionsLeg_Maneuver_Type::DirectionsLeg_Maneuver_Type_kStart,
                           DirectionsLeg_Maneuver_Type::DirectionsLeg_Maneuver_Type_kRight,
                           DirectionsLeg_Maneuver_Type::DirectionsLeg_Maneuver_Type_kLeft,
                           DirectionsLeg_Maneuver_Type::DirectionsLeg_Maneuver_Type_kRight,
                           DirectionsLeg_Maneuver_Type::DirectionsLeg_Maneuver_Type_kLeft,
                           DirectionsLeg_Maneuver_Type::DirectionsLeg_Maneuver_Type_kParkVehicle,
                           DirectionsLeg_Maneuver_Type::DirectionsLeg_Maneuver_Type_kLeft,
                           DirectionsLeg_Maneuver_Type::DirectionsLeg_Maneuver_Type_kRight,
                           DirectionsLeg_Maneuver_Type::DirectionsLeg_Maneuver_Type_kDestination,
                       });
  auto& leg = result.directions().routes(0).legs(0);

  // travel mode changes along the route
  EXPECT_EQ(leg.maneuver(0).travel_mode(), TravelMode::kDrive);
  EXPECT_EQ(leg.maneuver(leg.maneuver_size() - 1).travel_mode(), TravelMode::kPedestrian);
  gurka::assert::raw::expect_path(result, {"AB", "BE", "E1XY2F", "E1XY2F", "E1XY2F", "E1XY2F",
                                           "E1XY2F", "FC", "CD"});
}

TEST(Standalone, TrivialRoute) {
  const std::string ascii_map = R"(
      A------2---1---B-----------C-----------D
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

  auto result = gurka::do_action(valhalla::Options::route, map, {"1", "2"}, "auto_pedestrian", {});

  gurka::assert::raw::
      expect_maneuvers(result,
                       {
                           DirectionsLeg_Maneuver_Type::DirectionsLeg_Maneuver_Type_kStart,
                           DirectionsLeg_Maneuver_Type::DirectionsLeg_Maneuver_Type_kParkVehicle,
                           DirectionsLeg_Maneuver_Type::DirectionsLeg_Maneuver_Type_kDestination,
                       });

  // travel mode changes along the route
  auto& leg = result.directions().routes(0).legs(0);
  EXPECT_EQ(leg.maneuver(0).travel_mode(), TravelMode::kDrive);
  EXPECT_EQ(leg.maneuver(leg.maneuver_size() - 1).travel_mode(), TravelMode::kPedestrian);
}
