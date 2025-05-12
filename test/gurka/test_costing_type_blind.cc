#include "gurka.h"
#include <gtest/gtest.h>

using namespace valhalla;

// Make sure we get instructions for passing a bridge
TEST(CostingTypeBlind, Bridge) {

  const std::string& ascii_map = R"(
        A---B-------C---D
    )";
  const gurka::ways ways = {
      {"AB",
       {
           {"name", "normal street"},
           {"highway", "footway"},
       }},
      {"BC", {{"name", ""}, {"highway", "footway"}, {"bridge", "yes"}}},
      {"CD",
       {
           {"name", "normal street"},
           {"highway", "footway"},
       }},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 50);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/costing_type_blind_bridge");
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "D"}, "pedestrian",
                                 {{"/costing_options/pedestrian/type", "blind"}});

  // Verify instructions for start and end maneuvers
  gurka::assert::raw::expect_instructions_at_maneuver_index(result, 1, "Continue on the bridge.", "",
                                                            "Continue on the bridge.",
                                                            "Continue on the bridge.",
                                                            "Continue for 400 meters.");
}

// Make sure we get instructions for passing a tunnel
TEST(CostingTypeBlind, Tunnel) {

  const std::string& ascii_map = R"(
        A---B-------C---D
    )";
  const gurka::ways ways = {
      {"AB",
       {
           {"name", "normal street"},
           {"highway", "footway"},
       }},
      {"BC", {{"name", ""}, {"highway", "footway"}, {"tunnel", "yes"}}},
      {"CD",
       {
           {"name", "normal street"},
           {"highway", "footway"},
       }},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 50);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/costing_type_blind_tunnel");
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "D"}, "pedestrian",
                                 {{"/costing_options/pedestrian/type", "blind"}});

  // Verify instructions for start and end maneuvers
  gurka::assert::raw::expect_instructions_at_maneuver_index(result, 1, "Continue on the tunnel.", "",
                                                            "Continue on the tunnel.",
                                                            "Continue on the tunnel.",
                                                            "Continue for 400 meters.");
}

// Make sure we get instructions for passing a traffic sign
TEST(CostingTypeBlind, TrafficSignal) {

  const std::string& ascii_map = R"(
            E
            |
        A---B-------C---D
            |
            F
    )";
  const gurka::ways ways = {
      {"AB",
       {
           {"name", "normal street"},
           {"highway", "footway"},
       }},
      {"BC", {{"name", ""}, {"highway", "footway"}}},
      {"CD",
       {
           {"name", "normal street"},
           {"highway", "footway"},
       }},
      {"BE",
       {
           {"name", "crossing street"},
           {"highway", "secondary"},
       }},
      {"BF",
       {
           {"name", "crossing street"},
           {"highway", "secondary"},
       }},
  };

  const gurka::nodes nodes = {{"B", {{"highway", "traffic_signals"}}}};

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 50);
  auto map = gurka::buildtiles(layout, ways, nodes, {}, "test/data/costing_type_blind_traffic_sign");
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "D"}, "pedestrian",
                                 {{"/costing_options/pedestrian/type", "blind"}});

  // Verify instructions for start and end maneuvers
  gurka::assert::raw::
      expect_instructions_at_maneuver_index(result, 1, "Pass traffic signals on crossing street.", "",
                                            "", "Pass traffic signals on crossing street.", "");
}

TEST(CostingTypeBlind, Bollard) {

  const std::string& ascii_map = R"(
        A---B-------C---D
    )";
  const gurka::ways ways = {
      {"AB",
       {
           {"name", "normal street"},
           {"highway", "footway"},
       }},
      {"BC", {{"name", "normal street"}, {"highway", "footway"}}},
      {"CD",
       {
           {"name", "normal street"},
           {"highway", "footway"},
       }},
  };

  const gurka::nodes nodes = {{"B", {{"barrier", "bollard"}}}};

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 50);
  auto map = gurka::buildtiles(layout, ways, nodes, {}, "test/data/costing_type_blind_bollard");
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "D"}, "pedestrian",
                                 {{"/costing_options/pedestrian/type", "blind"}});

  // Verify instructions for start and end maneuvers
  gurka::assert::raw::expect_instructions_at_maneuver_index(result, 1, "Pass the bollards.", "", "",
                                                            "Pass the bollards.", "");
}