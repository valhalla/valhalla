#include "gurka.h"
#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;

const std::unordered_map<std::string, std::string> build_config{
    {"mjolnir.admin", {VALHALLA_SOURCE_DIR "test/data/netherlands_admin.sqlite"}}};

const std::vector<std::string>& costing = {"auto",    "taxi",          "bus",        "truck",
                                           "bicycle", "motor_scooter", "motorcycle", "pedestrian"};

TEST(Standalone, Oneway) {
  constexpr double gridsize_metres = 100;

  const std::string ascii_map = R"(
               
        A---B---C---D---E
                    |   |
        J---I---H---G---F
    )";

  const gurka::ways ways = {
      {"AB", {{"highway", "unclassified"}}},
      {"BC", {{"highway", "unclassified"}}},
      {"CD", {{"highway", "unclassified"}}},
      {"DE", {{"highway", "unclassified"}}},
      {"EF", {{"highway", "unclassified"}}},
      {"FG", {{"highway", "unclassified"}}},
      {"DG",
       {{"highway", "unclassified"},
        {"oneway", "yes"},
        {"oneway:bicycle", "yes"},
        {"oneway:foot", "yes"}}},
      {"GH", {{"highway", "unclassified"}}},
      {"HI", {{"highway", "unclassified"}}},
      {"IJ", {{"highway", "unclassified"}}},

  };

  const auto layout =
      gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {5.1079374, 52.0887174});
  auto map =
      gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_oneway_unclassified", build_config);
  for (auto& c : costing) {
    auto result = gurka::do_action(valhalla::Options::route, map, {"A", "J"}, c);
    gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DG", "GH", "HI", "IJ"});
  }

  for (auto const& c : costing) {
    auto result = gurka::do_action(valhalla::Options::route, map, {"J", "A"}, c);
    gurka::assert::raw::expect_path(result, {"IJ", "HI", "GH", "FG", "EF", "DE", "CD", "BC", "AB"});
  }
}

TEST(Standalone, OnewayFootway) {
  constexpr double gridsize_metres = 10;

  const std::string ascii_map = R"(
               
        A---B---C---D---E
                    |   |
        J---I---H---G---F
    )";

  const gurka::ways ways = {
      {"AB", {{"highway", "unclassified"}}},
      {"BC", {{"highway", "unclassified"}}},
      {"CD", {{"highway", "unclassified"}}},
      {"DE", {{"highway", "unclassified"}}},
      {"EF", {{"highway", "unclassified"}}},
      {"FG", {{"highway", "unclassified"}}},
      {"DG",
       {{"highway", "footway"},
        {"bicycle", "yes"},
        {"oneway:bicycle", "yes"},
        {"oneway:foot", "yes"}}},
      {"GH", {{"highway", "unclassified"}}},
      {"HI", {{"highway", "unclassified"}}},
      {"IJ", {{"highway", "unclassified"}}},

  };

  const auto layout =
      gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {5.1079374, 52.0887174});
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_oneway_footway", build_config);
  for (auto& c : costing) {
    auto result = gurka::do_action(valhalla::Options::route, map, {"A", "J"}, c);
    if (c == "pedestrian" || c == "bicycle")
      gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DG", "GH", "HI", "IJ"});
    else
      gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DE", "EF", "FG", "GH", "HI", "IJ"});
  }

  for (auto const& c : costing) {
    auto result = gurka::do_action(valhalla::Options::route, map, {"J", "A"}, c);
    gurka::assert::raw::expect_path(result, {"IJ", "HI", "GH", "FG", "EF", "DE", "CD", "BC", "AB"});
  }
}

TEST(Standalone, OnewayFootwayAllowBike) {
  constexpr double gridsize_metres = 10;

  const std::string ascii_map = R"(
               
        A---B---C---D---E
                    |   |
        J---I---H---G---F
    )";

  const gurka::ways ways = {
      {"AB", {{"highway", "unclassified"}}},
      {"BC", {{"highway", "unclassified"}}},
      {"CD", {{"highway", "unclassified"}}},
      {"DE", {{"highway", "unclassified"}}},
      {"EF",
       {{"highway", "unclassified"},
        {"oneway", "yes"},
        {"bicycle", "no"}}}, // tend not to favor footways, so make the costing use it
      {"FG", {{"highway", "unclassified"}}},
      {"DG", {{"highway", "footway"}, {"bicycle", "yes"}, {"oneway", "yes"}}},
      {"GH", {{"highway", "unclassified"}}},
      {"HI", {{"highway", "unclassified"}}},
      {"IJ", {{"highway", "unclassified"}}},

  };

  const auto layout =
      gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {5.1079374, 52.0887174});
  auto map =
      gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_oneway_footway_bike", build_config);
  for (auto& c : costing) {
    auto result = gurka::do_action(valhalla::Options::route, map, {"A", "J"}, c);

    if (c == "pedestrian" || c == "bicycle")
      gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DG", "GH", "HI", "IJ"});
    else
      gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DE", "EF", "FG", "GH", "HI", "IJ"});
  }
}

TEST(Standalone, OnewayFlipped) {
  constexpr double gridsize_metres = 10;

  const std::string ascii_map = R"(
               
        A---B---C---D---E
                    |   |
        J---I---H---G---F
    )";

  const gurka::ways ways = {
      {"AB", {{"highway", "unclassified"}}},
      {"BC", {{"highway", "unclassified"}}},
      {"CD", {{"highway", "unclassified"}}},
      {"DE", {{"highway", "unclassified"}}},
      {"EF", {{"highway", "unclassified"}}},
      {"FG", {{"highway", "unclassified"}}},
      {"DG",
       {{"highway", "footway"}, {"bicycle", "yes"}, {"oneway:bicycle", "-1"}, {"oneway:foot", "-1"}}},
      {"GH", {{"highway", "unclassified"}}},
      {"HI", {{"highway", "unclassified"}}},
      {"IJ", {{"highway", "unclassified"}}},

  };

  const auto layout =
      gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {5.1079374, 52.0887174});
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_oneway_flipped", build_config);
  for (auto& c : costing) {
    auto result = gurka::do_action(valhalla::Options::route, map, {"A", "J"}, c);

    gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DE", "EF", "FG", "GH", "HI", "IJ"});
  }
}
