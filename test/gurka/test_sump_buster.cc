#include "gurka.h"
#include <gtest/gtest.h>

using namespace valhalla;

TEST(Standalone, SumpBusterDefaults) {
  const std::string ascii_map = R"(
      A----B----C----D------------E
           |         |            |
           F         G            H
           |         |            |
           I----J----K------------L
  )";

  const gurka::ways ways = {
      {"AB", {{"highway", "residential"}}}, {"BC", {{"highway", "residential"}}},
      {"CD", {{"highway", "residential"}}}, {"DE", {{"highway", "residential"}}},
      {"BF", {{"highway", "residential"}}}, {"DG", {{"highway", "residential"}}},
      {"EH", {{"highway", "residential"}}}, {"FI", {{"highway", "residential"}}},
      {"GK", {{"highway", "residential"}}}, {"HL", {{"highway", "residential"}}},
      {"IJ", {{"highway", "residential"}}}, {"JK", {{"highway", "residential"}}},
      {"KL", {{"highway", "residential"}}},
  };

  const gurka::nodes nodes = {{"F", {{"barrier", "sump_buster"}}},
                              {"G", {{"barrier", "sump_buster"}}}};

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, nodes, {}, "test/data/gurka_sump_busterdefaults");
  auto result = gurka::route(map, "A", "I", "auto");
  ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
  auto leg = result.trip().routes(0).legs(0);
  gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DE", "EH", "HL", "KL", "JK", "IJ"});

  result = gurka::route(map, "A", "I", "hov");
  ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
  leg = result.trip().routes(0).legs(0);
  gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DE", "EH", "HL", "KL", "JK", "IJ"});

  result = gurka::route(map, "A", "I", "taxi");
  ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
  leg = result.trip().routes(0).legs(0);
  gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DE", "EH", "HL", "KL", "JK", "IJ"});

  result = gurka::route(map, "A", "I", "bus");
  ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
  leg = result.trip().routes(0).legs(0);
  gurka::assert::raw::expect_path(result, {"AB", "BF", "FI"});

  result = gurka::route(map, "A", "I", "truck");
  ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
  leg = result.trip().routes(0).legs(0);
  gurka::assert::raw::expect_path(result, {"AB", "BF", "FI"});

  result = gurka::route(map, "A", "I", "bicycle");
  ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
  leg = result.trip().routes(0).legs(0);
  gurka::assert::raw::expect_path(result, {"AB", "BF", "FI"});

  result = gurka::route(map, "A", "I", "motor_scooter");
  ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
  leg = result.trip().routes(0).legs(0);
  gurka::assert::raw::expect_path(result, {"AB", "BF", "FI"});

  result = gurka::route(map, "A", "I", "motorcycle");
  ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
  leg = result.trip().routes(0).legs(0);
  gurka::assert::raw::expect_path(result, {"AB", "BF", "FI"});

  result = gurka::route(map, "A", "I", "pedestrian");
  ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
  leg = result.trip().routes(0).legs(0);
  gurka::assert::raw::expect_path(result, {"AB", "BF", "FI"});
}

TEST(Standalone, SumpBusterOverride) {
  const std::string ascii_map = R"(
      A----B----C----D------------E
           |         |            |
           F         G            H
           |         |            |
           I----J----K------------L
  )";

  const gurka::ways ways = {
      {"AB", {{"highway", "residential"}}}, {"BC", {{"highway", "residential"}}},
      {"CD", {{"highway", "residential"}}}, {"DE", {{"highway", "residential"}}},
      {"BF", {{"highway", "residential"}}}, {"DG", {{"highway", "residential"}}},
      {"EH", {{"highway", "residential"}}}, {"FI", {{"highway", "residential"}}},
      {"GK", {{"highway", "residential"}}}, {"HL", {{"highway", "residential"}}},
      {"IJ", {{"highway", "residential"}}}, {"JK", {{"highway", "residential"}}},
      {"KL", {{"highway", "residential"}}},
  };

  const gurka::nodes nodes = {{"F",
                               {{"barrier", "sump_buster"},
                                {"bus", "no"},
                                {"hgv", "no"},
                                {"motorcycle", "no"},
                                {"moped", "no"},
                                {"foot", "no"},
                                {"bicycle", "no"}}},
                              {"G", {{"barrier", "sump_buster"}, {"motorcar", "yes"}}}};

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, nodes, {}, "test/data/gurka_sump_busteroverride");
  auto result = gurka::route(map, "A", "I", "auto");
  ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
  auto leg = result.trip().routes(0).legs(0);
  gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DG", "GK", "JK", "IJ"});

  result = gurka::route(map, "A", "I", "hov");
  ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
  leg = result.trip().routes(0).legs(0);
  gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DG", "GK", "JK", "IJ"});

  result = gurka::route(map, "A", "I", "taxi");
  ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
  leg = result.trip().routes(0).legs(0);
  gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DG", "GK", "JK", "IJ"});

  result = gurka::route(map, "A", "I", "bus");
  ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
  leg = result.trip().routes(0).legs(0);
  gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DG", "GK", "JK", "IJ"});

  result = gurka::route(map, "A", "I", "truck");
  ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
  leg = result.trip().routes(0).legs(0);
  gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DG", "GK", "JK", "IJ"});

  result = gurka::route(map, "A", "I", "bicycle");
  ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
  leg = result.trip().routes(0).legs(0);
  gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DG", "GK", "JK", "IJ"});

  result = gurka::route(map, "A", "I", "motor_scooter");
  ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
  leg = result.trip().routes(0).legs(0);
  gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DG", "GK", "JK", "IJ"});

  result = gurka::route(map, "A", "I", "motorcycle");
  ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
  leg = result.trip().routes(0).legs(0);
  gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DG", "GK", "JK", "IJ"});

  result = gurka::route(map, "A", "I", "pedestrian");
  ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
  leg = result.trip().routes(0).legs(0);
  gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DG", "GK", "JK", "IJ"});
}
