#include "gurka.h"
#include <gtest/gtest.h>

using namespace valhalla;

const std::vector<std::string>& costing = {"auto",    "taxi",          "bus",        "truck",
                                           "bicycle", "motor_scooter", "motorcycle", "pedestrian"};

void validate_path(const valhalla::Api& result, const std::vector<std::string>& expected_names) {
  ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
  auto leg = result.trip().routes(0).legs(0);
  gurka::assert::raw::expect_path(result, expected_names);
}

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

  for (auto& c : costing) {
    if (c == "auto" || c == "taxi")
      validate_path(gurka::do_action(valhalla::Options::route, map, {"A", "I"}, c),
                    {"AB", "BC", "CD", "DE", "EH", "HL", "KL", "JK", "IJ"});
    else
      validate_path(gurka::do_action(valhalla::Options::route, map, {"A", "I"}, c),
                    {"AB", "BF", "FI"});
  }
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

  for (auto& c : costing) {
    validate_path(gurka::do_action(valhalla::Options::route, map, {"A", "I"}, c),
                  {"AB", "BC", "CD", "DG", "GK", "JK", "IJ"});
  }
}
