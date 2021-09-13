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

TEST(Standalone, NodeAccess) {
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

  const gurka::nodes nodes = {{"F", {{"motor_vehicle", "no"}}}, {"G", {{"motorcar", "no"}}}};

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, nodes, {}, "test/data/gurka_node_access");

  for (auto& c : costing) {
    if (c == "auto" || c == "taxi")
      validate_path(gurka::do_action(valhalla::Options::route, map, {"A", "I"}, c),
                    {"AB", "BC", "CD", "DE", "EH", "HL", "KL", "JK", "IJ"});
    else if (c == "bicycle" || c == "pedestrian")
      validate_path(gurka::do_action(valhalla::Options::route, map, {"A", "I"}, c),
                    {"AB", "BF", "FI"});
    else
      validate_path(gurka::do_action(valhalla::Options::route, map, {"A", "I"}, c),
                    {"AB", "BC", "CD", "DG", "GK", "JK", "IJ"});
  }
}
