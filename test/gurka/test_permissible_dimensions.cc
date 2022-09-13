#include "gurka.h"
#include <gtest/gtest.h>

using namespace valhalla;

namespace {
const std::vector<std::string> kSupportedCostingTypes = {"auto", "truck", "bus", "taxi"};
const std::vector<std::string> kUnsupportedCostingTypes = {"bicycle", "pedestrian", "motorcycle",
                                                           "motor_scooter"};
} // namespace

TEST(Standalone, Maxwidth) {
  const std::string ascii_map = R"(
      A----B----C----D----E
           |         |    |
           |         |    |
           I----H----G----F
  )";

  const gurka::ways ways = {
      {"AB", {{"highway", "residential"}}},
      {"BC", {{"highway", "residential"}, {"maxwidth", "2.0"}}},
      {"CD", {{"highway", "residential"}, {"maxwidth", "2.0m"}}},
      {"DE", {{"highway", "residential"}}},
      {"EF", {{"highway", "residential"}}},
      {"FG", {{"highway", "residential"}}},
      {"GH", {{"highway", "residential"}}},
      {"HI", {{"highway", "residential"}}},
      {"DG", {{"highway", "residential"}, {"maxwidth", "1.9"}}},
      {"BI", {{"highway", "residential"}, {"maxwidth", "1.9m"}}},

  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  const auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_maxwidth");

  for (const auto& costing : kSupportedCostingTypes) {
    const auto result = gurka::do_action(valhalla::Options::route, map, {"A", "I"}, costing,
                                         {{"/costing_options/" + costing + "/width", "2.0"}});
    ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
    gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DE", "EF", "FG", "GH", "HI"});
  }

  for (const auto& costing : kSupportedCostingTypes) {
    const auto result = gurka::do_action(valhalla::Options::route, map, {"A", "I"}, costing,
                                         {{"/costing_options/" + costing + "/width", "1.8"}});
    ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
    gurka::assert::raw::expect_path(result, {"AB", "BI"});
  }

  for (const auto& costing : kSupportedCostingTypes) {
    const auto result = gurka::do_action(valhalla::Options::route, map, {"A", "I"}, costing,
                                         {{"/costing_options/" + costing + "/width", "1.9"}});
    ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
    gurka::assert::raw::expect_path(result, {"AB", "BI"});
  }

  for (const auto& costing : kSupportedCostingTypes) {
    EXPECT_THROW(gurka::do_action(valhalla::Options::route, map, {"A", "I"}, costing,
                                  {{"/costing_options/" + costing + "/width", "2.1"}}),
                 valhalla_exception_t);
  }

  for (const auto& costing : kUnsupportedCostingTypes) {
    const auto result = gurka::do_action(valhalla::Options::route, map, {"A", "I"}, costing);
    ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
    gurka::assert::raw::expect_path(result, {"AB", "BI"});
  }
}

TEST(Standalone, Maxheight) {
  const std::string ascii_map = R"(
      A----B----C----D----E
           |         |    |
           |         |    |
           I----H----G----F
  )";

  const gurka::ways ways = {
      {"AB", {{"highway", "residential"}}},
      {"BC", {{"highway", "residential"}, {"maxheight", "2.0"}}},
      {"CD", {{"highway", "residential"}, {"maxheight", "2.0m"}}},
      {"DE", {{"highway", "residential"}}},
      {"EF", {{"highway", "residential"}}},
      {"FG", {{"highway", "residential"}}},
      {"GH", {{"highway", "residential"}}},
      {"HI", {{"highway", "residential"}}},
      {"DG", {{"highway", "residential"}, {"maxheight", "1.9"}}},
      {"BI", {{"highway", "residential"}, {"maxheight", "1.9m"}}},

  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  const auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_maxheight");

  for (const auto& costing : kSupportedCostingTypes) {
    const auto result = gurka::do_action(valhalla::Options::route, map, {"A", "I"}, costing,
                                         {{"/costing_options/" + costing + "/height", "2.0"}});
    ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
    gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DE", "EF", "FG", "GH", "HI"});
  }

  for (const auto& costing : kSupportedCostingTypes) {
    const auto result = gurka::do_action(valhalla::Options::route, map, {"A", "I"}, costing,
                                         {{"/costing_options/" + costing + "/height", "1.8"}});
    ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
    gurka::assert::raw::expect_path(result, {"AB", "BI"});
  }

  for (const auto& costing : kSupportedCostingTypes) {
    const auto result = gurka::do_action(valhalla::Options::route, map, {"A", "I"}, costing,
                                         {{"/costing_options/" + costing + "/height", "1.9"}});
    ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
    gurka::assert::raw::expect_path(result, {"AB", "BI"});
  }

  for (const auto& costing : kSupportedCostingTypes) {
    EXPECT_THROW(gurka::do_action(valhalla::Options::route, map, {"A", "I"}, costing,
                                  {{"/costing_options/" + costing + "/height", "2.1"}}),
                 valhalla_exception_t);
  }

  for (const auto& costing : kUnsupportedCostingTypes) {
    const auto result = gurka::do_action(valhalla::Options::route, map, {"A", "I"}, costing);
    ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
    gurka::assert::raw::expect_path(result, {"AB", "BI"});
  }
}
