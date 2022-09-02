#include "gurka.h"

#include <gtest/gtest.h>

using namespace valhalla;

namespace {
const std::vector<std::string> kSupportedCostingModels = {
    "auto",
    "taxi",
    "bus",
    "truck",
    "pedestrian",
};
} // namespace

class ExcludeTollsTest : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double grid_size_meters = 100.;

    const std::string ascii_map = R"(
   E----F----G----H----I----A----J----K----L
                                 |    |
                                 |    |
                                 |    |
                                 |    |
                                 M----N
    )";

    const gurka::ways ways = {
        {"EF", {{"highway", "residential"}, {"toll", "yes"}}},
        {"FG", {{"highway", "residential"}, {"toll", "yes"}}},
        {"GH", {{"highway", "residential"}}},
        {"HI", {{"highway", "residential"}, {"toll", "yes"}}},
        {"IA", {{"highway", "residential"}}},
        {"IJ", {{"highway", "residential"}}},
        {"JK", {{"highway", "residential"}, {"toll", "yes"}}},
        {"KL", {{"highway", "residential"}}},
        {"JM", {{"highway", "residential"}}},
        {"MN", {{"highway", "residential"}}},
        {"NK", {{"highway", "residential"}}},
    };

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, grid_size_meters);
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/exclude_tolls");
  }
};

gurka::map ExcludeTollsTest::map = {};

TEST_F(ExcludeTollsTest, TollsInTheMiddle) {
  // Without options
  for (const auto& costing : kSupportedCostingModels) {
    const auto result = gurka::do_action(valhalla::Options::route, map, {"I", "L"}, costing);
    gurka::assert::raw::expect_path(result, {"IJ", "JK", "KL"});
  }

  // Use tolls
  for (const auto& costing : kSupportedCostingModels) {
    const auto result = gurka::do_action(valhalla::Options::route, map, {"I", "L"}, costing,
                                         {{"/costing_options/" + costing + "/exclude_tolls", "0"}});
    gurka::assert::raw::expect_path(result, {"IJ", "JK", "KL"});
  }

  // Do not use tolls
  for (const auto& costing : kSupportedCostingModels) {
    const auto result = gurka::do_action(valhalla::Options::route, map, {"I", "L"}, costing,
                                         {{"/costing_options/" + costing + "/exclude_tolls", "1"}});
    gurka::assert::raw::expect_path(result, {"IJ", "JM", "MN", "NK", "KL"});
  }
}

TEST_F(ExcludeTollsTest, TollsUnsupported) {
  const std::string start = "E";
  const std::string end = "L";
  for (const auto& costing : std::vector<std::string>{"bicycle"}) {
    const auto result_0 =
        gurka::do_action(valhalla::Options::route, map, {start, end}, costing,
                         {{"/costing_options/" + costing + "/exclude_tolls", "1"}});
    EXPECT_EQ(result_0.trip().routes_size(), 1);
    const auto result_1 =
        gurka::do_action(valhalla::Options::route, map, {start, end}, costing,
                         {{"/costing_options/" + costing + "/exclude_tolls", "0"}});
    EXPECT_EQ(result_1.trip().routes_size(), 1);
    EXPECT_EQ(gurka::detail::get_paths(result_0), gurka::detail::get_paths(result_1));
  }
}

TEST_F(ExcludeTollsTest, TollsInTheBeginning) {
  // Without options
  for (const auto& costing : kSupportedCostingModels) {
    const auto result = gurka::do_action(valhalla::Options::route, map, {"E", "H"}, costing);
    gurka::assert::raw::expect_path(result, {"EF", "FG", "GH"});
  }

  // Use tolls
  for (const auto& costing : kSupportedCostingModels) {
    const auto result = gurka::do_action(valhalla::Options::route, map, {"E", "H"}, costing,
                                         {{"/costing_options/" + costing + "/exclude_tolls", "0"}});
    gurka::assert::raw::expect_path(result, {"EF", "FG", "GH"});
  }

  // Do not use tolls
  for (const auto& costing : kSupportedCostingModels) {
    const auto result = gurka::do_action(valhalla::Options::route, map, {"E", "H"}, costing,
                                         {{"/costing_options/" + costing + "/exclude_tolls", "1"}});
    gurka::assert::raw::expect_path(result, {"EF", "FG", "GH"});
  }
}

TEST_F(ExcludeTollsTest, TollsInTheEnd) {
  // Without options
  for (const auto& costing : kSupportedCostingModels) {
    const auto result = gurka::do_action(valhalla::Options::route, map, {"H", "E"}, costing);
    gurka::assert::raw::expect_path(result, {"GH", "FG", "EF"});
  }

  // Use tolls
  for (const auto& costing : kSupportedCostingModels) {
    const auto result = gurka::do_action(valhalla::Options::route, map, {"H", "E"}, costing,
                                         {{"/costing_options/" + costing + "/exclude_tolls", "0"}});
    gurka::assert::raw::expect_path(result, {"GH", "FG", "EF"});
  }

  // Do not use tolls
  for (const auto& costing : kSupportedCostingModels) {
    const auto result = gurka::do_action(valhalla::Options::route, map, {"H", "E"}, costing,
                                         {{"/costing_options/" + costing + "/exclude_tolls", "1"}});
    gurka::assert::raw::expect_path(result, {"GH", "FG", "EF"});
  }
}
