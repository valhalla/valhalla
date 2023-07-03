#include "gurka.h"

#include <gtest/gtest.h>

using namespace valhalla;

namespace {
const std::vector<std::string> kSupportedCostingModels = {
    "auto",
    "taxi",
    "bus",
    "truck",
};
} // namespace

class ExcludeUnpavedTest : public ::testing::Test {
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
        {"EF", {{"highway", "residential"}, {"surface", "compacted"}}},
        {"FG", {{"highway", "residential"}, {"surface", "compacted"}}},
        {"GH", {{"highway", "residential"}}},
        {"HI", {{"highway", "residential"}, {"surface", "unpaved"}}},
        {"IA", {{"highway", "residential"}, {"surface", "unpaved"}}},
        {"IJ", {{"highway", "residential"}}},
        {"JK", {{"highway", "residential"}, {"surface", "gravel"}}},
        {"KL", {{"highway", "residential"}}},
        {"JM", {{"highway", "residential"}}},
        {"MN", {{"highway", "residential"}}},
        {"NK", {{"highway", "residential"}}},
    };

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, grid_size_meters);
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/exclude_unpaved");
  }
};

gurka::map ExcludeUnpavedTest::map = {};

TEST_F(ExcludeUnpavedTest, UnpavedRoadsInTheMiddle) {
  // Without options
  for (const auto& costing : kSupportedCostingModels) {
    const auto result = gurka::do_action(valhalla::Options::route, map, {"I", "L"}, costing);
    gurka::assert::raw::expect_path(result, {"IJ", "JK", "KL"});
  }

  // Use unpaved roads
  for (const auto& costing : kSupportedCostingModels) {
    const auto result = gurka::do_action(valhalla::Options::route, map, {"I", "L"}, costing,
                                         {{"/costing_options/" + costing + "/exclude_unpaved", "0"}});
    gurka::assert::raw::expect_path(result, {"IJ", "JK", "KL"});
  }

  // Do not use unpaved roads
  for (const auto& costing : kSupportedCostingModels) {
    const auto result = gurka::do_action(valhalla::Options::route, map, {"I", "L"}, costing,
                                         {{"/costing_options/" + costing + "/exclude_unpaved", "1"}});
    gurka::assert::raw::expect_path(result, {"IJ", "JM", "MN", "NK", "KL"});
  }
}

TEST_F(ExcludeUnpavedTest, UnpavedRoadsUnsupported) {
  const std::string start = "E";
  const std::string end = "L";
  for (const auto& costing : std::vector<std::string>{"bicycle", "pedestrian"}) {
    const auto result_0 =
        gurka::do_action(valhalla::Options::route, map, {start, end}, costing,
                         {{"/costing_options/" + costing + "/exclude_unpaved", "1"}});
    EXPECT_EQ(result_0.trip().routes_size(), 1);
    const auto result_1 =
        gurka::do_action(valhalla::Options::route, map, {start, end}, costing,
                         {{"/costing_options/" + costing + "/exclude_unpaved", "0"}});
    EXPECT_EQ(result_1.trip().routes_size(), 1);
    EXPECT_EQ(gurka::detail::get_paths(result_0), gurka::detail::get_paths(result_1));
  }

  // motor_scooter, motorcycle are unsupported costing models too, but the engine does not get routes
  // through unpaved roads. These edges are not allowed in the Allowed and ReverseAllowed methods. It
  // includes only roads that have surface greater than Surface::kDirt(See kMinimumMotorcycleSurface,
  // kMinimumScooterSurface constants).
}

TEST_F(ExcludeUnpavedTest, UnpavedRoadsInTheBeginning) {
  // Without options
  for (const auto& costing : kSupportedCostingModels) {
    const auto result = gurka::do_action(valhalla::Options::route, map, {"E", "H"}, costing);
    gurka::assert::raw::expect_path(result, {"EF", "FG", "GH"});
  }

  // Use unpaved roads
  for (const auto& costing : kSupportedCostingModels) {
    const auto result = gurka::do_action(valhalla::Options::route, map, {"E", "H"}, costing,
                                         {{"/costing_options/" + costing + "/exclude_unpaved", "0"}});
    gurka::assert::raw::expect_path(result, {"EF", "FG", "GH"});
  }

  // Do not use unpaved roads
  for (const auto& costing : kSupportedCostingModels) {
    const auto result = gurka::do_action(valhalla::Options::route, map, {"E", "H"}, costing,
                                         {{"/costing_options/" + costing + "/exclude_unpaved", "1"}});
    gurka::assert::raw::expect_path(result, {"EF", "FG", "GH"});
  }
}

TEST_F(ExcludeUnpavedTest, UnpavedRoadsInTheEnd) {
  // Without options
  for (const auto& costing : kSupportedCostingModels) {
    const auto result = gurka::do_action(valhalla::Options::route, map, {"G", "A"}, costing);
    gurka::assert::raw::expect_path(result, {"GH", "HI", "IA"});
  }

  // Use unpaved roads
  for (const auto& costing : kSupportedCostingModels) {
    const auto result = gurka::do_action(valhalla::Options::route, map, {"G", "A"}, costing,
                                         {{"/costing_options/" + costing + "/exclude_unpaved", "0"}});
    gurka::assert::raw::expect_path(result, {"GH", "HI", "IA"});
  }

  // Do not use unpaved roads
  for (const auto& costing : kSupportedCostingModels) {
    const auto result = gurka::do_action(valhalla::Options::route, map, {"G", "A"}, costing,
                                         {{"/costing_options/" + costing + "/exclude_unpaved", "1"}});
    gurka::assert::raw::expect_path(result, {"GH", "HI", "IA"});
  }
}
