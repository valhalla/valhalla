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

//==========================================================================================
class ExcludeCashOnlyTollNodeTest : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double grid_size_meters = 10;

    const std::string ascii_map = R"(
        A-----------B-------------C-------------D-------------E
                    |                           |
                    |                           |
                    |                           |
                    F---------------------------G
    )";

    const gurka::ways ways = {
        {"AB", {{"highway", "primary"}}}, {"BC", {{"highway", "primary"}}},
        {"CD", {{"highway", "primary"}}}, {"DE", {{"highway", "primary"}}},
        {"BF", {{"highway", "primary"}}}, {"FG", {{"highway", "residential"}, {"surface", "gravel"}}},
        {"GD", {{"highway", "primary"}}},
    };

    const gurka::nodes nodes = {
        {"C", {{"barrier", "toll_booth"}, {"payment:coins", "true"}}},
    };

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, grid_size_meters);
    map = gurka::buildtiles(layout, ways, nodes, {}, "test/data/exclude_cash_only_toll_node");
  }
};

gurka::map ExcludeCashOnlyTollNodeTest::map = {};

TEST_F(ExcludeCashOnlyTollNodeTest, Basic) {
  // Without exclude_cash_only_tolls option
  {
    for (const auto& costing : kSupportedCostingModels) {
      const auto result = gurka::do_action(valhalla::Options::route, map, {"A", "E"}, costing);
      gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DE"});
    }
  }

  // exclude_cash_only_tolls=0
  {
    for (const auto& costing : kSupportedCostingModels) {
      const auto result =
          gurka::do_action(valhalla::Options::route, map, {"A", "E"}, costing,
                           {{"/costing_options/" + costing + "/exclude_cash_only_tolls", "0"}});
      gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DE"});
    }
  }

  // exclude_cash_only_tolls=1
  {
    for (const auto& costing : kSupportedCostingModels) {
      const auto result =
          gurka::do_action(valhalla::Options::route, map, {"A", "E"}, costing,
                           {{"/costing_options/" + costing + "/exclude_cash_only_tolls", "1"}});
      gurka::assert::raw::expect_path(result, {"AB", "BF", "FG", "GD", "DE"});
    }
  }

  // Force "no route": exclude_cash_only_tolls=1 && exclude_unpaved=1
  {
    for (const auto& costing : kSupportedCostingModels) {
      try {
        const auto result =
            gurka::do_action(valhalla::Options::route, map, {"A", "E"}, costing,
                             {{"/costing_options/" + costing + "/exclude_cash_only_tolls", "1"},
                              {"/costing_options/" + costing + "/exclude_unpaved", "1"}});
      } catch (const std::runtime_error& e) {
        EXPECT_STREQ(e.what(), "No path could be found for input");
      }
    }
  }
}
