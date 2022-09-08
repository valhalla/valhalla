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
      // make sure the right exception is thrown
    try {
      gurka::do_action(valhalla::Options::route, map, {"I", "L"}, costing,
                                          {{"/costing_options/" + costing + "/exclude_tolls", "0"}});

    } catch (const valhalla_exception_t& err) { EXPECT_EQ(err.code, 145); } catch (...) {
      FAIL() << "Expected valhalla_exception_t.";
    };
  }
}
