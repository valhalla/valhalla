#include "gurka.h"

#include <gtest/gtest.h>

using namespace valhalla;

namespace {
const std::vector<std::string> kSupportedCostingModels = {"auto", "taxi", "bus", "truck",
                                                          "motorcycle"};

const std::vector<std::string> kExclusionParameters = {"exclude_ferries"};

constexpr double grid_size_meters = 100.;

const std::string ascii_map = R"(
    A----1---B----C--D------E
  )";

const gurka::ways ways = {
    {"AB", {{"highway", "secondary"}}},
    {"BC", {{"route", "ferry"}}},
    {"CD", {{"highway", "secondary"}}},
    {"DE", {{"highway", "secondary"}}},
};

const auto layout = gurka::detail::map_to_coordinates(ascii_map, grid_size_meters);

void check_result(const std::string& exclude_parameter_value,
                  const std::vector<std::string>& waypoints,
                  const std::vector<std::string>& expected_names,
                  const gurka::map& map,
                  const std::vector<std::string>& props = {}) {

  const std::string& costing = props[0];
  const std::string& exclude_parameter = props[1];
  if (exclude_parameter_value == "1") {
    try {
      const auto result = gurka::do_action(valhalla::Options::route, map, waypoints, costing,
                                           {{"/costing_options/" + costing + "/" + exclude_parameter,
                                             exclude_parameter_value}});
      FAIL() << "Expected no path to be found";
    } catch (valhalla_exception_t& e) { EXPECT_EQ(e.code, 442); } catch (...) {
      FAIL() << "Failed with unexpected error code";
    }
  }

  if (exclude_parameter_value == "0") {
    const auto result = gurka::do_action(valhalla::Options::route, map, waypoints, costing,
                                         {{"/costing_options/" + costing + "/" + exclude_parameter,
                                           exclude_parameter_value}});

    gurka::assert::raw::expect_path(result, expected_names);
  }
}
} // namespace

class ExclusionTest : public ::testing::TestWithParam<std::vector<std::string>> {
protected:
  static gurka::map map;
  static gurka::map mapNotAllowed;

  static void SetUpTestSuite() {

    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/hard_exclude",
                            {{"service_limits.allow_hard_exclusions", "true"}});
    mapNotAllowed = gurka::buildtiles(layout, ways, {}, {}, "test/data/hard_exclude");
  }
};

gurka::map ExclusionTest::map = {};
gurka::map ExclusionTest::mapNotAllowed = {};

TEST_P(ExclusionTest, ExcludeFerries) {
  check_result("0", {"A", "D"}, {"AB", "BC", "CD"}, map, GetParam());
  check_result("1", {"A", "D"}, {"AB", "BC", "CD"}, map, GetParam());
}

INSTANTIATE_TEST_SUITE_P(ExcludePropsTest, ExclusionTest, ::testing::ValuesIn([]() {
                           std::vector<std::vector<std::string>> values;
                           for (const auto& costing : kSupportedCostingModels) {
                             for (const auto& param : kExclusionParameters) {
                               values.push_back({costing, param});
                             }
                           }
                           return values;
                         }()));
