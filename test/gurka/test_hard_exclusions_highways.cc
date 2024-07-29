#include "gurka.h"

#include <gtest/gtest.h>

using namespace valhalla;

namespace {
const std::vector<std::string> kCostingModelsExcludeHighways = {"auto",          "taxi",
                                                                "bus",           "truck",
                                                                "pedestrian",    "bicycle",
                                                                "motor_scooter", "motorcycle"};

const std::vector<std::string> kCostingModelsNoHardExcludeSetA = {"auto", "taxi", "bus", "truck",
                                                                  "motorcycle"};

const std::vector<std::string> kCostingModelsNoHardExcludeSetB = {"pedestrian", "bicycle",
                                                                  "motor_scooter"};

const std::vector<std::string> kExclusionParameters = {"exclude_highways"};

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
    {"EF", {{"highway", "residential"}, {"bridge", "yes"}, {"tunnel", "yes"}, {"toll", "yes"}}},
    {"FG", {{"highway", "residential"}, {"bridge", "yes"}, {"tunnel", "yes"}, {"toll", "yes"}}},
    {"GH", {{"highway", "residential"}}},
    {"HI", {{"highway", "residential"}, {"bridge", "yes"}, {"tunnel", "yes"}, {"toll", "yes"}}},
    {"IA", {{"highway", "residential"}}},
    {"IJ", {{"highway", "residential"}}},
    {"JK", {{"highway", "motorway"}, {"bridge", "yes"}, {"tunnel", "yes"}, {"toll", "yes"}}},
    {"KL", {{"highway", "residential"}}},
    {"JM", {{"highway", "residential"}}},
    {"MN", {{"highway", "residential"}}},
    {"NK", {{"highway", "residential"}}},
};

const auto layout = gurka::detail::map_to_coordinates(ascii_map, grid_size_meters);

void check_result(const std::string& exclude_parameter_value,
                  const std::vector<std::string>& waypoints,
                  const std::vector<std::string>& expected_names,
                  const gurka::map& map,
                  const std::vector<std::string>& props = {}) {

  const std::string& costing = props[0];
  const std::string& exclude_parameter = props[1];
  const auto result = gurka::do_action(valhalla::Options::route, map, waypoints, costing,
                                       {{"/costing_options/" + costing + "/" + exclude_parameter,
                                         exclude_parameter_value}});
  gurka::assert::raw::expect_path(result, expected_names);
}

} // namespace

class ExclusionTestExcludeHighways : public ::testing::TestWithParam<std::vector<std::string>> {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/hard_exclude",
                            {{"service_limits.allow_hard_exclusions", "true"}});
  }
};

gurka::map ExclusionTestExcludeHighways::map = {};

TEST_P(ExclusionTestExcludeHighways, ExcludeHighways) {
  check_result("1", {"I", "L"}, {"IJ", "JM", "MN", "NK", "KL"}, map, GetParam());
}

INSTANTIATE_TEST_SUITE_P(ExcludeHighwaysTests,
                         ExclusionTestExcludeHighways,
                         ::testing::ValuesIn([]() {
                           std::vector<std::vector<std::string>> values;
                           for (const auto& costing : kCostingModelsExcludeHighways) {
                             for (const auto& param : kExclusionParameters) {
                               values.push_back({costing, param});
                             }
                           }
                           return values;
                         }()));

class ExclusionTestNoHardExcludeHighways : public ::testing::TestWithParam<std::vector<std::string>> {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/hard_exclude",
                            {{"service_limits.allow_hard_exclusions", "true"}});
  }
};

gurka::map ExclusionTestNoHardExcludeHighways::map = {};

TEST_P(ExclusionTestNoHardExcludeHighways, NoHardExcludeHighways) {
  check_result("0", {"I", "L"}, {"IJ", "JK", "KL"}, map, GetParam());
}

INSTANTIATE_TEST_SUITE_P(NoHardExcludeHighwaysTests,
                         ExclusionTestNoHardExcludeHighways,
                         ::testing::ValuesIn([]() {
                           std::vector<std::vector<std::string>> values;
                           for (const auto& costing : kCostingModelsNoHardExcludeSetA) {
                             for (const auto& param : kExclusionParameters) {
                               values.push_back({costing, param});
                             }
                           }
                           return values;
                         }()));

class ExclusionTestNoHardExcludeHighwaysForOtherModes
    : public ::testing::TestWithParam<std::vector<std::string>> {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/hard_exclude",
                            {{"service_limits.allow_hard_exclusions", "true"}});
  }
};

gurka::map ExclusionTestNoHardExcludeHighwaysForOtherModes::map = {};

TEST_P(ExclusionTestNoHardExcludeHighwaysForOtherModes, NoHardExcludeHighwaysForOtherModes) {
  check_result("0", {"I", "L"}, {"IJ", "JM", "MN", "NK", "KL"}, map, GetParam());
}

INSTANTIATE_TEST_SUITE_P(NoHardExcludeHighwaysForOtherModesTests,
                         ExclusionTestNoHardExcludeHighwaysForOtherModes,
                         ::testing::ValuesIn([]() {
                           std::vector<std::vector<std::string>> values;
                           for (const auto& costing : kCostingModelsNoHardExcludeSetB) {
                             for (const auto& param : kExclusionParameters) {
                               values.push_back({costing, param});
                             }
                           }
                           return values;
                         }()));
