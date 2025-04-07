#include "gurka.h"

#include <gtest/gtest.h>

using namespace valhalla;

namespace {
const std::vector<std::string> kCostingModelsExcludeHighways = {"auto", "taxi", "bus", "truck",
                                                                "motorcycle"};

const std::vector<std::string> kCostingModelsNoHardExcludeSetA = {"auto", "taxi", "bus", "truck",
                                                                  "motorcycle"};

const std::vector<std::string> kCostingModelsNoHardExcludeSetB = {"pedestrian", "bicycle",
                                                                  "motor_scooter"};

const std::vector<std::string> kExclusionParameters = {"exclude_highways"};

constexpr double grid_size_meters = 100.;

const std::string ascii_map = R"(
  E----F----G----H----I----A----J----K----L----O----P----Q----R
                                |    |         |    |
                                |    |         |    |
                                |    |         |    |
                                |    |         |    |
                                M----N         T----U
  )";

const gurka::ways ways = {
    {"EF", {{"highway", "motorway"}}},    {"FG", {{"highway", "residential"}}},
    {"GH", {{"highway", "residential"}}}, {"HI", {{"highway", "residential"}}},
    {"IA", {{"highway", "residential"}}}, {"IJ", {{"highway", "residential"}}},
    {"JK", {{"highway", "motorway"}}},    {"KL", {{"highway", "residential"}}},
    {"JM", {{"highway", "residential"}}}, {"MN", {{"highway", "residential"}}},
    {"NK", {{"highway", "residential"}}}, {"LO", {{"highway", "motorway"}}},
    {"OP", {{"highway", "motorway"}}},    {"PQ", {{"highway", "residential"}}},
    {"QR", {{"highway", "residential"}}}, {"OT", {{"highway", "residential"}}},
    {"TU", {{"highway", "residential"}}}, {"UP", {{"highway", "residential"}}},
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
  static gurka::map mapNotAllowed;

  static void SetUpTestSuite() {
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/hard_exclude_highways",
                            {{"service_limits.allow_hard_exclusions", "true"}});

    mapNotAllowed = gurka::buildtiles(layout, ways, {}, {}, "test/data/hard_exclude_highways");
  }
};

gurka::map ExclusionTestExcludeHighways::map = {};
gurka::map ExclusionTestExcludeHighways::mapNotAllowed = {};

TEST_P(ExclusionTestExcludeHighways, ExcludeHighways) {
  check_result("1", {"I", "L"}, {"IJ", "JM", "MN", "NK", "KL"}, map, GetParam());
}

TEST_P(ExclusionTestExcludeHighways, InTheBeginningWithNoExit) {
  check_result("0", {"E", "H"}, {"EF", "FG", "GH"}, map, GetParam());
  check_result("1", {"E", "H"}, {"EF", "FG", "GH"}, map, GetParam());
}

// It does not exit to the residential OT but keeps driving on the motorway OP
TEST_P(ExclusionTestExcludeHighways, InTheBeginningWithPossibleExit) {
  check_result("0", {"L", "U"}, {"LO", "OP", "UP"}, map, GetParam());
  check_result("1", {"L", "U"}, {"LO", "OP", "UP"}, map, GetParam());
}

TEST_P(ExclusionTestExcludeHighways, InTheBeginningNotAllowed) {
  check_result("0", {"E", "H"}, {"EF", "FG", "GH"}, mapNotAllowed, GetParam());
  try {
    check_result("1", {"E", "H"}, {"EF", "FG", "GH"}, mapNotAllowed, GetParam());
  } catch (const valhalla_exception_t& err) { EXPECT_EQ(err.code, 145); } catch (...) {
    FAIL() << "Expected valhalla_exception_t.";
  };
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
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/hard_exclude_highways",
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
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/hard_exclude_highways",
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
