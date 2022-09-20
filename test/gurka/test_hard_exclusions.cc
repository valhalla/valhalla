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
    "bicycle",
    "motor_scooter",
    "motorcycle"
  };

  const std::vector<std::string> kExclusionParameters = {
    "exclude_bridges",
    "exclude_tunnels",
    "exclude_tolls"
  };

  // const std::vector<std::tuple<std::string, std::string>> kParameterTuples(kSupportedCostingModels.size() * kExclusionParameters.size());

} // namespace

class ExclusionTest : public ::testing::TestWithParam<std::vector<std::string>> {
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
        {"EF", {{"highway", "residential"}, {"bridge", "yes"}, {"tunnel", "yes"}, {"toll", "yes"}}},
        {"FG", {{"highway", "residential"}, {"bridge", "yes"}, {"tunnel", "yes"}, {"toll", "yes"}}},
        {"GH", {{"highway", "residential"}}},
        {"HI", {{"highway", "residential"}, {"bridge", "yes"}, {"tunnel", "yes"}, {"toll", "yes"}}},
        {"IA", {{"highway", "residential"}}},
        {"IJ", {{"highway", "residential"}}},
        {"JK", {{"highway", "residential"}, {"bridge", "yes"}, {"tunnel", "yes"}, {"toll", "yes"}}},
        {"KL", {{"highway", "residential"}}},
        {"JM", {{"highway", "residential"}}},
        {"MN", {{"highway", "residential"}}},
        {"NK", {{"highway", "residential"}}},
    };

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, grid_size_meters);
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/hard_exclude",
                            {{"service_limits.allow_hard_exclusions", "true"}});

    //for (size_t i = 0, s = kSupportedCostingModels.size(); i < kParameterTuples.size(); ++i) {
    //  kParameterTuples[i] = {kExclusionParameters[i/s], kSupportedCostingModels[i%s]};
    //}
  }

  void check_result(const std::string& exclude_parameter_value,
                    const std::vector<std::string>& waypoints,
                    const std::vector<std::string>& expected_names,
                    const std::vector<std::string>& props = {}) {

    const std::string& costing = props[0];
    const std::string& exclude_parameter = props[1];
    const auto result = gurka::do_action(valhalla::Options::route, map, waypoints, costing,
                                          {{"/costing_options/" + costing + "/" + exclude_parameter, exclude_parameter_value}});
    gurka::assert::raw::expect_path(result, expected_names);
  }

};

gurka::map ExclusionTest::map = {};

TEST_P(ExclusionTest, InTheMiddle) {
  check_result("0", {"I","L"}, {"IJ", "JK", "KL"}, GetParam());
  check_result("1", {"I","L"}, {"IJ", "JM", "MN", "NK", "KL"}, GetParam());
}

TEST_P(ExclusionTest, InTheBeginning) {
  check_result("0", {"E", "H"}, {"EF", "FG", "GH"}, GetParam());
  check_result("1", {"E", "H"}, {"EF", "FG", "GH"}, GetParam());
}

TEST_P(ExclusionTest, InTheEnd) {
  check_result("0", {"H", "E"}, {"GH", "FG", "EF"}, GetParam());
  check_result("1", {"H", "E"}, {"GH", "FG", "EF"}, GetParam());
}

INSTANTIATE_TEST_SUITE_P(ExcludePropsTest,
                         ExclusionTest,
                         ::testing::Values(::testing::Combine(::testing::Values(kSupportedCostingModels),
                                                               ::testing::Values(kExclusionParameters))));
