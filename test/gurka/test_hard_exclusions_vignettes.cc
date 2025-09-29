#include "gurka.h"
#include "test.h"

#include <gtest/gtest.h>
#include <boost/format.hpp>
#include <boost/property_tree/json_parser.hpp>


using namespace valhalla;

namespace {
const std::vector<std::string> kCostingModelsExcludeVignettes = {"auto", "truck"};
const std::vector<std::string> kCostingModelsNoHardExcludeSetA = {"auto", "truck"};
const std::vector<std::string> kExclusionParameters = {"exclude_vignettes"};

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
    {"EF", {{"highway", "motorway"}, {"vignette", "true"}}},
    {"FG", {{"highway", "motorway"}, {"vignette", "true"}}},
    {"GH", {{"highway", "motorway"}}},
    {"HI", {{"highway", "motorway"}, {"vignette", "true"}}},
    {"IA", {{"highway", "motorway"}}},
    {"IJ", {{"highway", "motorway"}}},
    {"JK", {{"highway", "motorway"}, {"vignette", "true"}}},
    {"KL", {{"highway", "motorway"}}},
    {"JM", {{"highway", "motorway"}}},
    {"MN", {{"highway", "motorway"}}},
    {"NK", {{"highway", "motorway"}}},
};

//const auto layout = gurka::detail::map_to_coordinates(ascii_map, grid_size_meters);

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

class ExclusionTestExcludeVignettes : public ::testing::TestWithParam<std::vector<std::string>> {
protected:
  static gurka::map map;
  static gurka::map mapNotAllowed;

  static void SetUpTestSuite() {
    const auto layout = gurka::detail::map_to_coordinates(ascii_map, grid_size_meters);

    // Explicitly build config with service_limits
    auto conf_with_exclusions =
        test::make_config("/workspaces/way/test/data/hard_exclude_vignettes",
                           {{"service_limits.allow_hard_exclusions", "true"}});
    
    auto conf_without_exclusions =
        test::make_config("/workspaces/way/test/data/hard_exclude_vignettes", {});
    std::stringstream mapSS;
    boost::property_tree::json_parser::write_json(mapSS, conf_without_exclusions);
    valhalla::config(mapSS.str());

    map = gurka::buildtiles(layout, ways, {}, {}, conf_with_exclusions);
    mapNotAllowed = gurka::buildtiles(layout, ways, {}, {}, conf_without_exclusions);
  }
};

gurka::map ExclusionTestExcludeVignettes::map = {};
gurka::map ExclusionTestExcludeVignettes::mapNotAllowed = {};

TEST_P(ExclusionTestExcludeVignettes, ExcludeVignettes) {
  check_result("1", {"I", "L"}, {"IJ", "JM", "MN", "NK", "KL"}, map, GetParam());
}

TEST_P(ExclusionTestExcludeVignettes, InTheBeginningWithNoExit) {
  check_result("0", {"E", "H"}, {"EF", "FG", "GH"}, map, GetParam());
  check_result("1", {"E", "H"}, {"EF", "FG", "GH"}, map, GetParam());
}

TEST_P(ExclusionTestExcludeVignettes, InTheMiddleNotAllowed) {
  check_result("0", {"I", "L"}, {"IJ", "JK", "KL"}, mapNotAllowed, GetParam());
  try {
    check_result("1", {"I", "L"}, {"IJ", "JM", "MN", "NK", "KL"}, mapNotAllowed, GetParam());
  } catch (const valhalla_exception_t& err) { EXPECT_EQ(err.code, 145); } catch (...) {
    FAIL() << "Expected valhalla_exception_t.";
  };
}

INSTANTIATE_TEST_SUITE_P(ExcludeVignettesTests,
                         ExclusionTestExcludeVignettes,
                         ::testing::ValuesIn([]() {
                           std::vector<std::vector<std::string>> values;
                           for (const auto& costing : kCostingModelsExcludeVignettes) {
                             for (const auto& param : kExclusionParameters) {
                               values.push_back({costing, param});
                             }
                           }
                           return values;
                         }()));

class ExclusionTestNoHardExcludeVignettes : public ::testing::TestWithParam<std::vector<std::string>> {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    const auto layout = gurka::detail::map_to_coordinates(ascii_map, grid_size_meters);

    // Explicitly build config with service_limits
    auto conf_with_exclusions =
        test::make_config("/workspaces/way/test/data/hard_exclude_vignettes",
                           {{"service_limits.allow_hard_exclusions", "true"}});
    map = gurka::buildtiles(layout, ways, {}, {}, conf_with_exclusions);
  }
};

gurka::map ExclusionTestNoHardExcludeVignettes::map = {};

TEST_P(ExclusionTestNoHardExcludeVignettes, NoHardExcludeVignettes) {
  check_result("0", {"I", "L"}, {"IJ", "JK", "KL"}, map, GetParam());
}

INSTANTIATE_TEST_SUITE_P(NoHardExcludeVignettesTests,
                         ExclusionTestNoHardExcludeVignettes,
                         ::testing::ValuesIn([]() {
                           std::vector<std::vector<std::string>> values;
                           for (const auto& costing : kCostingModelsNoHardExcludeSetA) {
                             for (const auto& param : kExclusionParameters) {
                               values.push_back({costing, param});
                             }
                           }
                           return values;
                         }()));