
#include "baldr/graphconstants.h"
#include "gurka.h"
#include "test.h"
#include <gtest/gtest.h>

using namespace valhalla;

namespace {
const std::vector<std::string> kSupportedCostingModels = {"auto", "bus", "truck", "motorcycle"};

const std::map<std::string, std::string> kParameters = {{"tolls", "toll"},
                                                        {"highways", "highway"},
                                                        {"ferry", "ferry"}};
std::vector<std::string> kUseParameters;

constexpr double grid_size_meters = 100.;

const std::string ascii_map = R"(
                          B------C
                          |      |
                          |      |
  I---------J--------L----A      D
            |        |    |      |
             \      /     |      |
              M----N      E      F
                            \  /
                             G
  )";

const gurka::ways ways = {
    {"IJ", {{"highway", "trunk"}}},
    {"JL", {{"highway", "motorway"}, {"toll", "yes"}}},
    {"LA", {{"highway", "trunk"}}},
    {"ABCD", {{"maxspeed", "50"}, {"route", "ferry"}, {"motor_vehicle", "yes"}}},
    {"AEGFD", {{"highway", "trunk"}, {"maxspeed", "10"}}},
    {"JM", {{"highway", "trunk"}}},
    {"MN", {{"highway", "trunk"}}},
    {"NL", {{"highway", "trunk"}}},
};

const std::vector<std::string> trip = {"I", "D"};
const std::map<std::string, std::vector<std::string>> waypoints_on =
    {{"tolls", {"IJ", "JL", "LA", "ABCD"}},
     {"highways", {"IJ", "JL", "LA", "ABCD"}},
     {"ferry", {"IJ", "JL", "LA", "ABCD"}}};
const std::map<std::string, std::vector<std::string>> waypoints_off =
    {{"tolls", {"IJ", "JM", "MN", "NL", "LA", "ABCD"}},
     {"highways", {"IJ", "JM", "MN", "NL", "LA", "ABCD"}},
     {"ferry", {"IJ", "JL", "LA", "AEGFD"}}};
const auto layout = gurka::detail::map_to_coordinates(ascii_map, grid_size_meters);

} // namespace

class UseTest : public ::testing::TestWithParam<std::tuple<std::string, std::string>> {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/use_feature");
  }

  /**
   * Test if a found path has the expected feature, with the expected value
   *
   * @param raw_result the result of a /route or /match request
   * @param feature expected feature
   * @param expected_value the expected value that the expected_value should be
   */
  void
  expect_feature(valhalla::Api& raw_result, const std::string& feature, const bool expected_value) {
    // std::string json = tyr::serializeDirections(raw_result);
    // std::cout << json.c_str() << std::endl;
    rapidjson::Document result = gurka::convert_to_json(raw_result, valhalla::Options_Format_json);
    if (result.HasParseError()) {
      FAIL() << "Error converting route response to JSON";
    }
    EXPECT_TRUE(result.HasMember("trip"));
    EXPECT_TRUE(result["trip"].IsObject());

    EXPECT_TRUE(result["trip"].HasMember("legs"));
    EXPECT_TRUE(result["trip"]["legs"].IsArray());

    const auto& leg = result["trip"]["legs"][0];
    {
      EXPECT_TRUE(leg.HasMember("maneuvers"));
      EXPECT_TRUE(leg["maneuvers"].IsArray());
      const auto actual_value =
          std::any_of(leg["maneuvers"].Begin(), leg["maneuvers"].End(), [&](const auto& maneuver) {
            return maneuver.HasMember(feature) && maneuver[feature].IsBool() &&
                   maneuver[feature].GetBool();
          });
      EXPECT_EQ(actual_value, expected_value);
    }
    {
      EXPECT_TRUE(leg.HasMember("summary"));
      const auto& summary = leg["summary"];
      EXPECT_TRUE(summary.IsObject());
      EXPECT_TRUE(summary.HasMember("has_" + feature));
      EXPECT_TRUE(summary["has_" + feature].IsBool());
      const auto actual_value = summary["has_" + feature].GetBool();
      EXPECT_EQ(actual_value, expected_value);
    }
    {
      EXPECT_TRUE(result["trip"].HasMember("summary"));
      const auto& summary = result["trip"]["summary"];
      EXPECT_TRUE(summary.IsObject());
      EXPECT_TRUE(summary.HasMember("has_" + feature));
      EXPECT_TRUE(summary["has_" + feature].IsBool());
      const auto actual_value = summary["has_" + feature].GetBool();
      EXPECT_EQ(actual_value, expected_value);
    }
  }

public:
  void check_result(const std::string& use_parameter_value,
                    const std::vector<std::string>& waypoints,
                    const std::vector<std::string>& expected_names,
                    const gurka::map& map,
                    const std::tuple<std::string, std::string>& props = {}) {

    const std::string& costing = std::get<0>(props);
    const std::string& use_parameter = std::get<1>(props);
    auto result = gurka::do_action(valhalla::Options::route, map, waypoints, costing,
                                   {{"/costing_options/" + costing + "/use_" + use_parameter,
                                     use_parameter_value}});
    gurka::assert::raw::expect_path(result, expected_names);
    expect_feature(result, kParameters.at(use_parameter), use_parameter_value == "1");
  }
};

gurka::map UseTest::map = {};

TEST_P(UseTest, Autocost) {
  const std::string& param = std::get<1>(GetParam());
  check_result("0", trip, waypoints_off.at(param), map, GetParam());
  check_result("1", trip, waypoints_on.at(param), map, GetParam());
}

INSTANTIATE_TEST_SUITE_P(UsePropsTest,
                         UseTest,
                         ::testing::Combine(::testing::ValuesIn(kSupportedCostingModels),
                                            ::testing::ValuesIn(kUseParameters)));

int main(int argc, char** argv) {
  std::transform(kParameters.begin(), kParameters.end(), std::back_inserter(kUseParameters),
                 [](auto& pair) { return pair.first; });
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
