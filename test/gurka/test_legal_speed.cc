#include "gurka.h"
#include <boost/format.hpp>
#include <gtest/gtest.h>

using namespace valhalla;

class LegalSpeedTest : public ::testing::TestWithParam<std::string> {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize = 10;

    const std::string ascii_map = R"(
      A-------E
      |       |
      |       |
      |       |
      B-------F
      |       |
      |       |
      C       G
      |       |
      |       |
      D-------H
    )";

    // TODO: extend the ways
    const gurka::ways ways =
        {{"BAEF", {{"highway", "tertiary"}, {"maxspeed", "20"}}},
         {"CDHG", {{"highway", "tertiary"}, {"maxspeed", "20"}}},
         {"BC", {{"highway", "tertiary"}, {"maxspeed:forward", "60"}, {"maxspeed:backward", "20"}}},
         {"FG", {{"highway", "tertiary"}, {"maxspeed:forward", "20"}, {"maxspeed:backward", "60"}}},
         {"BF", {{"highway", "tertiary"}, {"maxspeed", "none"}}}};

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/legal_speed");
  }

  void doTests(const std::string& costing,
               const std::vector<std::string>& waypoints,
               const std::vector<std::string>& expected_default,
               const std::vector<std::string>& expected_legal) {

    const std::unordered_map<std::string, std::string> options = {
        {"/costing_options/" + costing + "/legal_speed", "50"}};

    valhalla::Api default_route = gurka::do_action(valhalla::Options::route, map, waypoints, costing);
    valhalla::Api legal_set_route =
        gurka::do_action(valhalla::Options::route, map, waypoints, costing, options);

    gurka::assert::raw::expect_path(default_route, expected_default);
    gurka::assert::raw::expect_path(legal_set_route, expected_legal);
  }
};

gurka::map LegalSpeedTest::map = {};

TEST_P(LegalSpeedTest, TestMaxSpeed) {
  doTests(GetParam(), {"C", "F"}, {"BC", "BF"}, {"BC", "BAEF"});
}

TEST_P(LegalSpeedTest, TestForwardMaxSpeed) {
  doTests(GetParam(), {"A", "D"}, {"BAEF", "BC", "CDHG"}, {"BAEF", "FG", "CDHG"});
}

TEST_P(LegalSpeedTest, TestBackwardMaxSpeed) {
  doTests(GetParam(), {"H", "E"}, {"CDHG", "FG", "BAEF"}, {"CDHG", "BC", "BAEF"});
}

TEST_P(LegalSpeedTest, TestFailRoute) {
  try {
    valhalla::Api route_legal =
        gurka::do_action(valhalla::Options::route, map, {"A", "D"}, GetParam(),
                         {{"/costing_options/" + GetParam() + "/legal_speed", "10"}});
  } catch (const valhalla_exception_t& err) { EXPECT_EQ(err.code, 442); } catch (...) {
    FAIL() << "Expected valhalla_exception_t.";
  };
}

INSTANTIATE_TEST_SUITE_P(LegalSpeedProfilesTest,
                         LegalSpeedTest,
                         ::testing::Values("auto",
                                           "truck",
                                           "bicycle",
                                           "motorcycle",
                                           "motor_scooter",
                                           "hov",
                                           "taxi",
                                           "bus"));
