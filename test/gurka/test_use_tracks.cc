#include "gurka.h"
#include "test.h"
#include <gtest/gtest.h>

using namespace valhalla;

const std::vector<std::string>& costing = {"auto",          "taxi",       "bus",       "truck",
                                           "motor_scooter", "motorcycle", "pedestrian"};

void validate_path(const valhalla::Api& result, const std::vector<std::string>& expected_names) {
  ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
  auto leg = result.trip().routes(0).legs(0);
  gurka::assert::raw::expect_path(result, expected_names);
}

class UseTracksTest : public ::testing::Test {
protected:
  static gurka::map use_tracks_map;

  static void SetUpTestSuite() {
    const std::string ascii_map = R"(
    A-1----B--------E-----2-F
           |        |       |
           |        |       |
           |        |       |
           |        |       |
           C--------D       G-----H
                     \
                      I
    )";

    const gurka::ways ways = {
        {"AB", {{"highway", "residential"}}}, {"BE", {{"highway", "track"}, {"surface", "paved"}}},
        {"EF", {{"highway", "residential"}}}, {"FG", {{"highway", "track"}}},
        {"BC", {{"highway", "residential"}}}, {"CD", {{"highway", "residential"}}},
        {"DE", {{"highway", "residential"}}}, {"DI", {{"highway", "secondary"}}},
        {"GH", {{"highway", "residential"}}},
    };

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
    use_tracks_map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_use_tracks");

    // set the same historical speeds for all roads
    test::customize_historical_traffic(use_tracks_map.config, [](baldr::DirectedEdge& e) {
      e.set_free_flow_speed(40);
      e.set_constrained_flow_speed(40);
      return boost::none;
    });
  }
};

gurka::map UseTracksTest::use_tracks_map = {};

TEST_F(UseTracksTest, test_default_value) {
  for (const auto& c : costing)
    if (c == "auto" || c == "taxi" || c == "bus" || c == "truck")
      // avoid tracks by default; use tracks only if the route starts or ends at 'track' edge
      validate_path(gurka::do_action(valhalla::Options::route, use_tracks_map, {"1", "2"}, c),
                    {"AB", "BC", "CD", "DE", "EF"});
    else
      // track tag shouldn't affect other costings too much
      validate_path(gurka::do_action(valhalla::Options::route, use_tracks_map, {"1", "2"}, c),
                    {"AB", "BE", "EF"});
}

TEST_F(UseTracksTest, test_use_tracks) {
  // use routes with tracks
  for (const auto& c : costing)
    validate_path(gurka::do_action(valhalla::Options::route, use_tracks_map, {"1", "2"}, c,
                                   {{"/costing_options/" + c + "/use_tracks", "1"}}),
                  {"AB", "BE", "EF"});
}

TEST_F(UseTracksTest, test_avoid_tracks) {
  // avoid tracks
  for (const auto& c : costing)
    validate_path(gurka::do_action(valhalla::Options::route, use_tracks_map, {"1", "2"}, c,
                                   {{"/costing_options/" + c + "/use_tracks", "0"}}),
                  {"AB", "BC", "CD", "DE", "EF"});
}

TEST_F(UseTracksTest, test_use_tracks_if_no_other_roads) {
  // use track if it's needed to complete the route
  for (const auto& c : costing) {
    validate_path(gurka::do_action(valhalla::Options::route, use_tracks_map, {"2", "H"}, c,
                                   {{"/costing_options/" + c + "/use_tracks", "0"}}),
                  {"EF", "FG", "GH"});
  }
}
