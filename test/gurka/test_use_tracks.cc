#include "gurka.h"
#include "test.h"
#include <gtest/gtest.h>

using namespace valhalla;

// set costings impacted by `use_tracks` parameter
const std::vector<std::string>& costing = {"auto", "hov", "taxi", "bus", "truck"};

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
           |        |       3
           |        |       |
           C--------D       G
    )";

    const gurka::ways ways = {
        {"AB", {{"highway", "track"}}},       {"BE", {{"highway", "track"}}},
        {"EF", {{"highway", "track"}}},       {"FG", {{"highway", "residential"}}},
        {"BC", {{"highway", "residential"}}}, {"CD", {{"highway", "residential"}}},
        {"DE", {{"highway", "residential"}}},
    };

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
    use_tracks_map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_use_tracks");

    // set the same historical speeds for all roads
    test::customize_historical_traffic(use_tracks_map.config, [](baldr::DirectedEdge& e) {
      e.set_free_flow_speed(40);
      e.set_constrained_flow_speed(40);
      return std::vector<int16_t>{};
    });
  }
};

gurka::map UseTracksTest::use_tracks_map = {};

TEST_F(UseTracksTest, test_default_value) {
  // don't set `use_tracks` parameter; avoid tracks by default
  for (const auto& c : costing)
    validate_path(gurka::route(use_tracks_map, "1", "2", c), {"AB", "BC", "CD", "DE", "EF"});
}

TEST_F(UseTracksTest, test_favor_tracks) {
  // prefer routes with tracks
  for (const auto& c : costing)
    validate_path(gurka::route(use_tracks_map, "1", "D", c,
                               {{"/costing_options/" + c + "/use_tracks", "1"}}),
                  {"AB", "BE", "DE"});
}

TEST_F(UseTracksTest, test_avoid_tracks) {
  // prefer routes with tracks
  for (const auto& c : costing)
    validate_path(gurka::route(use_tracks_map, "1", "2", c,
                               {{"/costing_options/" + c + "/use_tracks", "0"}}),
                  {"AB", "BC", "CD", "DE", "EF"});
}

TEST_F(UseTracksTest, test_use_tracks_if_no_other_roads) {
  // prefer routes with tracks
  for (const auto& c : costing) {
    if (c != "auto")
      continue;
    validate_path(gurka::route(use_tracks_map, "1", "3", c,
                               {{"/costing_options/" + c + "/use_tracks", "1"}}),
                  {"AB", "BC", "CD", "DE", "EF", "FG"});
  }
}