#include "gurka.h"
#include "test.h"

using namespace valhalla;

std::string deprecated_costing_methods[3] = {"auto_shorter", "hov", "auto_data_fix"};

// test case for routes endpoint
TEST(warnings, routes_endpoint) {
  const std::string ascii_map = R"(
          A----------B-------------C-----P
          |          |                   |
          |          |                   |
          |          3--------U     G----S
          |          |        |     |    |
          D----------E--------4-----F----L
     )";
  const gurka::ways ways = {
      {"AB", {{"highway", "primary"}, {"name", "RT 1"}}},
      {"BC", {{"highway", "motorway"}, {"name", "RT 2"}}},
      {"CP", {{"highway", "motorway"}, {"name", "RT 3"}}},
      {"AD", {{"highway", "motorway"}, {"name", "RT 4"}}},
      {"B3", {{"highway", "motorway"}, {"name", "RT 5"}}},
      {"3E", {{"highway", "motorway"}, {"name", "RT 6"}}},
      {"3U", {{"highway", "motorway"}, {"name", "RT 7"}}},
      {"E4", {{"highway", "motorway"}, {"name", "RT 8"}}},
      {"U4", {{"highway", "motorway"}, {"name", "RT 9"}}},
      {"4F", {{"highway", "motorway"}, {"name", "RT 10"}}},
      {"GF", {{"highway", "motorway"}, {"name", "RT 11"}}},
      {"GS", {{"highway", "motorway"}, {"name", "RT 12"}}},
      {"PS", {{"highway", "motorway"}, {"name", "RT 13"}}},
      {"FL", {{"highway", "motorway"}, {"name", "RT 14"}}},
      {"SL", {{"highway", "motorway"}, {"name", "RT 15"}}},
  };
  const double gridsize = 100;
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/routes_warnings");
  for (int i = 0; i < 3; i++) {
    valhalla::Api result = gurka::do_action(valhalla::Options::route, map, {"A", "L"},
                                            deprecated_costing_methods[i], {{"/best_paths", "2"}});
    ASSERT_TRUE(result.info().warnings_size() != 0);
    EXPECT_EQ(result.info().warnings_size(), 2);
  }
}

// test case for locate endpoint
TEST(location_warnings, locate_endpoint) {
  const std::string ascii_map = R"(
            1------------2-----------A
            |            |           |
            |            |           |
            a------------b-----------c
            |            |           |
            |            B-----------K
            |  E---------|           |
            | /          |           6
            |/           |           |
            D------------F-----------3
  )";
  const gurka::ways ways = {
      {"AE", {{"highway", "primary"}, {"name", "RT 1"}}},
      {"BE", {{"highway", "motorway"}, {"name", "RT 2"}}},
      {"DE", {{"highway", "motorway"}, {"name", "RT 3"}}},
      {"BF", {{"highway", "motorway"}, {"name", "RT 4"}}},
      {"DF", {{"highway", "motorway"}, {"name", "RT 5"}}},
  };
  const double gridsize = 100;
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/locate_warnings");
  for (int i = 0; i < 3; i++) {
    valhalla::Api result =
        gurka::do_action(valhalla::Options::locate, map, {"6"}, deprecated_costing_methods[i]);
    ASSERT_TRUE(result.info().warnings_size() != 0);
    EXPECT_EQ(result.info().warnings_size(), 1);
  }
}

// test case for isochrone endpoint
TEST(isochrone_warnings, isochrone_endpoint) {
  const std::string ascii_map = R"(
            E------------M-----------A--------Z
            |            |                    |
            |            |                    |
            3------------N-----------C--------|
            |            |           |        |
            |            B           2--------|
            |                                 |
            |                                 |
            |                                /  
            P------------4-----------1------/
  )";
  const gurka::ways ways = {
      {"EM", {{"highway", "primary"}, {"name", "RT 1"}}},
      {"MA", {{"highway", "motorway"}, {"name", "RT 2"}}},
      {"AZ", {{"highway", "motorway"}, {"name", "RT 3"}}},
      {"NC", {{"highway", "motorway"}, {"name", "RT 4"}}},
      {"NB", {{"highway", "motorway"}, {"name", "RT 5"}}},
  };
  const double gridsize = 100;
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/isochrone_warnings");
  for (int i = 0; i < 3; i++) {
    valhalla::Api result =
        gurka::do_action(valhalla::Options::isochrone, map, {"B"}, deprecated_costing_methods[i],
                         {{"/contours/0/time", "10"}, {"/denoise", "0"}, {"/generalize", "0"}});
    ASSERT_FALSE(result.info().warnings_size() == 0);
    EXPECT_EQ(result.info().warnings_size(), 1);
  }
}

// test case for transit available endpoint
TEST(transit_available_warnings, transit_available_endpoint) {
  const std::string ascii_map = R"(
            A------------B-----------C--------D
            |            |           |        |
            |            |           |        |
            E------------1-----------s--------H-------------------------Q
            |            |           |        |
            |            |           |        |
            I------------2-----------t--------L-------------------------R
            |            |           |        |
            |            |           |        |  
            M------------N-----------O--------P
  )";
  const gurka::ways ways = {
      {"AD", {{"highway", "primary"}, {"name", "RT 1"}}},
      {"EQ", {{"highway", "motorway"}, {"name", "RT 2"}}},
      {"IR", {{"highway", "motorway"}, {"name", "RT 3"}}},
      {"MP", {{"highway", "motorway"}, {"name", "RT 4"}}},
      {"DP", {{"highway", "motorway"}, {"name", "RT 5"}}},
      {"AM", {{"highway", "motorway"}, {"name", "RT 6"}}},
  };
  const double gridsize = 100;
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/transit_available_warnings");
  valhalla::Api result = gurka::do_action(valhalla::Options::transit_available, map, {"A"}, "",
                                          {{"/locations/0/radius", "5"}, {"/best_paths", "2"}});
  ASSERT_FALSE(result.info().warnings_size() == 0);
  EXPECT_EQ(result.info().warnings_size(), 1);
}
