#include "gurka.h"
#include "test.h"

using namespace valhalla;

TEST(Alternates, test_short_route) {
  const std::string ascii_map = R"(
               E---------F
               |         |
       A-------B---------C-------D
               |         |
               |         |
               G---------H
    )";

  const gurka::ways ways = {
      {"AB", {{"highway", "primary"}, {"maxspeed", "60"}}},
      {"BC", {{"highway", "primary"}, {"maxspeed", "60"}}},
      {"CD", {{"highway", "primary"}, {"maxspeed", "60"}}},

      {"BGHC", {{"highway", "primary"}, {"maxspeed", "60"}}},
      {"BEFC", {{"highway", "primary"}, {"maxspeed", "60"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 1000);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/alternates_short");

  auto result =
      gurka::do_action(valhalla::Options::route, map, {"A", "D"}, "auto", {{"/alternates", "2"}});
  const auto paths = gurka::detail::get_paths(result);

  ASSERT_EQ(paths.size(), 3) << "Unexpected number of routes";

  EXPECT_EQ(paths[0], std::vector<std::string>({"AB", "BC", "CD"})) << "Wrong shortest route";
  // ~4 min longer
  EXPECT_EQ(paths[1], std::vector<std::string>({"AB", "BEFC", "CD"}))
      << "Wrong first alternative route";
  // ~6 min longer
  EXPECT_EQ(paths[2], std::vector<std::string>({"AB", "BGHC", "CD"}))
      << "Wrong second alternative route";
}

TEST(Alternates, test_long_route) {
  const std::string ascii_map = R"(
                             E---------------------------F
                             |                           |
       A---------------------B---------------------------C----------------------D
                             |                           |
                             |                           |
                             G---------------------------H
    )";

  const gurka::ways ways = {
      {"AB", {{"highway", "primary"}, {"maxspeed", "60"}}},
      {"BC", {{"highway", "motorway"}, {"maxspeed", "90"}}},
      {"CD", {{"highway", "primary"}, {"maxspeed", "60"}}},

      {"BGHC", {{"highway", "motorway"}, {"maxspeed", "90"}}},
      {"BEFC", {{"highway", "motorway"}, {"maxspeed", "90"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10000);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/alternates_long");

  auto result =
      gurka::do_action(valhalla::Options::route, map, {"A", "D"}, "auto", {{"/alternates", "2"}});
  const auto paths = gurka::detail::get_paths(result);

  ASSERT_EQ(paths.size(), 3) << "Unexpected number of routes";

  EXPECT_EQ(paths[0], std::vector<std::string>({"AB", "BC", "CD"})) << "Wrong shortest route";
  // ~27 min longer
  EXPECT_EQ(paths[1], std::vector<std::string>({"AB", "BEFC", "CD"}))
      << "Wrong first alternative route";
  // ~40 min longer
  EXPECT_EQ(paths[2], std::vector<std::string>({"AB", "BGHC", "CD"}))
      << "Wrong second alternative route";
}

TEST(Alternates, test_too_long_detour) {
  const std::string ascii_map = R"(
       A------------B-C------------D
                    | |
                    | |
                    | |
                    | |
                    | |
                    E-F
    )";

  const gurka::ways ways = {
      {"ABCD", {{"highway", "primary"}, {"maxspeed", "60"}}},
      {"BE", {{"highway", "motorway"}, {"maxspeed", "120"}}},
      {"EF", {{"highway", "motorway"}, {"maxspeed", "120"}}},
      {"FC", {{"highway", "motorway"}, {"maxspeed", "120"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 2000);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/alternates_too_long_detour");

  auto result =
      gurka::do_action(valhalla::Options::route, map, {"A", "D"}, "auto", {{"/alternates", "1"}});
  const auto paths = gurka::detail::get_paths(result);

  ASSERT_EQ(paths.size(), 1) << "Got alternative with too long detour";
}
