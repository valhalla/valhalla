#include "gurka.h"
#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;

const std::unordered_map<std::string, std::string> build_config{
    {"mjolnir.admin", {VALHALLA_SOURCE_DIR "test/data/netherlands_admin.sqlite"}}};

const std::vector<std::string>& costing = {"auto",    "taxi",          "bus",        "truck",
                                           "bicycle", "motor_scooter", "motorcycle", "pedestrian"};

TEST(Standalone, AccessPsvWay) {
  constexpr double gridsize_metres = 10;

  const std::string ascii_map = R"(
                            L
                            |
                            |
        A---B---C---D---E---I---J
                |       |       |
                F-------G-------K
                |
                H

        M------N------O
    )";

  const gurka::ways ways = {
      {"AB", {{"highway", "primary"}}},
      {"BC", {{"highway", "primary"}}},
      {"CD", {{"highway", "primary"}}},
      {"DE", {{"highway", "primary"}}},
      {"EG", {{"highway", "primary"}}},
      {"FG", {{"highway", "primary"}}},
      {"CF",
       {
           {"highway", "primary"},
           {"access", "psv"}, // access key wins over bus or taxi tag
           {"bike", "no"},
           {"bus", "no"},
       }},
      {"FH", {{"highway", "primary"}}},
      {"EI", {{"highway", "bus_guideway"}}},
      {"JI", {{"highway", "busway"}}},
      {"GK", {{"highway", "primary"}}},
      {"KJ", {{"highway", "primary"}}},
      {"LI", {{"highway", "primary"}}},
      {"MN", {{"highway", "residential"}, {"access", "no"}, {"bus", "permit"}, {"taxi", "permit"}}},
      {"NO", {{"highway", "residential"}, {"access", "no"}, {"bus", "permit"}, {"taxi", "permit"}}},
  };

  const auto layout =
      gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {5.1079374, 52.0887174});
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_access_psv_way", build_config);
  for (auto& c : costing) {
    auto result = gurka::do_action(valhalla::Options::route, map, {"A", "H"}, c);

    if (c == "bus" || c == "taxi")
      gurka::assert::raw::expect_path(result, {"AB", "BC", "CF", "FH"});
    else
      gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DE", "EG", "FG", "FH"});
  }

  for (auto& c : costing) {
    auto result = gurka::do_action(valhalla::Options::route, map, {"D", "J"}, c);

    if (c == "bus")
      gurka::assert::raw::expect_path(result, {"DE", "EI", "JI"});
    else
      gurka::assert::raw::expect_path(result, {"DE", "EG", "GK", "KJ"});
  }

  for (auto& c : costing) {
    if (c == "bus")
      EXPECT_NO_THROW(gurka::do_action(valhalla::Options::route, map, {"D", "L"}, c));
    else
      EXPECT_THROW(gurka::do_action(valhalla::Options::route, map, {"D", "L"}, c),
                   std::runtime_error);
  }

  // Test bus=permit overriding access=no
  auto result = gurka::do_action(valhalla::Options::route, map, {"M", "O"}, "bus");
  gurka::assert::raw::expect_path(result, {"MN", "NO"});

  // Test taxi=permit overriding access=no
  result = gurka::do_action(valhalla::Options::route, map, {"M", "O"}, "taxi");
  gurka::assert::raw::expect_path(result, {"MN", "NO"});
}

TEST(Standalone, AccessPsvNode) {
  constexpr double gridsize_metres = 10;

  const std::string ascii_map = R"(
               
        A---B---C---D---E
                |       |
                F       |
                |       |
                H-------G
    )";

  const gurka::ways ways = {
      {"AB", {{"highway", "primary"}}}, {"BC", {{"highway", "primary"}}},
      {"CD", {{"highway", "primary"}}}, {"DE", {{"highway", "primary"}}},
      {"EG", {{"highway", "primary"}}}, {"HG", {{"highway", "primary"}}},
      {"CF", {{"highway", "primary"}}}, {"FH", {{"highway", "primary"}}},

  };

  const gurka::nodes nodes = {{"F",
                               {
                                   {"access", "psv"}, // access tag wins over bus or taxi tag
                                   {"taxi", "no"},
                                   {"bus", "no"},
                               }}};

  const auto layout =
      gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {5.1079374, 52.0887174});
  auto map =
      gurka::buildtiles(layout, ways, nodes, {}, "test/data/gurka_access_psv_way", build_config);
  for (auto& c : costing) {
    auto result = gurka::do_action(valhalla::Options::route, map, {"A", "H"}, c);

    if (c == "bus" || c == "taxi")
      gurka::assert::raw::expect_path(result, {"AB", "BC", "CF", "FH"});
    else
      gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DE", "EG", "HG"});
  }
}
