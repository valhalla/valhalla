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
               
        A---B---C---D---E
                |       |
                F-------G
                |
                H
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
