#include "gurka.h"
#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;

const std::unordered_map<std::string, std::string> build_config{
    {"mjolnir.admin", {VALHALLA_SOURCE_DIR "test/data/netherlands_admin.sqlite"}}};

const std::vector<std::string>& costing = {"auto",          "hov",        "taxi",
                                           "bus",           "truck",      "bicycle",
                                           "motor_scooter", "motorcycle", "pedestrian"};

TEST(Standalone, AccessPsv) {
  constexpr double gridsize_metres = 10;

  const std::string ascii_map = R"(
               
        A---B---C---D
                |   |
                E---F---G
    )";

  const gurka::ways ways = {
      {"AB", {{"highway", "primary"}}},
      {"BC", {{"highway", "primary"}}},
      {"CD", {{"highway", "primary"}}},
      {"CE",
       {
           {"highway", "primary"},
           {"bus", "no"}, // access tag wins over bus tag
           {"access", "psv"},
       }},
      {"EF", {{"highway", "primary"}}},
      {"FG", {{"highway", "primary"}}},
      {"DF", {{"highway", "primary"}}},
  };

  const auto layout =
      gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {5.1079374, 52.0887174});
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_access_psv", build_config);
  for (auto& c : costing) {
    auto result = gurka::do_action(valhalla::Options::route, map, {"A", "G"}, c);

    if (c == "bus" || c == "taxi")
      gurka::assert::raw::expect_path(result, {"AB", "BC", "CE", "EF", "FG"});
    else
      gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DF", "FG"});
  }
}
