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

TEST(Standalone, RouteOnPrivateAccess) {
  constexpr double gridsize_metres = 10;

  const std::string ascii_map = R"(    
        A---B---C---D
            |   |   |
            E   F   G
    )";

  const gurka::ways ways = {
      {"AB", {{"highway", "primary"}}},
      {"BC", {{"highway", "primary"}}},
      {"CD", {{"highway", "primary"}}},
      {"BE", {{"highway", "service"}, {"access", "private"}}},
      {"CF", {{"highway", "service"}, {"access", "private"}, {"service", "driveway"}}},
      {"DG", {{"highway", "service"}, {"access", "private"}, {"service", "parking_aisle"}}},
  };

  const auto layout =
      gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {5.1079374, 52.0887174});
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_route_on_private_access",
                               build_config);

  for (auto& c : costing) {
    auto result = gurka::do_action(valhalla::Options::route, map, {"A", "E"}, c);
    gurka::assert::raw::expect_path(result, {"AB", "BE"});

    result = gurka::do_action(valhalla::Options::route, map, {"A", "F"}, c);
    gurka::assert::raw::expect_path(result, {"AB", "BC", "CF"});

    result = gurka::do_action(valhalla::Options::route, map, {"A", "G"}, c);
    gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DG"});
  }
}
