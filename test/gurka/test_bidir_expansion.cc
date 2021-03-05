#include "gurka.h"
#include "test.h"

#include <gtest/gtest.h>

using namespace valhalla;
using LiveTrafficCustomize = test::LiveTrafficCustomize;

inline void SetLiveSpeed(baldr::TrafficSpeed* live_speed, uint64_t speed) {
  live_speed->breakpoint1 = 255;
  live_speed->overall_speed = speed >> 1;
  live_speed->speed1 = speed >> 1;
}

TEST(BidirExpansion, expansion_exhaustion) {
  const std::string ascii_map = R"(
  A---B---C---D---E
  )";

  // Make AB oneway so that reverse expansion can't access the
  // origin location. Also we need at least 3 edges so that we
  // dont fallback to using time-dep fwd A* due to trivial edge
  // case (excluding the oneway)
  const gurka::ways ways = {{"AB", {{"highway", "secondary"},
                                    {"oneway","yes"}}},
                            {"BC", {{"highway", "primary"}}},
                            {"CD", {{"highway", "primary"}}},
                            {"DE", {{"highway", "primary"}}}};
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 25);
  const std::string tile_dir = "test/data/bidir_expansion";
  auto map = gurka::buildtiles(layout, ways, {}, {}, tile_dir);
  map.config.put("mjolnir.traffic_extract", tile_dir + "/traffic.tar");
  test::build_live_traffic_data(map.config);
  std::shared_ptr<baldr::GraphReader> reader = test::make_clean_graphreader(map.config.get_child("mjolnir"));

  test::customize_live_traffic_data(map.config,
                                    [](baldr::GraphReader&, baldr::TrafficTile&,
                                       uint32_t, baldr::TrafficSpeed* current) -> void {
                                      SetLiveSpeed(current, 100);
                                    });

  LiveTrafficCustomize slow_edge = [&](baldr::GraphReader& reader, baldr::TrafficTile& tile,
                                       uint32_t index, baldr::TrafficSpeed* current) -> void {
    baldr::GraphId tile_id(tile.header->tile_id);
    auto edge_AB = std::get<0>(gurka::findEdge(reader, map.nodes, "AB", "B"));
    auto edge_BC = std::get<0>(gurka::findEdge(reader, map.nodes, "BC", "C"));
    if ((edge_AB.Tile_Base() == tile_id && edge_AB.id() == index) ||
        (edge_BC.Tile_Base() == tile_id && edge_BC.id() == index)) {
      SetLiveSpeed(current, 1);
    }
  };
  // Make AB really slow, increasing its cost and causing the expansion
  // to only be in the reverse dir till it exhausts
  test::customize_live_traffic_data(map.config, slow_edge);

  {
    // Route from A->E should work (A->B is a slow 1-way edge)
    auto result = gurka::do_action(valhalla::Options::route, map, {"A", "E"},
                                   "auto",
                                   {{"/date_time/type", "3"},
                                    {"/date_time/value", "current"},
                                    {"/costing_options/auto/speed_types/0", "current"}},
                                   reader);
    gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DE"});
  }
}
