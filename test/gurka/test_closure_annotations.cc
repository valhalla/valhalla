#include "gurka.h"
#include "test.h"

#include <boost/format.hpp>
#include <gtest/gtest.h>

using namespace valhalla;
using LiveTrafficCustomize = test::LiveTrafficCustomize;

namespace {

inline void
SetSubsegmentLiveSpeed(baldr::TrafficSpeed* live_speed, uint64_t speed, uint8_t subsegment) {
  live_speed->breakpoint1 = subsegment;
  live_speed->overall_speed = speed >> 1;
  live_speed->speed1 = speed >> 1;
}

inline void SetLiveSpeed(baldr::TrafficSpeed* live_speed, uint64_t speed) {
  SetSubsegmentLiveSpeed(live_speed, speed, 255);
}

void close_partial_dir_edge(baldr::GraphReader& reader,
                            baldr::TrafficTile& tile,
                            uint32_t index,
                            double percent_along,
                            baldr::TrafficSpeed* current,
                            const std::string& edge_name,
                            const std::string& start_node,
                            const gurka::map& map) {
  baldr::GraphId tile_id(tile.header->tile_id);
  auto edge = std::get<0>(gurka::findEdge(reader, map.nodes, edge_name, start_node));
  if (edge.Tile_Base() == tile_id && edge.id() == index) {
    SetSubsegmentLiveSpeed(current, 0, static_cast<uint8_t>(255 * percent_along));
  }
}

void close_dir_edge(baldr::GraphReader& reader,
                    baldr::TrafficTile& tile,
                    uint32_t index,
                    baldr::TrafficSpeed* current,
                    const std::string& edge_name,
                    const std::string& start_node,
                    const gurka::map& map) {
  close_partial_dir_edge(reader, tile, index, 1., current, edge_name, start_node, map);
}

void close_bidir_edge(baldr::GraphReader& reader,
                      baldr::TrafficTile& tile,
                      uint32_t index,
                      baldr::TrafficSpeed* current,
                      const std::string& edge_name,
                      const gurka::map& map) {
  baldr::GraphId tile_id(tile.header->tile_id);
  std::string start_node(1, edge_name.front());
  std::string end_node(1, edge_name.back());

  close_dir_edge(reader, tile, index, current, edge_name, start_node, map);
  close_dir_edge(reader, tile, index, current, edge_name, end_node, map);
}

} // namespace

class ClosureAnnotations : public ::testing::Test {
protected:
  static gurka::map closure_map;
  static int const default_speed;
  static std::string const tile_dir;
  static std::shared_ptr<baldr::GraphReader> reader;

  static void SetUpTestSuite() {
    const std::string ascii_map = R"(

    A-1----2-B
             |
             3
             |
             C
             |
             4
             |
             D-5----6-E
    )";

    const std::string speed_str = std::to_string(default_speed);
    const gurka::ways ways = {{"AB", {{"highway", "primary"}, {"maxspeed", speed_str}}},
                              {"BC", {{"highway", "primary"}, {"maxspeed", speed_str}}},
                              {"CD", {{"highway", "primary"}, {"maxspeed", speed_str}}},
                              {"DE", {{"highway", "primary"}, {"maxspeed", speed_str}}}};

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100, {.05f, .2f});
    closure_map = gurka::buildtiles(layout, ways, {}, {}, tile_dir);

    closure_map.config.put("mjolnir.traffic_extract", tile_dir + "/traffic.tar");
    test::build_live_traffic_data(closure_map.config);

    reader = test::make_clean_graphreader(closure_map.config.get_child("mjolnir"));
  }

  void set_default_speed_on_all_edges() {
    test::customize_live_traffic_data(closure_map.config,
                                      [](baldr::GraphReader&, baldr::TrafficTile&, uint32_t,
                                         baldr::TrafficSpeed* current) -> void {
                                        SetLiveSpeed(current, default_speed);
                                      });
  }

  virtual void SetUp() {
    set_default_speed_on_all_edges();

    // Partially(50%) close 12
    test::customize_live_traffic_data(closure_map.config,
                                      [](baldr::GraphReader& reader, baldr::TrafficTile& tile,
                                         uint32_t index, baldr::TrafficSpeed* current) -> void {
                                        // close_partial_dir_edge(reader, tile, index, 0.5, current,
                                        // "AB", "B", closure_map);
                                        close_partial_dir_edge(reader, tile, index, 0.5, current,
                                                               "AB", "B", closure_map);
                                      });

#if 0
    // Fully close BC
    test::customize_live_traffic_data(closure_map.config,
                                      [](baldr::GraphReader& reader, baldr::TrafficTile& tile,
                                         uint32_t index, baldr::TrafficSpeed* current) -> void {
      close_bidir_edge(reader, tile, index, current, "BC", closure_map);
                                      });
    // Partially(50%) close DE
    test::customize_live_traffic_data(closure_map.config,
                                      [](baldr::GraphReader& reader, baldr::TrafficTile& tile,
                                         uint32_t index, baldr::TrafficSpeed* current) -> void {
      close_partial_dir_edge(reader, tile, index, 0.5, current, "DE", "D", closure_map);
                                      });
#endif
  }

  virtual void TearDown() {
    set_default_speed_on_all_edges();
  }
};

gurka::map ClosureAnnotations::closure_map = {};
const int ClosureAnnotations::default_speed = 36;
const std::string ClosureAnnotations::tile_dir = "test/data/closure_annotations";
std::shared_ptr<baldr::GraphReader> ClosureAnnotations::reader;

const std::string req_without_closure_annotations = R"({
  "locations": [
    {"lat": %s, "lon": %s},
    {"lat": %s, "lon": %s}
  ],
  "costing": "auto",
  "costing_options": {
    "auto": {
      "speed_types": [ "freeflow", "constrained", "predicted", "current"],
      "ignore_closures": true
    }
  },
  "date_time": { "type": 3, "value": "current" },
  "format": "osrm"
}
)";

const std::string req_with_closure_annotations = R"({
  "locations": [
    {"lat": %s, "lon": %s},
    {"lat": %s, "lon": %s}
  ],
  "costing": "auto",
  "costing_options": {
    "auto": {
      "speed_types": [ "freeflow", "constrained", "predicted", "current"],
      "ignore_closures": true
    }
  },
  "date_time": { "type": 3, "value": "current" },
  "format": "osrm",
  "shape_format": "geojson",
  "filters": {
    "attributes": [
      "shape_attributes.closure"
    ],
    "action": "include"
  }
}
)";

TEST_F(ClosureAnnotations, EndOnClosure) {
  const std::string& req =
      (boost::format(req_with_closure_annotations) % std::to_string(closure_map.nodes.at("1").lat()) %
       std::to_string(closure_map.nodes.at("1").lng()) %
       std::to_string(closure_map.nodes.at("2").lat()) %
       std::to_string(closure_map.nodes.at("2").lng()))
          .str();
  auto result = gurka::do_action(Options::route, closure_map, req, reader);
  gurka::assert::raw::expect_path(result, {"AB"});

  // rapidjson::Document response = gurka::convert_to_json(result, valhalla::Options_Format_osrm);
}
