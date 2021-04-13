#include "gurka.h"
#include "test.h"

#include <boost/format.hpp>
#include <gtest/gtest.h>

using namespace valhalla;
using LiveTrafficCustomize = test::LiveTrafficCustomize;

namespace {

inline void SetLiveSpeedFrom(baldr::TrafficSpeed* live_speed, uint8_t speed, uint8_t breakpoint1) {
  live_speed->breakpoint1 = breakpoint1;
  live_speed->breakpoint2 = 255;
  live_speed->encoded_speed2 = speed >> 1;
  live_speed->encoded_speed3 = UNKNOWN_TRAFFIC_SPEED_RAW;
}

inline void SetLiveSpeedUpto(baldr::TrafficSpeed* live_speed, uint8_t speed, uint8_t breakpoint1) {
  live_speed->breakpoint1 = breakpoint1;
  live_speed->encoded_speed1 = speed >> 1;
}

inline void SetLiveSpeed(baldr::TrafficSpeed* live_speed, uint8_t speed) {
  live_speed->breakpoint1 = 255;
  live_speed->overall_encoded_speed = speed >> 1;
  live_speed->encoded_speed1 = speed >> 1;
}

void close_partial_dir_edge_from(baldr::GraphReader& reader,
                                 baldr::TrafficTile& tile,
                                 uint32_t index,
                                 double percent_along,
                                 baldr::TrafficSpeed* current,
                                 const std::string& edge_name,
                                 const std::string& end_node,
                                 const gurka::map& map) {
  baldr::GraphId tile_id(tile.header->tile_id);
  auto edge = std::get<0>(gurka::findEdge(reader, map.nodes, edge_name, end_node));
  if (edge.Tile_Base() == tile_id && edge.id() == index) {
    uint8_t breakpoint1 = static_cast<uint8_t>(255 * percent_along);
    SetLiveSpeedFrom(current, 0, breakpoint1);
  }
}

void close_partial_dir_edge_upto(baldr::GraphReader& reader,
                                 baldr::TrafficTile& tile,
                                 uint32_t index,
                                 double percent_along,
                                 baldr::TrafficSpeed* current,
                                 const std::string& edge_name,
                                 const std::string& end_node,
                                 const gurka::map& map) {
  baldr::GraphId tile_id(tile.header->tile_id);
  auto edge = std::get<0>(gurka::findEdge(reader, map.nodes, edge_name, end_node));
  if (edge.Tile_Base() == tile_id && edge.id() == index) {
    uint8_t breakpoint1 = static_cast<uint8_t>(255 * percent_along);
    SetLiveSpeedUpto(current, 0, breakpoint1);
  }
}

void close_dir_edge(baldr::GraphReader& reader,
                    baldr::TrafficTile& tile,
                    uint32_t index,
                    baldr::TrafficSpeed* current,
                    const std::string& edge_name,
                    const std::string& end_node,
                    const gurka::map& map) {
  baldr::GraphId tile_id(tile.header->tile_id);
  auto edge = std::get<0>(gurka::findEdge(reader, map.nodes, edge_name, end_node));
  if (edge.Tile_Base() == tile_id && edge.id() == index) {
    SetLiveSpeed(current, 0);
  }
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
              D-5---E
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

    // Partially close AB from 50%
    test::customize_live_traffic_data(closure_map.config,
                                      [](baldr::GraphReader& reader, baldr::TrafficTile& tile,
                                         uint32_t index, baldr::TrafficSpeed* current) -> void {
                                        close_partial_dir_edge_from(reader, tile, index, 0.5, current,
                                                                    "AB", "B", closure_map);
                                      });

    // Fully close BC
    test::customize_live_traffic_data(closure_map.config,
                                      [](baldr::GraphReader& reader, baldr::TrafficTile& tile,
                                         uint32_t index, baldr::TrafficSpeed* current) -> void {
                                        close_bidir_edge(reader, tile, index, current, "BC",
                                                         closure_map);
                                      });
    // Partially close DE upto 50%
    test::customize_live_traffic_data(closure_map.config,
                                      [](baldr::GraphReader& reader, baldr::TrafficTile& tile,
                                         uint32_t index, baldr::TrafficSpeed* current) -> void {
                                        close_partial_dir_edge_upto(reader, tile, index, 0.5, current,
                                                                    "DE", "E", closure_map);
                                      });
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

void expect_closures(const rapidjson::Document& response,
                     const std::vector<std::pair<int, int>>& expected_closures,
                     int expected_coord_count) {
  auto route = response["routes"][0].GetObject();
  auto leg = route["legs"][0].GetObject();
  size_t num_coordinates = route["geometry"]["coordinates"].Size();
  size_t num_closures = leg["closures"].Size();

  ASSERT_EQ(num_coordinates, expected_coord_count);
  ASSERT_EQ(num_closures, expected_closures.size());
  int idx = 0;
  for (const auto& start_end_pair : expected_closures) {
    EXPECT_EQ(leg["closures"][idx]["geometry_index_start"].GetInt(), start_end_pair.first);
    EXPECT_EQ(leg["closures"][idx]["geometry_index_end"].GetInt(), start_end_pair.second);
    idx++;
  }
  auto last_closure = leg["closures"][num_closures - 1].GetObject();
  ASSERT_LE(last_closure["geometry_index_end"].GetInt(), num_coordinates);
}

TEST_F(ClosureAnnotations, EndOnClosure) {
  const std::string& req =
      (boost::format(req_with_closure_annotations) % std::to_string(closure_map.nodes.at("1").lat()) %
       std::to_string(closure_map.nodes.at("1").lng()) %
       std::to_string(closure_map.nodes.at("2").lat()) %
       std::to_string(closure_map.nodes.at("2").lng()))
          .str();
  auto result = gurka::do_action(Options::route, closure_map, req, reader);
  gurka::assert::raw::expect_path(result, {"AB"});

  rapidjson::Document response = gurka::convert_to_json(result, valhalla::Options_Format_osrm);
  expect_closures(response, {{1, 2}}, 3);
}

TEST_F(ClosureAnnotations, EndWithConsecutiveClosures) {
  const std::string& req =
      (boost::format(req_with_closure_annotations) % std::to_string(closure_map.nodes.at("1").lat()) %
       std::to_string(closure_map.nodes.at("1").lng()) %
       std::to_string(closure_map.nodes.at("3").lat()) %
       std::to_string(closure_map.nodes.at("3").lng()))
          .str();
  auto result = gurka::do_action(Options::route, closure_map, req, reader);
  gurka::assert::raw::expect_path(result, {"AB", "BC"});

  rapidjson::Document response = gurka::convert_to_json(result, valhalla::Options_Format_osrm);
  expect_closures(response, {{1, 3}}, 4);
}

TEST_F(ClosureAnnotations, IntermediateClosure) {
  const std::string& req =
      (boost::format(req_with_closure_annotations) % std::to_string(closure_map.nodes.at("1").lat()) %
       std::to_string(closure_map.nodes.at("1").lng()) %
       std::to_string(closure_map.nodes.at("4").lat()) %
       std::to_string(closure_map.nodes.at("4").lng()))
          .str();
  auto result = gurka::do_action(Options::route, closure_map, req, reader);
  gurka::assert::raw::expect_path(result, {"AB", "BC", "CD"});

  rapidjson::Document response = gurka::convert_to_json(result, valhalla::Options_Format_osrm);
  expect_closures(response, {{1, 3}}, 5);
}

TEST_F(ClosureAnnotations, BeginAtClosure) {
  const std::string& req =
      (boost::format(req_with_closure_annotations) % std::to_string(closure_map.nodes.at("2").lat()) %
       std::to_string(closure_map.nodes.at("2").lng()) %
       std::to_string(closure_map.nodes.at("4").lat()) %
       std::to_string(closure_map.nodes.at("4").lng()))
          .str();
  auto result = gurka::do_action(Options::route, closure_map, req, reader);
  gurka::assert::raw::expect_path(result, {"AB", "BC", "CD"});

  rapidjson::Document response = gurka::convert_to_json(result, valhalla::Options_Format_osrm);
  expect_closures(response, {{0, 2}}, 4);
}

TEST_F(ClosureAnnotations, AllWithinClosure) {
  const std::string& req =
      (boost::format(req_with_closure_annotations) % std::to_string(closure_map.nodes.at("2").lat()) %
       std::to_string(closure_map.nodes.at("2").lng()) %
       std::to_string(closure_map.nodes.at("3").lat()) %
       std::to_string(closure_map.nodes.at("3").lng()))
          .str();
  auto result = gurka::do_action(Options::route, closure_map, req, reader);
  gurka::assert::raw::expect_path(result, {"AB", "BC"});

  rapidjson::Document response = gurka::convert_to_json(result, valhalla::Options_Format_osrm);
  expect_closures(response, {{0, 2}}, 3);
}

TEST_F(ClosureAnnotations, DiscontinuousClosures) {
  const std::string& req =
      (boost::format(req_with_closure_annotations) % std::to_string(closure_map.nodes.at("A").lat()) %
       std::to_string(closure_map.nodes.at("A").lng()) %
       std::to_string(closure_map.nodes.at("E").lat()) %
       std::to_string(closure_map.nodes.at("E").lng()))
          .str();
  auto result = gurka::do_action(Options::route, closure_map, req, reader);
  gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DE"});

  rapidjson::Document response = gurka::convert_to_json(result, valhalla::Options_Format_osrm);
  expect_closures(response, {{1, 3}, {4, 5}}, 7);
}
