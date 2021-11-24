#include "gurka.h"
#include "test.h"

#include "baldr/graphreader.h"
#include "baldr/traffictile.h"

using namespace valhalla;
using LiveTrafficCustomize = test::LiveTrafficCustomize;

namespace {
inline void SetLiveSpeed(baldr::TrafficSpeed* live_speed, uint64_t speed) {
  live_speed->breakpoint1 = 255;
  live_speed->overall_encoded_speed = speed >> 1;
  live_speed->encoded_speed1 = speed >> 1;
}

void close_dir_edge(baldr::GraphReader& reader,
                    baldr::TrafficTile& tile,
                    uint32_t index,
                    baldr::TrafficSpeed* current,
                    const std::string& edge_name,
                    const std::string& end_node,
                    const gurka::map& closure_map) {
  baldr::GraphId tile_id(tile.header->tile_id);
  auto edge = std::get<0>(gurka::findEdge(reader, closure_map.nodes, edge_name, end_node));
  if (edge.Tile_Base() == tile_id && edge.id() == index) {
    SetLiveSpeed(current, 0);
  }
}

void close_bidir_edge(baldr::GraphReader& reader,
                      baldr::TrafficTile& tile,
                      uint32_t index,
                      baldr::TrafficSpeed* current,
                      const std::string& edge_name,
                      const gurka::map& closure_map) {
  baldr::GraphId tile_id(tile.header->tile_id);
  std::string start_node(1, edge_name.front());
  std::string end_node(1, edge_name.back());

  close_dir_edge(reader, tile, index, current, edge_name, start_node, closure_map);
  close_dir_edge(reader, tile, index, current, edge_name, end_node, closure_map);
}
} // namespace

class ClosurePenalty : public ::testing::TestWithParam<std::string> {
protected:
  static gurka::map closure_map;
  static int const default_speed;
  static std::string const tile_dir;
  static std::shared_ptr<baldr::GraphReader> reader;

  static void SetUpTestSuite() {
    const std::string ascii_map = R"(

      A-----B-----C
      |     2     |
      |     |     |
      D-----E-----F
      |     |     |
      G-----H-----I
      |     |     |
      J-----K-----L
      |     |     |
      M-----N-----O
      |     |     |
      P-----Q-----R
      |     |     |
      |     1     |
      S-----T-----U
    )";

    const gurka::ways ways = {
        {"AB", {{"highway", "primary"}, {"maxspeed", "30"}}},
        {"BC", {{"highway", "primary"}, {"maxspeed", "30"}}},
        {"DE", {{"highway", "primary"}, {"maxspeed", "30"}}},
        {"EF", {{"highway", "primary"}, {"maxspeed", "30"}}},
        {"GH", {{"highway", "primary"}, {"maxspeed", "30"}}},
        {"HI", {{"highway", "primary"}, {"maxspeed", "30"}}},
        {"JK", {{"highway", "primary"}, {"maxspeed", "30"}}},
        {"KL", {{"highway", "primary"}, {"maxspeed", "30"}}},
        {"MN", {{"highway", "primary"}, {"maxspeed", "30"}}},
        {"NO", {{"highway", "primary"}, {"maxspeed", "30"}}},
        {"PQ", {{"highway", "primary"}, {"maxspeed", "30"}}},
        {"QR", {{"highway", "primary"}, {"maxspeed", "30"}}},
        {"ST", {{"highway", "primary"}, {"maxspeed", "30"}}},
        {"TU", {{"highway", "primary"}, {"maxspeed", "30"}}},
        // left section is slow
        {"AD", {{"highway", "primary"}, {"maxspeed", "20"}}},
        {"DG", {{"highway", "primary"}, {"maxspeed", "20"}}},
        {"GJ", {{"highway", "primary"}, {"maxspeed", "20"}}},
        {"JM", {{"highway", "primary"}, {"maxspeed", "20"}}},
        {"MP", {{"highway", "primary"}, {"maxspeed", "20"}}},
        {"PS", {{"highway", "primary"}, {"maxspeed", "20"}}},
        // middle section is fastest
        {"BE", {{"highway", "primary"}, {"maxspeed", "45"}}},
        {"EH", {{"highway", "primary"}, {"maxspeed", "45"}}},
        {"HK", {{"highway", "primary"}, {"maxspeed", "45"}}},
        {"KN", {{"highway", "primary"}, {"maxspeed", "45"}}},
        {"NQ", {{"highway", "primary"}, {"maxspeed", "45"}}},
        {"QT", {{"highway", "primary"}, {"maxspeed", "45"}}},
        // right section slower than middle
        {"CF", {{"highway", "primary"}, {"maxspeed", "35"}}},
        {"FI", {{"highway", "primary"}, {"maxspeed", "35"}}},
        {"IL", {{"highway", "primary"}, {"maxspeed", "35"}}},
        {"LO", {{"highway", "primary"}, {"maxspeed", "35"}}},
        {"OR", {{"highway", "primary"}, {"maxspeed", "35"}}},
        {"RU", {{"highway", "primary"}, {"maxspeed", "35"}}},
    };

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100, {.05f, .2f});
    closure_map = gurka::buildtiles(layout, ways, {}, {}, tile_dir);

    closure_map.config.put("mjolnir.traffic_extract", tile_dir + "/traffic.tar");
    test::build_live_traffic_data(closure_map.config);

    reader = test::make_clean_graphreader(closure_map.config.get_child("mjolnir"));
  }

  void set_unknown_live_speed_on_all_edges() {
    test::customize_live_traffic_data(closure_map.config,
                                      [](baldr::GraphReader& reader, baldr::TrafficTile& tile,
                                         uint32_t index, baldr::TrafficSpeed* current) -> void {
                                        (void)reader, (void)tile, (void)index;
                                        SetLiveSpeed(current, UNKNOWN_TRAFFIC_SPEED_RAW);
                                      });
  }

  virtual void SetUp() {
    set_unknown_live_speed_on_all_edges();
  }

  virtual void TearDown() {
    set_unknown_live_speed_on_all_edges();
  }
};

gurka::map ClosurePenalty::closure_map = {};
const std::string ClosurePenalty::tile_dir = "test/data/closure_penalty";
std::shared_ptr<baldr::GraphReader> ClosurePenalty::reader;

TEST_P(ClosurePenalty, AvoidClosure) {
  std::string costing = GetParam();

  {
    auto result =
        gurka::do_action(valhalla::Options::route, closure_map, {"1", "C"}, costing,
                         {{"/date_time/type", "3"}, {"/date_time/value", "current"}}, reader);
    gurka::assert::raw::expect_path(result, {"QT", "NQ", "KN", "HK", "EH", "BE", "BC"});
  }

  // Close entire stretch of center road
  test::LiveTrafficCustomize close_edge = [](baldr::GraphReader& reader, baldr::TrafficTile& tile,
                                             int index, baldr::TrafficSpeed* current) -> void {
    close_bidir_edge(reader, tile, index, current, "BE", closure_map);
    close_bidir_edge(reader, tile, index, current, "EH", closure_map);
    close_bidir_edge(reader, tile, index, current, "HK", closure_map);
    close_bidir_edge(reader, tile, index, current, "NQ", closure_map);
    close_bidir_edge(reader, tile, index, current, "QT", closure_map);
  };
  test::customize_live_traffic_data(closure_map.config, close_edge);

  // Route from closed edge to an open one
  {
    const std::string& req_include_closures =
        (boost::format(
             R"({"locations":[{"lat":%s,"lon":%s,"search_filter":{"exclude_closures":false}},{"lat":%s,"lon":%s}],"costing":"%s", "costing_options": {"%s": {"speed_types":["freeflow","constrained","predicted","current"]}}, "date_time":{"type":"3", "value": "current"}})") %
         std::to_string(closure_map.nodes.at("1").lat()) %
         std::to_string(closure_map.nodes.at("1").lng()) %
         std::to_string(closure_map.nodes.at("C").lat()) %
         std::to_string(closure_map.nodes.at("C").lng()) % costing % costing)
            .str();

    std::vector<std::string> expected_path = {"QT", "TU", "RU", "OR", "LO", "IL", "FI", "CF"};
    if (costing == "motor_scooter") {
      expected_path = {"QT", "QR", "OR", "LO", "IL", "FI", "CF"};
    }
    auto result =
        gurka::do_action(valhalla::Options::route, closure_map, req_include_closures, reader);
    gurka::assert::raw::expect_path(result, expected_path);
  }

  // Route from & to closed edge
  {
    const std::string& req_include_closures =
        (boost::format(
             R"({"locations":[{"lat":%s,"lon":%s,"search_filter":{"exclude_closures":false}},{"lat":%s,"lon":%s,"search_filter":{"exclude_closures":false}}],"costing":"%s", "costing_options": {"%s": {"speed_types":["freeflow","constrained","predicted","current"]}}, "date_time":{"type":"3", "value": "current"}})") %
         std::to_string(closure_map.nodes.at("1").lat()) %
         std::to_string(closure_map.nodes.at("1").lng()) %
         std::to_string(closure_map.nodes.at("2").lat()) %
         std::to_string(closure_map.nodes.at("2").lng()) % costing % costing)
            .str();

    std::vector<std::string> expected_path = {"QT", "TU", "RU", "OR", "LO",
                                              "IL", "FI", "CF", "BC", "BE"};
    if (costing == "motor_scooter") {
      expected_path = {"QT", "NQ", "KN", "HK", "EH", "BE"};
    }
    auto result =
        gurka::do_action(valhalla::Options::route, closure_map, req_include_closures, reader);
    gurka::assert::raw::expect_path(result, expected_path);
  }

  {
    const std::string& req_include_closures =
        (boost::format(
             R"({"locations":[{"lat":%s,"lon":%s,"search_filter":{"exclude_closures":false}},{"lat":%s,"lon":%s}],"costing":"%s", "costing_options": {"%s": {"speed_types":["freeflow","constrained","predicted","current"]}}, "date_time":{"type":"3", "value": "current"}})") %
         std::to_string(closure_map.nodes.at("1").lat()) %
         std::to_string(closure_map.nodes.at("1").lng()) %
         std::to_string(closure_map.nodes.at("A").lat()) %
         std::to_string(closure_map.nodes.at("A").lng()) % costing % costing)
            .str();

    std::vector<std::string> expected_path = {"QT", "ST", "PS", "MP", "JM", "GJ", "DG", "AD"};
    if (costing == "motor_scooter") {
      expected_path = {"QT", "PQ", "MP", "JM", "GJ", "DG", "AD"};
    }
    auto result =
        gurka::do_action(valhalla::Options::route, closure_map, req_include_closures, reader);
    gurka::assert::raw::expect_path(result, expected_path);
  }

  // Change closure factor to 1.0, this means no added penalty to closed edges
  {
    const std::string& req_include_closures =
        (boost::format(
             R"({"locations":[{"lat":%s,"lon":%s,"search_filter":{"exclude_closures":false}},{"lat":%s,"lon":%s,"search_filter":{"exclude_closures":false}}],"costing":"%s", "costing_options": {"%s": {"closure_factor": 1.0, "speed_types":["freeflow","constrained","predicted","current"]}}, "date_time":{"type":"3", "value": "current"}})") %
         std::to_string(closure_map.nodes.at("1").lat()) %
         std::to_string(closure_map.nodes.at("1").lng()) %
         std::to_string(closure_map.nodes.at("2").lat()) %
         std::to_string(closure_map.nodes.at("2").lng()) % costing % costing)
            .str();
    auto result =
        gurka::do_action(valhalla::Options::route, closure_map, req_include_closures, reader);
    gurka::assert::raw::expect_path(result, {"QT", "NQ", "KN", "HK", "EH", "BE"});
  }
}

std::vector<std::string> buildParams() {
  // Return the different costings we want to test closures against
  return {
      "auto", "motorcycle", "motor_scooter", "bus", "truck", "taxi",
  };
}

INSTANTIATE_TEST_SUITE_P(Test, ClosurePenalty, ::testing::ValuesIn(buildParams()));
