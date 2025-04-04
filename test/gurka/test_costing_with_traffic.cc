#include "gurka.h"
#include "test.h"
#include <boost/format.hpp>
#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;
using LiveTrafficCustomize = test::LiveTrafficCustomize;

const std::vector<std::string>& costing = {"auto",  "taxi",          "bus",
                                           "truck", "motor_scooter", "motorcycle"};

void SetLiveSpeed(baldr::TrafficSpeed* live_speed, uint64_t speed) {
  live_speed->breakpoint1 = 255;
  live_speed->overall_encoded_speed = speed >> 1;
  live_speed->encoded_speed1 = speed >> 1;
}

void update_dir_edge(baldr::GraphReader& reader,
                     baldr::TrafficTile& tile,
                     uint32_t index,
                     baldr::TrafficSpeed* current,
                     const std::string& edge_name,
                     const std::string& end_node,
                     const gurka::map& closure_map,
                     uint64_t speed) {
  baldr::GraphId tile_id(tile.header->tile_id);
  auto edge = std::get<0>(gurka::findEdge(reader, closure_map.nodes, edge_name, end_node));
  if (edge.Tile_Base() == tile_id && edge.id() == index) {
    SetLiveSpeed(current, speed);
  }
}

void update_bidir_edges(baldr::GraphReader& reader,
                        baldr::TrafficTile& tile,
                        uint32_t index,
                        baldr::TrafficSpeed* current,
                        const std::string& edge_name,
                        const gurka::map& closure_map,
                        uint64_t speed) {
  baldr::GraphId tile_id(tile.header->tile_id);
  std::string start_node(1, edge_name.front());
  std::string end_node(1, edge_name.back());

  update_dir_edge(reader, tile, index, current, edge_name, start_node, closure_map, speed);
  update_dir_edge(reader, tile, index, current, edge_name, end_node, closure_map, speed);
}

gurka::map map = {};
const std::unordered_map<std::string, std::string> build_config{
    {"mjolnir.admin", {VALHALLA_SOURCE_DIR "test/data/netherlands_admin.sqlite"}}};

TEST(Standalone, CostingWithTraffic) {
  constexpr double gridsize_metres = 10;

  const std::string ascii_map = R"(
       r---A---B---C---D---E
       |   1   |   |   |   | \
       |   |   |   |   |   |  \
       |   |   y---x   |   |   F
       |   |   |   |   |   |   |\
       |   |   |   |   w---v   |  G---H---I---J---K---L---M
       |   |   |   |   |   |   |  |   |   |   |   |   |   |
       s---Z---Y---X---W---V---U--T---S---R---Q---P---O---N-2-p
       |   |   |   |   |   |   |  |   |   |   |   |   |   |
       |   |   |   |   |   |   |  |   |   |   |   |   |   |
       |   |   |   |   |   |   |  |   |   |   |   |   |   |
       a---b---c---d---e---f---g--h---i---j---k---l---m---o
    )";

  const gurka::ways ways = {
      {"rA", {{"highway", "primary"}}},
      {"AB", {{"highway", "primary"}}},
      {"BC", {{"highway", "primary"}}},
      {"CD", {{"highway", "primary"}}},
      {"DE", {{"highway", "primary"}}},
      {"EF", {{"highway", "primary"}}},
      {"FG", {{"highway", "primary"}}},
      {"GH", {{"highway", "primary"}}},
      {"HI", {{"highway", "primary"}}},
      {"IJ", {{"highway", "primary"}}},
      {"JK", {{"highway", "primary"}}},
      {"KL", {{"highway", "primary"}}},
      {"LM", {{"highway", "primary"}}},
      {"pN", {{"highway", "tertiary"}}},
      {"NO", {{"highway", "tertiary"}}},
      {"OP", {{"highway", "tertiary"}}},
      {"PQ", {{"highway", "tertiary"}}},
      {"QR", {{"highway", "tertiary"}}},
      {"RS", {{"highway", "tertiary"}}},
      {"ST", {{"highway", "tertiary"}}},
      {"TU", {{"highway", "tertiary"}}},
      {"UV", {{"highway", "tertiary"}}},
      {"VW", {{"highway", "tertiary"}}},
      {"WX", {{"highway", "tertiary"}}},
      {"XY", {{"highway", "tertiary"}}},
      {"YZ", {{"highway", "tertiary"}}},
      {"Zs", {{"highway", "tertiary"}}},
      {"ab", {{"highway", "tertiary"}}},
      {"bc", {{"highway", "tertiary"}}},
      {"cd", {{"highway", "tertiary"}}},
      {"de", {{"highway", "tertiary"}}},
      {"ef", {{"highway", "tertiary"}}},
      {"fg", {{"highway", "tertiary"}}},
      {"gh", {{"highway", "tertiary"}}},
      {"hi", {{"highway", "tertiary"}}},
      {"ij", {{"highway", "tertiary"}}},
      {"jk", {{"highway", "tertiary"}}},
      {"kl", {{"highway", "tertiary"}}},
      {"lm", {{"highway", "tertiary"}}},
      {"mo", {{"highway", "tertiary"}}},
      {"oNM", {{"highway", "residential"}, {"oneway", "yes"}}},
      {"LOm", {{"highway", "residential"}, {"oneway", "yes"}}},
      {"lPK", {{"highway", "residential"}, {"oneway", "yes"}}},
      {"JQk", {{"highway", "residential"}, {"oneway", "yes"}}},
      {"jRI", {{"highway", "residential"}, {"oneway", "yes"}}},
      {"HSi", {{"highway", "residential"}, {"oneway", "yes"}}},
      {"hTG", {{"highway", "residential"}, {"oneway", "yes"}}},
      {"FUg", {{"highway", "residential"}, {"oneway", "yes"}}},
      {"fVvE", {{"highway", "residential"}, {"oneway", "yes"}}},
      {"DwWe", {{"highway", "residential"}, {"oneway", "yes"}}},
      {"dXxC", {{"highway", "residential"}, {"oneway", "yes"}}},
      {"ByYc", {{"highway", "residential"}, {"oneway", "yes"}}},
      {"bZA", {{"highway", "residential"}, {"oneway", "yes"}}},
      {"rsa", {{"highway", "residential"}, {"oneway", "yes"}}},
      {"wv", {{"highway", "residential"}}},
      {"yx", {{"highway", "residential"}}},
  };

  const gurka::nodes nodes = {
      {"p", {{"highway", "traffic_signals"}}}, {"N", {{"highway", "traffic_signals"}}},
      {"O", {{"highway", "traffic_signals"}}}, {"P", {{"highway", "traffic_signals"}}},
      {"Q", {{"highway", "traffic_signals"}}}, {"R", {{"highway", "traffic_signals"}}},
      {"S", {{"highway", "traffic_signals"}}}, {"T", {{"highway", "traffic_signals"}}},
      {"U", {{"highway", "traffic_signals"}}}, {"V", {{"highway", "traffic_signals"}}},
      {"W", {{"highway", "traffic_signals"}}}, {"X", {{"highway", "traffic_signals"}}},
      {"Y", {{"highway", "traffic_signals"}}}, {"Z", {{"highway", "traffic_signals"}}},
      {"s", {{"highway", "traffic_signals"}}}, {"r", {{"highway", "traffic_signals"}}},
      {"A", {{"highway", "traffic_signals"}}}, {"B", {{"highway", "traffic_signals"}}},
      {"C", {{"highway", "traffic_signals"}}}, {"D", {{"highway", "traffic_signals"}}},
      {"E", {{"highway", "traffic_signals"}}}, {"F", {{"highway", "traffic_signals"}}},
      {"G", {{"highway", "traffic_signals"}}}, {"H", {{"highway", "traffic_signals"}}},
      {"I", {{"highway", "traffic_signals"}}}, {"J", {{"highway", "traffic_signals"}}},
      {"K", {{"highway", "traffic_signals"}}}, {"L", {{"highway", "traffic_signals"}}},
      {"M", {{"highway", "traffic_signals"}}}, {"a", {{"highway", "traffic_signals"}}},
      {"b", {{"highway", "traffic_signals"}}}, {"c", {{"highway", "traffic_signals"}}},
      {"d", {{"highway", "traffic_signals"}}}, {"e", {{"highway", "traffic_signals"}}},
      {"f", {{"highway", "traffic_signals"}}}, {"g", {{"highway", "traffic_signals"}}},
      {"h", {{"highway", "traffic_signals"}}}, {"i", {{"highway", "traffic_signals"}}},
      {"j", {{"highway", "traffic_signals"}}}, {"k", {{"highway", "traffic_signals"}}},
      {"l", {{"highway", "traffic_signals"}}}, {"m", {{"highway", "traffic_signals"}}},
      {"o", {{"highway", "traffic_signals"}}},
  };

  const auto layout =
      gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {5.1079374, 52.0887174});
  map = gurka::buildtiles(layout, ways, nodes, {}, "test/data/gurka_costing_with_traffic",
                          build_config);

  for (auto& c : costing) {
    auto result = gurka::do_action(valhalla::Options::route, map, {"2", "1"}, c);
    if (c == "truck" || c == "motor_scooter") // favor tertiary road - no traffic in the data
      gurka::assert::raw::expect_path(result, {"pN", "NO", "OP", "PQ", "QR", "RS", "ST", "TU", "UV",
                                               "VW", "WX", "XY", "YZ", "bZA"});
    else // favor primary road - no traffic in the data
      gurka::assert::raw::expect_path(result, {"pN", "oNM", "LM", "KL", "JK", "IJ", "HI", "GH", "FG",
                                               "EF", "DE", "CD", "BC", "ByYc", "ByYc", "YZ", "bZA"});
  }

  {
    map.config.put("mjolnir.traffic_extract", "test/data/gurka_costing_with_traffic/traffic.tar");
    test::build_live_traffic_data(map.config);
    std::shared_ptr<valhalla::baldr::GraphReader> reader =
        test::make_clean_graphreader(map.config.get_child("mjolnir"));

    LiveTrafficCustomize edges_with_traffic = [](baldr::GraphReader& reader, baldr::TrafficTile& tile,
                                                 uint32_t index,
                                                 baldr::TrafficSpeed* current) -> void {
      // update speeds on primary road
      update_bidir_edges(reader, tile, index, current, "rA", map, 35);
      update_bidir_edges(reader, tile, index, current, "AB", map, 35);
      update_bidir_edges(reader, tile, index, current, "BC", map, 35);
      update_bidir_edges(reader, tile, index, current, "CD", map, 35);
      update_bidir_edges(reader, tile, index, current, "DE", map, 35);
      update_bidir_edges(reader, tile, index, current, "EF", map, 35);
      update_bidir_edges(reader, tile, index, current, "FG", map, 35);
      update_bidir_edges(reader, tile, index, current, "GH", map, 35);
      update_bidir_edges(reader, tile, index, current, "HI", map, 35);
      update_bidir_edges(reader, tile, index, current, "IJ", map, 35);
      update_bidir_edges(reader, tile, index, current, "JK", map, 35);
      update_bidir_edges(reader, tile, index, current, "KL", map, 35);
      update_bidir_edges(reader, tile, index, current, "LM", map, 35);

      // update speeds on tertiary road
      update_bidir_edges(reader, tile, index, current, "pN", map, 27);
      update_bidir_edges(reader, tile, index, current, "NO", map, 27);
      update_bidir_edges(reader, tile, index, current, "OP", map, 27);
      update_bidir_edges(reader, tile, index, current, "PQ", map, 27);
      update_bidir_edges(reader, tile, index, current, "QR", map, 27);
      update_bidir_edges(reader, tile, index, current, "RS", map, 27);
      update_bidir_edges(reader, tile, index, current, "ST", map, 27);
      update_bidir_edges(reader, tile, index, current, "TU", map, 27);
      update_bidir_edges(reader, tile, index, current, "UV", map, 27);
      update_bidir_edges(reader, tile, index, current, "VW", map, 27);
      update_bidir_edges(reader, tile, index, current, "WX", map, 27);
      update_bidir_edges(reader, tile, index, current, "XY", map, 27);
      update_bidir_edges(reader, tile, index, current, "YZ", map, 27);
      update_bidir_edges(reader, tile, index, current, "Zs", map, 27);
    };
    test::customize_live_traffic_data(map.config, edges_with_traffic);

    for (auto& c : costing) {
      const std::string& req_no_traffic =
          (boost::format(
               R"({"locations":[{"lat":%s,"lon":%s},{"lat":%s,"lon":%s}],"costing":"%s"})") %
           std::to_string(map.nodes.at("2").lat()) % std::to_string(map.nodes.at("2").lng()) %
           std::to_string(map.nodes.at("1").lat()) % std::to_string(map.nodes.at("1").lng()) % c)
              .str();
      auto result = gurka::do_action(valhalla::Options::route, map, req_no_traffic, reader);

      if (c == "truck" ||
          c == "motor_scooter") // favor tertiary road - traffic added but not requested
        gurka::assert::raw::expect_path(result, {"pN", "NO", "OP", "PQ", "QR", "RS", "ST", "TU", "UV",
                                                 "VW", "WX", "XY", "YZ", "bZA"});
      else // favor primary road - traffic added but not requested
        gurka::assert::raw::expect_path(result,
                                        {"pN", "oNM", "LM", "KL", "JK", "IJ", "HI", "GH", "FG", "EF",
                                         "DE", "CD", "BC", "ByYc", "ByYc", "YZ", "bZA"});
    }

    std::string date_type = "3"; // invariant time
    for (auto& c : costing) {
      const std::string& req_with_traffic =
          (boost::format(
               R"({"locations":[{"lat":%s,"lon":%s},{"lat":%s,"lon":%s}],"costing":"%s", "costing_options": {"%s": {"speed_types":["freeflow","constrained","predicted","current"]}}, "date_time":{"type":3, "value": "current"}})") %
           std::to_string(map.nodes.at("2").lat()) % std::to_string(map.nodes.at("2").lng()) %
           std::to_string(map.nodes.at("1").lat()) % std::to_string(map.nodes.at("1").lng()) % c % c)
              .str();
      auto result = gurka::do_action(valhalla::Options::route, map, req_with_traffic, reader);

      // favor tertiary road - traffic in use
      gurka::assert::raw::expect_path(result, {"pN", "NO", "OP", "PQ", "QR", "RS", "ST", "TU", "UV",
                                               "VW", "WX", "XY", "YZ", "bZA"});
    }
  }
}
