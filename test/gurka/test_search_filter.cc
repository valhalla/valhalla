
#include "gurka.h"
#include "test.h"

#include <boost/format.hpp>
#include <gtest/gtest.h>

using namespace valhalla::baldr;
using valhalla_exception_t = valhalla::valhalla_exception_t;
namespace gurka = valhalla::gurka;
using LiveTrafficCustomize = test::LiveTrafficCustomize;

/*************************************************************/
class SearchFilter : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize = 100;

    // remark: without admin database, left-side driving is the
    // default driving side.
    const std::string ascii_map = R"(
    B---------C
    |   2   8 |
    |         ↑
    |1    4  3|
    |7        |
    A---------D
     \        |
      \ 5     |
       \   6  |
        F-----E
        |     |      
        |     y      
        |     |      
        G-x---H
         )";
    const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
    const gurka::ways ways = {{"AB", {{"highway", "motorway"}}},
                              {"BC", {{"highway", "residential"}, {"tunnel", "yes"}}},
                              {"CD", {{"highway", "primary"}, {"oneway", "-1"}}},
                              {"AD", {{"highway", "primary"}}},
                              {"DE", {{"highway", "primary"}}},
                              {"EF", {{"highway", "primary"}, {"bridge", "yes"}, {"toll", "yes"}}},
                              {"AF", {{"highway", "motorway_link"}}},
                              {"FG", {{"highway", "secondary"}}},
                              {"GH", {{"route", "ferry"}}},
                              {"HE", {{"highway", "secondary"}}}};
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/search_filter");
  }
};

gurka::map SearchFilter::map = {};

TEST_F(SearchFilter, Unfiltered) {
  auto from = "1";
  auto to = "2";

  const std::string& request =
      (boost::format(R"({"locations":[{"lat":%s,"lon":%s},{"lat":%s,"lon":%s}],"costing":"auto"})") %
       std::to_string(map.nodes.at(from).lat()) % std::to_string(map.nodes.at(from).lng()) %
       std::to_string(map.nodes.at(to).lat()) % std::to_string(map.nodes.at(to).lng()))
          .str();

  auto result = gurka::do_action(valhalla::Options::route, map, request);

  // should take the shortest path
  gurka::assert::osrm::expect_steps(result, {"AB", "BC"});
  gurka::assert::raw::expect_path(result, {"AB", "BC"});
}
TEST_F(SearchFilter, NodeSnapped) {
  auto from = "B";
  auto to = "C";

  const std::string& request =
      (boost::format(
           R"({"locations":[{"lat":%s,"lon":%s,"search_filter":{"exclude_tunnel":true}},{"lat":%s,"lon":%s}],"costing":"auto"})") %
       std::to_string(map.nodes.at(from).lat()) % std::to_string(map.nodes.at(from).lng()) %
       std::to_string(map.nodes.at(to).lat()) % std::to_string(map.nodes.at(to).lng()))
          .str();

  auto result = gurka::do_action(valhalla::Options::route, map, request);

  // should take the shortest path
  gurka::assert::osrm::expect_steps(result, {"AB", "AD", "CD"});
  gurka::assert::raw::expect_path(result, {"AB", "AD", "CD"});
}
TEST_F(SearchFilter, Heading) {
  auto from = "1";
  auto to = "2";

  const std::string& request =
      (boost::format(
           R"({"locations":[{"lat":%s,"lon":%s,"heading":180,"heading_tolerance":45},{"lat":%s,"lon":%s}],"costing":"auto"})") %
       std::to_string(map.nodes.at(from).lat()) % std::to_string(map.nodes.at(from).lng()) %
       std::to_string(map.nodes.at(to).lat()) % std::to_string(map.nodes.at(to).lng()))
          .str();
  auto result = gurka::do_action(valhalla::Options::route, map, request);

  // should take the long way around starting southbound due to heading at origin
  gurka::assert::osrm::expect_steps(result, {"AB", "AD", "CD", "BC"});
  gurka::assert::raw::expect_path(result, {"AB", "AD", "CD", "BC"});
}
TEST_F(SearchFilter, PreferredSide) {
  auto from = "7";
  auto to = "8";

  const std::string& request =
      (boost::format(
           R"({"locations":[{"lat":%s,"lon":%s},{"lat":%s,"lon":%s,"preferred_side":"same"}],"costing":"auto"})") %
       std::to_string(map.nodes.at(from).lat()) % std::to_string(map.nodes.at(from).lng()) %
       std::to_string(map.nodes.at(to).lat()) % std::to_string(map.nodes.at(to).lng()))
          .str();
  auto result = gurka::do_action(valhalla::Options::route, map, request);

  // should take the long way around starting southbound due to preferred side at destination
  gurka::assert::osrm::expect_steps(result, {"AB", "AD", "CD", "BC"});
  gurka::assert::raw::expect_path(result, {"AB", "AD", "CD", "BC"});
}
TEST_F(SearchFilter, StreetSideCutoff) {
  auto from = "7";
  auto to = "8";

  const std::string& request =
      (boost::format(
           R"({"locations":[{"lat":%s,"lon":%s},{"lat":%s,"lon":%s,"preferred_side":"same","street_side_cutoff":"primary"}],"costing":"auto"})") %
       std::to_string(map.nodes.at(from).lat()) % std::to_string(map.nodes.at(from).lng()) %
       std::to_string(map.nodes.at(to).lat()) % std::to_string(map.nodes.at(to).lng()))
          .str();
  auto result = gurka::do_action(valhalla::Options::route, map, request);

  // should take the short way in the north
  gurka::assert::osrm::expect_steps(result, {"AB", "BC"});
  gurka::assert::raw::expect_path(result, {"AB", "BC"});
}
TEST_F(SearchFilter, MaxRoadClass) {
  auto from = "1";
  auto to = "2";

  // Should snap origin to CD as the search_filter disallows motorways
  const std::string& request =
      (boost::format(
           R"({"locations":[{"lat":%s,"lon":%s,"search_filter":{"max_road_class":"primary"}},{"lat":%s,"lon":%s}],"costing":"auto"})") %
       std::to_string(map.nodes.at(from).lat()) % std::to_string(map.nodes.at(from).lng()) %
       std::to_string(map.nodes.at(to).lat()) % std::to_string(map.nodes.at(to).lng()))
          .str();
  auto result = gurka::do_action(valhalla::Options::route, map, request);
  gurka::assert::osrm::expect_steps(result, {"AD", "AB", "BC"});
  gurka::assert::raw::expect_path(result, {"AD", "AB", "BC"});
}
TEST_F(SearchFilter, MinRoadClass) {
  auto from = "1";
  auto to = "2";
  // Should snap destination to AB as the search_filter disallows primary
  const std::string& request =
      (boost::format(
           R"({"locations":[{"lat":%s,"lon":%s},{"lat":%s,"lon":%s,"search_filter":{"min_road_class":"motorway"}}],"costing":"auto"})") %
       std::to_string(map.nodes.at(from).lat()) % std::to_string(map.nodes.at(from).lng()) %
       std::to_string(map.nodes.at(to).lat()) % std::to_string(map.nodes.at(to).lng()))
          .str();
  auto result = gurka::do_action(valhalla::Options::route, map, request);
  gurka::assert::osrm::expect_steps(result, {"AB"});
  gurka::assert::raw::expect_path(result, {"AB"});
}
TEST_F(SearchFilter, ExcludeTunnel) {
  auto from = "2";
  auto to = "1";
  const std::string& request_unfiltered =
      (boost::format(R"({"locations":[{"lat":%s,"lon":%s},{"lat":%s,"lon":%s}],"costing":"auto"})") %
       std::to_string(map.nodes.at(from).lat()) % std::to_string(map.nodes.at(from).lng()) %
       std::to_string(map.nodes.at(to).lat()) % std::to_string(map.nodes.at(to).lng()))
          .str();
  auto result_unfiltered = gurka::do_action(valhalla::Options::route, map, request_unfiltered);
  gurka::assert::osrm::expect_steps(result_unfiltered, {"BC", "AB"});
  gurka::assert::raw::expect_path(result_unfiltered, {"BC", "AB"});

  const std::string& request_filtered =
      (boost::format(
           R"({"locations":[{"lat":%s,"lon":%s,"search_filter":{"exclude_tunnel":true}},{"lat":%s,"lon":%s}],"costing":"auto"})") %
       std::to_string(map.nodes.at(from).lat()) % std::to_string(map.nodes.at(from).lng()) %
       std::to_string(map.nodes.at(to).lat()) % std::to_string(map.nodes.at(to).lng()))
          .str();
  auto result_filtered = gurka::do_action(valhalla::Options::route, map, request_filtered);
  gurka::assert::osrm::expect_steps(result_filtered, {"AB"});
  gurka::assert::raw::expect_path(result_filtered, {"AB"});
}
TEST_F(SearchFilter, ExcludeBridge) {
  auto from = "6";
  auto to = "3";
  const std::string& request_unfiltered =
      (boost::format(R"({"locations":[{"lat":%s,"lon":%s},{"lat":%s,"lon":%s}],"costing":"auto"})") %
       std::to_string(map.nodes.at(from).lat()) % std::to_string(map.nodes.at(from).lng()) %
       std::to_string(map.nodes.at(to).lat()) % std::to_string(map.nodes.at(to).lng()))
          .str();
  auto result_unfiltered = gurka::do_action(valhalla::Options::route, map, request_unfiltered);
  gurka::assert::osrm::expect_steps(result_unfiltered, {"EF", "DE"});
  gurka::assert::raw::expect_path(result_unfiltered, {"EF", "DE", "CD"});

  const std::string& request_filtered =
      (boost::format(
           R"({"locations":[{"lat":%s,"lon":%s,"search_filter":{"exclude_bridge":true}},{"lat":%s,"lon":%s}],"costing":"auto"})") %
       std::to_string(map.nodes.at(from).lat()) % std::to_string(map.nodes.at(from).lng()) %
       std::to_string(map.nodes.at(to).lat()) % std::to_string(map.nodes.at(to).lng()))
          .str();
  auto result_filtered = gurka::do_action(valhalla::Options::route, map, request_filtered);
  gurka::assert::osrm::expect_steps(result_filtered, {"AD", "CD"});
  gurka::assert::raw::expect_path(result_filtered, {"AD", "CD"});
}
TEST_F(SearchFilter, ExcludeRamp) {
  auto from = "5";
  auto to = "2";
  const std::string& request_unfiltered =
      (boost::format(R"({"locations":[{"lat":%s,"lon":%s},{"lat":%s,"lon":%s}],"costing":"auto"})") %
       std::to_string(map.nodes.at(from).lat()) % std::to_string(map.nodes.at(from).lng()) %
       std::to_string(map.nodes.at(to).lat()) % std::to_string(map.nodes.at(to).lng()))
          .str();
  auto result_unfiltered = gurka::do_action(valhalla::Options::route, map, request_unfiltered);
  gurka::assert::osrm::expect_steps(result_unfiltered, {"AB", "BC"});
  gurka::assert::raw::expect_path(result_unfiltered, {"AF", "AB", "BC"});

  const std::string& request_filtered =
      (boost::format(
           R"({"locations":[{"lat":%s,"lon":%s,"search_filter":{"exclude_ramp":true}},{"lat":%s,"lon":%s}],"costing":"auto"})") %
       std::to_string(map.nodes.at(from).lat()) % std::to_string(map.nodes.at(from).lng()) %
       std::to_string(map.nodes.at(to).lat()) % std::to_string(map.nodes.at(to).lng()))
          .str();
  auto result_filtered = gurka::do_action(valhalla::Options::route, map, request_filtered);

  gurka::assert::osrm::expect_steps(result_filtered, {"AD", "AB", "BC"});
  gurka::assert::raw::expect_path(result_filtered, {"AD", "AB", "BC"});
}
TEST_F(SearchFilter, ExcludeFerry) {
  auto from = "x";
  auto to = "y";
  const std::string& request_unfiltered =
      (boost::format(R"({"locations":[{"lat":%s,"lon":%s},{"lat":%s,"lon":%s}],"costing":"auto"})") %
       std::to_string(map.nodes.at(from).lat()) % std::to_string(map.nodes.at(from).lng()) %
       std::to_string(map.nodes.at(to).lat()) % std::to_string(map.nodes.at(to).lng()))
          .str();
  auto result_unfiltered = gurka::do_action(valhalla::Options::route, map, request_unfiltered);
  gurka::assert::osrm::expect_steps(result_unfiltered, {"GH", "HE"});
  gurka::assert::raw::expect_path(result_unfiltered, {"GH", "HE"});

  const std::string& request_filtered =
      (boost::format(
           R"({"locations":[{"lat":%s,"lon":%s,"search_filter":{"exclude_ferry":true}},{"lat":%s,"lon":%s}],"costing":"auto"})") %
       std::to_string(map.nodes.at(from).lat()) % std::to_string(map.nodes.at(from).lng()) %
       std::to_string(map.nodes.at(to).lat()) % std::to_string(map.nodes.at(to).lng()))
          .str();
  auto result_filtered = gurka::do_action(valhalla::Options::route, map, request_filtered);

  gurka::assert::osrm::expect_steps(result_filtered, {"FG", "EF", "HE"});
  gurka::assert::raw::expect_path(result_filtered, {"FG", "EF", "HE"});
}
TEST_F(SearchFilter, ExcludeToll) {
  auto from = "6";
  auto to = "3";
  const std::string& request_unfiltered =
      (boost::format(R"({"locations":[{"lat":%s,"lon":%s},{"lat":%s,"lon":%s}],"costing":"auto"})") %
       std::to_string(map.nodes.at(from).lat()) % std::to_string(map.nodes.at(from).lng()) %
       std::to_string(map.nodes.at(to).lat()) % std::to_string(map.nodes.at(to).lng()))
          .str();
  auto result_unfiltered = gurka::do_action(valhalla::Options::route, map, request_unfiltered);
  gurka::assert::osrm::expect_steps(result_unfiltered, {"EF", "DE"});
  gurka::assert::raw::expect_path(result_unfiltered, {"EF", "DE", "CD"});

  const std::string& request_filtered =
      (boost::format(
           R"({"locations":[{"lat":%s,"lon":%s,"search_filter":{"exclude_toll":true}},{"lat":%s,"lon":%s}],"costing":"auto"})") %
       std::to_string(map.nodes.at(from).lat()) % std::to_string(map.nodes.at(from).lng()) %
       std::to_string(map.nodes.at(to).lat()) % std::to_string(map.nodes.at(to).lng()))
          .str();
  auto result_filtered = gurka::do_action(valhalla::Options::route, map, request_filtered);
  gurka::assert::osrm::expect_steps(result_filtered, {"AD", "CD"});
  gurka::assert::raw::expect_path(result_filtered, {"AD", "CD"});
}

/*************************************************************/
namespace {
inline void SetLiveSpeed(TrafficSpeed* live_speed, uint64_t speed) {
  live_speed->breakpoint1 = 255;
  live_speed->overall_encoded_speed = speed >> 1;
  live_speed->encoded_speed1 = speed >> 1;
}

void close_dir_edge(GraphReader& reader,
                    TrafficTile& tile,
                    uint32_t index,
                    TrafficSpeed* current,
                    const std::string& edge_name,
                    const std::string& end_node,
                    const gurka::map& closure_map) {
  GraphId tile_id(tile.header->tile_id);
  auto edge = std::get<0>(gurka::findEdge(reader, closure_map.nodes, edge_name, end_node));
  if (edge.Tile_Base() == tile_id && edge.id() == index) {
    SetLiveSpeed(current, 0);
  }
}

void close_bidir_edge(GraphReader& reader,
                      TrafficTile& tile,
                      uint32_t index,
                      TrafficSpeed* current,
                      const std::string& edge_name,
                      const gurka::map& closure_map) {
  GraphId tile_id(tile.header->tile_id);
  std::string start_node(1, edge_name.front());
  std::string end_node(1, edge_name.back());

  close_dir_edge(reader, tile, index, current, edge_name, start_node, closure_map);
  close_dir_edge(reader, tile, index, current, edge_name, end_node, closure_map);
}

} // namespace

class ExcludeClosuresOnWaypoints : public ::testing::TestWithParam<std::string> {
protected:
  static gurka::map closure_map;
  static int const default_speed;
  static std::string const tile_dir;
  static std::shared_ptr<GraphReader> reader;

  static void SetUpTestSuite() {
    const std::string ascii_map = R"(
              F----G
              |    |
    A--1--B---C----D--2--E--J--3--K
             /
            I
            |
            H

   L4----5-M-6-N
  )";

    const std::string speed_str = std::to_string(default_speed);
    const gurka::ways ways = {{"AB", {{"highway", "primary"}, {"maxspeed", speed_str}}},
                              {"BC", {{"highway", "primary"}, {"maxspeed", speed_str}}},
                              {"CD", {{"highway", "primary"}, {"maxspeed", speed_str}}},
                              {"DE", {{"highway", "primary"}, {"maxspeed", speed_str}}},
                              {"EJ", {{"highway", "primary"}, {"maxspeed", speed_str}}},
                              {"JK", {{"highway", "primary"}, {"maxspeed", speed_str}}},
                              {"CFGD", {{"highway", "residential"}, {"maxspeed", "10"}}},
                              {"HIC", {{"highway", "primary"}, {"maxspeed", speed_str}}},
                              {"LM", {{"highway", "primary"}, {"maxspeed", speed_str}}},
                              {"MN", {{"highway", "primary"}, {"maxspeed", speed_str}}}};

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100, {.05f, .2f});
    closure_map = gurka::buildtiles(layout, ways, {}, {}, tile_dir);

    closure_map.config.put("mjolnir.traffic_extract", tile_dir + "/traffic.tar");
    test::build_live_traffic_data(closure_map.config);

    reader = test::make_clean_graphreader(closure_map.config.get_child("mjolnir"));
  }

  void set_default_speed_on_all_edges() {
    test::customize_live_traffic_data(closure_map.config,
                                      [](GraphReader& reader, TrafficTile& tile, uint32_t index,
                                         TrafficSpeed* current) -> void {
                                        (void)reader, (void)tile, (void)index;
                                        SetLiveSpeed(current, default_speed);
                                      });
  }

  virtual void SetUp() {
    // Enable extend search, since search in 1 directions exhausts
    // quickly with small gurka map and high cost closed edges
    closure_map.config.put("thor.extended_search", true);
    set_default_speed_on_all_edges();
  }

  virtual void TearDown() {
    closure_map.config.put("thor.extended_search", false);
    set_default_speed_on_all_edges();
  }
};

gurka::map ExcludeClosuresOnWaypoints::closure_map = {};
const int ExcludeClosuresOnWaypoints::default_speed = 36;
const std::string ExcludeClosuresOnWaypoints::tile_dir =
    "test/data/traffic_exclude_closures_on_waypoints";
std::shared_ptr<GraphReader> ExcludeClosuresOnWaypoints::reader;

/*
 *  Tests search_filter.exclude_closures at departure
 */
TEST_P(ExcludeClosuresOnWaypoints, ExcludeClosuresAtDeparture) {
  std::string costing = GetParam();
  std::string date_type = "3"; // invariant time
  std::string costing_speed_type =
      (boost::format("/costing_options/%s/speed_types/0") % costing).str();

  // None of the edges are closed
  {
    auto result = gurka::do_action(valhalla::Options::route, closure_map, {"1", "2"}, costing,
                                   {{"/date_time/type", date_type},
                                    {"/date_time/value", "current"},
                                    {costing_speed_type, "current"}},
                                   reader);
    gurka::assert::osrm::expect_steps(result, {"AB"});
    gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DE"});
  }
  {
    LiveTrafficCustomize close_edge = [](GraphReader& reader, TrafficTile& tile, uint32_t index,
                                         TrafficSpeed* current) -> void {
      close_bidir_edge(reader, tile, index, current, "AB", closure_map);
      close_bidir_edge(reader, tile, index, current, "BC", closure_map);
    };
    test::customize_live_traffic_data(closure_map.config, close_edge);

    auto result = gurka::do_action(valhalla::Options::route, closure_map, {"1", "2"}, costing,
                                   {{"/date_time/type", date_type},
                                    {"/date_time/value", "current"},
                                    {costing_speed_type, "current"}},
                                   reader);
    gurka::assert::osrm::expect_steps(result, {"HIC", "CD"});
    gurka::assert::raw::expect_path(result, {"HIC", "CD", "DE"});

    // Specify search filter to disable exclude_closures at departure
    const std::string& req_disable_exclude_closures =
        (boost::format(
             R"({"locations":[{"lat":%s,"lon":%s,"search_filter":{"exclude_closures":false}},{"lat":%s,"lon":%s}],"costing":"%s", "costing_options": {"%s": {"speed_types":["freeflow","constrained","predicted","current"]}}, "date_time":{"type":"%s", "value": "current"}})") %
         std::to_string(closure_map.nodes.at("1").lat()) %
         std::to_string(closure_map.nodes.at("1").lng()) %
         std::to_string(closure_map.nodes.at("2").lat()) %
         std::to_string(closure_map.nodes.at("2").lng()) % costing % costing % date_type)
            .str();
    result =
        gurka::do_action(valhalla::Options::route, closure_map, req_disable_exclude_closures, reader);
    gurka::assert::osrm::expect_steps(result, {"AB"});
    gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DE"});
  }
}

/*
 *  Tests search_filter.exclude_closures at destination
 */
TEST_P(ExcludeClosuresOnWaypoints, ExcludeClosuresAtDestination) {
  std::string costing = GetParam();
  std::string date_type = "3"; // invariant time
  std::string costing_speed_type =
      (boost::format("/costing_options/%s/speed_types/0") % costing).str();

  // None of the edges are closed
  {
    auto result = gurka::do_action(valhalla::Options::route, closure_map, {"1", "2"}, costing,
                                   {{"/date_time/type", date_type},
                                    {"/date_time/value", "current"},
                                    {costing_speed_type, "current"}},
                                   reader);
    gurka::assert::osrm::expect_steps(result, {"AB"});
    gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DE"});
  }
  {
    LiveTrafficCustomize close_edge = [](GraphReader& reader, TrafficTile& tile, uint32_t index,
                                         TrafficSpeed* current) -> void {
      close_bidir_edge(reader, tile, index, current, "DE", closure_map);
      close_bidir_edge(reader, tile, index, current, "CD", closure_map);
    };
    test::customize_live_traffic_data(closure_map.config, close_edge);

    auto result = gurka::do_action(valhalla::Options::route, closure_map, {"1", "2"}, costing,
                                   {{"/date_time/type", date_type},
                                    {"/date_time/value", "current"},
                                    {costing_speed_type, "current"}},
                                   reader);
    // TODO: See https://github.com/valhalla/valhalla/issues/2709 on why its
    // taking CFGD instead of DE
    gurka::assert::osrm::expect_steps(result, {"AB", "CFGD"});
    gurka::assert::raw::expect_path(result, {"AB", "BC", "CFGD"});

    // Specify search filter to disable exclude_closures at destination
    const std::string& req_disable_exclude_closures =
        (boost::format(
             R"({"locations":[{"lat":%s,"lon":%s},{"lat":%s,"lon":%s,"search_filter":{"exclude_closures":false}}],"costing":"%s", "costing_options": {"%s": {"speed_types":["freeflow","constrained","predicted","current"]}}, "date_time":{"type":"%s", "value": "current"}})") %
         std::to_string(closure_map.nodes.at("1").lat()) %
         std::to_string(closure_map.nodes.at("1").lng()) %
         std::to_string(closure_map.nodes.at("2").lat()) %
         std::to_string(closure_map.nodes.at("2").lng()) % costing % costing % date_type)
            .str();
    result =
        gurka::do_action(valhalla::Options::route, closure_map, req_disable_exclude_closures, reader);
    gurka::assert::osrm::expect_steps(result, {"AB", "CFGD", "DE"});
    gurka::assert::raw::expect_path(result, {"AB", "BC", "CFGD", "DE"});
  }
}

/*
 *  Tests search_filter.exclude_closures at midway location
 */
TEST_P(ExcludeClosuresOnWaypoints, ExcludeClosuresAtMidway) {
  std::string costing = GetParam();
  std::string date_type = "3"; // invariant time
  std::string costing_speed_type =
      (boost::format("/costing_options/%s/speed_types/0") % costing).str();
  // None of the edges are closed. Route has multiple waypoints
  {
    auto result = gurka::do_action(valhalla::Options::route, closure_map, {"1", "D", "3"}, costing,
                                   {{"/date_time/type", date_type},
                                    {"/date_time/value", "current"},
                                    {costing_speed_type, "current"}},
                                   reader);
    gurka::assert::osrm::expect_steps(result, {"AB", "DE"});
    gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DE", "EJ", "JK"});
  }
  {
    LiveTrafficCustomize close_edge = [](GraphReader& reader, TrafficTile& tile, uint32_t index,
                                         TrafficSpeed* current) -> void {
      close_bidir_edge(reader, tile, index, current, "CD", closure_map);
      close_bidir_edge(reader, tile, index, current, "DE", closure_map);
    };
    test::customize_live_traffic_data(closure_map.config, close_edge);

    // The route from 2->3 fails since there's no suitable edge from 2->3
    EXPECT_THROW((gurka::do_action(valhalla::Options::route, closure_map, {"1", "2", "3"}, costing,
                                   {{"/date_time/type", date_type},
                                    {"/date_time/value", "current"},
                                    {costing_speed_type, "current"}},
                                   reader)),
                 valhalla_exception_t);
    // Specify search filter to disable exclude_closures at midway waypoint
    const std::string& req_disable_exclude_closures =
        (boost::format(
             R"({"locations":[{"lat":%s,"lon":%s},{"lat":%s,"lon":%s,"search_filter":{"exclude_closures":false}},{"lat":%s,"lon":%s}],"costing":"%s", "costing_options": {"%s": {"speed_types":["freeflow","constrained","predicted","current"]}}, "date_time":{"type":"%s", "value": "current"}})") %
         std::to_string(closure_map.nodes.at("1").lat()) %
         std::to_string(closure_map.nodes.at("1").lng()) %
         std::to_string(closure_map.nodes.at("D").lat()) %
         std::to_string(closure_map.nodes.at("D").lng()) %
         std::to_string(closure_map.nodes.at("3").lat()) %
         std::to_string(closure_map.nodes.at("3").lng()) % costing % costing % date_type)
            .str();
    auto result =
        gurka::do_action(valhalla::Options::route, closure_map, req_disable_exclude_closures, reader);
    gurka::assert::osrm::expect_steps(result, {"AB", "CFGD", "DE"});
    gurka::assert::raw::expect_path(result, {"AB", "BC", "CFGD", "DE", "EJ", "JK"});
  }
}

/*
 *  Tests costing_options.ignore_closures:true overrides
 *  search_filter.exclude_closures:true (when exclude_closures:true is not
 *  explicitly set)
 */
TEST_P(ExcludeClosuresOnWaypoints, IgnoreClosuresOverridesExcludeClosures) {
  std::string costing = GetParam();
  std::string date_type = "3"; // invariant time
  std::string costing_speed_type =
      (boost::format("/costing_options/%s/speed_types/0") % costing).str();

  // None of the edges are closed
  {
    auto result = gurka::do_action(valhalla::Options::route, closure_map, {"1", "2"}, costing,
                                   {{"/date_time/type", date_type},
                                    {"/date_time/value", "current"},
                                    {costing_speed_type, "current"}},
                                   reader);
    gurka::assert::osrm::expect_steps(result, {"AB"});
    gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DE"});
  }

  // CD edge is closed in both directions. Route should avoid CD with
  // exclude_closures set to true (default)
  {
    LiveTrafficCustomize close_edge = [](GraphReader& reader, TrafficTile& tile, uint32_t index,
                                         TrafficSpeed* current) -> void {
      close_bidir_edge(reader, tile, index, current, "CD", closure_map);
      close_bidir_edge(reader, tile, index, current, "DE", closure_map);
    };
    test::customize_live_traffic_data(closure_map.config, close_edge);

    auto result = gurka::do_action(valhalla::Options::route, closure_map, {"1", "2"}, costing,
                                   {{"/date_time/type", date_type},
                                    {"/date_time/value", "current"},
                                    {costing_speed_type, "current"}},
                                   reader);
    gurka::assert::osrm::expect_steps(result, {"AB", "CFGD"});
    gurka::assert::raw::expect_path(result, {"AB", "BC", "CFGD"});

    // set ignore_closures in costing, while leaving exclude_closures unset
    // (which defaults to true). ignore_closures should override
    // exclude_closures
    const std::string& req_disable_exclude_closures =
        (boost::format(
             R"({"locations":[{"lat":%s,"lon":%s},{"lat":%s,"lon":%s}],"costing":"%s", "costing_options": {"%s": {"speed_types":["freeflow","constrained","predicted","current"], "ignore_closures": true}}, "date_time":{"type":"%s", "value": "current"}})") %
         std::to_string(closure_map.nodes.at("1").lat()) %
         std::to_string(closure_map.nodes.at("1").lng()) %
         std::to_string(closure_map.nodes.at("2").lat()) %
         std::to_string(closure_map.nodes.at("2").lng()) % costing % costing % date_type)
            .str();
    result =
        gurka::do_action(valhalla::Options::route, closure_map, req_disable_exclude_closures, reader);
    gurka::assert::osrm::expect_steps(result, {"AB"});
    gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DE"});
  }
}

TEST_P(ExcludeClosuresOnWaypoints, AvoidIntermediateClosures) {
  std::string costing = GetParam();
  std::string date_type = "3"; // invariant time
  std::string costing_speed_type =
      (boost::format("/costing_options/%s/speed_types/0") % costing).str();

  // None of the edges are closed
  {
    auto result = gurka::do_action(valhalla::Options::route, closure_map, {"1", "3"}, costing,
                                   {{"/date_time/type", date_type},
                                    {"/date_time/value", "current"},
                                    {costing_speed_type, "current"}},
                                   reader);
    gurka::assert::osrm::expect_steps(result, {"AB"});
    gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DE", "EJ", "JK"});
  }

  // Close edges at departure, intermediate & destination edges
  {
    LiveTrafficCustomize close_edge = [](GraphReader& reader, TrafficTile& tile, uint32_t index,
                                         TrafficSpeed* current) -> void {
      close_bidir_edge(reader, tile, index, current, "AB", closure_map);
      close_bidir_edge(reader, tile, index, current, "CD", closure_map);
      close_bidir_edge(reader, tile, index, current, "JK", closure_map);
    };
    test::customize_live_traffic_data(closure_map.config, close_edge);

    auto result = gurka::do_action(valhalla::Options::route, closure_map, {"1", "3"}, costing,
                                   {{"/date_time/type", date_type},
                                    {"/date_time/value", "current"},
                                    {costing_speed_type, "current"}},
                                   reader);
    gurka::assert::osrm::expect_steps(result, {"BC", "CFGD", "DE"});
    gurka::assert::raw::expect_path(result, {"BC", "CFGD", "DE", "EJ"});

    const std::string& req_disable_exclude_closures =
        (boost::format(
             R"({"locations":[{"lat":%s,"lon":%s,"search_filter":{"exclude_closures":false}},{"lat":%s,"lon":%s,"search_filter":{"exclude_closures":false}}],"costing":"%s", "costing_options": {"%s": {"speed_types":["freeflow","constrained","predicted","current"]}}, "date_time":{"type":"%s", "value": "current"}})") %
         std::to_string(closure_map.nodes.at("1").lat()) %
         std::to_string(closure_map.nodes.at("1").lng()) %
         std::to_string(closure_map.nodes.at("3").lat()) %
         std::to_string(closure_map.nodes.at("3").lng()) % costing % costing % date_type)
            .str();
    result =
        gurka::do_action(valhalla::Options::route, closure_map, req_disable_exclude_closures, reader);
    gurka::assert::osrm::expect_steps(result, {"AB", "CFGD", "DE"});
    gurka::assert::raw::expect_path(result, {"AB", "BC", "CFGD", "DE", "EJ", "JK"});
  }
}

TEST_P(ExcludeClosuresOnWaypoints, TrivialRouteSameEdge) {
  std::string costing = GetParam();
  std::string date_type = "3"; // invariant time
  std::string costing_speed_type =
      (boost::format("/costing_options/%s/speed_types/0") % costing).str();

  // Route starts & ends on a single edge
  {
    auto result = gurka::do_action(valhalla::Options::route, closure_map, {"4", "5"}, costing,
                                   {{"/date_time/type", date_type},
                                    {"/date_time/value", "current"},
                                    {costing_speed_type, "current"}},
                                   reader);
    gurka::assert::osrm::expect_steps(result, {"LM"});
    gurka::assert::raw::expect_path(result, {"LM"});
  }

  // Close LM edge
  {
    LiveTrafficCustomize close_edge = [](GraphReader& reader, TrafficTile& tile, uint32_t index,
                                         TrafficSpeed* current) -> void {
      close_bidir_edge(reader, tile, index, current, "LM", closure_map);
    };
    test::customize_live_traffic_data(closure_map.config, close_edge);

    // No route since the edge is closed
    EXPECT_THROW(gurka::do_action(valhalla::Options::route, closure_map, {"4", "5"}, costing,
                                  {{"/date_time/type", date_type},
                                   {"/date_time/value", "current"},
                                   {costing_speed_type, "current"}},
                                  reader),
                 valhalla_exception_t);

    const std::string& req_disable_exclude_closures =
        (boost::format(
             R"({"locations":[{"lat":%s,"lon":%s,"search_filter":{"exclude_closures":false}},{"lat":%s,"lon":%s,"search_filter":{"exclude_closures":false}}],"costing":"%s", "costing_options": {"%s": {"speed_types":["freeflow","constrained","predicted","current"]}}, "date_time":{"type":"%s", "value": "current"}})") %
         std::to_string(closure_map.nodes.at("4").lat()) %
         std::to_string(closure_map.nodes.at("4").lng()) %
         std::to_string(closure_map.nodes.at("5").lat()) %
         std::to_string(closure_map.nodes.at("5").lng()) % costing % costing % date_type)
            .str();
    auto result =
        gurka::do_action(valhalla::Options::route, closure_map, req_disable_exclude_closures, reader);
    gurka::assert::osrm::expect_steps(result, {"LM"});
    gurka::assert::raw::expect_path(result, {"LM"});
  }
}

TEST_P(ExcludeClosuresOnWaypoints, DISABLED_TrivialRouteAdjacentEdges) {
  // Test disabled since trivial case use timedep-fwd, which currently
  // does not work with closures at destination
  std::string costing = GetParam();
  std::string date_type = "3"; // invariant time
  std::string costing_speed_type =
      (boost::format("/costing_options/%s/speed_types/0") % costing).str();

  // Start and end locations are on adjacent edges. This will use timedep_fwd even with date_type 3
  {
    auto result = gurka::do_action(valhalla::Options::route, closure_map, {"4", "6"}, costing,
                                   {{"/date_time/type", date_type},
                                    {"/date_time/value", "current"},
                                    {costing_speed_type, "current"}},
                                   reader);
    gurka::assert::osrm::expect_steps(result, {"LM"});
    gurka::assert::raw::expect_path(result, {"LM", "MN"});
  }

  // Close adjacent edges LM & MN
  {
    LiveTrafficCustomize close_edge = [](GraphReader& reader, TrafficTile& tile, uint32_t index,
                                         TrafficSpeed* current) -> void {
      close_bidir_edge(reader, tile, index, current, "LM", closure_map);
      close_bidir_edge(reader, tile, index, current, "MN", closure_map);
    };
    test::customize_live_traffic_data(closure_map.config, close_edge);

    // Snaps to the nearby edge HIC, since candidate edges are closed
    auto res = gurka::do_action(valhalla::Options::route, closure_map, {"4", "6"}, costing,
                                {{"/date_time/type", date_type},
                                 {"/date_time/value", "current"},
                                 {costing_speed_type, "current"}},
                                reader);
    gurka::assert::osrm::expect_steps(res, {"HIC"});
    gurka::assert::raw::expect_path(res, {"HIC", "HIC"});

    const std::string& req_disable_exclude_closures =
        (boost::format(
             R"({"locations":[{"lat":%s,"lon":%s,"search_filter":{"exclude_closures":false}},{"lat":%s,"lon":%s,"search_filter":{"exclude_closures":false}}],"costing":"%s", "costing_options": {"%s": {"speed_types":["freeflow","constrained","predicted","current"]}}, "date_time":{"type":"%s", "value": "current"}})") %
         std::to_string(closure_map.nodes.at("4").lat()) %
         std::to_string(closure_map.nodes.at("4").lng()) %
         std::to_string(closure_map.nodes.at("6").lat()) %
         std::to_string(closure_map.nodes.at("6").lng()) % costing % costing % date_type)
            .str();
    // TODO: Enable once timedep-fwd handles clsures at destination edges
    auto result =
        gurka::do_action(valhalla::Options::route, closure_map, req_disable_exclude_closures, reader);
    gurka::assert::osrm::expect_steps(result, {"LM"});
    gurka::assert::raw::expect_path(result, {"LM", "MN"});
  }
}

/*
 *  Tests costing_options.ignore_closures and search_filter.exclude_closures
 *  cannot both be specified
 */
TEST_P(ExcludeClosuresOnWaypoints, ConflictingOptions) {
  std::string costing = GetParam();
  std::string date_type = "3"; // invariant time

  // ignore_closures:true & exclude_closures:true on all locations
  {
    const std::string& bad_request =
        (boost::format(
             R"({"locations":[{"lat":%s,"lon":%s,"search_filter":{"exclude_closures":true}},{"lat":%s,"lon":%s,"search_filter":{"exclude_closures":true}}],"costing":"%s", "costing_options": {"%s": {"speed_types":["freeflow","constrained","predicted","current"], "ignore_closures": true}}, "date_time":{"type":"%s", "value": "current"}})") %
         std::to_string(closure_map.nodes.at("A").lat()) %
         std::to_string(closure_map.nodes.at("A").lng()) %
         std::to_string(closure_map.nodes.at("E").lat()) %
         std::to_string(closure_map.nodes.at("E").lng()) % costing % costing % date_type)
            .str();

    EXPECT_THROW(
        {
          try {
            gurka::do_action(valhalla::Options::route, closure_map, bad_request, reader);
          } catch (const valhalla_exception_t& e) {
            EXPECT_EQ(e.code, 143);
            EXPECT_STREQ(
                "ignore_closures in costing and exclude_closures in search_filter cannot both be specified",
                e.what());
            throw;
          }
        },
        valhalla_exception_t);
  }
  // ignore_closures:true & exclude_closures:false on all locations
  {
    const std::string& bad_request =
        (boost::format(
             R"({"locations":[{"lat":%s,"lon":%s,"search_filter":{"exclude_closures":false}},{"lat":%s,"lon":%s,"search_filter":{"exclude_closures":false}}],"costing":"%s", "costing_options": {"%s": {"speed_types":["freeflow","constrained","predicted","current"], "ignore_closures": true}}, "date_time":{"type":"%s", "value": "current"}})") %
         std::to_string(closure_map.nodes.at("A").lat()) %
         std::to_string(closure_map.nodes.at("A").lng()) %
         std::to_string(closure_map.nodes.at("E").lat()) %
         std::to_string(closure_map.nodes.at("E").lng()) % costing % costing % date_type)
            .str();

    EXPECT_THROW(
        {
          try {
            gurka::do_action(valhalla::Options::route, closure_map, bad_request, reader);
          } catch (const valhalla_exception_t& e) {
            EXPECT_EQ(e.code, 143);
            EXPECT_STREQ(
                "ignore_closures in costing and exclude_closures in search_filter cannot both be specified",
                e.what());
            throw;
          }
        },
        valhalla_exception_t);
  }
  // ignore_closures:false & exclude_closures:true on all locations
  {
    const std::string& bad_request =
        (boost::format(
             R"({"locations":[{"lat":%s,"lon":%s,"search_filter":{"exclude_closures":true}},{"lat":%s,"lon":%s,"search_filter":{"exclude_closures":true}}],"costing":"%s", "costing_options": {"%s": {"speed_types":["freeflow","constrained","predicted","current"], "ignore_closures":false}}, "date_time":{"type":"%s", "value": "current"}})") %
         std::to_string(closure_map.nodes.at("A").lat()) %
         std::to_string(closure_map.nodes.at("A").lng()) %
         std::to_string(closure_map.nodes.at("E").lat()) %
         std::to_string(closure_map.nodes.at("E").lng()) % costing % costing % date_type)
            .str();

    EXPECT_THROW(
        {
          try {
            gurka::do_action(valhalla::Options::route, closure_map, bad_request, reader);
          } catch (const valhalla_exception_t& e) {
            EXPECT_EQ(e.code, 143);
            EXPECT_STREQ(
                "ignore_closures in costing and exclude_closures in search_filter cannot both be specified",
                e.what());
            throw;
          }
        },
        valhalla_exception_t);
  }
  // ignore_closures:false & exclude_closures:false on all locations
  {
    const std::string& bad_request =
        (boost::format(
             R"({"locations":[{"lat":%s,"lon":%s,"search_filter":{"exclude_closures":false}},{"lat":%s,"lon":%s,"search_filter":{"exclude_closures":false}}],"costing":"%s", "costing_options": {"%s": {"speed_types":["freeflow","constrained","predicted","current"], "ignore_closures":false}}, "date_time":{"type":"%s", "value": "current"}})") %
         std::to_string(closure_map.nodes.at("A").lat()) %
         std::to_string(closure_map.nodes.at("A").lng()) %
         std::to_string(closure_map.nodes.at("E").lat()) %
         std::to_string(closure_map.nodes.at("E").lng()) % costing % costing % date_type)
            .str();

    EXPECT_THROW(
        {
          try {
            gurka::do_action(valhalla::Options::route, closure_map, bad_request, reader);
          } catch (const valhalla_exception_t& e) {
            EXPECT_EQ(e.code, 143);
            EXPECT_STREQ(
                "ignore_closures in costing and exclude_closures in search_filter cannot both be specified",
                e.what());
            throw;
          }
        },
        valhalla_exception_t);
  }
  // ignore_closures:true & exclude_closures:false on one location
  {
    const std::string& bad_request =
        (boost::format(
             R"({"locations":[{"lat":%s,"lon":%s},{"lat":%s,"lon":%s,"search_filter":{"exclude_closures":false}},{"lat":%s,"lon":%s}],"costing":"%s", "costing_options": {"%s": {"speed_types":["freeflow","constrained","predicted","current"], "ignore_closures":false}}, "date_time":{"type":"%s", "value": "current"}})") %
         std::to_string(closure_map.nodes.at("A").lat()) %
         std::to_string(closure_map.nodes.at("A").lng()) %
         std::to_string(closure_map.nodes.at("C").lat()) %
         std::to_string(closure_map.nodes.at("C").lng()) %
         std::to_string(closure_map.nodes.at("E").lat()) %
         std::to_string(closure_map.nodes.at("E").lng()) % costing % costing % date_type)
            .str();

    EXPECT_THROW(
        {
          try {
            gurka::do_action(valhalla::Options::route, closure_map, bad_request, reader);
          } catch (const valhalla_exception_t& e) {
            EXPECT_EQ(e.code, 143);
            EXPECT_STREQ(
                "ignore_closures in costing and exclude_closures in search_filter cannot both be specified",
                e.what());
            throw;
          }
        },
        valhalla_exception_t);
  }
}

std::vector<std::string> buildParams() {
  // Return the different costings we want to test closures against
  return {
      "auto", "motorcycle", "motor_scooter", "bus", "truck", "taxi",
  };
}

INSTANTIATE_TEST_SUITE_P(SearchFilter,
                         ExcludeClosuresOnWaypoints,
                         ::testing::ValuesIn(buildParams()));

class ClosuresWithRestrictions : public ::testing::TestWithParam<std::string> {
protected:
  static gurka::map closure_map;
  static int const default_speed;
  static std::string const tile_dir;
  static std::shared_ptr<GraphReader> reader;

  static void SetUpTestSuite() {
    const std::string ascii_map = R"(
    A---------B
    |         |
    C         D
    |         |
    E----F----G
         |
         H
  )";

    const std::string speed_str = std::to_string(default_speed);
    const gurka::ways ways = {{"AB", {{"highway", "primary"}, {"maxspeed", speed_str}}},
                              {"AC", {{"highway", "primary"}, {"maxspeed", speed_str}}},
                              {"CE", {{"highway", "primary"}, {"maxspeed", speed_str}}},
                              {"BD", {{"highway", "primary"}, {"maxspeed", speed_str}}},
                              {"DG", {{"highway", "primary"}, {"maxspeed", speed_str}}},
                              {"EF", {{"highway", "primary"}, {"maxspeed", speed_str}}},
                              {"FG", {{"highway", "primary"}, {"maxspeed", speed_str}}},
                              {"FH", {{"highway", "primary"}, {"maxspeed", speed_str}}}};

    const gurka::relations relations = {{{
                                             {gurka::way_member, "FH", "from"},
                                             {gurka::way_member, "EF", "to"},
                                             {gurka::node_member, "F", "via"},
                                         },
                                         {
                                             {"type", "restriction"},
                                             {"restriction", "no_left_turn"},
                                         }}};

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, 20, {.05f, .2f});
    closure_map = gurka::buildtiles(layout, ways, {}, relations, tile_dir);

    closure_map.config.put("mjolnir.traffic_extract", tile_dir + "/traffic.tar");
    test::build_live_traffic_data(closure_map.config);
    closure_map.config.put("thor.extended_search", true);

    reader = test::make_clean_graphreader(closure_map.config.get_child("mjolnir"));
  }

  void set_default_speed_on_all_edges() {
    test::customize_live_traffic_data(closure_map.config,
                                      [](GraphReader& reader, TrafficTile& tile, uint32_t index,
                                         TrafficSpeed* current) -> void {
                                        (void)reader, (void)tile, (void)index;
                                        SetLiveSpeed(current, default_speed);
                                      });
  }

  virtual void SetUp() {
    set_default_speed_on_all_edges();
  }

  virtual void TearDown() {
    set_default_speed_on_all_edges();
  }
};

gurka::map ClosuresWithRestrictions::closure_map = {};
const int ClosuresWithRestrictions::default_speed = 36;
const std::string ClosuresWithRestrictions::tile_dir = "test/data/traffic_exclude_closures";
std::shared_ptr<GraphReader> ClosuresWithRestrictions::reader;

TEST_P(ClosuresWithRestrictions, AvoidClosureWithRestriction) {
  std::string costing = GetParam();
  std::string date_type = "3";
  std::string costing_speed_type =
      (boost::format("/costing_options/%s/speed_types/0") % costing).str();
  {
    LiveTrafficCustomize close_edge = [](GraphReader& reader, TrafficTile& tile, uint32_t index,
                                         TrafficSpeed* current) -> void {
      close_bidir_edge(reader, tile, index, current, "EF", closure_map);
      close_bidir_edge(reader, tile, index, current, "FG", closure_map);
      close_bidir_edge(reader, tile, index, current, "FH", closure_map);
    };
    test::customize_live_traffic_data(closure_map.config, close_edge);

    const std::string& req =
        (boost::format(
             R"({"locations":[{"lat":%s,"lon":%s,"search_filter":{"exclude_closures":false}},{"lat":%s,"lon":%s}],"costing":"%s", "costing_options": {"%s": {"speed_types":["freeflow","constrained","predicted","current"]}}, "date_time":{"type":"%s", "value": "current"}})") %
         std::to_string(closure_map.nodes.at("H").lat()) %
         std::to_string(closure_map.nodes.at("H").lng()) %
         std::to_string(closure_map.nodes.at("C").lat()) %
         std::to_string(closure_map.nodes.at("C").lng()) % costing % costing % date_type)
            .str();
    auto result = gurka::do_action(valhalla::Options::route, closure_map, req, reader);
    gurka::assert::osrm::expect_steps(result, {"FH", "FG", "DG", "AB", "AC"});
    gurka::assert::raw::expect_path(result, {"FH", "FG", "DG", "BD", "AB", "AC"});

    // For G->C since closed edges have high cost, the route will take the longer loop (GDBAC).
    // To prevent that and force it to take closed edges (since we want to test the left-turn
    // restriction created above does not take effect for going straight), we close AB
    LiveTrafficCustomize close_edge_AB = [](GraphReader& reader, TrafficTile& tile, uint32_t index,
                                            TrafficSpeed* current) -> void {
      close_bidir_edge(reader, tile, index, current, "AB", closure_map);
    };
    test::customize_live_traffic_data(closure_map.config, close_edge_AB);

    const std::string& req_disable_exclude_closures =
        (boost::format(
             R"({"locations":[{"lat":%s,"lon":%s,"search_filter":{"exclude_closures":false}},{"lat":%s,"lon":%s}],"costing":"%s", "costing_options": {"%s": {"speed_types":["freeflow","constrained","predicted","current"]}}, "date_time":{"type":"%s", "value": "current"}})") %
         std::to_string(closure_map.nodes.at("G").lat()) %
         std::to_string(closure_map.nodes.at("G").lng()) %
         std::to_string(closure_map.nodes.at("C").lat()) %
         std::to_string(closure_map.nodes.at("C").lng()) % costing % costing % date_type)
            .str();
    result =
        gurka::do_action(valhalla::Options::route, closure_map, req_disable_exclude_closures, reader);
    gurka::assert::osrm::expect_steps(result, {"FG", "CE"});
    gurka::assert::raw::expect_path(result, {"FG", "EF", "CE"});
  }
}

INSTANTIATE_TEST_SUITE_P(SearchFilter, ClosuresWithRestrictions, ::testing::ValuesIn(buildParams()));

class ClosuresWithTimedepRoutes : public ::testing::TestWithParam<std::string> {
protected:
  static gurka::map closure_map;
  static int const default_speed;
  static std::string const tile_dir;
  static std::shared_ptr<GraphReader> reader;

  static void SetUpTestSuite() {
    const std::string ascii_map = R"(

             I--J
             |  |
    A--B--C--D--E--F--G--H

    )";

    const std::string speed_str = std::to_string(default_speed);
    const gurka::ways ways = {{"AB", {{"highway", "primary"}, {"maxspeed", speed_str}}},
                              {"BC", {{"highway", "primary"}, {"maxspeed", speed_str}}},
                              {"CD", {{"highway", "primary"}, {"maxspeed", speed_str}}},
                              {"DE", {{"highway", "primary"}, {"maxspeed", speed_str}}},
                              {"EF", {{"highway", "primary"}, {"maxspeed", speed_str}}},
                              {"FG", {{"highway", "primary"}, {"maxspeed", speed_str}}},
                              {"GH", {{"highway", "primary"}, {"maxspeed", speed_str}}},
                              {"DIJE", {{"highway", "primary"}, {"maxspeed", speed_str}}}};

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, 20, {.05f, .2f});
    closure_map = gurka::buildtiles(layout, ways, {}, {}, tile_dir);

    closure_map.config.put("mjolnir.traffic_extract", tile_dir + "/traffic.tar");
    test::build_live_traffic_data(closure_map.config);
    reader = test::make_clean_graphreader(closure_map.config.get_child("mjolnir"));
  }

  void set_default_speed_on_all_edges() {
    test::customize_live_traffic_data(closure_map.config,
                                      [](GraphReader& reader, TrafficTile& tile, uint32_t index,
                                         TrafficSpeed* current) -> void {
                                        (void)reader, (void)tile, (void)index;
                                        SetLiveSpeed(current, default_speed);
                                      });
  }

  virtual void SetUp() {
    set_default_speed_on_all_edges();
  }

  virtual void TearDown() {
    set_default_speed_on_all_edges();
  }
};

gurka::map ClosuresWithTimedepRoutes::closure_map = {};
const int ClosuresWithTimedepRoutes::default_speed = 36;
const std::string ClosuresWithTimedepRoutes::tile_dir = "test/data/traffic_exclude_closures";
std::shared_ptr<GraphReader> ClosuresWithTimedepRoutes::reader;

TEST_P(ClosuresWithTimedepRoutes, IgnoreClosureWithTimedepForward) {
  std::string costing = GetParam();
  // use current departure time which makes use of timedep fwd A*
  std::string date_type = "0";
  std::string costing_speed_type =
      (boost::format("/costing_options/%s/speed_types/0") % costing).str();

  {
    auto result = gurka::do_action(valhalla::Options::route, closure_map, {"A", "G"}, costing,
                                   {{"/date_time/type", date_type},
                                    {"/date_time/value", "current"},
                                    {costing_speed_type, "current"}},
                                   reader);
    gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DE", "EF", "FG"});
  }
  {
    LiveTrafficCustomize close_edge = [](GraphReader& reader, TrafficTile& tile, uint32_t index,
                                         TrafficSpeed* current) -> void {
      close_bidir_edge(reader, tile, index, current, "AB", closure_map);
      close_bidir_edge(reader, tile, index, current, "BC", closure_map);
    };
    test::customize_live_traffic_data(closure_map.config, close_edge);

    const std::string& req =
        (boost::format(
             R"({"locations":[{"lat":%s,"lon":%s,"search_filter":{"exclude_closures":false}},{"lat":%s,"lon":%s}],"costing":"%s", "costing_options": {"%s": {"speed_types":["freeflow","constrained","predicted","current"]}}, "date_time":{"type":"%s", "value": "current"}})") %
         std::to_string(closure_map.nodes.at("A").lat()) %
         std::to_string(closure_map.nodes.at("A").lng()) %
         std::to_string(closure_map.nodes.at("G").lat()) %
         std::to_string(closure_map.nodes.at("G").lng()) % costing % costing % date_type)
            .str();
    auto result = gurka::do_action(valhalla::Options::route, closure_map, req, reader);
    gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DE", "EF", "FG"});
  }

  // Close an interdmediate edge. Route should avoid it while not ignoring the
  // consecutive closures at origin
  {
    LiveTrafficCustomize close_edge = [](GraphReader& reader, TrafficTile& tile, uint32_t index,
                                         TrafficSpeed* current) -> void {
      close_bidir_edge(reader, tile, index, current, "DE", closure_map);
    };
    test::customize_live_traffic_data(closure_map.config, close_edge);

    const std::string& req =
        (boost::format(
             R"({"locations":[{"lat":%s,"lon":%s,"search_filter":{"exclude_closures":false}},{"lat":%s,"lon":%s}],"costing":"%s", "costing_options": {"%s": {"speed_types":["freeflow","constrained","predicted","current"]}}, "date_time":{"type":"%s", "value": "current"}})") %
         std::to_string(closure_map.nodes.at("A").lat()) %
         std::to_string(closure_map.nodes.at("A").lng()) %
         std::to_string(closure_map.nodes.at("G").lat()) %
         std::to_string(closure_map.nodes.at("G").lng()) % costing % costing % date_type)
            .str();
    auto result = gurka::do_action(valhalla::Options::route, closure_map, req, reader);
    gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DIJE", "EF", "FG"});
  }
  // TODO: Ensure this test work with timedep reverse (date_type = "2")
  // Refer https://github.com/valhalla/valhalla/issues/2733
}

TEST_P(ClosuresWithTimedepRoutes, IgnoreClosureWithTimedepReverse) {
  std::string costing = GetParam();
  // use arrive by time which makes use of timedep reverse A*
  std::string date_type = "2";
  std::string costing_speed_type =
      (boost::format("/costing_options/%s/speed_types/0") % costing).str();

  {
    auto result = gurka::do_action(valhalla::Options::route, closure_map, {"B", "H"}, costing,
                                   {{"/date_time/type", date_type},
                                    {"/date_time/value", "current"},
                                    {costing_speed_type, "current"}},
                                   reader);
    gurka::assert::raw::expect_path(result, {"BC", "CD", "DE", "EF", "FG", "GH"});
  }
  {
    LiveTrafficCustomize close_edge = [](GraphReader& reader, TrafficTile& tile, uint32_t index,
                                         TrafficSpeed* current) -> void {
      close_bidir_edge(reader, tile, index, current, "FG", closure_map);
      close_bidir_edge(reader, tile, index, current, "GH", closure_map);
    };
    test::customize_live_traffic_data(closure_map.config, close_edge);

    const std::string& req =
        (boost::format(
             R"({"locations":[{"lat":%s,"lon":%s},{"lat":%s,"lon":%s,"search_filter":{"exclude_closures":false}}],"costing":"%s", "costing_options": {"%s": {"speed_types":["freeflow","constrained","predicted","current"]}}, "date_time":{"type":"%s", "value": "current"}})") %
         std::to_string(closure_map.nodes.at("B").lat()) %
         std::to_string(closure_map.nodes.at("B").lng()) %
         std::to_string(closure_map.nodes.at("H").lat()) %
         std::to_string(closure_map.nodes.at("H").lng()) % costing % costing % date_type)
            .str();
    auto result = gurka::do_action(valhalla::Options::route, closure_map, req, reader);
    gurka::assert::raw::expect_path(result, {"BC", "CD", "DE", "EF", "FG", "GH"});
  }
  // Close an interdmediate edge. Route should avoid it while not ignoring the
  // consecutive closures at destination
  {
    LiveTrafficCustomize close_edge = [](GraphReader& reader, TrafficTile& tile, uint32_t index,
                                         TrafficSpeed* current) -> void {
      close_bidir_edge(reader, tile, index, current, "DE", closure_map);
    };
    test::customize_live_traffic_data(closure_map.config, close_edge);

    const std::string& req =
        (boost::format(
             R"({"locations":[{"lat":%s,"lon":%s},{"lat":%s,"lon":%s,"search_filter":{"exclude_closures":false}}],"costing":"%s", "costing_options": {"%s": {"speed_types":["freeflow","constrained","predicted","current"]}}, "date_time":{"type":"%s", "value": "current"}})") %
         std::to_string(closure_map.nodes.at("B").lat()) %
         std::to_string(closure_map.nodes.at("B").lng()) %
         std::to_string(closure_map.nodes.at("H").lat()) %
         std::to_string(closure_map.nodes.at("H").lng()) % costing % costing % date_type)
            .str();
    auto result = gurka::do_action(valhalla::Options::route, closure_map, req, reader);
    gurka::assert::raw::expect_path(result, {"BC", "CD", "DIJE", "EF", "FG", "GH"});
  }

  // TODO: Ensure this test work with timedep fwd (date_type = "0")
  // Refer https://github.com/valhalla/valhalla/issues/2733
}

INSTANTIATE_TEST_SUITE_P(SearchFilter, ClosuresWithTimedepRoutes, ::testing::ValuesIn(buildParams()));

/*********************************************************************/

struct Waypoint {
  std::string node;
  int16_t preferred_level;
};
class LevelSearchFilter : public ::testing::Test {
protected:
  static gurka::map map;
  static std::string ascii_map;
  static gurka::nodelayout layout;
  static void SetUpTestSuite() {
    constexpr double gridsize_metres = 50;

    /**
     * Difficult to represent visually, so here is a stacked view:
     *
     * ground level:
     *  C-----------------D
     *  |                 |
     *  |                 |
     *  |                 |
     * (A)---------------(B)
     *
     * first floor:
     *   G---------------H
     *   |               |
     *   |               |
     *   |               |
     *  (E)-------------(F)
     *
     * () = connected via stairs
     */
    ascii_map = R"(
      C-G-----------H-D
      | |  x    y   | |
      | |           | |
      | |           | |                z
      | |           | |                       w
      | |           | |
      A~E-----------F~B
    )";

    const gurka::ways ways = {
        // ground floor
        {"AB", {{"highway", "corridor"}, {"level", "0"}}},
        {"AC", {{"highway", "corridor"}, {"level", "0"}}},
        {"CD", {{"highway", "corridor"}, {"level", "0"}}},
        {"DB", {{"highway", "corridor"}, {"level", "0"}}},
        // level 1
        {"EG", {{"highway", "corridor"}, {"level", "1"}}},
        {"EF", {{"highway", "corridor"}, {"level", "1"}}},
        {"GH", {{"highway", "corridor"}, {"level", "1"}}},
        {"HF", {{"highway", "corridor"}, {"level", "1"}}},
        // stairs
        {"AE", {{"highway", "steps"}, {"level", "0;1"}}},
        {"FB", {{"highway", "steps"}, {"level", "0;1"}}},

    };

    layout = gurka::detail::map_to_coordinates(ascii_map, gridsize_metres);
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_multi_level_loki", {});
  }

  valhalla::Api Route(const std::vector<Waypoint>& waypoints, unsigned int cutoff = 0) {
    std::vector<std::string> nodes;
    std::unordered_map<std::string, std::string> options;
    for (size_t index = 0; index < waypoints.size(); ++index) {
      const auto& wp = waypoints[index];
      nodes.emplace_back(wp.node);
      options["/locations/" + std::to_string(index) + "/search_filter/level"] =
          std::to_string(wp.preferred_level);
      if (cutoff > 0) {
        options["/locations/" + std::to_string(index) + "/search_cutoff"] = std::to_string(cutoff);
      }
    }
    return gurka::do_action(valhalla::Options::route, map, nodes, "pedestrian", options);
  }
};
gurka::map LevelSearchFilter::map = {};
std::string LevelSearchFilter::ascii_map = {};
gurka::nodelayout LevelSearchFilter::layout = {};

TEST_F(LevelSearchFilter, TraverseLevels) {
  auto result = Route({{"x", 0}, {"y", 1}});
  ASSERT_EQ(result.info().warnings().size(), 1);
  EXPECT_EQ(result.info().warnings().Get(0).code(), 302);
  gurka::assert::raw::expect_path(result, {"CD", "AC", "AE", "EG", "GH"});
}

TEST_F(LevelSearchFilter, NonExistentLevel) {
  try {
    auto result = Route({{"x", 0}, {"y", 6}});
    FAIL() << "We should not get to here";
  } catch (const valhalla_exception_t& e) {
    EXPECT_EQ(e.code, 171);
    EXPECT_STREQ(e.what(), "No suitable edges near location");
  } catch (...) { FAIL() << "Failed with unexpected exception type"; }
}

TEST_F(LevelSearchFilter, Cutoff) {
  try {
    auto result = Route({{"x", 0}, {"z", 1}});
    FAIL() << "We should not get to here";
  } catch (const valhalla_exception_t& e) {
    EXPECT_EQ(e.code, 171);
    EXPECT_STREQ(e.what(), "No suitable edges near location");
  } catch (...) { FAIL() << "Failed with unexpected exception type"; }
}

TEST_F(LevelSearchFilter, CutoffOverride) {
  try {
    auto result = Route({{"x", 0}, {"z", 1}}, 9000);
    EXPECT_EQ(result.info().warnings().size(), 1);
  } catch (...) { FAIL() << "Shoud succeed"; }
}

TEST_F(LevelSearchFilter, CutoffClamped) {
  try {
    // w is about 1300m away, so the search_cutoff being clamped to 1000 should result
    // in an exception
    auto result = Route({{"x", 0}, {"w", 1}}, 2000);
    FAIL() << "Should fail";
  } catch (const valhalla_exception_t& e) { EXPECT_EQ(e.code, 171); } catch (...) {
    FAIL() << "Failed with unexpected exception type";
  };
}