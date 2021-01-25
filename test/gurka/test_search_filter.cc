#include "gurka.h"
#include "test.h"

#include <boost/format.hpp>
#include <gtest/gtest.h>

using namespace valhalla;
using LiveTrafficCustomize = test::LiveTrafficCustomize;

/*************************************************************/
class SearchFilter : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize = 100;

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
         )";
    const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
    const gurka::ways ways = {{"AB", {{"highway", "motorway"}}},
                              {"BC", {{"highway", "residential"}, {"tunnel", "yes"}}},
                              {"CD", {{"highway", "primary"}, {"oneway", "-1"}}},
                              {"AD", {{"highway", "primary"}}},
                              {"DE", {{"highway", "primary"}}},
                              {"EF", {{"highway", "primary"}, {"bridge", "yes"}}},
                              {"AF", {{"highway", "motorway_link"}}}};
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

/*************************************************************/
namespace {
inline void SetLiveSpeed(baldr::TrafficSpeed* live_speed, uint64_t speed) {
  live_speed->breakpoint1 = 255;
  live_speed->overall_speed = speed >> 1;
  live_speed->speed1 = speed >> 1;
}

void close_dir_edge(baldr::GraphReader& reader,
                    baldr::TrafficTile& tile,
                    uint32_t index,
                    baldr::TrafficSpeed* current,
                    const std::string& edge_name,
                    const std::string& start_node,
                    const gurka::map& closure_map) {
  baldr::GraphId tile_id(tile.header->tile_id);
  auto edge = std::get<0>(gurka::findEdge(reader, closure_map.nodes, edge_name, start_node));
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

using costing_and_datetype = std::tuple<std::string, std::string>;

class ExcludeClosuresOnWaypoints : public ::testing::TestWithParam<costing_and_datetype> {
protected:
  static gurka::map closure_map;
  static int const default_speed;
  static std::string const tile_dir;
  static std::shared_ptr<baldr::GraphReader> reader;

  static void SetUpTestSuite() {
    const std::string ascii_map = R"(
              F----G
              |    |
    A--1--B---C----D--2--E--J--3--K
             /
            I
            |
            H

   L-4---5-M-6-N
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
                                      [](baldr::GraphReader& reader, baldr::TrafficTile& tile,
                                         uint32_t index, baldr::TrafficSpeed* current) -> void {
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

gurka::map ExcludeClosuresOnWaypoints::closure_map = {};
const int ExcludeClosuresOnWaypoints::default_speed = 36;
const std::string ExcludeClosuresOnWaypoints::tile_dir =
    "test/data/traffic_exclude_closures_on_waypoints";
std::shared_ptr<baldr::GraphReader> ExcludeClosuresOnWaypoints::reader;

/*
 *  Tests search_filter.exclude_closures at departure
 */
TEST_P(ExcludeClosuresOnWaypoints, ExcludeClosuresAtDeparture) {
  std::string costing = std::get<0>(GetParam());
  std::string date_type = std::get<1>(GetParam());
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

  // AB edge is closed in both directions. Route should avoid AB with
  // exclude_closures set to true (default) & and use it otherwise
  {
    LiveTrafficCustomize close_edge = [](baldr::GraphReader& reader, baldr::TrafficTile& tile,
                                         uint32_t index, baldr::TrafficSpeed* current) -> void {
      close_bidir_edge(reader, tile, index, current, "AB", closure_map);
    };
    test::customize_live_traffic_data(closure_map.config, close_edge);

    auto result = gurka::do_action(valhalla::Options::route, closure_map, {"1", "2"}, costing,
                               {{"/date_time/type", date_type},
                                {"/date_time/value", "current"},
                                {costing_speed_type, "current"}},
                               reader);
    gurka::assert::osrm::expect_steps(result, {"BC"});
    gurka::assert::raw::expect_path(result, {"BC", "CD", "DE"});

    // Specify search filter to disable exclude_closures at departure
    const std::string& req_disable_exclude_closures =
        (boost::format(
             R"({"locations":[{"lat":%s,"lon":%s,"search_filter":{"exclude_closures":false}},{"lat":%s,"lon":%s}],"costing":"%s", "costing_options": {"%s": {"speed_types":["freeflow","constrained","predicted","current"]}}, "date_time":{"type":"%s", "value": "current"}})") %
         std::to_string(closure_map.nodes.at("1").lat()) %
         std::to_string(closure_map.nodes.at("1").lng()) %
         std::to_string(closure_map.nodes.at("2").lat()) %
         std::to_string(closure_map.nodes.at("2").lng()) % costing % costing % date_type)
            .str();
    result = gurka::do_action(valhalla::Options::route, closure_map, req_disable_exclude_closures, reader);
    gurka::assert::osrm::expect_steps(result, {"AB"});
    gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DE"});
  }
}

/*
 *  Tests search_filter.exclude_closures at destination
 */
TEST_P(ExcludeClosuresOnWaypoints, ExcludeClosuresAtDestination) {
  std::string costing = std::get<0>(GetParam());
  std::string date_type = std::get<1>(GetParam());
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

  // DE edge is closed in both directions. Route should avoid DE with
  // exclude_closures set to true (default) & and use it otherwise
  {
    LiveTrafficCustomize close_edge = [](baldr::GraphReader& reader, baldr::TrafficTile& tile,
                                         uint32_t index, baldr::TrafficSpeed* current) -> void {
      close_bidir_edge(reader, tile, index, current, "DE", closure_map);
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
    result = gurka::do_action(valhalla::Options::route, closure_map, req_disable_exclude_closures, reader);
    gurka::assert::osrm::expect_steps(result, {"AB"});
    gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DE"});
  }
}

/*
 *  Tests search_filter.exclude_closures at midway location
 */
TEST_P(ExcludeClosuresOnWaypoints, ExcludeClosuresAtMidway) {
  std::string costing = std::get<0>(GetParam());
  std::string date_type = std::get<1>(GetParam());
  std::string costing_speed_type =
      (boost::format("/costing_options/%s/speed_types/0") % costing).str();

  // None of the edges are closed. Route has multiple waypoints
  {
    auto result = gurka::do_action(valhalla::Options::route, closure_map, {"1", "2", "3"}, costing,
                               {{"/date_time/type", date_type},
                                {"/date_time/value", "current"},
                                {costing_speed_type, "current"}},
                               reader);
    gurka::assert::osrm::expect_steps(result, {"AB", "DE"});
    gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DE", "DE", "EJ", "JK"});
  }

  {
    LiveTrafficCustomize close_edge = [](baldr::GraphReader& reader, baldr::TrafficTile& tile,
                                         uint32_t index, baldr::TrafficSpeed* current) -> void {
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
    auto result = gurka::do_action(valhalla::Options::route, closure_map, req_disable_exclude_closures, reader);
    gurka::assert::osrm::expect_steps(result, {"AB", "DE"});
    gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DE", "EJ", "JK"});
  }
}

/*
 *  Tests costing_options.ignore_closures:true overrides
 *  search_filter.exclude_closures:true (when exclude_closures:true is not
 *  explicitly set)
 */
TEST_P(ExcludeClosuresOnWaypoints, IgnoreClosuresOverridesExcludeClosures) {
  std::string costing = std::get<0>(GetParam());
  std::string date_type = std::get<1>(GetParam());
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
    LiveTrafficCustomize close_edge = [](baldr::GraphReader& reader, baldr::TrafficTile& tile,
                                         uint32_t index, baldr::TrafficSpeed* current) -> void {
      close_bidir_edge(reader, tile, index, current, "CD", closure_map);
    };
    test::customize_live_traffic_data(closure_map.config, close_edge);

    auto result = gurka::do_action(valhalla::Options::route, closure_map, {"1", "2"}, costing,
                               {{"/date_time/type", date_type},
                                {"/date_time/value", "current"},
                                {costing_speed_type, "current"}},
                               reader);
    gurka::assert::osrm::expect_steps(result, {"AB", "CFGD", "DE"});
    gurka::assert::raw::expect_path(result, {"AB", "BC", "CFGD", "DE"});

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
    result = gurka::do_action(valhalla::Options::route, closure_map, req_disable_exclude_closures, reader);
    gurka::assert::osrm::expect_steps(result, {"AB"});
    gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DE"});
  }
}

TEST_P(ExcludeClosuresOnWaypoints, ConsecutiveClosuresAtDeparture) {
  std::string costing = std::get<0>(GetParam());
  std::string date_type = std::get<1>(GetParam());
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

  // Close departure & destination edges
  {
    LiveTrafficCustomize close_edge = [](baldr::GraphReader& reader, baldr::TrafficTile& tile,
                                         uint32_t index, baldr::TrafficSpeed* current) -> void {
      close_bidir_edge(reader, tile, index, current, "AB", closure_map);
      close_bidir_edge(reader, tile, index, current, "BC", closure_map);
    };
    test::customize_live_traffic_data(closure_map.config, close_edge);

    auto result = gurka::do_action(valhalla::Options::route, closure_map, {"1", "3"}, costing,
                               {{"/date_time/type", date_type},
                                {"/date_time/value", "current"},
                                {costing_speed_type, "current"}},
                               reader);
    gurka::assert::osrm::expect_steps(result, {"HIC", "CD"});
    gurka::assert::raw::expect_path(result, {"HIC", "CD", "DE", "EJ", "JK"});

    // Specify search filter to disable exclude_closures at departure
    const std::string& req_disable_exclude_closures =
        (boost::format(
             R"({"locations":[{"lat":%s,"lon":%s,"search_filter":{"exclude_closures":false}},{"lat":%s,"lon":%s}],"costing":"%s", "costing_options": {"%s": {"speed_types":["freeflow","constrained","predicted","current"]}}, "date_time":{"type":"%s", "value": "current"}})") %
         std::to_string(closure_map.nodes.at("1").lat()) %
         std::to_string(closure_map.nodes.at("1").lng()) %
         std::to_string(closure_map.nodes.at("3").lat()) %
         std::to_string(closure_map.nodes.at("3").lng()) % costing % costing % date_type)
            .str();
    EXPECT_THROW(gurka::do_action(valhalla::Options::route, closure_map, req_disable_exclude_closures, reader),
                 valhalla_exception_t);
  }
}

TEST_P(ExcludeClosuresOnWaypoints, ConsecutiveClosuresWithLowReachability) {
  // Tests that edge candidates with low reachability can be selected when
  // routing out of consecutive closures
  std::string costing = std::get<0>(GetParam());
  std::string date_type = std::get<1>(GetParam());
  std::string costing_speed_type =
      (boost::format("/costing_options/%s/speed_types/0") % costing).str();

  // Close departure & destination edges
  {
    LiveTrafficCustomize close_edge = [](baldr::GraphReader& reader, baldr::TrafficTile& tile,
                                         uint32_t index, baldr::TrafficSpeed* current) -> void {
      close_bidir_edge(reader, tile, index, current, "AB", closure_map);
      close_bidir_edge(reader, tile, index, current, "BC", closure_map);
    };
    test::customize_live_traffic_data(closure_map.config, close_edge);

    // Specify search filter to disable exclude_closures at departure
    const std::string& req_disable_exclude_closures =
        (boost::format(
             R"({"locations":[{"lat":%s,"lon":%s,"search_filter":{"exclude_closures":false}},{"lat":%s,"lon":%s}],"costing":"%s", "costing_options": {"%s": {"speed_types":["freeflow","constrained","predicted","current"]}}, "date_time":{"type":"%s", "value": "current"}})") %
         std::to_string(closure_map.nodes.at("1").lat()) %
         std::to_string(closure_map.nodes.at("1").lng()) %
         std::to_string(closure_map.nodes.at("2").lat()) %
         std::to_string(closure_map.nodes.at("2").lng()) % costing % costing % date_type)
            .str();
    EXPECT_THROW(gurka::do_action(valhalla::Options::route, closure_map, req_disable_exclude_closures, reader),
                 valhalla_exception_t);

    // Specify minimum reachability so that nearby edges (which were previously
    // rejected due to low-reachbility), can be selected
    // Note: we must turn off candidate ranking because of a bug in bidira* that
    // doesnt continue the other direction of the search if one becomes exhausted
    const std::string& req_low_reachbility =
        (boost::format(
             R"({"locations":[{"lat":%s,"lon":%s,"search_filter":{"exclude_closures":false}, "minimum_reachability": 2, "rank_candidates":false},{"lat":%s,"lon":%s}],"costing":"%s", "costing_options": {"%s": {"speed_types":["freeflow","constrained","predicted","current"]}}, "date_time":{"type":"%s", "value": "current"}})") %
         std::to_string(closure_map.nodes.at("1").lat()) %
         std::to_string(closure_map.nodes.at("1").lng()) %
         std::to_string(closure_map.nodes.at("2").lat()) %
         std::to_string(closure_map.nodes.at("2").lng()) % costing % costing % date_type)
            .str();
    auto result = gurka::do_action(valhalla::Options::route, closure_map, req_low_reachbility, reader);
    gurka::assert::osrm::expect_steps(result, {"BC"});
    gurka::assert::raw::expect_path(result, {"BC", "CD", "DE"});
  }
}

TEST_P(ExcludeClosuresOnWaypoints, AvoidOnlyMidwayClosures) {
  std::string costing = std::get<0>(GetParam());
  std::string date_type = std::get<1>(GetParam());
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

  // Close edges at departure, midway & destination edges
  {
    LiveTrafficCustomize close_edge = [](baldr::GraphReader& reader, baldr::TrafficTile& tile,
                                         uint32_t index, baldr::TrafficSpeed* current) -> void {
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
    result = gurka::do_action(valhalla::Options::route, closure_map, req_disable_exclude_closures, reader);
    gurka::assert::osrm::expect_steps(result, {"AB", "CFGD", "DE"});
    gurka::assert::raw::expect_path(result, {"AB", "BC", "CFGD", "DE", "EJ", "JK"});
  }
}

TEST_P(ExcludeClosuresOnWaypoints, TrivialRouteSameEdge) {
  std::string costing = std::get<0>(GetParam());
  std::string date_type = std::get<1>(GetParam());
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
    LiveTrafficCustomize close_edge = [](baldr::GraphReader& reader, baldr::TrafficTile& tile,
                                         uint32_t index, baldr::TrafficSpeed* current) -> void {
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
    auto result = gurka::do_action(valhalla::Options::route, closure_map, req_disable_exclude_closures, reader);
    gurka::assert::osrm::expect_steps(result, {"LM"});
    gurka::assert::raw::expect_path(result, {"LM"});
  }
}

TEST_P(ExcludeClosuresOnWaypoints, DISABLED_TrivialRouteAdjacentEdges) {
  // Test disabled since trivial case use timedep-fwd, which currently
  // does not work with closures at destination
  std::string costing = std::get<0>(GetParam());
  std::string date_type = std::get<1>(GetParam());
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
    LiveTrafficCustomize close_edge = [](baldr::GraphReader& reader, baldr::TrafficTile& tile,
                                         uint32_t index, baldr::TrafficSpeed* current) -> void {
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
    auto result = gurka::do_action(valhalla::Options::route, closure_map, req_disable_exclude_closures, reader);
    gurka::assert::osrm::expect_steps(result, {"LM"});
    gurka::assert::raw::expect_path(result, {"LM", "MN"});
  }
}

/*
 *  Tests costing_options.ignore_closures and search_filter.exclude_closures
 *  cannot both be specified
 */
TEST_P(ExcludeClosuresOnWaypoints, ConflictingOptions) {
  std::string costing = std::get<0>(GetParam());
  std::string date_type = std::get<1>(GetParam());

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

// Generate different combinations of costing (auto, bus, etc) & date_type (0, 3)
std::vector<costing_and_datetype> buildParams() {
  std::vector<costing_and_datetype> params;

  std::vector<std::string> costings = {
      "auto", "motorcycle", "motor_scooter", "bus", "truck", "hov", "taxi",
  };
  params.reserve(costings.size());
  for (const auto& costing : costings) {
    // Add date_type:3 for time-invariant bidir a*
    params.emplace_back(std::make_tuple(costing, "3"));
    // Add date_type:0 for timedep-fwd a* using current time
    // TODO: Currently, ignoring closures at destinations does not work for
    // time dependent a* implmentations. Enable this once
    // https://github.com/valhalla/valhalla/issues/2733 is addressed
    // params.emplace_back(std::make_tuple(costing, "0"));
  }
  return params;
}

INSTANTIATE_TEST_SUITE_P(SearchFilter,
                         ExcludeClosuresOnWaypoints,
                         ::testing::ValuesIn(buildParams()));

// TODO: Enable once https://github.com/valhalla/valhalla/issues/2732 is addressed
class DISABLED_ExcludeConsecutiveEdgeClosures
    : public ::testing::TestWithParam<costing_and_datetype> {
protected:
  static gurka::map closure_map;
  static int const default_speed;
  static std::string const tile_dir;
  static std::shared_ptr<baldr::GraphReader> reader;

  static void SetUpTestSuite() {
    const std::string ascii_map = R"(

    A--------B
    |        |
    C-1---D--E-----F
          |        |
          G  H-----I
             |
          J--K--L
             |
             M
  )";

    const std::string speed_str = std::to_string(default_speed);
    const gurka::ways ways = {{"AB", {{"highway", "primary"}, {"maxspeed", speed_str}}},
                              {"AC", {{"highway", "primary"}, {"maxspeed", speed_str}}},
                              {"CD", {{"highway", "primary"}, {"maxspeed", speed_str}}},
                              {"BE", {{"highway", "primary"}, {"maxspeed", speed_str}}},
                              {"DE", {{"highway", "primary"}, {"maxspeed", speed_str}}},
                              {"EF", {{"highway", "primary"}, {"maxspeed", speed_str}}},
                              {"DG", {{"highway", "primary"}, {"maxspeed", speed_str}}},
                              {"FI", {{"highway", "primary"}, {"maxspeed", speed_str}}},
                              {"HI", {{"highway", "primary"}, {"maxspeed", speed_str}}},
                              {"HK", {{"highway", "primary"}, {"maxspeed", speed_str}}},
                              {"KM", {{"highway", "primary"}, {"maxspeed", speed_str}}},
                              {"JK", {{"highway", "primary"}, {"maxspeed", speed_str}}},
                              {"KL", {{"highway", "primary"}, {"maxspeed", speed_str}}}};

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, 20, {.05f, .2f});
    closure_map = gurka::buildtiles(layout, ways, {}, {}, tile_dir);

    closure_map.config.put("mjolnir.traffic_extract", tile_dir + "/traffic.tar");
    test::build_live_traffic_data(closure_map.config);

    reader = test::make_clean_graphreader(closure_map.config.get_child("mjolnir"));
  }

  void set_default_speed_on_all_edges() {
    test::customize_live_traffic_data(closure_map.config,
                                      [](baldr::GraphReader& reader, baldr::TrafficTile& tile,
                                         uint32_t index, baldr::TrafficSpeed* current) -> void {
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

gurka::map DISABLED_ExcludeConsecutiveEdgeClosures::closure_map = {};
const int DISABLED_ExcludeConsecutiveEdgeClosures::default_speed = 36;
const std::string DISABLED_ExcludeConsecutiveEdgeClosures::tile_dir =
    "test/data/traffic_exclude_closures";
std::shared_ptr<baldr::GraphReader> DISABLED_ExcludeConsecutiveEdgeClosures::reader;

TEST_P(DISABLED_ExcludeConsecutiveEdgeClosures, UturnDueToClosure) {
  std::string costing = std::get<0>(GetParam());
  std::string date_type = std::get<1>(GetParam());
  std::string costing_speed_type =
      (boost::format("/costing_options/%s/speed_types/0") % costing).str();

  {
    const std::string& req_with_heading =
        (boost::format(
             R"({"locations":[{"lat":%s,"lon":%s,"heading":90},{"lat":%s,"lon":%s}],"costing":"%s", "costing_options": {"%s": {"speed_types":["freeflow","constrained","predicted","current"]}}, "date_time":{"type":"%s", "value": "current"}})") %
         std::to_string(closure_map.nodes.at("1").lat()) %
         std::to_string(closure_map.nodes.at("1").lng()) %
         std::to_string(closure_map.nodes.at("E").lat()) %
         std::to_string(closure_map.nodes.at("E").lng()) % costing % costing % date_type)
            .str();
    auto result = gurka::do_action(valhalla::Options::route, closure_map, req_with_heading, reader);
    gurka::assert::osrm::expect_steps(result, {"CD"});
    gurka::assert::raw::expect_path(result, {"CD", "DE"});
  }

  // Close consecutive edges CD & DE in fwd direction
  {
    LiveTrafficCustomize close_edge = [](baldr::GraphReader& reader, baldr::TrafficTile& tile,
                                         uint32_t index, baldr::TrafficSpeed* current) -> void {
      close_dir_edge(reader, tile, index, current, "CD", "D", closure_map);
      close_dir_edge(reader, tile, index, current, "DE", "E", closure_map);
    };
    test::customize_live_traffic_data(closure_map.config, close_edge);

    const std::string& req_with_heading =
        (boost::format(
             R"({"locations":[{"lat":%s,"lon":%s,"heading":90},{"lat":%s,"lon":%s}],"costing":"%s", "costing_options": {"%s": {"speed_types":["freeflow","constrained","predicted","current"]}}, "date_time":{"type":"%s", "value": "current"}})") %
         std::to_string(closure_map.nodes.at("1").lat()) %
         std::to_string(closure_map.nodes.at("1").lng()) %
         std::to_string(closure_map.nodes.at("E").lat()) %
         std::to_string(closure_map.nodes.at("E").lng()) % costing % costing % date_type)
            .str();
    auto result = gurka::do_action(valhalla::Options::route, closure_map, req_with_heading, reader);
    gurka::assert::osrm::expect_steps(result, {"AC", "AB", "BE"});
    gurka::assert::raw::expect_path(result, {"AC", "AB", "BE"});

    const std::string& req_disable_exclude_closures =
        (boost::format(
             R"({"locations":[{"lat":%s,"lon":%s,"heading":90,"search_filter":{"exclude_closures":false}},{"lat":%s,"lon":%s,"search_filter":{"exclude_closures":false}}],"costing":"%s", "costing_options": {"%s": {"speed_types":["freeflow","constrained","predicted","current"]}}, "date_time":{"type":"%s", "value": "current"}})") %
         std::to_string(closure_map.nodes.at("1").lat()) %
         std::to_string(closure_map.nodes.at("1").lng()) %
         std::to_string(closure_map.nodes.at("E").lat()) %
         std::to_string(closure_map.nodes.at("E").lng()) % costing % costing % date_type)
            .str();
    result = gurka::do_action(valhalla::Options::route, closure_map, req_disable_exclude_closures, reader);
    gurka::assert::osrm::expect_steps(result, {"CD", "AC", "AB", "BE"});
    gurka::assert::raw::expect_path(result, {"CD", "CD", "AC", "AB", "BE"});
  }
}

TEST_P(DISABLED_ExcludeConsecutiveEdgeClosures, DistantSnapDueToClosure) {
  std::string costing = std::get<0>(GetParam());
  std::string date_type = std::get<1>(GetParam());
  std::string costing_speed_type =
      (boost::format("/costing_options/%s/speed_types/0") % costing).str();

  {
    const std::string& req_with_heading =
        (boost::format(
             R"({"locations":[{"lat":%s,"lon":%s,"heading":0},{"lat":%s,"lon":%s}],"costing":"%s", "costing_options": {"%s": {"speed_types":["freeflow","constrained","predicted","current"]}}, "date_time":{"type":"%s", "value": "current"}})") %
         std::to_string(closure_map.nodes.at("M").lat()) %
         std::to_string(closure_map.nodes.at("M").lng()) %
         std::to_string(closure_map.nodes.at("B").lat()) %
         std::to_string(closure_map.nodes.at("B").lng()) % costing % costing % date_type)
            .str();
    auto result = gurka::do_action(valhalla::Options::route, closure_map, req_with_heading, reader);
    gurka::assert::osrm::expect_steps(result, {"KM", "HI", "FI", "EF", "BE"});
    gurka::assert::raw::expect_path(result, {"KM", "HK", "HI", "FI", "EF", "BE"});
  }

  // Close consecutive edges HK & KM
  {
    LiveTrafficCustomize close_edge = [](baldr::GraphReader& reader, baldr::TrafficTile& tile,
                                         uint32_t index, baldr::TrafficSpeed* current) -> void {
      close_bidir_edge(reader, tile, index, current, "HK", closure_map);
      close_bidir_edge(reader, tile, index, current, "KM", closure_map);
    };
    test::customize_live_traffic_data(closure_map.config, close_edge);

    const std::string& req_with_heading =
        (boost::format(
             R"({"locations":[{"lat":%s,"lon":%s,"heading":0},{"lat":%s,"lon":%s}],"costing":"%s", "costing_options": {"%s": {"speed_types":["freeflow","constrained","predicted","current"]}}, "date_time":{"type":"%s", "value": "current"}})") %
         std::to_string(closure_map.nodes.at("M").lat()) %
         std::to_string(closure_map.nodes.at("M").lng()) %
         std::to_string(closure_map.nodes.at("B").lat()) %
         std::to_string(closure_map.nodes.at("B").lng()) % costing % costing % date_type)
            .str();
    EXPECT_THROW(gurka::do_action(valhalla::Options::route, closure_map, req_with_heading, reader), valhalla_exception_t);

    const std::string& req_disable_exclude_closures =
        (boost::format(
             R"({"locations":[{"lat":%s,"lon":%s,"heading":0,"search_filter":{"exclude_closures":false}},{"lat":%s,"lon":%s,"search_filter":{"exclude_closures":false}}],"costing":"%s", "costing_options": {"%s": {"speed_types":["freeflow","constrained","predicted","current"]}}, "date_time":{"type":"%s", "value": "current"}})") %
         std::to_string(closure_map.nodes.at("M").lat()) %
         std::to_string(closure_map.nodes.at("M").lng()) %
         std::to_string(closure_map.nodes.at("B").lat()) %
         std::to_string(closure_map.nodes.at("B").lng()) % costing % costing % date_type)
            .str();
    auto result = gurka::do_action(valhalla::Options::route, closure_map, req_disable_exclude_closures, reader);
    gurka::assert::osrm::expect_steps(result, {"HI", "FI", "EF", "BE"});
    gurka::assert::raw::expect_path(result, {"HI", "FI", "EF", "BE"});
  }
}

INSTANTIATE_TEST_SUITE_P(SearchFilter,
                         DISABLED_ExcludeConsecutiveEdgeClosures,
                         ::testing::ValuesIn(buildParams()));
