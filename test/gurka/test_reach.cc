#include "gurka.h"
#include "loki/reach.h"
#include "sif/costfactory.h"
#include "sif/dynamiccost.h"
#include "test.h"

#include <gtest/gtest.h>
#include <valhalla/proto/options.pb.h>

using namespace valhalla;
using namespace valhalla::loki;
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

class TestReach : public ::testing::Test {
protected:
  static gurka::map closure_map;
  static int const default_speed;
  static std::string const tile_dir;
  static std::shared_ptr<baldr::GraphReader> reader;

  static void SetUpTestSuite() {
    const std::string ascii_map = R"(

      A---B---C
      |   |   |
      D---E---F
      |   |   |
      G---H---I


      K--L--M--N--O
     )";

    const std::string speed_str = std::to_string(default_speed);
    const gurka::ways ways = {{"AB", {{"highway", "primary"}, {"maxspeed", speed_str}}},
                              {"BC", {{"highway", "primary"}, {"maxspeed", speed_str}}},
                              {"AD", {{"highway", "primary"}, {"maxspeed", speed_str}}},
                              {"BE", {{"highway", "primary"}, {"maxspeed", speed_str}}},
                              {"CF", {{"highway", "primary"}, {"maxspeed", speed_str}}},
                              {"DE", {{"highway", "primary"}, {"maxspeed", speed_str}}},
                              {"EF", {{"highway", "primary"}, {"maxspeed", speed_str}}},
                              {"DG", {{"highway", "primary"}, {"maxspeed", speed_str}}},
                              {"EH", {{"highway", "primary"}, {"maxspeed", speed_str}}},
                              {"FI", {{"highway", "primary"}, {"maxspeed", speed_str}}},
                              {"GH", {{"highway", "primary"}, {"maxspeed", speed_str}}},
                              {"HI", {{"highway", "primary"}, {"maxspeed", speed_str}}},
                              {"KL", {{"highway", "primary"}, {"maxspeed", speed_str}}},
                              {"LM", {{"highway", "primary"}, {"maxspeed", speed_str}}},
                              {"MN", {{"highway", "primary"}, {"maxspeed", speed_str}}},
                              {"NO", {{"highway", "primary"}, {"maxspeed", speed_str}}}};

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10, {.05f, .2f});
    closure_map = gurka::buildtiles(layout, ways, {}, {}, tile_dir);

    closure_map.config.put("mjolnir.traffic_extract", tile_dir + "/traffic.tar");
    test::build_live_traffic_data(closure_map.config);

    reader = test::make_clean_graphreader(closure_map.config.get_child("mjolnir"));
  }

  void set_default_speed_on_all_edges() {
    test::customize_live_traffic_data(closure_map.config,
                                      [](baldr::GraphReader&, baldr::TrafficTile&, int,
                                         baldr::TrafficSpeed* current) -> void {
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

gurka::map TestReach::closure_map = {};
const int TestReach::default_speed = 36;
const std::string TestReach::tile_dir = "test/data/reach_with_closures";
std::shared_ptr<baldr::GraphReader> TestReach::reader;

TEST_F(TestReach, ReachWithNoClosures) {
  // Create costing
  // - type: auto
  // - flow mask: "default" (includes current) to consider live speeds
  // - filter_closures: disable filter closed edges
  sif::CostFactory factory;
  Costing c;
  c.set_type(Costing::auto_);
  c.mutable_options()->set_flow_mask(kDefaultFlowMask);
  c.set_filter_closures(false);

  auto costing = factory.Create(c);
  auto edge = gurka::findEdgeByNodes(*reader, closure_map.nodes, "A", "B");

  // check its reach
  loki::Reach reach_checker;
  auto reach = reach_checker(std::get<1>(edge), std::get<0>(edge), 50, *reader, costing);

  // all edges should have the same in/outbound reach
  EXPECT_EQ(reach.inbound, reach.outbound);
  EXPECT_EQ(reach.inbound, 9);
  EXPECT_EQ(reach.outbound, 9);
}

TEST_F(TestReach, ReachWithClosures) {
  LiveTrafficCustomize close_edge = [](baldr::GraphReader& reader, baldr::TrafficTile& tile,
                                       uint32_t index, baldr::TrafficSpeed* current) -> void {
    close_bidir_edge(reader, tile, index, current, "AB", closure_map);
    close_bidir_edge(reader, tile, index, current, "BC", closure_map);
    close_bidir_edge(reader, tile, index, current, "AD", closure_map);
    close_bidir_edge(reader, tile, index, current, "BE", closure_map);
  };
  test::customize_live_traffic_data(closure_map.config, close_edge);

  // Create costing
  // - type: auto
  // - flow mask: "default" (includes current) to consider live speeds
  // - filter_closures: enabled, to remove closed edges
  sif::CostFactory factory;
  loki::Reach reach_checker;

  Costing c;
  c.set_type(Costing::auto_);
  c.mutable_options()->set_flow_mask(kDefaultFlowMask);
  c.set_filter_closures(true);
  // Check reach for AB - enclosed by closures, so reach should be 0
  {
    auto costing = factory.Create(c);
    auto edge = gurka::findEdgeByNodes(*reader, closure_map.nodes, "A", "B");
    auto reach = reach_checker(std::get<1>(edge), std::get<0>(edge), 50, *reader, costing);

    EXPECT_EQ(reach.inbound, reach.outbound);
    EXPECT_EQ(reach.inbound, 0);
    EXPECT_EQ(reach.outbound, 0);
  }
  // Check reach for HI - open, but reduced reach without ignoring closures
  {
    auto costing = factory.Create(c);
    auto edge = gurka::findEdgeByNodes(*reader, closure_map.nodes, "H", "I");
    auto reach = reach_checker(std::get<1>(edge), std::get<0>(edge), 50, *reader, costing);

    EXPECT_EQ(reach.inbound, 7);
    EXPECT_EQ(reach.outbound, 7);
  }

  // Check reach for AB ignoring closures
  {
    c.set_filter_closures(false);
    auto costing = factory.Create(c);
    auto edge = gurka::findEdgeByNodes(*reader, closure_map.nodes, "A", "B");
    auto reach = reach_checker(std::get<1>(edge), std::get<0>(edge), 50, *reader, costing);

    EXPECT_EQ(reach.inbound, 8);
    EXPECT_EQ(reach.outbound, 8);
  }
  // Check reach for HI ignoring closures
  {
    c.set_filter_closures(false);
    auto costing = factory.Create(c);
    auto edge = gurka::findEdgeByNodes(*reader, closure_map.nodes, "H", "I");
    auto reach = reach_checker(std::get<1>(edge), std::get<0>(edge), 50, *reader, costing);

    EXPECT_EQ(reach.inbound, 7);
    EXPECT_EQ(reach.outbound, 7);
  }
}

TEST_F(TestReach, DISABLED_ReachWithClosures2) {
  LiveTrafficCustomize close_edge = [](baldr::GraphReader& reader, baldr::TrafficTile& tile,
                                       uint32_t index, baldr::TrafficSpeed* current) -> void {
    close_bidir_edge(reader, tile, index, current, "LM", closure_map);
    close_bidir_edge(reader, tile, index, current, "MN", closure_map);
  };
  test::customize_live_traffic_data(closure_map.config, close_edge);

  // Create costing
  // - type: auto
  // - flow mask: "default" (includes current) to consider live speeds
  // - filter_closures: enabled, to remove closed edges
  sif::CostFactory factory;
  loki::Reach reach_checker;

  Costing c;
  c.set_type(Costing::auto_);
  c.mutable_options()->set_flow_mask(kDefaultFlowMask);
  c.set_filter_closures(false);
  auto costing = factory.Create(c);
  auto edge = gurka::findEdgeByNodes(*reader, closure_map.nodes, "K", "L");
  auto reach = reach_checker(std::get<1>(edge), std::get<0>(edge), 50, *reader, costing);

  EXPECT_EQ(reach.inbound, reach.outbound);
  EXPECT_EQ(reach.inbound, 3);
  EXPECT_EQ(reach.outbound, 3);
}
