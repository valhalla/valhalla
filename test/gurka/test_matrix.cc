#include "gurka.h"
#include "test.h"

#include <boost/format.hpp>
#include <gtest/gtest.h>

using namespace valhalla;

void update_traffic_on_edges(baldr::GraphReader& reader,
                             baldr::TrafficTile& tile,
                             uint32_t index,
                             baldr::TrafficSpeed* current,
                             const std::string& edge_name,
                             const gurka::map& closure_map,
                             uint64_t speed) {
  for (const auto& node : {edge_name.front(), edge_name.back()}) {
    baldr::GraphId tile_id(tile.header->tile_id);
    auto edge =
        std::get<0>(gurka::findEdge(reader, closure_map.nodes, edge_name, std::string(1, node)));
    if (edge.Tile_Base() == tile_id && edge.id() == index) {
      current->breakpoint1 = 255;
      current->overall_encoded_speed = speed >> 1;
      current->encoded_speed1 = speed >> 1;
    }
  }
}

class MatrixTest : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize = 100;

    // upper ways are motorways, lower are residential
    const std::string ascii_map = R"(
        A-----B
        |     |
        |     |
        |     |
     E--F     G--H
        |     |
        C-----D
    )";

    const gurka::ways ways = {{"FA", {{"highway", "motorway"}}},
                              {"AB", {{"highway", "motorway"}}},
                              {"BG", {{"highway", "motorway"}}},
                              {"FC", {{"highway", "residential"}}},
                              {"CD", {{"highway", "residential"}}},
                              {"DG", {{"highway", "residential"}}},
                              {"EF", {{"highway", "residential"}}},
                              {"GH", {{"highway", "residential"}}}};

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/matrix_traffic",
                            {{"service_limits.max_timedep_distance_matrix", "50000"}});
  }
};

gurka::map MatrixTest::map = {};

TEST_F(MatrixTest, MatrixNoTraffic) {
  // we expect to take the motorways, residential path is 1.6f
  const std::vector<float> exp_dists = {0.0f, 2.0f, 2.0f, 0.0f};
  std::string res;
  const auto result = gurka::do_action(Options::sources_to_targets, map, {"E", "H"}, {"E", "H"},
                                       "auto", {}, {}, &res);

  rapidjson::Document res_doc;
  res_doc.Parse(res.c_str());

  size_t i = 0;
  for (const auto& origin_row : res_doc["sources_to_targets"].GetArray()) {
    auto origin_td = origin_row.GetArray();
    for (const auto& v : origin_td) {
      std::string msg = "Problem at source " + std::to_string(i / origin_td.Size()) + " and target " +
                        std::to_string(i % origin_td.Size());
      EXPECT_EQ(v.GetObject()["distance"].GetFloat(), exp_dists[i]) << msg;
      i++;
    }
  }
}

TEST_F(MatrixTest, MatrixWithLiveTraffic) {
  map.config.put("mjolnir.traffic_extract", "test/data/matrix_traffic/traffic.tar");

  test::build_live_traffic_data(map.config);
  std::shared_ptr<valhalla::baldr::GraphReader> reader =
      test::make_clean_graphreader(map.config.get_child("mjolnir"));

  test::LiveTrafficCustomize edges_with_traffic = [](baldr::GraphReader& reader,
                                                     baldr::TrafficTile& tile, uint32_t index,
                                                     baldr::TrafficSpeed* current) -> void {
    // update speeds on primary road
    update_traffic_on_edges(reader, tile, index, current, "FA", map, 5);
    update_traffic_on_edges(reader, tile, index, current, "AB", map, 5);
    update_traffic_on_edges(reader, tile, index, current, "BG", map, 5);
  };
  test::customize_live_traffic_data(map.config, edges_with_traffic);

  // we expect to take the shorter lower path on residential
  const std::vector<float> exp_dists = {0.0f, 1.6f, 1.6f, 0.0f};
  const std::unordered_map<std::string, std::string> options = {{"/date_time/type", "0"}};
  std::string res;
  const auto result = gurka::do_action(Options::sources_to_targets, map, {"E", "H"}, {"E", "H"},
                                       "auto", options, reader, &res);

  rapidjson::Document res_doc;
  res_doc.Parse(res.c_str());

  size_t i = 0;
  for (const auto& origin_row : res_doc["sources_to_targets"].GetArray()) {
    auto origin_td = origin_row.GetArray();
    for (const auto& v : origin_td) {
      std::string msg = "Problem at source " + std::to_string(i / origin_td.Size()) + " and target " +
                        std::to_string(i % origin_td.Size());
      EXPECT_EQ(v.GetObject()["distance"].GetFloat(), exp_dists[i]) << msg;
      i++;
    }
  }
}
