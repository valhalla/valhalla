#include "baldr/rapidjson_utils.h"
#include "gurka.h"
#include "midgard/encoded.h"
#include "proto/api.pb.h"
#include "test.h"
#include "valhalla/proto_conversions.h"
#include "valhalla/worker.h"

#include <gtest/gtest.h>

using namespace valhalla;
using namespace valhalla::thor;
using namespace valhalla::midgard;

namespace {
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

void check_matrix(const rapidjson::Document& result,
                  const std::vector<float>& exp_times,
                  bool valid_traffic,
                  const std::string& metric,
                  Matrix::Algorithm matrix_type) {
  size_t i = 0;
  for (const auto& origin_row : result["sources_to_targets"].GetArray()) {
    auto origin_td = origin_row.GetArray();
    for (const auto& v : origin_td) {
      std::string msg = "Problem at source " + std::to_string(i / origin_td.Size()) + " and target " +
                        std::to_string(i % origin_td.Size()) + " (index " + std::to_string(i) +
                        "), " + metric;
      EXPECT_TRUE(v.HasMember(metric));
      EXPECT_NEAR(v.GetObject()[metric].GetFloat(), exp_times[i], 0.01) << msg;
      if (valid_traffic) {
        ASSERT_TRUE(v.GetObject().HasMember("date_time")) << msg;
        EXPECT_TRUE(v.GetObject()["date_time"] != "") << msg;
      }
      i++;
    }
  }
  const std::string algo = result["algorithm"].GetString();
  const std::string& exp_algo = MatrixAlgoToString(matrix_type);
  EXPECT_EQ(algo, exp_algo);
}

} // namespace

class MatrixTrafficTest : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize = 100;

    const std::string ascii_map = R"(
      A----1---B-----C-----D------E------F---2--------G
    )";

    const gurka::ways ways = {
        {"AB", {{"highway", "primary"}, {"maxspeed", "100"}, {"bicycle", "yes"}}},
        {"BC", {{"highway", "primary"}, {"maxspeed", "100"}, {"bicycle", "no"}}},
        {"CD", {{"highway", "primary"}, {"maxspeed", "100"}, {"bicycle", "yes"}}},
        {"DE", {{"highway", "primary"}, {"maxspeed", "100"}, {"bicycle", "no"}}},
        {"EF", {{"highway", "primary"}, {"maxspeed", "100"}, {"bicycle", "yes"}}},
        {"FG", {{"highway", "primary"}, {"maxspeed", "100"}, {"bicycle", "no"}}},
    };

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
    // also turn on the reverse connection search; there's no real test for it
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/matrix_traffic_allowed",
                            {{"service_limits.max_timedep_distance_matrix", "50000"},
                             {"mjolnir.traffic_extract",
                              "test/data/matrix_traffic_allowed/traffic.tar"},
                             {"mjolnir.timezone", VALHALLA_BUILD_DIR "test/data/tz.sqlite"},
                             {"thor.costmatrix_check_reverse_connection", "1"},
                             {"mjolnir.shortcuts", "0"}});

    test::build_live_traffic_data(map.config);
    test::LiveTrafficCustomize edges_with_traffic = [&](baldr::GraphReader& reader,
                                                        baldr::TrafficTile& tile, uint32_t index,
                                                        baldr::TrafficSpeed* current) -> void {
      // update traffic speeds, set them to some low value
      for (const auto& way : ways) {

        auto fwd = way.first;
        auto rev = fwd;
        std::reverse(rev.begin(), rev.end());

        update_traffic_on_edges(reader, tile, index, current, fwd, map, 4);
        update_traffic_on_edges(reader, tile, index, current, rev, map, 4);
      };
    };
    test::customize_live_traffic_data(map.config, edges_with_traffic);
  }
};

gurka::map MatrixTrafficTest::map = {};

TEST_F(MatrixTrafficTest, LiveTraffic) {
  std::unordered_map<std::string, std::string> options = {{"/date_time/type", "0"},
                                                          {"/costing_options/auto/speed_types/0",
                                                           "current"}};

  // forward tree
  std::string res;
  auto result = gurka::do_action(Options::sources_to_targets, map, {"1", "2"}, {"1", "2"}, "auto",
                                 options, nullptr, &res);
  rapidjson::Document res_doc;
  res_doc.Parse(res.c_str());
  check_matrix(res_doc, {0.0f, 234.0f, 548.0f, 0.0f}, true, "time", Matrix::TimeDistanceMatrix);
  ASSERT_EQ(result.info().warnings().size(), 0);

  // forward search, date_time on the locations, 2nd location has pointless date_time
  options = {{"/sources/0/date_time", "current"},
             {"/sources/1/date_time", "2016-07-03T08:06"},
             {"/costing_options/auto/speed_types/0", "current"}};
  res.erase();
  result = gurka::do_action(Options::sources_to_targets, map, {"1", "2"}, {"1", "2"}, "auto", options,
                            nullptr, &res);
  res_doc.Parse(res.c_str());
  // the second origin can't respect time (no historical data)
  check_matrix(res_doc, {0.0f, 234.0f, 122.0f, 0.0f}, false, "time", Matrix::TimeDistanceMatrix);
  ASSERT_EQ(result.info().warnings().size(), 0);

  res.erase();
  options = {
      {"/sources/0/date_time", "current"},
      {"/sources/1/date_time", "2016-07-03T08:06"},
      {"/costing_options/auto/speed_types/0", "current"},
      {"/prioritize_bidirectional", "1"},
      {"/shape_format", "polyline6"},
  };
  result = gurka::do_action(Options::sources_to_targets, map, {"1", "2"}, {"1", "2"}, "auto", options,
                            nullptr, &res);
  res_doc.Parse(res.c_str());
  check_matrix(res_doc, {0.0f, 924.0f, 122.0f, 0.0f}, false, "time", Matrix::CostMatrix);
}
