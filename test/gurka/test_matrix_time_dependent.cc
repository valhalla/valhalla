#include "gurka.h"
#include "test.h"
#include <valhalla/thor/matrix_common.h>

#include <gtest/gtest.h>

using namespace valhalla;
using namespace valhalla::thor;

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
                  const std::vector<float>& exp_dists,
                  bool valid_traffic,
                  MatrixType matrix_type) {
  size_t i = 0;
  for (const auto& origin_row : result["sources_to_targets"].GetArray()) {
    auto origin_td = origin_row.GetArray();
    for (const auto& v : origin_td) {
      std::string msg = "Problem at source " + std::to_string(i / origin_td.Size()) + " and target " +
                        std::to_string(i % origin_td.Size());
      EXPECT_NEAR(v.GetObject()["distance"].GetFloat(), exp_dists[i], 0.01) << msg;
      if (valid_traffic) {
        EXPECT_TRUE(v.GetObject().HasMember("date_time")) << msg;
        EXPECT_TRUE(v.GetObject()["date_time"] != "") << msg;
      }
      i++;
    }
  }
  const std::string algo = result["algorithm"].GetString();
  const std::string exp_algo = matrix_type == MatrixType::Cost ? "costmatrix" : "timedistancematrix";
  EXPECT_EQ(algo, exp_algo);
}
} // namespace

class MatrixTest : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize = 100;

    // upper ways are motorways, lower are residential
    // unless traffic is enabled, matrix should prefer the longer motorways
    // provoke shortcuts around the start & end
    const std::string ascii_map = R"(
          A-----B
          2     |
          |     |
          1     |
 E--F--G--H     I--J--K--L
          |     |
          C-----D
    )";

    const gurka::ways ways = {{"HA", {{"highway", "motorway"}}},
                              {"AB", {{"highway", "motorway"}}},
                              {"BI", {{"highway", "motorway"}}},
                              {"HC", {{"highway", "residential"}}},
                              {"CD", {{"highway", "residential"}}},
                              {"DI", {{"highway", "residential"}}},
                              {"EF", {{"highway", "primary"}}},
                              {"FG", {{"highway", "motorway"}, {"name", "Left Street"}}},
                              {"GH", {{"highway", "motorway"}, {"name", "Left Street"}}},
                              {"IJ", {{"highway", "motorway"}, {"name", "Right Street"}}},
                              {"JK", {{"highway", "motorway"}, {"name", "Right Street"}}},
                              {"KL", {{"highway", "primary"}}}};

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/matrix_traffic_allowed",
                            {{"service_limits.max_timedep_distance_matrix", "50000"},
                             {"mjolnir.traffic_extract",
                              "test/data/matrix_traffic_allowed/traffic.tar"},
                             {"mjolnir.timezone", "test/data/tz.sqlite"}});

    // verify shortcut edges being built
    // TODO: need to hack HierarchyLimits to allow shortcuts being seen by the algo
    baldr::GraphReader graph_reader(map.config.get_child("mjolnir"));
    auto shortcut_edge_rev = std::get<1>(gurka::findEdgeByNodes(graph_reader, layout, "I", "K"));
    auto shortcut_edge_fwd = std::get<1>(gurka::findEdgeByNodes(graph_reader, layout, "F", "H"));
    EXPECT_TRUE(shortcut_edge_fwd->is_shortcut());
    EXPECT_TRUE(shortcut_edge_rev->is_shortcut());

    test::build_live_traffic_data(map.config);
    test::LiveTrafficCustomize edges_with_traffic = [](baldr::GraphReader& reader,
                                                       baldr::TrafficTile& tile, uint32_t index,
                                                       baldr::TrafficSpeed* current) -> void {
      // update speeds on primary road
      update_traffic_on_edges(reader, tile, index, current, "HA", map, 5);
      update_traffic_on_edges(reader, tile, index, current, "AB", map, 5);
      update_traffic_on_edges(reader, tile, index, current, "BI", map, 5);
    };
    test::customize_live_traffic_data(map.config, edges_with_traffic);
  }
};

gurka::map MatrixTest::map = {};

TEST_F(MatrixTest, MatrixNoTraffic) {
  // no traffic, so this is CostMatrix
  std::string res;
  const auto result = gurka::do_action(Options::sources_to_targets, map, {"E", "L"}, {"E", "L"},
                                       "auto", {}, {}, &res);

  rapidjson::Document res_doc;
  res_doc.Parse(res.c_str());

  // we expect to take the motorways, residential path is 2.8f
  check_matrix(res_doc, {0.0f, 3.2f, 3.2f, 0.0f}, false, MatrixType::Cost);
}

TEST_F(MatrixTest, TDMatrixWithLiveTraffic) {
  std::unordered_map<std::string, std::string> options = {{"/date_time/type", "0"},
                                                          {"/costing_options/auto/speed_types/0",
                                                           "current"}};

  // forward tree
  std::string res;
  auto result = gurka::do_action(Options::sources_to_targets, map, {"E", "L"}, {"E", "L"}, "auto",
                                 options, nullptr, &res);
  rapidjson::Document res_doc;
  res_doc.Parse(res.c_str());
  check_matrix(res_doc, {0.0f, 2.8f, 2.8f, 0.0f}, true, MatrixType::TimeDist);
  ASSERT_EQ(result.info().warnings().size(), 0);

  // forward tree, date_time on the locations, 2nd location has pointless date_time
  options = {{"/sources/0/date_time", "current"},
             {"/sources/1/date_time", "2016-07-03T08:06"},
             {"/costing_options/auto/speed_types/0", "current"}};
  res.erase();
  result = gurka::do_action(Options::sources_to_targets, map, {"E", "L"}, {"E", "L"}, "auto", options,
                            nullptr, &res);
  res_doc.Parse(res.c_str());
  // the second origin can't respect time (no historical data)
  check_matrix(res_doc, {0.0f, 2.8f, 3.2f, 0.0f}, false, MatrixType::TimeDist);
  ASSERT_EQ(result.info().warnings().size(), 0);

  // TODO: there's still a bug in CostMatrix which chooses the wrong correlated edges:
  // https://github.com/valhalla/valhalla/issues/3803
  // this should really take the longer route since it's not using traffic here for TDMatrix
  res.erase();
  result = gurka::do_action(Options::sources_to_targets, map, {"E", "L"}, {"L"}, "auto", options,
                            nullptr, &res);
  res_doc.Parse(res.c_str());
  check_matrix(res_doc, {2.8f, 0.0f}, false, MatrixType::Cost);
  ASSERT_EQ(result.info().warnings().size(), 1);
  ASSERT_EQ(result.info().warnings(0).code(), 201);

  // forward tree, source & target within a single edge
  options = {{"/sources/0/date_time", "current"}, {"/costing_options/auto/speed_types/0", "current"}};
  res.erase();
  result = gurka::do_action(Options::sources_to_targets, map, {"1"}, {"1", "2"}, "auto", options,
                            nullptr, &res);
  res_doc.Parse(res.c_str());
  check_matrix(res_doc, {0.0f, 0.2f}, true, MatrixType::TimeDist);
  ASSERT_EQ(result.info().warnings().size(), 0);
}

TEST_F(MatrixTest, CostMatrixWithLiveTraffic) {
  std::unordered_map<std::string, std::string> options = {{"/date_time/type", "0"},
                                                          {"/costing_options/auto/speed_types/0",
                                                           "current"},
                                                          {"/prioritize_bidirectional", "1"}};

  // forward tree
  std::string res;
  auto result = gurka::do_action(Options::sources_to_targets, map, {"E", "L"}, {"E", "L"}, "auto",
                                 options, nullptr, &res);
  rapidjson::Document res_doc;
  res_doc.Parse(res.c_str());
  check_matrix(res_doc, {0.0f, 2.8f, 2.8f, 0.0f}, true, MatrixType::Cost);
  ASSERT_EQ(result.info().warnings().size(), 0);

  // forward tree, date_time on the locations, 2nd location has pointless date_time
  options = {{"/sources/0/date_time", "current"},
             {"/sources/1/date_time", "2016-07-03T08:06"},
             {"/costing_options/auto/speed_types/0", "current"},
             {"/prioritize_bidirectional", "1"}};
  res.erase();
  result = gurka::do_action(Options::sources_to_targets, map, {"E", "L"}, {"E", "L"}, "auto", options,
                            nullptr, &res);
  res_doc.Parse(res.c_str());
  // the second origin can't respect time (no historical data)
  check_matrix(res_doc, {0.0f, 2.8f, 3.2f, 0.0f}, false, MatrixType::Cost);
  ASSERT_EQ(result.info().warnings().size(), 0);

  // forward tree, source & target within a single edge
  options = {{"/sources/0/date_time", "current"},
             {"/costing_options/auto/speed_types/0", "current"},
             {"/prioritize_bidirectional", "1"}};
  res.erase();
  result = gurka::do_action(Options::sources_to_targets, map, {"1"}, {"1", "2"}, "auto", options,
                            nullptr, &res);
  res_doc.Parse(res.c_str());
  check_matrix(res_doc, {0.0f, 0.2f}, true, MatrixType::Cost);
  ASSERT_EQ(result.info().warnings().size(), 0);

  // bidir matrix allows less targets than sources and date_time on the sources
  options = {{"/sources/0/date_time", "2016-07-03T08:06"},
             {"/sources/1/date_time", "current"},
             {"/costing_options/auto/speed_types/0", "current"},
             {"/prioritize_bidirectional", "1"}};
  res.erase();
  result = gurka::do_action(Options::sources_to_targets, map, {"E", "L"}, {"E"}, "auto", options,
                            nullptr, &res);
  res_doc.Parse(res.c_str());
  check_matrix(res_doc, {0.0f, 2.8f}, true, MatrixType::Cost);
  ASSERT_EQ(result.info().warnings().size(), 0);

  // we don't support date_time on the targets
  options = {{"/targets/0/date_time", "2016-07-03T08:06"},
             {"/costing_options/auto/speed_types/0", "current"},
             {"/prioritize_bidirectional", "1"}};
  res.erase();
  result = gurka::do_action(Options::sources_to_targets, map, {"E", "L"}, {"E"}, "auto", options,
                            nullptr, &res);
  res_doc.Parse(res.c_str());
  // TODO: there's still a bug in CostMatrix which chooses the wrong correlated edges:
  // https://github.com/valhalla/valhalla/issues/3803
  // this should really take the longer route since it's not using traffic here
  check_matrix(res_doc, {0.0f, 2.8f}, false, MatrixType::Cost);
  ASSERT_EQ(result.info().warnings().size(), 1);
  ASSERT_EQ(result.info().warnings(0).code(), 206);
}

TEST_F(MatrixTest, DisallowedRequest) {
  map.config.put("service_limits.max_timedep_distance_matrix", "0");
  const std::unordered_map<std::string, std::string> options = {{"/date_time/type", "0"}};
  const auto result =
      gurka::do_action(Options::sources_to_targets, map, {"E", "L"}, {"E", "L"}, "auto", options);

  ASSERT_EQ(result.info().warnings().size(), 0);
  for (auto& loc : result.options().sources()) {
    ASSERT_TRUE(loc.date_time().empty());
  }
  for (auto& loc : result.options().targets()) {
    ASSERT_TRUE(loc.date_time().empty());
  }

  // revert for other tests
  map.config.put("service_limits.max_timedep_distance_matrix", "50000");
}

TEST_F(MatrixTest, TDSources) {
  // more sources than targets and arrive_by should work
  rapidjson::Document res_doc;
  std::string res;
  std::unordered_map<std::string, std::string> options = {{"/date_time/type", "2"},
                                                          {"/date_time/value", "2016-07-03T08:06"}};
  auto result = gurka::do_action(Options::sources_to_targets, map, {"E", "L"}, {"E"}, "auto", options,
                                 nullptr, &res);
  res_doc.Parse(res.c_str());
  check_matrix(res_doc, {0.0f, 3.2f}, true, MatrixType::TimeDist);
  ASSERT_EQ(result.info().warnings().size(), 0);

  // more targets than sources with date_time.type = 2 are disallowed
  options = {{"/date_time/type", "2"}, {"/date_time/value", "2016-07-03T08:06"}};
  res.erase();
  result = gurka::do_action(Options::sources_to_targets, map, {"E"}, {"E", "L"}, "auto", options,
                            nullptr, &res);
  res_doc.Parse(res.c_str());
  check_matrix(res_doc, {0.0f, 3.2f}, false, MatrixType::Cost);
  ASSERT_EQ(result.info().warnings().Get(0).code(), 202);
  ASSERT_EQ(result.info().warnings().size(), 1);

  // date_time on the sources, disallowed reverse
  options = {{"/sources/0/date_time", "current"},
             {"/sources/1/date_time", "2016-07-03T08:06"},
             {"/costing_options/auto/speed_types/0", "current"}};
  res.erase();
  result = gurka::do_action(Options::sources_to_targets, map, {"E", "L"}, {"E"}, "auto", options,
                            nullptr, &res);
  res_doc.Parse(res.c_str());
  check_matrix(res_doc, {0.0f, 3.2f}, false, MatrixType::Cost);
  ASSERT_EQ(result.info().warnings().Get(0).code(), 201);
  ASSERT_EQ(result.info().warnings().size(), 1);
}

TEST_F(MatrixTest, TDTargets) {
  // more targets than sources are allowed
  rapidjson::Document res_doc;
  std::string res;
  std::unordered_map<std::string, std::string> options = {{"/date_time/type", "0"},
                                                          {"/date_time/value", "current"}};
  auto result = gurka::do_action(Options::sources_to_targets, map, {"E"}, {"E", "L"}, "auto", options,
                                 nullptr, &res);
  res_doc.Parse(res.c_str());
  check_matrix(res_doc, {0.0f, 2.8f}, true, MatrixType::TimeDist);
  ASSERT_EQ(result.info().warnings().size(), 0);

  // more sources than targets with date_time.type = 1 are disallowed
  options = {{"/date_time/type", "1"}, {"/date_time/value", "2016-07-03T08:06"}};
  res.erase();
  result = gurka::do_action(Options::sources_to_targets, map, {"E", "L"}, {"E"}, "auto", options,
                            nullptr, &res);
  res_doc.Parse(res.c_str());
  check_matrix(res_doc, {0.0f, 3.2f}, false, MatrixType::Cost);
  ASSERT_EQ(result.info().warnings().size(), 1);
  ASSERT_EQ(result.info().warnings().Get(0).code(), 201);

  // date_time on the targets, disallowed forward
  options = {{"/targets/0/date_time", "current"}, {"/targets/1/date_time", "2016-07-03T08:06"}};
  res.erase();
  result = gurka::do_action(Options::sources_to_targets, map, {"E"}, {"E", "L"}, "auto", options,
                            nullptr, &res);
  res_doc.Parse(res.c_str());
  check_matrix(res_doc, {0.0f, 3.2f}, false, MatrixType::Cost);
  ASSERT_EQ(result.info().warnings().size(), 1);
  ASSERT_EQ(result.info().warnings().Get(0).code(), 202);
}
