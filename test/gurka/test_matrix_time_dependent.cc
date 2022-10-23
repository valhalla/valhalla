#include "gurka.h"
#include "test.h"

#include <boost/format.hpp>
#include <gtest/gtest.h>

using namespace valhalla;

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

void check_dist(const rapidjson::Document& result, const std::vector<float>& exp_dists) {
  size_t i = 0;
  for (const auto& origin_row : result["sources_to_targets"].GetArray()) {
    auto origin_td = origin_row.GetArray();
    for (const auto& v : origin_td) {
      std::string msg = "Problem at source " + std::to_string(i / origin_td.Size()) + " and target " +
                        std::to_string(i % origin_td.Size());
      EXPECT_EQ(v.GetObject()["distance"].GetFloat(), exp_dists[i]) << msg;
      i++;
    }
  }
}
} // namespace

class MatrixTest : public ::testing::Test {
protected:
  static gurka::map allowed_map;
  static gurka::map disallowed_map;

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
    allowed_map = gurka::buildtiles(layout, ways, {}, {}, "test/data/matrix_traffic_allowed",
                                    {{"service_limits.max_timedep_distance_matrix", "50000"}});
    disallowed_map = gurka::buildtiles(layout, ways, {}, {}, "test/data/matrix_traffic_disallowed",
                                       {{"service_limits.max_timedep_distance_matrix", "0"}});
  }
};

gurka::map MatrixTest::allowed_map = {};
gurka::map MatrixTest::disallowed_map = {};

TEST_F(MatrixTest, MatrixNoTraffic) {
  std::string res;
  const auto result = gurka::do_action(Options::sources_to_targets, allowed_map, {"E", "H"},
                                       {"E", "H"}, "auto", {}, {}, &res);

  rapidjson::Document res_doc;
  res_doc.Parse(res.c_str());

  // we expect to take the motorways, residential path is 1.6f
  check_dist(res_doc, {0.0f, 2.0f, 2.0f, 0.0f});
}

TEST_F(MatrixTest, MatrixWithLiveTraffic) {
  allowed_map.config.put("mjolnir.traffic_extract", "test/data/matrix_traffic_allowed/traffic.tar");

  test::build_live_traffic_data(allowed_map.config);
  std::shared_ptr<valhalla::baldr::GraphReader> reader =
      test::make_clean_graphreader(allowed_map.config.get_child("mjolnir"));

  test::LiveTrafficCustomize edges_with_traffic = [](baldr::GraphReader& reader,
                                                     baldr::TrafficTile& tile, uint32_t index,
                                                     baldr::TrafficSpeed* current) -> void {
    // update speeds on primary road
    update_traffic_on_edges(reader, tile, index, current, "FA", allowed_map, 5);
    update_traffic_on_edges(reader, tile, index, current, "AB", allowed_map, 5);
    update_traffic_on_edges(reader, tile, index, current, "BG", allowed_map, 5);
  };
  test::customize_live_traffic_data(allowed_map.config, edges_with_traffic);

  const std::vector<float> exp_dists = {0.0f, 1.6f, 1.6f, 0.0f};
  std::unordered_map<std::string, std::string> options = {{"/date_time/type", "0"}};

  // forward tree
  std::string res;
  auto result = gurka::do_action(Options::sources_to_targets, allowed_map, {"E", "H"}, {"E", "H"},
                                 "auto", options, reader, &res);
  rapidjson::Document res_doc;
  res_doc.Parse(res.c_str());
  // we expect to take the shorter lower path on residential
  check_dist(res_doc, exp_dists);
  ASSERT_EQ(result.info().warnings().size(), 0);

  // forward tree, date_time on the locations, 2nd location has pointless date_time
  options = {{"/sources/0/date_time", "current"},
             {"/sources/1/date_time", "2016-07-03T08:06"},
             {"/costing_options/auto/speed_types/0", "current"}};
  res.erase();
  result = gurka::do_action(Options::sources_to_targets, allowed_map, {"E", "H"}, {"E", "H"}, "auto",
                            options, nullptr, &res);
  res_doc.Parse(res.c_str());
  // the second origin can't respect time (no historical data)
  check_dist(res_doc, {0.0f, 1.6f, 2.0f, 0.0f});
  ASSERT_EQ(result.info().warnings().size(), 0);

  // reverse tree, no time_info
  res.erase();
  result = gurka::do_action(Options::sources_to_targets, allowed_map, {"E", "H"}, {"H"}, "auto",
                            options, reader, &res);
  res_doc.Parse(res.c_str());
  // without time, we'd expect to take the longer, faster route
  // TODO: be aware that it's actually 2.0f, but CostMatrix has a bug here
  check_dist(res_doc, {1.6f, 0.0f});
  ASSERT_EQ(result.info().warnings().size(), 1);
  ASSERT_EQ(result.info().warnings().Get(0).code(), 401);
}

TEST_F(MatrixTest, DisallowedRequest) {
  const std::unordered_map<std::string, std::string> options = {{"/date_time/type", "0"}};
  const auto result = gurka::do_action(Options::sources_to_targets, disallowed_map, {"E", "H"},
                                       {"E", "H"}, "auto", options);

  ASSERT_EQ(result.info().warnings().size(), 1);
  ASSERT_EQ(result.info().warnings().Get(0).code(), 400);
  for (auto& loc : result.options().sources()) {
    ASSERT_TRUE(loc.date_time().empty());
  }
  for (auto& loc : result.options().targets()) {
    ASSERT_TRUE(loc.date_time().empty());
  }
}

TEST_F(MatrixTest, Sources) {
  // more sources than targets are allowed, but since we only have live traffic it won't have an
  // effect
  rapidjson::Document res_doc;
  std::string res;
  std::unordered_map<std::string, std::string> options = {{"/date_time/type", "2"},
                                                          {"/date_time/value", "2016-07-03T08:06"}};
  auto result = gurka::do_action(Options::sources_to_targets, allowed_map, {"E", "H"}, {"E"}, "auto",
                                 options, nullptr, &res);
  res_doc.Parse(res.c_str());
  check_dist(res_doc, {0.0f, 2.0f});
  ASSERT_EQ(result.info().warnings().size(), 0);

  // more targets than sources with date_time.type = 2 are disallowed
  options = {{"/date_time/type", "2"}, {"/date_time/value", "2016-07-03T08:06"}};
  res.erase();
  result = gurka::do_action(Options::sources_to_targets, allowed_map, {"E"}, {"E", "H"}, "auto",
                            options, nullptr, &res);
  res_doc.Parse(res.c_str());
  check_dist(res_doc, {0.0f, 2.0f});
  ASSERT_EQ(result.info().warnings().Get(0).code(), 402);
  ASSERT_EQ(result.info().warnings().size(), 1);

  // date_time on the sources, disallowed reverse
  options = {{"/sources/0/date_time", "current"}, {"/sources/1/date_time", "2016-07-03T08:06"}};
  res.erase();
  result = gurka::do_action(Options::sources_to_targets, allowed_map, {"E", "H"}, {"E"}, "auto",
                            options, nullptr, &res);
  res_doc.Parse(res.c_str());
  check_dist(res_doc, {0.0f, 2.0f});
  ASSERT_EQ(result.info().warnings().Get(0).code(), 401);
  ASSERT_EQ(result.info().warnings().size(), 1);
}

TEST_F(MatrixTest, Targets) {
  // more targets than sources are allowed, but since we only have live traffic it won't have an
  // effect
  rapidjson::Document res_doc;
  std::string res;
  std::unordered_map<std::string, std::string> options = {{"/date_time/type", "1"},
                                                          {"/date_time/value", "2016-07-03T08:06"}};
  auto result = gurka::do_action(Options::sources_to_targets, allowed_map, {"E"}, {"E", "H"}, "auto",
                                 options, nullptr, &res);
  res_doc.Parse(res.c_str());
  check_dist(res_doc, {0.0f, 2.0f});
  ASSERT_EQ(result.info().warnings().size(), 0);

  // more sources than targets with date_time.type = 1 are disallowed
  options = {{"/date_time/type", "1"}, {"/date_time/value", "2016-07-03T08:06"}};
  res.erase();
  result = gurka::do_action(Options::sources_to_targets, allowed_map, {"E", "H"}, {"E"}, "auto",
                            options, nullptr, &res);
  res_doc.Parse(res.c_str());
  check_dist(res_doc, {0.0f, 2.0f});
  ASSERT_EQ(result.info().warnings().Get(0).code(), 401);
  ASSERT_EQ(result.info().warnings().size(), 1);

  // date_time on the targets, disallowed reverse
  options = {{"/targets/0/date_time", "current"}, {"/targets/1/date_time", "2016-07-03T08:06"}};
  res.erase();
  result = gurka::do_action(Options::sources_to_targets, allowed_map, {"E"}, {"E", "H"}, "auto",
                            options, nullptr, &res);
  res_doc.Parse(res.c_str());
  check_dist(res_doc, {0.0f, 2.0f});
  ASSERT_EQ(result.info().warnings().Get(0).code(), 402);
  ASSERT_EQ(result.info().warnings().size(), 1);
}
