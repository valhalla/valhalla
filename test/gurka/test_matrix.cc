#include "gurka.h"
#include "test.h"
#include <valhalla/midgard/encoded.h>
#include <valhalla/thor/matrixalgorithm.h>

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
                  const std::vector<float>& exp_dists,
                  bool valid_traffic,
                  Matrix::Algorithm matrix_type) {
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
  const std::string& exp_algo = MatrixAlgoToString(matrix_type);
  EXPECT_EQ(algo, exp_algo);
}
} // namespace

class MatrixTrafficTest : public ::testing::Test {
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
    // also turn on the reverse connection search; there's no real test for it
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/matrix_traffic_allowed",
                            {{"service_limits.max_timedep_distance_matrix", "50000"},
                             {"mjolnir.traffic_extract",
                              "test/data/matrix_traffic_allowed/traffic.tar"},
                             {"mjolnir.timezone", "test/data/tz.sqlite"},
                             {"thor.costmatrix_check_reverse_connection", "1"}});

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

gurka::map MatrixTrafficTest::map = {};

TEST_F(MatrixTrafficTest, MatrixNoTraffic) {
  // no traffic, so this is CostMatrix
  std::string res;
  const auto result = gurka::do_action(Options::sources_to_targets, map, {"E", "L"}, {"E", "L"},
                                       "auto", {}, {}, &res);

  rapidjson::Document res_doc;
  res_doc.Parse(res.c_str());

  // we expect to take the motorways, residential path is 2.8f
  check_matrix(res_doc, {0.0f, 3.2f, 3.2f, 0.0f}, false, Matrix::CostMatrix);
}

TEST_F(MatrixTrafficTest, TDMatrixWithLiveTraffic) {
  std::unordered_map<std::string, std::string> options = {{"/date_time/type", "0"},
                                                          {"/costing_options/auto/speed_types/0",
                                                           "current"}};

  // forward tree
  std::string res;
  auto result = gurka::do_action(Options::sources_to_targets, map, {"E", "L"}, {"E", "L"}, "auto",
                                 options, nullptr, &res);
  rapidjson::Document res_doc;
  res_doc.Parse(res.c_str());
  check_matrix(res_doc, {0.0f, 2.8f, 2.8f, 0.0f}, true, Matrix::TimeDistanceMatrix);
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
  check_matrix(res_doc, {0.0f, 2.8f, 3.2f, 0.0f}, false, Matrix::TimeDistanceMatrix);
  ASSERT_EQ(result.info().warnings().size(), 0);

  // TODO: there's still a bug in CostMatrix which chooses the wrong correlated edges:
  // https://github.com/valhalla/valhalla/issues/3803
  // this should really take the longer route since it's not using traffic here for TDMatrix
  res.erase();
  result = gurka::do_action(Options::sources_to_targets, map, {"E", "L"}, {"L"}, "auto", options,
                            nullptr, &res);
  res_doc.Parse(res.c_str());
  check_matrix(res_doc, {2.8f, 0.0f}, false, Matrix::CostMatrix);
  ASSERT_EQ(result.info().warnings().size(), 1);
  ASSERT_EQ(result.info().warnings(0).code(), 201);

  // forward tree, source & target within a single edge
  options = {{"/sources/0/date_time", "current"}, {"/costing_options/auto/speed_types/0", "current"}};
  res.erase();
  result = gurka::do_action(Options::sources_to_targets, map, {"1"}, {"1", "2"}, "auto", options,
                            nullptr, &res);
  res_doc.Parse(res.c_str());
  check_matrix(res_doc, {0.0f, 0.2f}, true, Matrix::TimeDistanceMatrix);
  ASSERT_EQ(result.info().warnings().size(), 0);
}

TEST_F(MatrixTrafficTest, CostMatrixWithLiveTraffic) {
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
  check_matrix(res_doc, {0.0f, 2.8f, 2.8f, 0.0f}, true, Matrix::CostMatrix);
  ASSERT_EQ(result.info().warnings().size(), 0);
  res.erase();

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
  check_matrix(res_doc, {0.0f, 2.8f, 3.2f, 0.0f}, false, Matrix::CostMatrix);
  ASSERT_EQ(result.info().warnings().size(), 0);

  // forward tree, source & target within a single edge
  options = {{"/sources/0/date_time", "current"},
             {"/costing_options/auto/speed_types/0", "current"},
             {"/prioritize_bidirectional", "1"}};
  res.erase();
  result = gurka::do_action(Options::sources_to_targets, map, {"1"}, {"1", "2"}, "auto", options,
                            nullptr, &res);
  res_doc.Parse(res.c_str());
  check_matrix(res_doc, {0.0f, 0.2f}, true, Matrix::CostMatrix);
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
  check_matrix(res_doc, {0.0f, 2.8f}, true, Matrix::CostMatrix);
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
  check_matrix(res_doc, {0.0f, 2.8f}, false, Matrix::CostMatrix);
  ASSERT_EQ(result.info().warnings().size(), 1);
  ASSERT_EQ(result.info().warnings(0).code(), 206);
}

TEST_F(MatrixTrafficTest, DisallowedRequest) {
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

TEST_F(MatrixTrafficTest, TDSources) {
  // more sources than targets and arrive_by should work
  rapidjson::Document res_doc;
  std::string res;
  std::unordered_map<std::string, std::string> options = {{"/date_time/type", "2"},
                                                          {"/date_time/value", "2016-07-03T08:06"}};
  auto result = gurka::do_action(Options::sources_to_targets, map, {"E", "L"}, {"E"}, "auto", options,
                                 nullptr, &res);
  res_doc.Parse(res.c_str());
  check_matrix(res_doc, {0.0f, 3.2f}, true, Matrix::TimeDistanceMatrix);
  ASSERT_EQ(result.info().warnings().size(), 0);

  // more targets than sources with date_time.type = 2 are disallowed
  options = {{"/date_time/type", "2"}, {"/date_time/value", "2016-07-03T08:06"}};
  res.erase();
  result = gurka::do_action(Options::sources_to_targets, map, {"E"}, {"E", "L"}, "auto", options,
                            nullptr, &res);
  res_doc.Parse(res.c_str());
  check_matrix(res_doc, {0.0f, 3.2f}, false, Matrix::CostMatrix);
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
  check_matrix(res_doc, {0.0f, 3.2f}, false, Matrix::CostMatrix);
  ASSERT_EQ(result.info().warnings().Get(0).code(), 201);
  ASSERT_EQ(result.info().warnings().size(), 1);
}

TEST_F(MatrixTrafficTest, TDTargets) {
  // more targets than sources are allowed
  rapidjson::Document res_doc;
  std::string res;
  std::unordered_map<std::string, std::string> options = {{"/date_time/type", "0"},
                                                          {"/date_time/value", "current"}};
  auto result = gurka::do_action(Options::sources_to_targets, map, {"E"}, {"E", "L"}, "auto", options,
                                 nullptr, &res);
  res_doc.Parse(res.c_str());
  check_matrix(res_doc, {0.0f, 2.8f}, true, Matrix::TimeDistanceMatrix);
  ASSERT_EQ(result.info().warnings().size(), 0);

  // more sources than targets with date_time.type = 1 are disallowed
  options = {{"/date_time/type", "1"}, {"/date_time/value", "2016-07-03T08:06"}};
  res.erase();
  result = gurka::do_action(Options::sources_to_targets, map, {"E", "L"}, {"E"}, "auto", options,
                            nullptr, &res);
  res_doc.Parse(res.c_str());
  check_matrix(res_doc, {0.0f, 3.2f}, false, Matrix::CostMatrix);
  ASSERT_EQ(result.info().warnings().size(), 1);
  ASSERT_EQ(result.info().warnings().Get(0).code(), 201);

  // date_time on the targets, disallowed forward
  options = {{"/targets/0/date_time", "current"}, {"/targets/1/date_time", "2016-07-03T08:06"}};
  res.erase();
  result = gurka::do_action(Options::sources_to_targets, map, {"E"}, {"E", "L"}, "auto", options,
                            nullptr, &res);
  res_doc.Parse(res.c_str());
  check_matrix(res_doc, {0.0f, 3.2f}, false, Matrix::CostMatrix);
  ASSERT_EQ(result.info().warnings().size(), 1);
  ASSERT_EQ(result.info().warnings().Get(0).code(), 202);
}

TEST(StandAlone, CostMatrixDeadends) {
  // ABI has a turn restriction
  // F is a blocking node
  const std::string ascii_map = R"(
       I
       |
    A--B--C
       |  |
       |  D
       E
      1|
       |
       F--H
       .
       G

  )";
  // clang-format off
  const gurka::ways ways = {
      {"AB", {{"highway", "residential"}, {"oneway", "yes"}}}, 
      {"BC", {{"highway", "residential"}}},
      {"CD", {{"highway", "residential"}}},
      {"BE", {{"highway", "residential"}}},
      {"EF", {{"highway", "residential"}}},
      {"FH", {{"highway", "residential"}}},
      {"FG", {{"highway", "residential"}}},
      {"BI", {{"highway", "residential"}}}
  };
  // clang-format on
  const gurka::nodes nodes = {{"F", {{"barrier", "block"}}}};
  const gurka::relations relations = {
      {{
           {gurka::way_member, "AB", "from"},
           {gurka::node_member, "B", "via"},
           {gurka::way_member, "BI", "to"},
       },
       {
           {"type", "restriction"},
           {"restriction", "no_left_turn"},
       }},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);

  auto map = gurka::buildtiles(layout, ways, nodes, relations,
                               VALHALLA_BUILD_DIR "test/data/costmatrix_deadends");

  rapidjson::Document res_doc;
  std::string res;

  // test that the we're taking the u-turn at D to get from A -> I
  // because of the ABI turn restriction
  {
    auto result = gurka::do_action(valhalla::Options::sources_to_targets, map, {"A"}, {"I"}, "auto",
                                   {}, nullptr, &res);
    res_doc.Parse(res.c_str());
    check_matrix(res_doc, {1.5f}, false, Matrix::CostMatrix);
    res.erase();
  }

  // then we force to go 1 -> F to hit a blocking node, doing a u-turn and go back the same way
  {
    auto result = gurka::do_action(valhalla::Options::sources_to_targets, map, {"1"}, {"B"}, "auto",
                                   {{"/sources/0/preferred_side", "opposite"}}, nullptr, &res);
    res_doc.Parse(res.c_str());
    check_matrix(res_doc, {0.8f}, false, Matrix::CostMatrix);
  }

  // throw if no connection can be found at all
  try {
    auto result = gurka::do_action(valhalla::Options::sources_to_targets, map, {"C"}, {"A"}, "auto");
    FAIL() << "No connection found should have thrown";
  } catch (const valhalla_exception_t& e) { EXPECT_EQ(e.code, 442); }
}

TEST(StandAlone, CostMatrixShapes) {
  // keep the same order in the map.nodes for encoding easily
  const std::string ascii_map = R"(
    A-B-C-D-E-F-G-H-I-J-K-------L
  )";
  // clang-format off
  const gurka::ways ways = {
      {"ABCDE", {{"highway", "residential"}}}, 
      {"EFGHIJK", {{"highway", "residential"}}},
      {"KL", {{"highway", "residential"}}},
  };
  // clang-format on

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);

  auto map =
      gurka::buildtiles(layout, ways, {}, {}, VALHALLA_BUILD_DIR "test/data/costmatrix_shapes");

  // points of all nodes
  std::vector<PointLL> vertices;
  for (const auto& node : map.nodes) {
    vertices.emplace_back(node.second);
  }

  std::string res;
  rapidjson::Document res_doc;

  // no shapes if not specified or "none"
  auto result = gurka::do_action(valhalla::Options::sources_to_targets, map, {"A"}, {"L"}, "auto", {},
                                 nullptr, &res);
  EXPECT_EQ(result.matrix().shapes(0), "");
  EXPECT_FALSE(res_doc.Parse(res.c_str())["sources_to_targets"]
                   .GetArray()[0]
                   .GetArray()[0]
                   .GetObject()
                   .HasMember("shape"));
  res.erase();

  std::unordered_map<std::string, std::string> options = {{"/shape_format", "no_shape"}};

  result = gurka::do_action(valhalla::Options::sources_to_targets, map, {"A"}, {"L"}, "auto", options,
                            nullptr, &res);
  EXPECT_EQ(result.matrix().shapes(0), "");
  EXPECT_FALSE(res_doc.Parse(res.c_str())["sources_to_targets"]
                   .GetArray()[0]
                   .GetArray()[0]
                   .GetObject()
                   .HasMember("shape"));
  res.erase();

  // polyline5/6

  options["/shape_format"] = "polyline5";
  auto encoded = encode<std::vector<PointLL>>(vertices, 1e5);
  result = gurka::do_action(valhalla::Options::sources_to_targets, map, {"A"}, {"L"}, "auto", options,
                            nullptr, &res);
  EXPECT_EQ(result.matrix().shapes(0), encoded);
  EXPECT_EQ(res_doc.Parse(res.c_str())["sources_to_targets"]
                .GetArray()[0]
                .GetArray()[0]
                .GetObject()["shape"],
            encoded);
  res.erase();

  options["/shape_format"] = "polyline6";
  encoded = encode<std::vector<PointLL>>(vertices, 1e6);
  result = gurka::do_action(valhalla::Options::sources_to_targets, map, {"A"}, {"L"}, "auto", options,
                            nullptr, &res);
  EXPECT_EQ(result.matrix().shapes(0), encoded);
  EXPECT_EQ(res_doc.Parse(res.c_str())["sources_to_targets"]
                .GetArray()[0]
                .GetArray()[0]
                .GetObject()["shape"],
            encoded);
  res.erase();

  // geojson

  options["/shape_format"] = "geojson";
  result = gurka::do_action(valhalla::Options::sources_to_targets, map, {"A"}, {"L"}, "auto", options,
                            nullptr, &res);
  EXPECT_EQ(result.matrix().shapes(0), encoded); // has the encoded polyline6 in PBF
  const auto& gj_shp = res_doc.Parse(res.c_str())["sources_to_targets"]
                           .GetArray()[0]
                           .GetArray()[0]
                           .GetObject()["shape"];
  EXPECT_TRUE(gj_shp.IsObject());
  EXPECT_EQ(gj_shp["coordinates"].GetArray().Size(), 12);
  EXPECT_EQ(gj_shp["type"], "LineString");
  res.erase();

  // trivial route
  // has a bug: https://github.com/valhalla/valhalla/issues/4433, but it's band-aided for now
  // floating point crap makes this fail though, it adds a tiny little bit on both ends, resulting in
  // 4 (not 2) points

  options["/shape_format"] = "polyline6";
  encoded = encode<std::vector<PointLL>>({map.nodes["G"], map.nodes["H"]});
  result = gurka::do_action(valhalla::Options::sources_to_targets, map, {"G"}, {"H"}, "auto", options,
                            nullptr, &res);
  /*
  EXPECT_EQ(result.matrix().shapes(0), encoded);
  EXPECT_EQ(res_doc.Parse(res.c_str())["sources_to_targets"].GetArray()[0].GetArray()[0].GetObject()["shape"],
  encoded);
  */
  res.erase();

  // trivial route reverse
  // has a bug: https://github.com/valhalla/valhalla/issues/4433, but it's band-aided for now

  options["/shape_format"] = "polyline6";
  encoded = encode<std::vector<PointLL>>({map.nodes["H"], map.nodes["G"]});
  result = gurka::do_action(valhalla::Options::sources_to_targets, map, {"H"}, {"G"}, "auto", options,
                            nullptr, &res);
  /*
  EXPECT_EQ(result.matrix().shapes(0), encoded);
  EXPECT_EQ(res_doc.Parse(res.c_str())["sources_to_targets"].GetArray()[0].GetArray()[0].GetObject()["shape"],
  encoded);
  */
  res.erase();

  // timedistancematrix

  options["/shape_format"] = "geojson";
  result = gurka::do_action(valhalla::Options::sources_to_targets, map, {"A"}, {"L"}, "pedestrian",
                            options, nullptr, &res);
  EXPECT_FALSE(res_doc.Parse(res.c_str())["sources_to_targets"]
                   .GetArray()[0]
                   .GetArray()[0]
                   .GetObject()
                   .HasMember("shape"));
  EXPECT_EQ(result.info().warnings().size(), 1);
  EXPECT_EQ(result.info().warnings(0).code(), 207);
  EXPECT_EQ(res_doc.Parse(res.c_str())["warnings"].GetArray().Size(), 1);
  EXPECT_EQ(res_doc.Parse(res.c_str())["warnings"].GetArray()[0].GetObject()["code"].GetUint64(),
            207);

  res.erase();
}

class DateTimeTest : public ::testing::Test {
protected:
  // check both with and without time zones present
  static gurka::map map;
  static gurka::map map_tz;

  static void SetUpTestSuite() {
    constexpr double gridsize = 1500;

    // ~ are approximate time zone crossings
    const std::string ascii_map = R"(
      A----------B
      |          |
      C          D
      |          |
      ~          ~
      |          |
      |          |
      E          F
      |          |
      G----------H
    )";

    const gurka::ways ways = {{"AC", {{"highway", "residential"}}},
                              {"CE", {{"highway", "residential"}}},
                              {"EG", {{"highway", "residential"}}},
                              {"GH", {{"highway", "residential"}}},
                              {"HF", {{"highway", "residential"}}},
                              {"FD", {{"highway", "residential"}}},
                              {"DB", {{"highway", "residential"}}},
                              {"BA", {{"highway", "residential"}}}};

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize, {-8.5755, 42.1079});
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/time_zone_matrix_no_tz");
    map_tz = gurka::buildtiles(layout, ways, {}, {}, "test/data/time_zone_matrix",
                               {{"mjolnir.timezone", "test/data/tz.sqlite"},
                                {"service_limits.max_timedep_distance_matrix", "50000"}});
  }
};
gurka::map DateTimeTest::map = {};
gurka::map DateTimeTest::map_tz = {};

TEST_F(DateTimeTest, DepartAtCostMatrix) {
  rapidjson::Document res_doc;
  std::string res;
  {
    auto api = gurka::do_action(valhalla::Options::sources_to_targets, map_tz, {"A", "G"}, {"A", "G"},
                                "auto",
                                {{"/prioritize_bidirectional", "1"},
                                 {"/date_time/type", "1"},
                                 {"/date_time/value", "2020-10-30T09:00"}},
                                nullptr, &res);

    res_doc.Parse(res.c_str());

    // sanity check
    EXPECT_EQ(api.matrix().algorithm(), Matrix::CostMatrix);

    EXPECT_TRUE(
        res_doc["sources_to_targets"].GetArray()[0].GetArray()[0].GetObject().HasMember("date_time"));

    EXPECT_EQ(res_doc["sources_to_targets"]
                  .GetArray()[0]
                  .GetArray()[0]
                  .GetObject()["time_zone_offset"],
              "+01:00");

    EXPECT_EQ(res_doc["sources_to_targets"]
                  .GetArray()[0]
                  .GetArray()[1]
                  .GetObject()["time_zone_offset"],
              "+00:00");

    EXPECT_EQ(res_doc["sources_to_targets"]
                  .GetArray()[1]
                  .GetArray()[0]
                  .GetObject()["time_zone_offset"],
              "+01:00");

    EXPECT_EQ(res_doc["sources_to_targets"]
                  .GetArray()[1]
                  .GetArray()[1]
                  .GetObject()["time_zone_offset"],
              "+00:00");

    EXPECT_EQ(res_doc["sources_to_targets"].GetArray()[0].GetArray()[0].GetObject()["time_zone_name"],
              "Europe/Madrid");

    EXPECT_EQ(res_doc["sources_to_targets"].GetArray()[0].GetArray()[1].GetObject()["time_zone_name"],
              "Europe/Lisbon");

    EXPECT_EQ(res_doc["sources_to_targets"].GetArray()[1].GetArray()[0].GetObject()["time_zone_name"],
              "Europe/Madrid");

    EXPECT_EQ(res_doc["sources_to_targets"].GetArray()[1].GetArray()[1].GetObject()["time_zone_name"],
              "Europe/Lisbon");
  }
}

TEST_F(DateTimeTest, DepartAtTimeDistanceMatrix) {
  rapidjson::Document res_doc;
  std::string res;
  {
    auto api = gurka::do_action(valhalla::Options::sources_to_targets, map_tz, {"A", "G"}, {"A", "G"},
                                "bicycle",
                                {{"/date_time/type", "1"}, {"/date_time/value", "2020-10-30T09:00"}},
                                nullptr, &res);

    res_doc.Parse(res.c_str());

    // sanity check
    EXPECT_EQ(api.matrix().algorithm(), Matrix::TimeDistanceMatrix);

    EXPECT_TRUE(
        res_doc["sources_to_targets"].GetArray()[0].GetArray()[0].GetObject().HasMember("date_time"));

    EXPECT_EQ(res_doc["sources_to_targets"]
                  .GetArray()[0]
                  .GetArray()[0]
                  .GetObject()["time_zone_offset"],
              "+01:00");

    EXPECT_EQ(res_doc["sources_to_targets"]
                  .GetArray()[0]
                  .GetArray()[1]
                  .GetObject()["time_zone_offset"],
              "+00:00");

    EXPECT_EQ(res_doc["sources_to_targets"]
                  .GetArray()[1]
                  .GetArray()[0]
                  .GetObject()["time_zone_offset"],
              "+01:00");

    EXPECT_EQ(res_doc["sources_to_targets"]
                  .GetArray()[1]
                  .GetArray()[1]
                  .GetObject()["time_zone_offset"],
              "+00:00");

    EXPECT_EQ(res_doc["sources_to_targets"].GetArray()[0].GetArray()[0].GetObject()["time_zone_name"],
              "Europe/Madrid");

    EXPECT_EQ(res_doc["sources_to_targets"].GetArray()[0].GetArray()[1].GetObject()["time_zone_name"],
              "Europe/Lisbon");

    EXPECT_EQ(res_doc["sources_to_targets"].GetArray()[1].GetArray()[0].GetObject()["time_zone_name"],
              "Europe/Madrid");

    EXPECT_EQ(res_doc["sources_to_targets"].GetArray()[1].GetArray()[1].GetObject()["time_zone_name"],
              "Europe/Lisbon");
  }
}

TEST_F(DateTimeTest, NoTimeZone) {
  rapidjson::Document res_doc;
  std::string res;
  {
    auto api =
        gurka::do_action(valhalla::Options::sources_to_targets, map, {"A", "G"}, {"A", "G"}, "auto",
                         {{"/prioritize_bidirectional", "true"},
                          {"/date_time/type", "1"},
                          {"/date_time/value", "2020-10-30T09:00"}},
                         nullptr, &res);
    res_doc.Parse(res.c_str());

    EXPECT_FALSE(
        res_doc["sources_to_targets"].GetArray()[0].GetArray()[0].GetObject().HasMember("date_time"));

    EXPECT_FALSE(res_doc["sources_to_targets"].GetArray()[0].GetArray()[0].GetObject().HasMember(
        "time_zone_offset"));

    EXPECT_FALSE(res_doc["sources_to_targets"].GetArray()[0].GetArray()[0].GetObject().HasMember(
        "time_zone_name"));
  }
}

TEST(StandAlone, MatrixSecondPass) {
  // from no-thru to no-thru should trigger a second pass
  // JL has a forward destination-only,
  //   so K -> I also triggers second pass (see oneway at HK), but I -> K doesn't (no oneway)
  const std::string ascii_map = R"(
    A---B           I---J
    |   |           |   |
    |   E---F---G---H   |
    |   |           ↓   |
    C---D           K---L
  )";

  gurka::ways ways;
  for (const auto& node_pair :
       {"AB", "BE", "AC", "CD", "DE", "EF", "FG", "GH", "HI", "IJ", "JL", "HK", "KL"}) {
    ways[node_pair] = {{"highway", "residential"}};
  }
  ways["JL"].emplace("motor_vehicle", "destination");
  ways["HK"].emplace("oneway", "true");

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 50);
  const auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/matrix_second_pass",
                                     {{"thor.costmatrix_allow_second_pass", "1"}});
  baldr::GraphReader graph_reader(map.config.get_child("mjolnir"));

  // Make sure the relevant edges are actually built as no-thru
  auto FE_edge = std::get<1>(gurka::findEdgeByNodes(graph_reader, layout, "F", "E"));
  auto GH_edge = std::get<1>(gurka::findEdgeByNodes(graph_reader, layout, "G", "H"));
  auto JL_edge = std::get<1>(gurka::findEdgeByNodes(graph_reader, layout, "J", "L"));
  EXPECT_TRUE(FE_edge->not_thru());
  EXPECT_TRUE(GH_edge->not_thru());
  EXPECT_TRUE(JL_edge->destonly());

  // Simple single route from no-thru to no-thru
  {
    auto api = gurka::do_action(valhalla::Options::sources_to_targets, map, {"A"}, {"J"}, "auto");
    EXPECT_GT(api.matrix().times(0), 0.f);
    EXPECT_TRUE(api.matrix().second_pass(0));
    EXPECT_TRUE(api.info().warnings(0).description().find('0') != std::string::npos);
  }

  // I -> K (idx 1) should pass on the first try
  // K -> I (idx 2) should need a second pass
  {
    auto api =
        gurka::do_action(valhalla::Options::sources_to_targets, map, {"I", "K"}, {"I", "K"}, "auto");
    EXPECT_GT(api.matrix().times(1), 0.f);
    EXPECT_FALSE(api.matrix().second_pass(1));
    EXPECT_GT(api.matrix().times(2), 0.f);
    EXPECT_TRUE(api.matrix().second_pass(2));
    EXPECT_GT(api.matrix().distances(2), api.matrix().distances(1));
    EXPECT_GT(api.matrix().times(2), api.matrix().times(1));

    // I -> I & K -> K shouldn't be processed a second time either
    EXPECT_FALSE(api.matrix().second_pass(0));
    EXPECT_FALSE(api.matrix().second_pass(3));
    EXPECT_TRUE(api.info().warnings(0).description().find('2') != std::string::npos);
  }
}

TEST(StandAlone, CostMatrixTrivialRoutes) {
  const std::string ascii_map = R"(
    A---B--2->-1--C---D
        |         |
        6         5
        |         |
        E--3---4--F
  )";
  const gurka::ways ways = {
      {"AB", {{"highway", "residential"}}}, {"BC", {{"highway", "residential"}, {"oneway", "yes"}}},
      {"CD", {{"highway", "residential"}}}, {"BE", {{"highway", "residential"}}},
      {"EF", {{"highway", "residential"}}}, {"FC", {{"highway", "residential"}}},
  };
  auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map =
      gurka::buildtiles(layout, ways, {}, {}, VALHALLA_BUILD_DIR "test/data/costmatrix_trivial");

  std::unordered_map<std::string, std::string> options = {{"/shape_format", "polyline6"}};

  // test the against-oneway case
  {
    auto matrix =
        gurka::do_action(valhalla::Options::sources_to_targets, map, {"1"}, {"2"}, "auto", options);
    EXPECT_EQ(matrix.matrix().distances(0), 2400);

    std::vector<PointLL> oneway_vertices;
    for (auto& node : {"1", "C", "F", "E", "B", "2"}) {
      oneway_vertices.push_back(layout[node]);
    }
    auto encoded = encode<std::vector<PointLL>>(oneway_vertices, 1e6);
    EXPECT_EQ(matrix.matrix().shapes(0), encoded);
  }

  // test the oneway case
  {
    auto matrix =
        gurka::do_action(valhalla::Options::sources_to_targets, map, {"2"}, {"1"}, "auto", options);
    EXPECT_EQ(matrix.matrix().distances(0), 400);

    auto encoded = encode<std::vector<PointLL>>({layout["2"], layout["1"]}, 1e6);
    EXPECT_EQ(matrix.matrix().shapes(0), encoded);
  }

  // test the normal trivial case
  {
    auto matrix =
        gurka::do_action(valhalla::Options::sources_to_targets, map, {"3"}, {"4"}, "auto", options);
    EXPECT_EQ(matrix.matrix().distances(0), 400);

    auto encoded = encode<std::vector<PointLL>>({layout["3"], layout["4"]}, 1e6);
    EXPECT_EQ(matrix.matrix().shapes(0), encoded);
  }

  // test trivial case via connecting edge
  {
    auto matrix =
        gurka::do_action(valhalla::Options::sources_to_targets, map, {"4"}, {"5"}, "auto", options);
    EXPECT_EQ(matrix.matrix().distances(0), 500);

    auto encoded = encode<std::vector<PointLL>>({layout["4"], layout["F"], layout["5"]}, 1e6);
    EXPECT_EQ(matrix.matrix().shapes(0), encoded);
  }
}

TEST(StandAlone, HGVNoAccessPenalty) {
  // if hgv_no_penalty is on we should still respect the maxweight restriction on CD
  // so we should take the next-best hgv=no edge with JK
  const std::string ascii_map = R"(
    A-1-------B----C----D----E--2-------F
                   |    |
                   J----K
                   |    |             
                   |    |
                   L----M
           )";

  const gurka::ways ways = {
      {"AB", {{"highway", "residential"}, {"hgv", "no"}}},
      {"BC", {{"highway", "residential"}}},
      {"CD", {{"highway", "residential"}, {"hgv", "no"}, {"maxweight", "3.5"}}},
      {"DE", {{"highway", "residential"}}},
      {"EF", {{"highway", "residential"}, {"hgv", "no"}}},
      {"CJ", {{"highway", "residential"}}},
      {"JK", {{"highway", "residential"}, {"hgv", "no"}}},
      {"JLMK", {{"highway", "residential"}}},
      {"KD", {{"highway", "residential"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  gurka::map map = gurka::buildtiles(layout, ways, {}, {}, "test/data/hgv_no_access_penalty",
                                     {{"service_limits.max_timedep_distance_matrix", "50000"}});

  std::unordered_map<std::string, std::string> cost_matrix =
      {{"/costing_options/truck/hgv_no_access_penalty", "2000"},
       {"/sources/0/date_time", "2024-03-20T09:00"},
       {"/prioritize_bidirectional", "1"}};
  std::unordered_map<std::string, std::string> td_matrix =
      {{"/costing_options/truck/hgv_no_access_penalty", "2000"},
       {"/sources/0/date_time", "2024-03-20T09:00"}};

  // do both costmatrix & timedistancematrix
  std::vector<std::unordered_map<std::string, std::string>> options = {cost_matrix, td_matrix};
  for (auto& truck_options : options) {

    // by default, take the detour via LM
    // NOTE, we're not snapping to the hgv=no edges either
    {
      auto matrix =
          gurka::do_action(valhalla::Options::sources_to_targets, map, {"1"}, {"2"}, "truck");
      EXPECT_EQ(matrix.matrix().distances(0), 2500);
    }

    // with a high hgv_no_penalty also take the detour via LM, but do snap to the hgv=no edges
    {
      auto matrix = gurka::do_action(valhalla::Options::sources_to_targets, map, {"1"}, {"2"},
                                     "truck", truck_options);
      // TODO(nils): timedistancematrix seems to have a tiny bug where time options result in slightly
      // less distances
      EXPECT_NEAR(matrix.matrix().distances(0), 3600, 2);
    }

    // with a low hgv_no_penalty take the JK edge
    {
      truck_options["/costing_options/truck/hgv_no_access_penalty"] = "10";
      auto matrix = gurka::do_action(valhalla::Options::sources_to_targets, map, {"1"}, {"2"},
                                     "truck", truck_options);
      // TODO(nils): timedistancematrix seems to have a tiny bug where time options result in slightly
      // less distances
      EXPECT_NEAR(matrix.matrix().distances(0), 3000, 2);
    }
  }
}
