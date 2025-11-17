#include "baldr/rapidjson_utils.h"
#include "gurka.h"
#include "loki/worker.h"
#include "midgard/encoded.h"
#include "proto/api.pb.h"
#include "test.h"
#include "thor/worker.h"
#include "valhalla/proto_conversions.h"
#include "valhalla/worker.h"

#include <gtest/gtest.h>

using namespace valhalla;
using namespace valhalla::thor;
using namespace valhalla::midgard;
using namespace valhalla::sif;
using namespace valhalla::baldr;

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
      A-------1---B-----C-----D-----E-----F---2------G
    )";

    const gurka::ways ways = {
        {"AB", {{"highway", "primary"}, {"maxspeed", "100"}}},
        {"BC", {{"highway", "primary"}, {"maxspeed", "100"}}},
        {"CD", {{"highway", "primary"}, {"maxspeed", "100"}}},
        {"DE", {{"highway", "primary"}, {"maxspeed", "100"}}},
        {"EF", {{"highway", "primary"}, {"maxspeed", "100"}}},
        {"FG", {{"highway", "primary"}, {"maxspeed", "100"}}},
    };

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
    // also turn on the reverse connection search; there's no real test for it
    map = gurka::buildtiles(layout, ways, {}, {}, VALHALLA_BUILD_DIR "test/data/matrix_traffic_time",
                            {{"service_limits.max_timedep_distance_matrix", "50000"},
                             {"mjolnir.traffic_extract",
                              VALHALLA_BUILD_DIR "test/data/matrix_traffic_time/traffic.tar"},
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

class SimpleCost final : public valhalla::sif::DynamicCost {
public:
  // keep track of the time information with which edge cost was called
  mutable std::vector<baldr::TimeInfo> time_infos;

  /**
   * Constructor.
   * @param  options Request options in a pbf
   */
  SimpleCost(const Costing& options) : DynamicCost(options, sif::TravelMode::kDrive, kAutoAccess) {
  }

  ~SimpleCost() {
  }

  bool Allowed(const DirectedEdge* edge,
               const bool /*is_dest*/,
               const EdgeLabel& pred,
               const graph_tile_ptr& /*tile*/,
               const GraphId& edgeid,
               const uint64_t /*current_time*/,
               const uint32_t /*tz_index*/,
               uint8_t& /*restriction_idx*/,
               uint8_t& /*destonly_access_restr_mask*/) const override {
    if (!IsAccessible(edge) || (!pred.deadend() && pred.opp_local_idx() == edge->localedgeidx()) ||
        (pred.restrictions() & (1 << edge->localedgeidx())) ||
        edge->surface() == Surface::kImpassable || IsUserAvoidEdge(edgeid) ||
        (!allow_destination_only_ && !pred.destonly() && edge->destonly())) {
      return false;
    }
    return true;
  }

  bool AllowedReverse(const DirectedEdge* edge,
                      const EdgeLabel& pred,
                      const DirectedEdge* opp_edge,
                      const graph_tile_ptr& /*tile*/,
                      const GraphId& opp_edgeid,
                      const uint64_t /*current_time*/,
                      const uint32_t /*tz_index*/,
                      uint8_t& /*restriction_idx*/,
                      uint8_t& /*destonly_access_restr_mask*/) const override {
    if (!IsAccessible(opp_edge) ||
        (!pred.deadend() && pred.opp_local_idx() == edge->localedgeidx()) ||
        (opp_edge->restrictions() & (1 << pred.opp_local_idx())) ||
        opp_edge->surface() == Surface::kImpassable || IsUserAvoidEdge(opp_edgeid) ||
        (!allow_destination_only_ && !pred.destonly() && opp_edge->destonly())) {
      return false;
    }
    return true;
  }

  Cost EdgeCost(const baldr::DirectedEdge* /*edge*/,
                const baldr::TransitDeparture* /*departure*/,
                const uint32_t /*curr_time*/) const override {
    throw std::runtime_error("We shouldnt be testing transit edges");
  }

  Cost EdgeCost(const DirectedEdge* edge,
                const GraphId& /*edgeid*/,
                const graph_tile_ptr& /*tile*/,
                const baldr::TimeInfo& time_info,
                uint8_t& /*flow_sources*/) const override {
    time_infos.push_back(time_info);
    float sec = static_cast<float>(edge->length());
    return {sec / 10.0f, sec};
  }

  Cost TransitionCost(const DirectedEdge* /*edge*/,
                      const NodeInfo* /*node*/,
                      const EdgeLabel& /*pred*/,
                      const graph_tile_ptr& /*tile*/,
                      const std::function<baldr::LimitedGraphReader()>& /*reader_getter*/
  ) const override {
    return {5.0f, 5.0f};
  }

  Cost TransitionCostReverse(const uint32_t /*idx*/,
                             const NodeInfo* /*node*/,
                             const DirectedEdge* /*opp_edge*/,
                             const DirectedEdge* /*opp_pred_edge*/,
                             const graph_tile_ptr& /*tile*/,
                             const baldr::GraphId& /*edge_id*/,
                             const std::function<baldr::LimitedGraphReader()>& /*reader_getter*/,
                             const bool /*has_measured_speed*/,
                             const InternalTurn /*internal_turn*/) const override {
    return {5.0f, 5.0f};
  }

  float AStarCostFactor() const override {
    return 0.1f;
  }

  bool Allowed(const baldr::DirectedEdge* edge, const graph_tile_ptr&, uint16_t) const override {
    auto access_mask = (ignore_access_ ? kAllAccess : access_mask_);
    bool accessible = (edge->forwardaccess() & access_mask) ||
                      (ignore_oneways_ && (edge->reverseaccess() & access_mask));
    if (edge->is_shortcut() || !accessible)
      return 0.0f;
    else {
      return 1.0f;
    }
  }
};

std::shared_ptr<SimpleCost> CreateSimpleCost(const Costing& options) {
  auto dcost = std::make_shared<SimpleCost>(options);
  return dcost;
}

TEST_F(MatrixTrafficTest, MatrixNoTraffic) {
  // no traffic, so this is CostMatrix
  std::string res;
  const auto result = gurka::do_action(Options::sources_to_targets, map, {"1", "2"}, {"1", "2"},
                                       "auto", {}, {}, &res);

  rapidjson::Document res_doc;
  res_doc.Parse(res.c_str());

  check_matrix(res_doc, {0.0f, 115.0f, 115.0f, 0.0f}, false, "time", Matrix::CostMatrix);
}

TEST_F(MatrixTrafficTest, TDMatrixLiveTraffic) {
  std::unordered_map<std::string, std::string> options = {{"/date_time/type", "0"},
                                                          {"/costing_options/auto/speed_types/0",
                                                           "current"}};

  // forward tree
  std::string res;
  auto result = gurka::do_action(Options::sources_to_targets, map, {"1", "2"}, {"1", "2"}, "auto",
                                 options, nullptr, &res);
  rapidjson::Document res_doc;
  res_doc.Parse(res.c_str());
  check_matrix(res_doc, {0.0f, 898.0f, 898.0f, 0.0f}, true, "time", Matrix::TimeDistanceMatrix);
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
  check_matrix(res_doc, {0.0f, 898.0f, 115.0f, 0.0f}, false, "time", Matrix::TimeDistanceMatrix);
  ASSERT_EQ(result.info().warnings().size(), 0);

  res.erase();
  options = {
      {"/sources/0/date_time", "current"},
      {"/sources/1/date_time", "2016-07-03T08:06"},
      {"/costing_options/auto/speed_types/0", "current"},
      {"/prioritize_bidirectional", "1"},
  };
  result = gurka::do_action(Options::sources_to_targets, map, {"1", "2"}, {"1", "2"}, "auto", options,
                            nullptr, &res);
  res_doc.Parse(res.c_str());

  check_matrix(res_doc, {0.0f, 898.0f, 115.0f, 0.0f}, false, "time", Matrix::CostMatrix);
}

TEST_F(MatrixTrafficTest, CostMatrixLiveTraffic) {
  std::unordered_map<std::string, std::string> options = {
      {"/date_time/type", "0"},
      {"/costing_options/auto/speed_types/0", "current"},
      {"/prioritize_bidirectional", "1"},
  };

  // forward tree
  std::string res;
  auto result = gurka::do_action(Options::sources_to_targets, map, {"1", "2"}, {"1", "2"}, "auto",
                                 options, nullptr, &res);
  rapidjson::Document res_doc;
  res_doc.Parse(res.c_str());
  check_matrix(res_doc, {0.0f, 898.0f, 898.0f, 0.0f}, true, "time", Matrix::CostMatrix);
  ASSERT_EQ(result.info().warnings().size(), 0);

  // bidir matrix allows less targets than sources and date_time on the sources
  options = {{"/sources/0/date_time", "2016-07-03T08:06"},
             {"/sources/1/date_time", "current"},
             {"/costing_options/auto/speed_types/0", "current"},
             {"/prioritize_bidirectional", "1"}};
  res.erase();
  result = gurka::do_action(Options::sources_to_targets, map, {"1", "2"}, {"1"}, "auto", options,
                            nullptr, &res);
  res_doc.Parse(res.c_str());
  check_matrix(res_doc, {0.0f, 898.0f}, true, "time", Matrix::CostMatrix);
  ASSERT_EQ(result.info().warnings().size(), 0);

  // we don't support date_time on the targets
  options = {{"/targets/0/date_time", "2016-07-03T08:06"},
             {"/costing_options/auto/speed_types/0", "current"},
             {"/prioritize_bidirectional", "1"}};
  res.erase();
  result = gurka::do_action(Options::sources_to_targets, map, {"1", "2"}, {"1"}, "auto", options,
                            nullptr, &res);
  res_doc.Parse(res.c_str());
  check_matrix(res_doc, {0.0f, 115.0f}, false, "time", Matrix::CostMatrix);
  ASSERT_EQ(result.info().warnings().size(), 1);
  ASSERT_EQ(result.info().warnings(0).code(), 206);
}

TEST_F(MatrixTrafficTest, DisallowedRequest) {
  map.config.put("service_limits.max_timedep_distance_matrix", "0");
  const std::unordered_map<std::string, std::string> options = {{"/date_time/type", "0"}};
  const auto result =
      gurka::do_action(Options::sources_to_targets, map, {"1", "2"}, {"1", "2"}, "auto", options);

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
  auto result = gurka::do_action(Options::sources_to_targets, map, {"1", "2"}, {"1"}, "auto", options,
                                 nullptr, &res);
  res_doc.Parse(res.c_str());
  check_matrix(res_doc, {0.0f, 115.0f}, true, "time", Matrix::TimeDistanceMatrix);
  ASSERT_EQ(result.info().warnings().size(), 0);

  // more targets than sources with date_time.type = 2 are disallowed
  options = {{"/date_time/type", "2"}, {"/date_time/value", "2016-07-03T08:06"}};
  res.erase();
  result = gurka::do_action(Options::sources_to_targets, map, {"1"}, {"1", "2"}, "auto", options,
                            nullptr, &res);
  res_doc.Parse(res.c_str());
  check_matrix(res_doc, {0.0f, 115.0f}, false, "time", Matrix::CostMatrix);
  ASSERT_EQ(result.info().warnings().Get(0).code(), 202);
  ASSERT_EQ(result.info().warnings().size(), 1);

  // date_time on the sources, disallowed reverse
  options = {{"/sources/0/date_time", "current"},
             {"/sources/1/date_time", "2016-07-03T08:06"},
             {"/costing_options/auto/speed_types/0", "current"}};
  res.erase();
  result = gurka::do_action(Options::sources_to_targets, map, {"1", "2"}, {"1"}, "auto", options,
                            nullptr, &res);
  res_doc.Parse(res.c_str());
  check_matrix(res_doc, {0.0f, 115.0f}, false, "time", Matrix::CostMatrix);
  ASSERT_EQ(result.info().warnings().Get(0).code(), 201);
  ASSERT_EQ(result.info().warnings().size(), 1);
}

TEST_F(MatrixTrafficTest, TDTargets) {
  // more targets than sources are allowed
  rapidjson::Document res_doc;
  std::string res;
  std::unordered_map<std::string, std::string> options = {{"/date_time/type", "0"},
                                                          {"/date_time/value", "current"}};
  auto result = gurka::do_action(Options::sources_to_targets, map, {"1"}, {"1", "2"}, "auto", options,
                                 nullptr, &res);
  res_doc.Parse(res.c_str());
  check_matrix(res_doc, {0.0f, 898.0f}, true, "time", Matrix::TimeDistanceMatrix);
  ASSERT_EQ(result.info().warnings().size(), 0);

  // more sources than targets with date_time.type = 1 are disallowed
  options = {{"/date_time/type", "1"}, {"/date_time/value", "2016-07-03T08:06"}};
  res.erase();
  result = gurka::do_action(Options::sources_to_targets, map, {"1", "2"}, {"1"}, "auto", options,
                            nullptr, &res);
  res_doc.Parse(res.c_str());
  check_matrix(res_doc, {0.0f, 115.0f}, false, "time", Matrix::CostMatrix);
  ASSERT_EQ(result.info().warnings().size(), 1);
  ASSERT_EQ(result.info().warnings().Get(0).code(), 201);

  // date_time on the targets, disallowed forward
  options = {{"/targets/0/date_time", "current"}, {"/targets/1/date_time", "2016-07-03T08:06"}};
  res.erase();
  result = gurka::do_action(Options::sources_to_targets, map, {"1"}, {"1", "2"}, "auto", options,
                            nullptr, &res);
  res_doc.Parse(res.c_str());
  check_matrix(res_doc, {0.0f, 115.0f}, false, "time", Matrix::CostMatrix);
  ASSERT_EQ(result.info().warnings().size(), 1);
  ASSERT_EQ(result.info().warnings().Get(0).code(), 202);
}

// hierarchy limits are managed by thor's worker, since we call the algorithms directly here,
// we have to do this manually
void set_hierarchy_limits(sif::cost_ptr_t& cost, boost::property_tree::ptree& pt) {
  Costing_Options opts;
  const auto hl_config = parse_hierarchy_limits_from_config(pt, "costmatrix", true);
  check_hierarchy_limits(cost->GetHierarchyLimits(), cost, opts, hl_config, false, true);
}

std::string build_matrix_request(const std::vector<std::string>& sources,
                                 const std::vector<std::string>& targets,
                                 const std::string& costing,
                                 const std::unordered_map<std::string, std::string>& options,
                                 const gurka::map& map) {
  auto sources_lls = gurka::detail::to_lls(map.nodes, sources);
  auto targets_lls = gurka::detail::to_lls(map.nodes, targets);
  return gurka::detail::build_valhalla_request({"sources", "targets"}, {sources_lls, targets_lls},
                                               costing, options);
}

/**
 * This test goes a little deeper in making sure that CostMatrix is actually pushing forward time on
 * one search tree and using invalid times on the reverse tree. We construct our own little costing
 * class that keeps track of the time info objects with which its EdgeCost method is called.
 *
 * We expect there to be similar counts of invalid and valid time info objects, and the maximum value
 * for "seconds_from_now" should be some non-zero value
 */
TEST_F(MatrixTrafficTest, CostMatrixPathfinding) {
  valhalla::loki::loki_worker_t loki_worker(map.config);

  auto test_request =
      build_matrix_request({"1", "2"}, {"1", "2"}, "auto", {{"/date_time/type", "0"}}, map);
  Api request;
  ParseApi(test_request, Options::sources_to_targets, request);
  loki_worker.matrix(request);
  thor_worker_t::adjust_scores(*request.mutable_options());

  GraphReader reader(map.config.get_child("mjolnir"));

  const auto hl_config = parse_hierarchy_limits_from_config(map.config, "costmatrix", true);
  sif::mode_costing_t mode_costing;
  auto cost =
      CreateSimpleCost(request.options().costings().find(request.options().costing_type())->second);

  mode_costing[0] = cost;
  set_hierarchy_limits(mode_costing[0], map.config);
  CostMatrix cost_matrix;
  cost_matrix.SourceToTarget(request, reader, mode_costing, sif::TravelMode::kDrive, 400000.0);
  uint64_t invalid_count = std::accumulate(cost->time_infos.begin(), cost->time_infos.end(), 0,
                                           [](uint64_t acc, const TimeInfo& a) {
                                             return acc + static_cast<uint64_t>(!a.valid);
                                           });
  uint64_t valid_count = std::accumulate(cost->time_infos.begin(), cost->time_infos.end(), 0,
                                         [](uint64_t acc, const TimeInfo& a) {
                                           return acc + static_cast<uint64_t>(a.valid);
                                         });

  auto max_seconds_from_now = std::max_element(cost->time_infos.begin(), cost->time_infos.end(),
                                               [](const TimeInfo& a, const TimeInfo& b) {
                                                 return a.seconds_from_now < b.seconds_from_now;
                                               });

  EXPECT_EQ(24, invalid_count);
  EXPECT_EQ(24, valid_count);
  EXPECT_EQ(7045, max_seconds_from_now->seconds_from_now);
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
    check_matrix(res_doc, {1.5f}, false, "distance", Matrix::CostMatrix);
    res.erase();
  }

  // then we force to go 1 -> F to hit a blocking node, doing a u-turn and go back the same way
  {
    auto result = gurka::do_action(valhalla::Options::sources_to_targets, map, {"1"}, {"B"}, "auto",
                                   {{"/sources/0/preferred_side", "opposite"}}, nullptr, &res);
    res_doc.Parse(res.c_str());
    check_matrix(res_doc, {0.8f}, false, "distance", Matrix::CostMatrix);
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
                               {{"mjolnir.timezone", VALHALLA_BUILD_DIR "test/data/tz.sqlite"},
                                {"service_limits.max_timedep_distance_matrix", "50000"}});
  }
};
gurka::map DateTimeTest::map = {};
gurka::map DateTimeTest::map_tz = {};

void check_date_times(const Api& request,
                      const rapidjson::Document& res,
                      const std::vector<std::string>& target_offsets,
                      const std::vector<std::string>& target_timezones) {
  for (size_t source_idx = 0; source_idx < static_cast<size_t>(request.options().sources().size());
       source_idx++) {
    for (size_t target_idx = 0; target_idx < static_cast<size_t>(request.options().targets().size());
         target_idx++) {

      ASSERT_TRUE(res["sources_to_targets"]
                      .GetArray()[source_idx]
                      .GetArray()[target_idx]
                      .GetObject()
                      .HasMember("date_time"));
      EXPECT_EQ(res["sources_to_targets"]
                    .GetArray()[source_idx]
                    .GetArray()[target_idx]
                    .GetObject()["time_zone_offset"]
                    .GetString(),
                target_offsets[target_idx]);
      EXPECT_EQ(res["sources_to_targets"]
                    .GetArray()[source_idx]
                    .GetArray()[target_idx]
                    .GetObject()["time_zone_name"]
                    .GetString(),
                target_timezones[target_idx]);
    }
  }
}
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
    check_date_times(api, res_doc, {"+01:00", "+00:00"}, {"Europe/Madrid", "Europe/Lisbon"});
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
    ASSERT_EQ(api.matrix().algorithm(), Matrix::TimeDistanceMatrix);
    check_date_times(api, res_doc, {"+01:00", "+00:00"}, {"Europe/Madrid", "Europe/Lisbon"});
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

// Parameterize check_reverse_connection
class TestConnectionCheck : public ::testing::TestWithParam<std::string> {};

TEST_P(TestConnectionCheck, MatrixSecondPass) {
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
                                     {{"thor.costmatrix.allow_second_pass", "1"},
                                      {"thor.costmatrix.check_reverse_connection", GetParam()}});
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

TEST_P(TestConnectionCheck, CostMatrixTrivialRoutes) {
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
      gurka::buildtiles(layout, ways, {}, {}, VALHALLA_BUILD_DIR "test/data/costmatrix_trivial",
                        {{"thor.costmatrix.check_reverse_connection", GetParam()}});

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

TEST_P(TestConnectionCheck, HGVNoAccessPenalty) {
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
                                     {{"service_limits.max_timedep_distance_matrix", "50000"},
                                      {"thor.costmatrix.check_reverse_connection", GetParam()}});

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

TEST_P(TestConnectionCheck, VerboseResponse) {

  const std::string ascii_map = R"(
    A-1-------B----C----D----E--2-------F
                   |    |
                   J----K--5------N
                   |    |         \
                   |    |          \
                   3    4           6
                   |    |            \
                   L----M             \
                                       O
                                       |
                                       7
                                       |
                                       P
           )";

  const gurka::ways ways = {
      {"AB", {{"highway", "residential"}}}, {"BC", {{"highway", "residential"}}},
      {"CD", {{"highway", "residential"}}}, {"DE", {{"highway", "residential"}}},
      {"FE", {{"highway", "residential"}}}, {"CJ", {{"highway", "residential"}}},
      {"JK", {{"highway", "residential"}}}, {"JLMK", {{"highway", "residential"}}},
      {"KD", {{"highway", "residential"}}}, {"KNOP", {{"highway", "residential"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  gurka::map map = gurka::buildtiles(layout, ways, {}, {}, "test/data/matrix_verbose_response",
                                     {{"service_limits.max_timedep_distance_matrix", "50000"},
                                      {"thor.costmatrix.check_reverse_connection", GetParam()}});
  {
    rapidjson::Document res_doc;
    std::string res;
    auto api = gurka::do_action(valhalla::Options::sources_to_targets, map, {"1", "2", "3", "4"},
                                {"1", "2", "3", "4"}, "auto", {{"/prioritize_bidirectional", "1"}},
                                nullptr, &res);

    res_doc.Parse(res.c_str());

    // sanity check
    EXPECT_EQ(api.matrix().algorithm(), Matrix::CostMatrix);

    for (size_t i = 0; i < 4; ++i) {
      for (size_t j = 0; j < 4; ++j) {
        bool key_should_exist = true;
        if (i == j)
          key_should_exist = false;
        EXPECT_EQ(res_doc["sources_to_targets"].GetArray()[i].GetArray()[j].GetObject().HasMember(
                      "begin_heading"),
                  key_should_exist);
        EXPECT_EQ(res_doc["sources_to_targets"].GetArray()[i].GetArray()[j].GetObject().HasMember(
                      "end_heading"),
                  key_should_exist);
        EXPECT_EQ(res_doc["sources_to_targets"].GetArray()[i].GetArray()[j].GetObject().HasMember(
                      "begin_lat"),
                  key_should_exist);
        EXPECT_EQ(res_doc["sources_to_targets"].GetArray()[i].GetArray()[j].GetObject().HasMember(
                      "begin_lon"),
                  key_should_exist);
        EXPECT_EQ(res_doc["sources_to_targets"].GetArray()[i].GetArray()[j].GetObject().HasMember(
                      "end_lat"),
                  key_should_exist);
        EXPECT_EQ(res_doc["sources_to_targets"].GetArray()[i].GetArray()[j].GetObject().HasMember(
                      "end_lon"),
                  key_should_exist);
      }
    }
    EXPECT_EQ(res_doc["sources_to_targets"]
                  .GetArray()[0]
                  .GetArray()[1]
                  .GetObject()["begin_heading"]
                  .GetDouble(),
              90);
    EXPECT_EQ(res_doc["sources_to_targets"]
                  .GetArray()[0]
                  .GetArray()[1]
                  .GetObject()["end_heading"]
                  .GetDouble(),
              90);
    EXPECT_EQ(res_doc["sources_to_targets"]
                  .GetArray()[1]
                  .GetArray()[0]
                  .GetObject()["begin_heading"]
                  .GetDouble(),
              270);
    EXPECT_EQ(res_doc["sources_to_targets"]
                  .GetArray()[1]
                  .GetArray()[0]
                  .GetObject()["end_heading"]
                  .GetDouble(),
              270);
    EXPECT_EQ(res_doc["sources_to_targets"]
                  .GetArray()[0]
                  .GetArray()[2]
                  .GetObject()["begin_heading"]
                  .GetDouble(),
              90);
    EXPECT_EQ(res_doc["sources_to_targets"]
                  .GetArray()[0]
                  .GetArray()[2]
                  .GetObject()["end_heading"]
                  .GetDouble(),
              180);
  }

  // check heading at long and winding edges
  {
    rapidjson::Document res_doc;
    std::string res;
    auto api = gurka::do_action(valhalla::Options::sources_to_targets, map, {"1", "5", "6", "7"},
                                {"1", "5", "6", "7"}, "auto",
                                {{"/prioritize_bidirectional", "1"}, {"/shape_format", "polyline6"}},
                                nullptr, &res);

    res_doc.Parse(res.c_str());

    // sanity check
    EXPECT_EQ(api.matrix().algorithm(), Matrix::CostMatrix);

    for (size_t i = 0; i < 4; ++i) {
      for (size_t j = 0; j < 4; ++j) {
        bool key_should_exist = true;
        if (i == j)
          key_should_exist = false;
        EXPECT_EQ(res_doc["sources_to_targets"].GetArray()[i].GetArray()[j].GetObject().HasMember(
                      "begin_heading"),
                  key_should_exist);
        EXPECT_EQ(res_doc["sources_to_targets"].GetArray()[i].GetArray()[j].GetObject().HasMember(
                      "end_heading"),
                  key_should_exist);
        EXPECT_EQ(res_doc["sources_to_targets"].GetArray()[i].GetArray()[j].GetObject().HasMember(
                      "begin_lat"),
                  key_should_exist);
        EXPECT_EQ(res_doc["sources_to_targets"].GetArray()[i].GetArray()[j].GetObject().HasMember(
                      "begin_lon"),
                  key_should_exist);
        EXPECT_EQ(res_doc["sources_to_targets"].GetArray()[i].GetArray()[j].GetObject().HasMember(
                      "end_lat"),
                  key_should_exist);
        EXPECT_EQ(res_doc["sources_to_targets"].GetArray()[i].GetArray()[j].GetObject().HasMember(
                      "end_lon"),
                  key_should_exist);
      }
    }
    size_t a, b;
    auto get_shape = [&res_doc](size_t i, size_t j) {
      return res_doc["sources_to_targets"]
          .GetArray()[i]
          .GetArray()[j]
          .GetObject()["shape"]
          .GetString();
    };
    auto get_heading = [&res_doc](size_t i, size_t j, const char* which) {
      return res_doc["sources_to_targets"].GetArray()[i].GetArray()[j].GetObject()[which].GetDouble();
    };

    // 1 -> 5
    a = 0;
    b = 1;
    EXPECT_EQ(get_heading(a, b, "begin_heading"), 90) << get_shape(a, b);
    EXPECT_EQ(get_heading(a, b, "end_heading"), 90) << get_shape(a, b);

    // 5 -> 1
    a = 1;
    b = 0;
    EXPECT_EQ(get_heading(a, b, "begin_heading"), 270) << get_shape(a, b);
    EXPECT_EQ(get_heading(a, b, "end_heading"), 270) << get_shape(a, b);

    // 1 -> 6
    a = 0;
    b = 2;
    EXPECT_EQ(get_heading(a, b, "begin_heading"), 90) << get_shape(a, b);
    EXPECT_EQ(get_heading(a, b, "end_heading"), 140.1) << get_shape(a, b);

    // 6 -> 1
    a = 2;
    b = 0;
    EXPECT_EQ(get_heading(a, b, "begin_heading"), 320.1) << get_shape(a, b);
    EXPECT_EQ(get_heading(a, b, "end_heading"), 270) << get_shape(a, b);

    // 1 -> 7
    a = 0;
    b = 3;
    EXPECT_EQ(get_heading(a, b, "begin_heading"), 90) << get_shape(a, b);
    EXPECT_EQ(get_heading(a, b, "end_heading"), 180) << get_shape(a, b);

    // 7 -> 1
    a = 3;
    b = 0;
    EXPECT_EQ(get_heading(a, b, "begin_heading"), 0) << get_shape(a, b);
    EXPECT_EQ(get_heading(a, b, "end_heading"), 270) << get_shape(a, b);
  }
}

/************************************************************************ */

std::string encode_shape(const std::vector<std::string>& nodes, valhalla::gurka::nodelayout& layout) {
  std::vector<PointLL> shape;
  shape.reserve(nodes.size());
  for (auto& node : nodes) {
    shape.push_back(layout[node]);
  }
  return encode<std::vector<PointLL>>(shape, 1e6);
}

void check_trivial_matrix(const gurka::map& map, gurka::nodelayout& layout) {
  std::unordered_map<std::string, std::string> options = {{"/shape_format", "polyline6"}};
  // 1 -> 2
  {
    auto matrix =
        gurka::do_action(valhalla::Options::sources_to_targets, map, {"1"}, {"2"}, "auto", options);
    EXPECT_EQ(matrix.matrix().distances(0), 300);
    EXPECT_EQ(matrix.matrix().shapes(0), encode_shape({"1", "2"}, layout));
  }

  // 1 -> 3
  {
    auto matrix =
        gurka::do_action(valhalla::Options::sources_to_targets, map, {"1"}, {"3"}, "auto", options);
    EXPECT_EQ(matrix.matrix().distances(0), 3300);
    EXPECT_EQ(matrix.matrix().shapes(0), encode_shape({"1", "A", "B", "C", "D", "3"}, layout));
  }

  // 1 -> 2,3
  {
    auto matrix = gurka::do_action(valhalla::Options::sources_to_targets, map, {"1"}, {"2", "3"},
                                   "auto", options);
    EXPECT_EQ(matrix.matrix().distances(0), 300);
    EXPECT_EQ(matrix.matrix().shapes(0), encode_shape({"1", "2"}, layout));
    EXPECT_EQ(matrix.matrix().distances(1), 3300);
    EXPECT_EQ(matrix.matrix().shapes(1), encode_shape({"1", "A", "B", "C", "D", "3"}, layout));
  }

  // 1 -> 3,2
  {
    auto matrix = gurka::do_action(valhalla::Options::sources_to_targets, map, {"1"}, {"3", "2"},
                                   "auto", options);
    EXPECT_EQ(matrix.matrix().distances(0), 3300);
    EXPECT_EQ(matrix.matrix().shapes(0), encode_shape({"1", "A", "B", "C", "D", "3"}, layout));
    EXPECT_EQ(matrix.matrix().distances(1), 300);
    EXPECT_EQ(matrix.matrix().shapes(1), encode_shape({"1", "2"}, layout));
  }
}

TEST_P(TestConnectionCheck, MultipleTrivialRoutes) {
  const std::string ascii_map = R"(
    B-------------C
    |             |
    |             |
    |             |
    A---2--1--3---D
  )";
  const gurka::ways ways = {
      {"AB", {{"highway", "residential"}}},
      {"BC", {{"highway", "residential"}, {"oneway", "yes"}}},
      {"CD", {{"highway", "residential"}}},
      {"DA", {{"highway", "residential"}, {"oneway", "yes"}}},
  };
  auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, {}, {}, VALHALLA_BUILD_DIR "test/data/costmatrix_fail",
                               {{"thor.costmatrix.check_reverse_connection", GetParam()}});
  check_trivial_matrix(map, layout);
  // ensure consistent behavior regardless of whether we do the reverse connection check
  map.config.put("thor.costmatrix_check_reverse_connection", "1");
  check_trivial_matrix(map, layout);
}

INSTANTIATE_TEST_SUITE_P(connection_check, TestConnectionCheck, ::testing::Values("1", "0"));

TEST(StandAlone, TrivialKeepExpanding) {
  // target candidates includes AB but should be penalized
  // so that path 1B, BC, Cx has less cost than the trivial one
  const std::string ascii_map = R"(
    A---1-----------B
                    |
                    |
                2   |
    D-----------x---C
  )";
  // clang-format off
  const gurka::ways ways = {
      {"AB", {{"highway", "residential"}}},
      {"BC", {{"highway", "residential"}}},
      {"CD", {{"highway", "residential"}}},
  };
  // clang-format on

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 25);

  auto map = gurka::buildtiles(layout, ways, {}, {},
                               VALHALLA_BUILD_DIR "test/data/costmatrix_keep_expanding",
                               {{"thor.costmatrix.check_reverse_connection", "1"}});

  auto result = gurka::do_action(valhalla::Options::sources_to_targets, map, {"1"}, {"2"}, "auto",
                                 {{"/targets/0/radius", "80"}}, nullptr);

  EXPECT_EQ(result.matrix().distances(0), 500);
}

/**
 * inbound source edges should be kept in case of
 * node snapping, as long as there are targets snapped
 * to the same node (and vice versa)
 */
TEST(StandAlone, TrivialCorrelation) {
  const std::string ascii_map = R"(
    2
    A------B------------C-----D
    1                   |     |
                        |     |
                        F-----E
  )";
  // clang-format off
  const gurka::ways ways = {
      {"AB", {{"highway", "residential"}}},
      {"BC", {{"highway", "residential"}}},
      {"CD", {{"highway", "residential"}}},
      {"DE", {{"highway", "residential"}}},
      {"EF", {{"highway", "residential"}}},
      {"FC", {{"highway", "residential"}}},
  };
  // clang-format on

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 2);

  auto map = gurka::buildtiles(layout, ways, {}, {},
                               VALHALLA_BUILD_DIR "test/data/costmatrix_trivial_correlation",
                               {{"thor.costmatrix.check_reverse_connection", "1"}});

  auto result =
      gurka::do_action(valhalla::Options::sources_to_targets, map, {"1"}, {"2"}, "auto", {}, nullptr);

  EXPECT_EQ(result.matrix().distances(0), 0);
}
