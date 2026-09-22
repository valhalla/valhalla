
#include "baldr/rapidjson_utils.h"
#include "gurka.h"
#include "loki/worker.h"
#include "thor/worker.h"

#include <boost/format.hpp>
#include <gtest/gtest.h>

using namespace valhalla;

std::string encode_shape(const std::vector<std::string>& nodes, valhalla::gurka::nodelayout& layout) {
  std::vector<midgard::PointLL> shape;
  shape.reserve(nodes.size());
  for (auto& node : nodes) {
    shape.push_back(layout[node]);
  }
  return midgard::encode<std::vector<midgard::PointLL>>(shape, 1e6);
}
std::tuple<baldr::GraphId, const baldr::DirectedEdge*> get_shortcut(baldr::GraphReader& reader,
                                                                    gurka::nodelayout& nodes,
                                                                    const std::string& begin_node,
                                                                    const std::string& end_node) {
  auto e = gurka::findEdgeByNodes(reader, nodes, begin_node, end_node);
  auto shortcut = reader.GetShortcut(std::get<0>(e));
  auto* de = reader.directededge(shortcut);
  return std::make_tuple(shortcut, de);
}

void check_cost_factor_edge(const ::google::protobuf::RepeatedPtrField<CostFactorEdge>& edges,
                            const std::string& begin_node,
                            const std::string& end_node,
                            baldr::GraphReader& reader,
                            gurka::nodelayout& nodes,
                            double expected_factor,
                            double expected_start,
                            double expected_end,
                            bool shortcut = false) {
  std::tuple<baldr::GraphId, const baldr::DirectedEdge*> e;
  if (shortcut) {
    e = get_shortcut(reader, nodes, begin_node, end_node);
  } else {
    e = gurka::findEdgeByNodes(reader, nodes, begin_node, end_node);
  }

  for (const auto& cfe : edges) {
    if (std::get<0>(e) == cfe.id()) {
      EXPECT_NEAR(expected_factor, cfe.factor(), 0.001)
          << "Check failed for " << begin_node << end_node;
      EXPECT_NEAR(expected_start, cfe.start(), 0.001);
      EXPECT_NEAR(expected_end, cfe.end(), 0.001);
      return;
    }
  }

  FAIL() << (shortcut ? "Shortcut " : "Edge ") << " not found: " << begin_node << end_node;
}

class LinearFeatureTest : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize_metres = 100;

    const std::string ascii_map = R"(
          I----J   M--N------7--O
          |    |   |             \
     G-8--H    K---L    Q--------9P
    /          |        |
    |          |        |
    A=====3====B========C=======D====2E=====F
    |          |                      |      \
    |          0                      Z-------b
    |          |                      |
    |          R--------S             5
    |                  /              |
    |                 /               |
    a----4----U------T-------1--------Y
              |                       |
              |                       |
              V---W---6---------------X
  )";

    const gurka::ways ways = {
        // A-F is a motorway and should form a single shortcut
        {"AB", {{"highway", "motorway"}, {"name", "A2"}}},
        {"BC", {{"highway", "motorway"}, {"name", "A2"}}},
        {"CD", {{"highway", "motorway"}, {"name", "A2"}}},
        {"DE", {{"highway", "motorway"}, {"name", "A2"}}},
        {"EF", {{"highway", "motorway"}, {"name", "A2"}}},

        // should also form a shortcut
        {"AG", {{"highway", "secondary"}, {"name", "Pretty Important Rd"}}},
        {"GH", {{"highway", "secondary"}, {"name", "Pretty Important Rd"}}},

        {"HI", {{"highway", "living_street"}}},
        {"IJ", {{"highway", "living_street"}}},
        {"JK", {{"highway", "living_street"}}},

        {"KB", {{"highway", "secondary"}}},
        {"KL", {{"highway", "tertiary"}}},
        {"LM", {{"highway", "tertiary"}}},
        {"MN", {{"highway", "tertiary"}}},
        {"NO", {{"highway", "tertiary"}}},
        {"OP", {{"highway", "tertiary"}}},

        {"PQ", {{"highway", "secondary"}}},
        {"QC", {{"highway", "secondary"}}},

        {"Aa", {{"highway", "secondary"}, {"name", "Relevant Ave"}}},
        {"aU", {{"highway", "secondary"}, {"name", "Relevant Ave"}}},

        {"UV", {{"highway", "secondary"}}},
        {"UT", {{"highway", "secondary"}}},
        {"TY", {{"highway", "secondary"}}},
        {"TS", {{"highway", "secondary"}}},
        {"SR", {{"highway", "tertiary"}}},
        {"RB", {{"highway", "tertiary"}}},

        // these should not form any shortcuts
        {"VW", {{"highway", "secondary"}, {"maxweight", "16"}, {"bicycle", "no"}}},
        {"WX", {{"highway", "secondary"}}},
        {"XY", {{"highway", "secondary"}, {"bicycle", "no"}}},
        {"YZ", {{"highway", "secondary"}, {"maxweight", "18"}, {"pedestrian", "no"}}},
        {"EZ", {{"highway", "secondary"}}},
        {"Zb", {{"highway", "secondary"}, {"bicycle", "no"}}},
        {"Fb", {{"highway", "secondary"}}},
    };

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize_metres);
    map =
        gurka::buildtiles(layout, ways, {}, {}, VALHALLA_BUILD_DIR "test/data/linear_feature_factors",
                          {{"service_limits.min_linear_cost_factor", "0.00001"}});
  }
};
gurka::map LinearFeatureTest::map = {};

/**
 * Simple case: one shape that starts and ends at a node
 * */
TEST_F(LinearFeatureTest, simple_high_factor) {
  loki::loki_worker_t loki_worker(map.config);
  thor::thor_worker_t thor_worker(map.config);

  std::string json_request = R"(
  {
    "locations": [
      {"lon": %s, "lat": %s},
      {"lon": %s, "lat": %s}
    ], 
    "linear_cost_factors": [
      {"shape": "%s", "factor": %s}
    ], 
    "costing": "auto"
  }
  )";

  auto json_str =
      (boost::format(json_request) % std::to_string(map.nodes.at("3").lng()) %
       std::to_string(map.nodes.at("3").lat()) % std::to_string(map.nodes.at("2").lng()) %
       std::to_string(map.nodes.at("2").lat()) % encode_shape({"A", "B", "C"}, map.nodes) % "200")
          .str();

  Api request;
  ParseApi(json_str, Options::route, request);
  loki_worker.route(request);
  loki_worker.cleanup();
  ASSERT_EQ(request.options().cost_factor_lines().size(), 1);
  EXPECT_EQ(request.options().cost_factor_lines().at(0).cost_factor(), 200);
  EXPECT_EQ(request.options().cost_factor_lines().at(0).shape().size(), 3);

  thor_worker.route(request);
  auto costing_options =
      request.options().costings().find(request.options().costing_type())->second.options();
  // AB, BC and the shortcut they make up, twice (once for each range)
  EXPECT_EQ(costing_options.cost_factor_edges().size(), 4);

  baldr::GraphReader reader(map.config.get_child("mjolnir"));

  auto AB = gurka::findEdgeByNodes(reader, map.nodes, "A", "B");
  auto BC = gurka::findEdgeByNodes(reader, map.nodes, "B", "C");
  auto shortcut = get_shortcut(reader, map.nodes, "A", "B");
  EXPECT_NE(std::get<0>(shortcut), baldr::kInvalidGraphId);
  for (const auto& e : costing_options.cost_factor_edges()) {
    EXPECT_EQ(e.factor(), 200);
    if (e.id() == std::get<0>(AB)) {
      EXPECT_NEAR(e.start(), 0., 0.01);
      EXPECT_NEAR(e.end(), 1., 0.01);
    } else if (e.id() == std::get<0>(BC)) {
      EXPECT_NEAR(e.start(), 0., 0.01);
      EXPECT_NEAR(e.end(), 1., 0.01);
    } else if (e.id() == std::get<0>(shortcut)) {
      if (e.start() < 0.01) {
        EXPECT_NEAR(e.end(), 0.275, 0.01);
      } else {
        EXPECT_NEAR(e.start(), 0.275, 0.01);
        EXPECT_NEAR(e.end(), 0.5, 0.01);
      }
    } else {
      FAIL() << "Unexpected edge: " + std::to_string(e.id());
    }
  }

  // finally check the route
  EXPECT_EQ(request.trip().routes(0).legs(0).shape(),
            encode_shape({"3", "A", "a", "U", "T", "Y", "Z", "E", "2"}, map.nodes));
}

/**
 * Similar as above but use a low factor
 * */
TEST_F(LinearFeatureTest, simple_low_factor) {
  loki::loki_worker_t loki_worker(map.config);
  thor::thor_worker_t thor_worker(map.config);

  std::string json_request = R"(
  {
    "locations": [
      {"lon": %s, "lat": %s},
      {"lon": %s, "lat": %s}
    ], 
    "linear_cost_factors": [
      {"shape": "%s", "factor": %s}
    ], 
    "costing": "auto"
  }
  )";

  auto json_str = (boost::format(json_request) % std::to_string(map.nodes.at("4").lng()) %
                   std::to_string(map.nodes.at("4").lat()) % std::to_string(map.nodes.at("1").lng()) %
                   std::to_string(map.nodes.at("1").lat()) %
                   encode_shape({"U", "V", "W", "X", "Y"}, map.nodes) % "0.01")
                      .str();

  std::cerr << "Valhalla request is: \n" << json_str << "\n";

  Api request;
  ParseApi(json_str, Options::route, request);
  loki_worker.route(request);
  loki_worker.cleanup();
  ASSERT_EQ(request.options().cost_factor_lines().size(), 1);
  EXPECT_EQ(request.options().cost_factor_lines().at(0).cost_factor(), 0.01f);
  EXPECT_EQ(request.options().cost_factor_lines().at(0).shape().size(), 5);

  thor_worker.route(request);
  auto costing_options =
      request.options().costings().find(request.options().costing_type())->second.options();
  EXPECT_EQ(costing_options.cost_factor_edges().size(), 4);

  baldr::GraphReader reader(map.config.get_child("mjolnir"));

  check_cost_factor_edge(costing_options.cost_factor_edges(), "U", "V", reader, map.nodes, 0.01f, 0.,
                         1.);
  check_cost_factor_edge(costing_options.cost_factor_edges(), "V", "W", reader, map.nodes, 0.01f, 0.,
                         1.);
  check_cost_factor_edge(costing_options.cost_factor_edges(), "W", "X", reader, map.nodes, 0.01f, 0.,
                         1.);
  check_cost_factor_edge(costing_options.cost_factor_edges(), "X", "Y", reader, map.nodes, 0.01f, 0.,
                         1.);

  sif::mode_costing_t mode_costing;
  auto costings = request.options().costings().find(request.options().costing_type())->second;
  auto auto_cost = valhalla::sif::CreateAutoCost(costings);

  // make sure there is no shortcut here
  auto shortcut = get_shortcut(reader, map.nodes, "U", "V");
  EXPECT_FALSE(std::get<0>(shortcut).is_valid());
  // finally check the route
  gurka::assert::raw::expect_path(request, {"Relevant Ave", "UV", "VW", "WX", "XY", "TY"});
}

TEST_F(LinearFeatureTest, partial_edges_shape) {
  loki::loki_worker_t loki_worker(map.config);
  thor::thor_worker_t thor_worker(map.config);

  std::string json_request = R"(
  {
    "locations": [
      {"lon": %s, "lat": %s},
      {"lon": %s, "lat": %s}
    ], 
    "linear_cost_factors": [
      {"shape": "%s", "factor": %s}
    ], 
    "costing": "auto"
  }
  )";

  auto json_str =
      (boost::format(json_request) % std::to_string(map.nodes.at("T").lng()) %
       std::to_string(map.nodes.at("T").lat()) % std::to_string(map.nodes.at("Z").lng()) %
       std::to_string(map.nodes.at("Z").lat()) % encode_shape({"1", "Y", "5"}, map.nodes) % "100")
          .str();

  std::cerr << "Valhalla request is: \n" << json_str << "\n";

  Api request;
  ParseApi(json_str, Options::route, request);
  loki_worker.route(request);
  loki_worker.cleanup();
  ASSERT_EQ(request.options().cost_factor_lines().size(), 1);
  EXPECT_NEAR(request.options().cost_factor_lines().at(0).cost_factor(), 100.f, 0.01);
  EXPECT_EQ(request.options().cost_factor_lines().at(0).shape().size(), 3);

  thor_worker.route(request);
  auto costing_options =
      request.options().costings().find(request.options().costing_type())->second.options();
  EXPECT_EQ(costing_options.cost_factor_edges().size(), 2);

  baldr::GraphReader reader(map.config.get_child("mjolnir"));

  check_cost_factor_edge(costing_options.cost_factor_edges(), "T", "Y", reader, map.nodes, 100.f,
                         0.4706, 1.);
  check_cost_factor_edge(costing_options.cost_factor_edges(), "Y", "Z", reader, map.nodes, 100.f, 0.,
                         0.6);

  sif::mode_costing_t mode_costing;
  auto costings = request.options().costings().find(request.options().costing_type())->second;
  auto auto_cost = valhalla::sif::CreateAutoCost(costings);

  // finally check the route
  gurka::assert::raw::expect_path(request, {"TS", "SR", "RB", "A2", "A2", "A2", "EZ"});
}

TEST_F(LinearFeatureTest, allow_access_restriction) {
  loki::loki_worker_t loki_worker(map.config);
  thor::thor_worker_t thor_worker(map.config);

  std::string json_request = R"(
  {
    "locations": [
      {"lon": %s, "lat": %s},
      {"lon": %s, "lat": %s}
    ], 
    "linear_cost_factors": [
      {"shape": "%s", "allow": %s}
    ], 
    "costing": "auto",
    "costing_options": {
      "auto": {
        "weight": 20
      }
    }
  }
  )";

  auto json_str =
      (boost::format(json_request) % std::to_string(map.nodes.at("4").lng()) %
       std::to_string(map.nodes.at("4").lat()) % std::to_string(map.nodes.at("6").lng()) %
       std::to_string(map.nodes.at("6").lat()) % encode_shape({"V", "W"}, map.nodes) % "true")
          .str();

  std::cerr << "Valhalla request is: \n" << json_str << "\n";

  Api request;
  ParseApi(json_str, Options::route, request);
  loki_worker.route(request);
  loki_worker.cleanup();
  ASSERT_EQ(request.options().cost_factor_lines().size(), 1);
  EXPECT_NEAR(request.options().cost_factor_lines().at(0).cost_factor(), 1.f, 0.01);
  EXPECT_TRUE(request.options().cost_factor_lines().at(0).allow());
  EXPECT_EQ(request.options().cost_factor_lines().at(0).shape().size(), 2);

  thor_worker.route(request);
  auto costing_options =
      request.options().costings().find(request.options().costing_type())->second.options();
  EXPECT_EQ(costing_options.cost_factor_edges().size(), 1);

  baldr::GraphReader reader(map.config.get_child("mjolnir"));

  bool found = false;
  for (auto& cfe : costing_options.cost_factor_edges()) {
    auto e = gurka::findEdgeByNodes(reader, map.nodes, "V", "W");
    if (std::get<0>(e) == cfe.id()) {
      EXPECT_TRUE(cfe.allow());
      EXPECT_NEAR(cfe.factor(), 1.f, 0.01f);
      found = true;
      break;
    }
  }
  EXPECT_TRUE(found);
  sif::mode_costing_t mode_costing;
  auto costings = request.options().costings().find(request.options().costing_type())->second;
  auto auto_cost = valhalla::sif::CreateAutoCost(costings);

  // finally check the route
  gurka::assert::raw::expect_path(request, {"Relevant Ave", "UV", "VW", "WX"});
}

/**
 * Test multiple shapes, sent as GeoJSON
 */
TEST_F(LinearFeatureTest, multi_shape_geojson) {
  loki::loki_worker_t loki_worker(map.config);
  thor::thor_worker_t thor_worker(map.config);

  std::string json_request = R"(
  {
    "locations": [
      {"lon": %s, "lat": %s},
      {"lon": %s, "lat": %s}
    ], 
    "linear_cost_factors": [
      {"type": "Feature", "geometry": {"type": "LineString", "coordinates": %s}, "properties": {"factor": %s}},
      {"type": "Feature", "geometry": {"type": "LineString", "coordinates": %s}, "properties": {"factor": %s}}
    ], 
    "costing": "auto"
  }
  )";

  auto format_coordinates = [&](const std::vector<std::string>& waypoints) {
    rapidjson::writer_wrapper_t writer;
    writer.set_precision(6);
    writer.start_array();
    for (const auto& c : waypoints) {
      writer.start_array();
      writer(map.nodes.at(c).lng());
      writer(map.nodes.at(c).lat());
      writer.end_array();
    }
    writer.end_array();
    return std::string(writer.get_buffer());
  };

  auto json_str =
      (boost::format(json_request) % std::to_string(map.nodes.at("E").lng()) %
       std::to_string(map.nodes.at("E").lat()) % std::to_string(map.nodes.at("Z").lng()) %
       std::to_string(map.nodes.at("Z").lat()) % format_coordinates({"2", "E", "Z", "5"}) % "100" %
       format_coordinates({"F", "b"}) % "0.1")
          .str();

  std::cerr << "Valhalla request is: \n" << json_str << "\n";

  Api request;
  ParseApi(json_str, Options::route, request);
  loki_worker.route(request);
  loki_worker.cleanup();
  ASSERT_EQ(request.options().cost_factor_lines().size(), 2);
  EXPECT_EQ(request.options().cost_factor_lines().at(0).cost_factor(), 100.f);
  EXPECT_EQ(request.options().cost_factor_lines().at(0).shape().size(), 4);
  EXPECT_EQ(request.options().cost_factor_lines().at(1).cost_factor(), 0.1f);
  EXPECT_EQ(request.options().cost_factor_lines().at(1).shape().size(), 2);

  thor_worker.route(request);
  auto costing_options =
      request.options().costings().find(request.options().costing_type())->second.options();
  EXPECT_EQ(costing_options.cost_factor_edges().size(), 5);

  baldr::GraphReader reader(map.config.get_child("mjolnir"));

  check_cost_factor_edge(costing_options.cost_factor_edges(), "F", "b", reader, map.nodes, 0.1f, 0.,
                         1.);
  check_cost_factor_edge(costing_options.cost_factor_edges(), "D", "E", reader, map.nodes, 100.f,
                         0.82498, 0.85, /*shortcut=*/true);

  sif::mode_costing_t mode_costing;
  auto costings = request.options().costings().find(request.options().costing_type())->second;
  auto auto_cost = valhalla::sif::CreateAutoCost(costings);

  // finally check the route
  gurka::assert::raw::expect_path(request, {"A2", "Fb", "Zb"});
}

/**
 * The same shape as simple_high_factor, but for CostMatrix.
 * */
TEST_F(LinearFeatureTest, matrix_high_factor) {
  std::string json_request = R"(
  {
    "sources": [{"lon": %s, "lat": %s}],
    "targets": [{"lon": %s, "lat": %s}],
    %s
    "costing": "auto"
  }
  )";

  auto build_request = [&](const std::string& factors) {
    return (boost::format(json_request) % std::to_string(map.nodes.at("3").lng()) %
            std::to_string(map.nodes.at("3").lat()) % std::to_string(map.nodes.at("2").lng()) %
            std::to_string(map.nodes.at("2").lat()) % factors)
        .str();
  };

  const auto factors = (boost::format(R"("linear_cost_factors": [{"shape": "%s", "factor": 200}],)") %
                        encode_shape({"A", "B", "C"}, map.nodes))
                           .str();

  loki::loki_worker_t loki_worker(map.config);
  thor::thor_worker_t thor_worker(map.config);

  // baseline: straight down the motorway
  Api baseline;
  ParseApi(build_request(""), Options::sources_to_targets, baseline);
  loki_worker.matrix(baseline);
  loki_worker.cleanup();
  thor_worker.matrix(baseline);
  thor_worker.cleanup();
  ASSERT_EQ(baseline.matrix().distances().size(), 1);

  Api request;
  ParseApi(build_request(factors), Options::sources_to_targets, request);
  loki_worker.matrix(request);
  loki_worker.cleanup();

  // the endpoints got correlated onto the lines without disturbing sources/targets
  ASSERT_EQ(request.options().cost_factor_lines().size(), 1);
  ASSERT_EQ(request.options().cost_factor_lines().at(0).locations().size(), 2);
  EXPECT_EQ(request.options().sources_size(), 1);
  EXPECT_EQ(request.options().targets_size(), 1);

  thor_worker.matrix(request);
  auto costing_options =
      request.options().costings().find(request.options().costing_type())->second.options();

  baldr::GraphReader reader(map.config.get_child("mjolnir"));
  // AB, BC and the shortcut they make up, twice (once for each range)
  ASSERT_EQ(costing_options.cost_factor_edges().size(), 4);
  check_cost_factor_edge(costing_options.cost_factor_edges(), "A", "B", reader, map.nodes, 200, 0, 1);
  check_cost_factor_edge(costing_options.cost_factor_edges(), "B", "C", reader, map.nodes, 200, 0, 1);

  // avoiding the motorway makes for a longer path
  ASSERT_EQ(request.matrix().distances().size(), 1);
  EXPECT_GT(request.matrix().distances(0), baseline.matrix().distances(0));
}

/**
 * Same, but through TimeDistanceMatrix
 * */
TEST_F(LinearFeatureTest, matrix_timedistancematrix) {
  std::string json_request = R"(
  {
    "sources": [{"lon": %s, "lat": %s}],
    "targets": [{"lon": %s, "lat": %s}],
    %s
    "costing": "auto"
  }
  )";

  auto build_request = [&](const std::string& factors) {
    return (boost::format(json_request) % std::to_string(map.nodes.at("3").lng()) %
            std::to_string(map.nodes.at("3").lat()) % std::to_string(map.nodes.at("2").lng()) %
            std::to_string(map.nodes.at("2").lat()) % factors)
        .str();
  };

  const auto factors = (boost::format(R"("linear_cost_factors": [{"shape": "%s", "factor": 200}],)") %
                        encode_shape({"A", "B", "C"}, map.nodes))
                           .str();

  auto config = map.config;
  config.put("thor.source_to_target_algorithm", "timedistancematrix");
  loki::loki_worker_t loki_worker(config);
  thor::thor_worker_t thor_worker(config);

  Api baseline;
  ParseApi(build_request(""), Options::sources_to_targets, baseline);
  loki_worker.matrix(baseline);
  loki_worker.cleanup();
  thor_worker.matrix(baseline);
  thor_worker.cleanup();
  ASSERT_EQ(baseline.matrix().algorithm(), Matrix::TimeDistanceMatrix);

  Api request;
  ParseApi(build_request(factors), Options::sources_to_targets, request);
  loki_worker.matrix(request);
  loki_worker.cleanup();
  thor_worker.matrix(request);

  ASSERT_EQ(request.matrix().algorithm(), Matrix::TimeDistanceMatrix);
  auto costing_options =
      request.options().costings().find(request.options().costing_type())->second.options();
  EXPECT_EQ(costing_options.cost_factor_edges().size(), 4);

  ASSERT_EQ(request.matrix().distances().size(), 1);
  EXPECT_GT(request.matrix().distances(0), baseline.matrix().distances(0));
}

/**
 * /optimized_route runs a matrix too, so it resolves the lines through the same path.
 * */
TEST_F(LinearFeatureTest, optimized_route) {
  std::string json_request = R"(
  {
    "locations": [
      {"lon": %s, "lat": %s},
      {"lon": %s, "lat": %s},
      {"lon": %s, "lat": %s}
    ],
    "linear_cost_factors": [{"shape": "%s", "factor": 200}],
    "costing": "auto"
  }
  )";

  auto json_str = (boost::format(json_request) % std::to_string(map.nodes.at("3").lng()) %
                   std::to_string(map.nodes.at("3").lat()) % std::to_string(map.nodes.at("2").lng()) %
                   std::to_string(map.nodes.at("2").lat()) % std::to_string(map.nodes.at("1").lng()) %
                   std::to_string(map.nodes.at("1").lat()) % encode_shape({"A", "B", "C"}, map.nodes))
                      .str();

  loki::loki_worker_t loki_worker(map.config);
  thor::thor_worker_t thor_worker(map.config);

  Api request;
  ParseApi(json_str, Options::optimized_route, request);
  loki_worker.matrix(request);
  loki_worker.cleanup();

  ASSERT_EQ(request.options().cost_factor_lines().size(), 1);
  ASSERT_EQ(request.options().cost_factor_lines().at(0).locations().size(), 2);
  EXPECT_EQ(request.options().sources_size(), 3);
  EXPECT_EQ(request.options().targets_size(), 3);

  thor_worker.optimized_route(request);
  auto costing_options =
      request.options().costings().find(request.options().costing_type())->second.options();
  EXPECT_EQ(costing_options.cost_factor_edges().size(), 4);
}

// shape starts and ends within the default node snap tolerance of a node
TEST_F(LinearFeatureTest, partial_edges_near_nodes) {
  loki::loki_worker_t loki_worker(map.config);
  thor::thor_worker_t thor_worker(map.config);

  std::string json_request = R"(
  {
    "locations": [
      {"lon": %s, "lat": %s},
      {"lon": %s, "lat": %s}
    ],
    "linear_cost_factors": [
      {"shape": "%s", "factor": %s}
    ],
    "costing": "auto"
  }
  )";

  const auto& B = map.nodes.at("B");
  const auto& C = map.nodes.at("C");
  const auto& D = map.nodes.at("D");
  double start = 3. / B.Distance(C);
  double end = 1. - 3. / C.Distance(D);
  std::vector<midgard::PointLL> shape{B.PointAlongSegment(C, start), C, C.PointAlongSegment(D, end)};

  auto json_str = (boost::format(json_request) % std::to_string(map.nodes.at("3").lng()) %
                   std::to_string(map.nodes.at("3").lat()) % std::to_string(map.nodes.at("2").lng()) %
                   std::to_string(map.nodes.at("2").lat()) % midgard::encode(shape, 1e6) % "10")
                      .str();

  Api request;
  ParseApi(json_str, Options::route, request);
  loki_worker.route(request);
  loki_worker.cleanup();
  ASSERT_EQ(request.options().cost_factor_lines().size(), 1);

  thor_worker.route(request);
  auto costing_options =
      request.options().costings().find(request.options().costing_type())->second.options();
  // BC, CD and the shortcut entry for each
  EXPECT_EQ(costing_options.cost_factor_edges().size(), 4);

  baldr::GraphReader reader(map.config.get_child("mjolnir"));
  check_cost_factor_edge(costing_options.cost_factor_edges(), "B", "C", reader, map.nodes, 10., start,
                         1.);
  check_cost_factor_edge(costing_options.cost_factor_edges(), "C", "D", reader, map.nodes, 10., 0.,
                         end);
}

TEST_F(LinearFeatureTest, empty_shape) {
  loki::loki_worker_t loki_worker(map.config);
  thor::thor_worker_t thor_worker(map.config);

  std::string json_request = R"(
  {
    "locations": [
      {"lon": %s, "lat": %s},
      {"lon": %s, "lat": %s}
    ], 
    "linear_cost_factors": [
      {"type": "Feature", "geometry": {"type": "LineString", "coordinates": %s}, "properties": {"factor": %s}},
      {"type": "Feature", "geometry": {"type": "LineString", "coordinates": %s}, "properties": {"factor": %s}}
    ], 
    "costing": "auto"
  }
  )";

  auto format_coordinates = [&](const std::vector<std::string>& waypoints) {
    rapidjson::writer_wrapper_t writer;
    writer.set_precision(6);
    writer.start_array();
    for (const auto& c : waypoints) {
      writer.start_array();
      writer(map.nodes.at(c).lng());
      writer(map.nodes.at(c).lat());
      writer.end_array();
    }
    writer.end_array();
    return std::string(writer.get_buffer());
  };

  auto json_str =
      (boost::format(json_request) % std::to_string(map.nodes.at("E").lng()) %
       std::to_string(map.nodes.at("E").lat()) % std::to_string(map.nodes.at("Z").lng()) %
       std::to_string(map.nodes.at("Z").lat()) % "" % "100" % format_coordinates({"F", "b"}) % "0.1")
          .str();

  std::cerr << "Valhalla request is: \n" << json_str << "\n";

  EXPECT_THROW(gurka::do_action(valhalla::Options::route, map, json_str), valhalla_exception_t);
}

// lines are correlated with a none costing, so they resolve onto edges the requested costing
// can't travel on
TEST(LinearFeature, none_costing) {
  const std::string ascii_map = R"(
    A----B----C
         |
         D
  )";
  const gurka::ways ways = {
      {"AB", {{"highway", "residential"}}},
      {"BC", {{"highway", "residential"}}},
      {"BD", {{"highway", "footway"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, {}, {},
                               VALHALLA_BUILD_DIR "test/data/linear_feature_none_costing");

  std::string json_request = R"(
  {
    "locations": [
      {"lon": %s, "lat": %s},
      {"lon": %s, "lat": %s}
    ],
    "linear_cost_factors": [
      {"shape": "%s", "factor": 200}
    ],
    "costing": "auto"
  }
  )";

  auto json_str = (boost::format(json_request) % std::to_string(map.nodes.at("A").lng()) %
                   std::to_string(map.nodes.at("A").lat()) % std::to_string(map.nodes.at("C").lng()) %
                   std::to_string(map.nodes.at("C").lat()) % encode_shape({"B", "D"}, map.nodes))
                      .str();

  loki::loki_worker_t loki_worker(map.config);

  Api request;
  ParseApi(json_str, Options::route, request);
  loki_worker.route(request);

  // loki resolved the line without disturbing the locations it has to correlate for the route
  EXPECT_EQ(request.options().locations_size(), 2);
  const auto& costing_options =
      request.options().costings().find(request.options().costing_type())->second.options();
  ASSERT_EQ(costing_options.cost_factor_edges().size(), 1);

  baldr::GraphReader reader(map.config.get_child("mjolnir"));
  check_cost_factor_edge(costing_options.cost_factor_edges(), "B", "D", reader, map.nodes, 200, 0, 1);
}

/**
 * "allow" has to cover more than the access restrictions it started out as: GH has no auto
 * access at all and is the only link between the two halves of the map, so the route either
 * exists or it doesn't and no cost comparison is involved. GH sits near the destination on
 * purpose, so the reverse search runs out of edges long before the forward one arrives.
 */
TEST(LinearFeature, allow_inaccessible_edge) {
  const std::string ascii_map = R"(
    A----B----C----D----E----F----G----H--I
  )";
  const gurka::ways ways = {
      {"AB", {{"highway", "residential"}}},
      {"BC", {{"highway", "residential"}}},
      {"CD", {{"highway", "residential"}}},
      {"DE", {{"highway", "residential"}}},
      {"EF", {{"highway", "residential"}}},
      {"FG", {{"highway", "residential"}}},
      {"GH", {{"highway", "residential"}, {"motor_vehicle", "no"}}},
      {"HI", {{"highway", "residential"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map =
      gurka::buildtiles(layout, ways, {}, {}, VALHALLA_BUILD_DIR "test/data/linear_feature_allow");

  std::string json_request = R"(
  {
    "locations": [
      {"lon": %s, "lat": %s},
      {"lon": %s, "lat": %s}
    ],
    %s
    "costing": "auto"
  }
  )";

  auto route = [&](const std::string& linear_cost_factors, Api& request) {
    auto json_str =
        (boost::format(json_request) % std::to_string(map.nodes.at("A").lng()) %
         std::to_string(map.nodes.at("A").lat()) % std::to_string(map.nodes.at("I").lng()) %
         std::to_string(map.nodes.at("I").lat()) % linear_cost_factors)
            .str();

    loki::loki_worker_t loki_worker(map.config);
    thor::thor_worker_t thor_worker(map.config);
    ParseApi(json_str, Options::route, request);
    loki_worker.route(request);
    loki_worker.cleanup();
    thor_worker.route(request);
  };

  Api without;
  EXPECT_THROW(route("", without), valhalla_exception_t);

  Api with;
  route((boost::format(R"("linear_cost_factors": [{"shape": "%s", "allow": true}],)") %
         encode_shape({"G", "H"}, map.nodes))
            .str(),
        with);

  const auto& costing_options =
      with.options().costings().find(with.options().costing_type())->second.options();
  ASSERT_EQ(costing_options.cost_factor_edges().size(), 1);
  EXPECT_TRUE(costing_options.cost_factor_edges().at(0).allow());

  gurka::assert::raw::expect_path(with, {"AB", "BC", "CD", "DE", "EF", "FG", "GH", "HI"});
}

/**
 * Dijkstras gates on the raw access bits before it ever asks costing, so the isochrone
 * expansion stops at B unless "allow" is honored there too
 */
TEST(LinearFeature, allow_in_dijkstras) {
  const std::string ascii_map = R"(
    A----B----C----D
  )";
  const gurka::ways ways = {
      {"AB", {{"highway", "residential"}}},
      {"BC", {{"highway", "residential"}, {"motor_vehicle", "no"}}},
      {"CD", {{"highway", "residential"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, {}, {},
                               VALHALLA_BUILD_DIR "test/data/linear_feature_allow_dijkstras");

  std::string json_request = R"(
  {
    "locations": [{"lon": %s, "lat": %s}],
    "contours": [{"time": 60}],
    "action": "isochrone",
    "skip_opposites": true,
    %s
    "costing": "auto"
  }
  )";

  auto expand = [&](const std::string& linear_cost_factors) {
    auto json_str = (boost::format(json_request) % std::to_string(map.nodes.at("A").lng()) %
                     std::to_string(map.nodes.at("A").lat()) % linear_cost_factors)
                        .str();
    return gurka::do_action(Options::expansion, map, json_str);
  };

  EXPECT_EQ(expand("").expansion().geometries_size(), 1);

  auto with = expand((boost::format(R"("linear_cost_factors": [{"shape": "%s", "allow": true}],)") %
                      encode_shape({"B", "C"}, map.nodes))
                         .str());
  EXPECT_EQ(with.expansion().geometries_size(), 3);
}

/**
 * CostMatrix checks the raw access bits before it asks costing, same as the route
 * algorithms. BC has no auto access and is the only link between source and target.
 */
TEST(LinearFeature, allow_in_costmatrix) {
  const std::string ascii_map = R"(
    A----B----C----D
  )";
  const gurka::ways ways = {
      {"AB", {{"highway", "residential"}}},
      {"BC", {{"highway", "residential"}, {"motor_vehicle", "no"}}},
      {"CD", {{"highway", "residential"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, {}, {},
                               VALHALLA_BUILD_DIR "test/data/linear_feature_allow_costmatrix");

  std::string json_request = R"(
  {
    "sources": [{"lon": %s, "lat": %s}],
    "targets": [{"lon": %s, "lat": %s}],
    %s
    "costing": "auto"
  }
  )";

  auto matrix = [&](const std::string& linear_cost_factors, Api& request) {
    auto json_str =
        (boost::format(json_request) % std::to_string(map.nodes.at("A").lng()) %
         std::to_string(map.nodes.at("A").lat()) % std::to_string(map.nodes.at("D").lng()) %
         std::to_string(map.nodes.at("D").lat()) % linear_cost_factors)
            .str();

    loki::loki_worker_t loki_worker(map.config);
    thor::thor_worker_t thor_worker(map.config);
    ParseApi(json_str, Options::sources_to_targets, request);
    loki_worker.matrix(request);
    loki_worker.cleanup();
    thor_worker.matrix(request);
  };

  Api without;
  matrix("", without);
  ASSERT_EQ(without.matrix().distances().size(), 1);
  // no auto path across BC, so the pair comes back unreachable
  EXPECT_GT(without.matrix().distances(0), 1e6);

  Api with;
  matrix((boost::format(R"("linear_cost_factors": [{"shape": "%s", "allow": true}],)") %
          encode_shape({"B", "C"}, map.nodes))
             .str(),
         with);
  ASSERT_EQ(with.matrix().distances().size(), 1);
  EXPECT_NEAR(with.matrix().distances(0), 1500, 1);
}
