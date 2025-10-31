
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
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/linear_feature_factors",
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
      if (e.start() == 0.) {
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
  EXPECT_FALSE(std::get<0>(shortcut).Is_Valid());
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
