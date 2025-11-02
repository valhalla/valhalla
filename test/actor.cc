#include "tyr/actor.h"
#include "test.h"

#include <boost/property_tree/ptree.hpp>
#include <vtzero/vector_tile.hpp>

#include <functional>
#include <string>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;

namespace {

// fake up config against pine grove traffic extract
const auto conf = test::make_config(VALHALLA_SOURCE_DIR "test/traffic_matcher_tiles");

TEST(Actor, Basic) {
  tyr::actor_t actor(conf);

  actor.route(R"({"locations":[{"lat":40.546115,"lon":-76.385076,"type":"break"},
      {"lat":40.544232,"lon":-76.385752,"type":"break"}],"costing":"auto"})");
  actor.cleanup();
  auto route_json = actor.route(R"({"locations":[{"lat":40.546115,"lon":-76.385076,"type":"break"},
          {"lat":40.544232,"lon":-76.385752,"type":"break"}],"costing":"auto"})");
  actor.cleanup();
  auto route = test::json_to_pt(route_json);
  ASSERT_NE(route_json.find("Tulpehocken"), std::string::npos);

  actor.trace_attributes(R"({"shape":[{"lat":40.546115,"lon":-76.385076},
      {"lat":40.544232,"lon":-76.385752}],"costing":"auto","shape_match":"map_snap"})");
  actor.cleanup();
  auto attributes_json = actor.trace_attributes(R"({"shape":[{"lat":40.546115,"lon":-76.385076},
      {"lat":40.544232,"lon":-76.385752}],"costing":"auto","shape_match":"map_snap"})");
  actor.cleanup();
  auto attributes = test::json_to_pt(attributes_json);
  ASSERT_NE(attributes_json.find("Tulpehocken"), std::string::npos);

  actor.transit_available(R"({"locations":[{"lat":35.647452, "lon":-79.597477, "radius":20},
      {"lat":34.766908, "lon":-80.325936,"radius":10}]})");
  actor.cleanup();
  auto transit_json =
      actor.transit_available(R"({"locations":[{"lat":35.647452, "lon":-79.597477, "radius":20},
      {"lat":34.766908, "lon":-80.325936,"radius":10}]})");
  actor.cleanup();
  auto transit = test::json_to_pt(transit_json);
  ASSERT_NE(transit_json.find(std::to_string(false)), std::string::npos);

  auto status_json = actor.status("");
  ASSERT_NE(status_json.find("tileset_last_modified"), std::string::npos);
  actor.cleanup();
  status_json = actor.status(R"({"verbose":true})");
  actor.cleanup();
  auto status = test::json_to_pt(status_json);
  ASSERT_NE(status_json.find("Polygon"), std::string::npos);

  // TODO: test the rest of them
}

struct test_exception_t {};

TEST(Actor, Route) {
  tyr::actor_t actor(conf);
  std::string request = R"({"locations":[{"lat":40.546115,"lon":-76.385076,"type":"break"},
        {"lat":40.544232,"lon":-76.385752,"type":"break"}],"costing":"auto"})";
  std::function<void()> interrupt = [] { throw test_exception_t{}; };
  EXPECT_THROW(actor.route(request, &interrupt), test_exception_t);
}

TEST(Actor, TraceAttributes) {
  tyr::actor_t actor(conf);
  std::string request = R"({"shape":[{"lat":40.546115,"lon":-76.385076},
        {"lat":40.544232,"lon":-76.385752}],"costing":"auto","shape_match":"map_snap"})";
  std::function<void()> interrupt = [] { throw test_exception_t{}; };
  EXPECT_THROW(actor.trace_attributes(request, &interrupt), test_exception_t);
}

TEST(Actor, Tile) {
  tyr::actor_t actor(conf);
  
  // Request a tile for the Pine Grove area (lat: 40.546115, lon: -76.385076)
  // At zoom 14, this is approximately tile 14/4714/6092
  std::string request = R"({"z":14,"x":4714,"y":6092})";
  
  auto tile_data = actor.tile(request);
  actor.cleanup();
  
  // Verify we got data back
  ASSERT_FALSE(tile_data.empty()) << "Tile data should not be empty";
  ASSERT_GT(tile_data.size(), 100) << "Tile data should be at least 100 bytes";
  
  // Parse the MVT tile
  vtzero::vector_tile tile{tile_data};
  
  // Verify the tile contains expected layers
  bool has_edges = false;
  bool has_nodes = false;
  
  while (auto layer = tile.next_layer()) {
    std::string layer_name = std::string(layer.name());
    
    if (layer_name == "edges") {
      has_edges = true;
      // Verify the edges layer has at least one feature
      ASSERT_GT(layer.num_features(), 0) << "Edges layer should have features";
    } else if (layer_name == "nodes") {
      has_nodes = true;
      // Verify the nodes layer has at least one feature
      ASSERT_GT(layer.num_features(), 0) << "Nodes layer should have features";
    }
  }
  
  // Verify both expected layers exist
  ASSERT_TRUE(has_edges) << "Tile should contain 'edges' layer";
  ASSERT_TRUE(has_nodes) << "Tile should contain 'nodes' layer";
}

// TODO: test the rest of them

TEST(Actor, SupportedFormats) {
  valhalla::Location loc1;
  loc1.mutable_ll()->set_lat(40.546115);
  loc1.mutable_ll()->set_lng(-76.385076);

  valhalla::Location loc2;
  loc2.mutable_ll()->set_lat(40.544232);
  loc2.mutable_ll()->set_lng(-76.385752);

  // Options that would work for all actions
  Options options;
  options.set_costing_type(Costing_Type::Costing_Type_auto_);

  auto* locations = options.mutable_locations();
  locations->Add()->CopyFrom(loc1);
  locations->Add()->CopyFrom(loc2);

  // for matrix
  options.mutable_sources()->CopyFrom(options.locations());
  options.mutable_targets()->CopyFrom(options.locations());

  // for trace_route and trace_attributes
  options.mutable_shape()->CopyFrom(options.locations());

  // for expansion
  options.set_expansion_action(Options::route);

  // for isochrone
  Options isochrone_options;
  auto* contour = isochrone_options.mutable_contours()->Add();
  contour->set_color("ff0000");
  contour->set_time(10.0);
  isochrone_options.set_costing_type(Costing_Type::Costing_Type_auto_);
  isochrone_options.mutable_locations()->Add()->CopyFrom(loc1);

  const struct {
    Options::Action action;
    std::string (tyr::actor_t::*action_fn)(const std::string& request_str,
                                           const std::function<void()>* interrupt,
                                           Api* api);
    Options options;
  } tests[] = {
      {Options::route, &tyr::actor_t::route, options},
      {Options::locate, &tyr::actor_t::locate, options},
      {Options::sources_to_targets, &tyr::actor_t::matrix, options},
      {Options::optimized_route, &tyr::actor_t::optimized_route, options},
      {Options::isochrone, &tyr::actor_t::isochrone, isochrone_options},
      {Options::trace_route, &tyr::actor_t::trace_route, options},
      {Options::trace_attributes, &tyr::actor_t::trace_attributes, options},
      {Options::height, &tyr::actor_t::height, options},
      {Options::transit_available, &tyr::actor_t::transit_available, options},
      {Options::expansion, &tyr::actor_t::expansion, options},
      {Options::centroid, &tyr::actor_t::centroid, options},
      {Options::status, &tyr::actor_t::status, options},
  };
  ASSERT_EQ(std::size(tests), Options::Action_ARRAYSIZE - 1) // -1 for `Options::no_action`
      << "Please add missing action to this test";

  tyr::actor_t actor(conf);
  for (const auto& t : tests) {
    for (int format = Options::Format_MIN; format <= Options::Format_MAX; format += 1) {
      ASSERT_TRUE(Options::Format_IsValid(format));

      valhalla::Api api;
      auto* options = api.mutable_options();
      options->CopyFrom(t.options);
      options->set_format(static_cast<Options::Format>(format));
      options->set_action(t.action);

      actor.cleanup();
      EXPECT_NO_THROW((actor.*(t.action_fn))("", nullptr, &api))
          << Options::Action_Name(t.action) << ": "
          << Options::Format_Name(static_cast<Options::Format>(format));
    }
  }
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
