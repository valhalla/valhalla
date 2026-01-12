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
  const auto utrecht_conf = test::make_config(VALHALLA_BUILD_DIR "test/data/utrecht_tiles");
  tyr::actor_t actor(utrecht_conf);

  // Request a tile for Utrecht center (52.08778°N, 5.13142°E)
  // At zoom 14, this is tile 14/8425/5405
  std::string request = R"({"tile": {"z": 14,"x": 8425,"y": 5405}})";

  auto tile_data = actor.tile(request);
  actor.cleanup();

  // should be 700-800kb
  EXPECT_GT(tile_data.size(), 100000);
  EXPECT_LT(tile_data.size(), 150000);

  vtzero::vector_tile tile{tile_data};

  bool has_edges = false;
  bool has_nodes = false;

  while (auto layer = tile.next_layer()) {
    std::string layer_name = std::string(layer.name());

    if (layer_name == "edges") {
      has_edges = true;
      EXPECT_EQ(layer.num_features(), 2278);
    } else if (layer_name == "nodes") {
      has_nodes = true;
      EXPECT_EQ(layer.num_features(), 1741);
    } else {
      FAIL() << "Unexpected layer: " << layer_name;
    }
  }

  EXPECT_TRUE(has_edges);
  EXPECT_TRUE(has_nodes);
}

// TEST(Actor, TileReturnShortcuts) {
//   // Use Utrecht tiles for this test
//   const auto utrecht_conf = test::make_config(VALHALLA_BUILD_DIR "test/data/utrecht_tiles");
//   tyr::actor_t actor(utrecht_conf);

//   // Request the same tile without shortcuts (default)
//   std::string request_no_shortcuts =
//       R"({"tile": {"z": 14,"x": 8425,"y": 5405}, "tile_options": {"return_shortcuts": false}})";
//   auto tile_data_no_shortcuts = actor.tile(request_no_shortcuts);
//   actor.cleanup();

//   // Request the same tile with shortcuts
//   std::string request_with_shortcuts =
//       R"({"tile": {"z": 14,"x": 8425,"y": 5405}, "tile_options": {"return_shortcuts": true}})";
//   auto tile_data_with_shortcuts = actor.tile(request_with_shortcuts);
//   actor.cleanup();

//   // Both should return valid data
//   ASSERT_FALSE(tile_data_no_shortcuts.empty()) << "Tile data without shortcuts should not be
//   empty"; ASSERT_FALSE(tile_data_with_shortcuts.empty()) << "Tile data with shortcuts should not be
//   empty";

//   // Parse both tiles
//   vtzero::vector_tile tile_no_shortcuts{tile_data_no_shortcuts};
//   vtzero::vector_tile tile_with_shortcuts{tile_data_with_shortcuts};

//   // Count features in edges layer for both tiles
//   uint32_t features_no_shortcuts = 0;
//   uint32_t features_with_shortcuts = 0;

//   while (auto layer = tile_no_shortcuts.next_layer()) {
//     if (std::string(layer.name()) == "edges") {
//       features_no_shortcuts = layer.num_features();
//       break;
//     }
//   }

//   while (auto layer = tile_with_shortcuts.next_layer()) {
//     if (std::string(layer.name()) == "edges") {
//       features_with_shortcuts = layer.num_features();
//       break;
//     }
//   }

//   ASSERT_EQ(features_with_shortcuts, 2317);
//   ASSERT_EQ(features_no_shortcuts, 2278);
// }

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

  // for tile
  Options tile_options;
  auto* xyz = tile_options.mutable_tile_xyz();
  xyz->set_x(8425);
  xyz->set_y(5405);
  xyz->set_z(14);

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
      {Options::tile, &tyr::actor_t::tile, tile_options},
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
