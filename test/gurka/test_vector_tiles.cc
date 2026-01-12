#include "gurka.h"
#include "loki/utils.h"
#include "loki/worker.h"
#include "midgard/constants.h"
#include "test.h"

#include <valhalla/exceptions.h>

#include <gmock/gmock-matchers.h>
#include <gtest/gtest.h>
#include <vtzero/vector_tile.hpp>

#include <format>
#include <set>

using namespace valhalla;
using namespace valhalla::midgard;

namespace {

// handler only counting vertices in a linestring
struct linestring_handler_t {
  uint32_t count = 0;
  void linestring_begin(uint32_t /*count*/) {
  }
  void linestring_point(vtzero::point /*point*/) noexcept {
    count++;
  }
  void linestring_end() const noexcept {
  }
  uint32_t result() {
    return count;
  }
};
} // namespace

TEST(VectorTilesBasic, ConfigFail) {
  // zoom config with descending levels fails
  auto config = test::make_config("/tmp");
  auto& mvt_min_zoom_road_class = config.get_child("loki.service_defaults.mvt_min_zoom_road_class");
  auto it = mvt_min_zoom_road_class.begin();
  it->second.put_value(30);
  EXPECT_THROW(loki::loki_worker_t loki_worker(config), std::runtime_error);

  // the wrong number of zoom levels fails too
  mvt_min_zoom_road_class.erase(it);
  EXPECT_THROW(
      {
        try {
          loki::loki_worker_t loki_worker(config);
        } catch (const std::runtime_error& e) {
          EXPECT_TRUE(
              std::string(e.what()).starts_with("mvt_min_zoom_road_class out of bounds, expected"));
          throw;
        }
      },
      std::runtime_error);
};

TEST(VectorTilesBasic, TileGeneralization) {
  const std::string ascii_map = R"(
      x
      A0  34  78
        12  56  9B
  )";
  const gurka::ways ways = {
      {"A0123456789B", {{"highway", "motorway"}}},
  };
  // huge grid size bcs of zoom 8
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 500);
  auto map = gurka::buildtiles(layout, ways, {}, {}, VALHALLA_BUILD_DIR "test/data/gurka_vt_basic");

  auto test_vertex_count = [&](const std::unordered_map<std::string, std::string>& options,
                               const uint32_t expected_count) {
    // by default we expect full generalization
    std::string tile_data;
    auto api = gurka::do_action(Options::tile, map, "x", 8, "auto", options, nullptr, &tile_data);

    vtzero::vector_tile tile{tile_data};
    auto layer = tile.get_layer_by_name("edges");
    EXPECT_EQ(layer.num_features(), 1);

    layer.for_each_feature([&](const vtzero::feature&& feat) {
      auto handler = linestring_handler_t{};
      vtzero::decode_linestring_geometry(feat.geometry(), handler);
      EXPECT_EQ(handler.count, expected_count);

      return true;
    });
  };

  // by default we expect full generalization
  test_vertex_count({}, 2);
  // with very low generalize, i.e. full resolution aka no generalize
  test_vertex_count({{"/generalize", "0.0001"}}, 12);
}

TEST(VectorTilesBasic, TileRendering) {
  constexpr double gridsize = 100;

  const std::string ascii_map = R"(
      A---B---C
          |
          D
  )";

  const gurka::ways ways = {
      {"AB", {{"highway", "primary"}, {"name", "Main Street"}}},
      {"BC", {{"highway", "primary"}, {"name", "Main Street"}}},
      {"BD", {{"highway", "secondary"}, {"name", "Side Street"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
  auto map = gurka::buildtiles(layout, ways, {}, {}, VALHALLA_BUILD_DIR "test/data/gurka_vt_basic");

  std::string tile_data;
  auto api = gurka::do_action(Options::tile, map, "B", 14, "auto", {}, nullptr, &tile_data);

  EXPECT_LT(tile_data.size(), 2850);
  EXPECT_GT(tile_data.size(), 2750);

  vtzero::vector_tile tile{tile_data};

  bool has_edges = false;
  bool has_nodes = false;

  while (auto layer = tile.next_layer()) {
    std::string layer_name = std::string(layer.name());

    if (layer_name == "edges") {
      has_edges = true;

      EXPECT_EQ(layer.version(), 2);
      EXPECT_EQ(layer.extent(), 4096);

      EXPECT_EQ(layer.num_features(), 1);

      auto feature = layer.next_feature();
      EXPECT_TRUE(feature.has_id());
      EXPECT_GT(feature.id(), 0);

      EXPECT_EQ(feature.geometry_type(), vtzero::GeomType::LINESTRING);

      std::set<std::string> expected_props =
          {"tile_level",       "tile_id",         "road_class",          "use",
           "length",           "edge_id:forward", "speed:forward",       "access:auto:forward",
           "edge_id:backward", "speed:backward",  "access:auto:backward"};
      std::set<std::string> found_props;
      while (auto property = feature.next_property()) {
        std::string key = std::string(property.key());
        found_props.insert(key);
      }

      // Check that all expected properties are present
      for (const auto& prop : expected_props) {
        EXPECT_TRUE(found_props.count(prop) > 0) << "Edge should have property: " << prop;
      }

    } else if (layer_name == "nodes") {
      has_nodes = true;

      EXPECT_EQ(layer.version(), 2);
      EXPECT_EQ(layer.extent(), 4096);

      EXPECT_EQ(layer.num_features(), 2);

      auto feature = layer.next_feature();

      EXPECT_TRUE(feature.has_id()) << "Node feature should have an ID";
      EXPECT_GT(feature.id(), 0) << "Node feature ID should be positive";
      EXPECT_EQ(feature.geometry_type(), vtzero::GeomType::POINT) << "Node should be a point";

      std::set<std::string> expected_props = {"tile_level",        "tile_id",
                                              "node_id",           "type",
                                              "edge_count",        "access:auto",
                                              "access:pedestrian", "access:bicycle"};
      std::set<std::string> found_props;

      while (auto property = feature.next_property()) {
        std::string key = std::string(property.key());
        found_props.insert(key);
      }

      for (const auto& prop : expected_props) {
        EXPECT_TRUE(found_props.count(prop) > 0) << "Node should have property: " << prop;
      }
    } else {
      FAIL() << "Unexpected layer: " << layer_name;
    }
  }
  EXPECT_TRUE(has_edges);
  EXPECT_TRUE(has_nodes);
}

class VectorTiles : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize = 50;
    const std::string ascii_map = R"(
    x
      A---B---C---D
      |   |   |   |
      E---F---G---H
      |   |   |   |
      I---J---K---L
    )";

    const gurka::ways ways = {
        {"AB", {{"highway", "primary"}, {"name", "Main Street"}}},
        {"BC", {{"highway", "primary"}, {"name", "Main Street"}}},
        {"CD", {{"highway", "primary"}, {"name", "Main Street"}}},
        {"AE", {{"highway", "secondary"}, {"name", "First Avenue"}}},
        {"BF", {{"highway", "secondary"}, {"name", "Second Avenue"}}},
        {"CG", {{"highway", "secondary"}, {"name", "Third Avenue"}}},
        {"DH", {{"highway", "secondary"}, {"name", "Fourth Avenue"}}},
        {"EF", {{"highway", "tertiary"}, {"name", "Oak Street"}}},
        {"FG", {{"highway", "tertiary"}, {"name", "Oak Street"}}},
        {"GH", {{"highway", "tertiary"}, {"name", "Oak Street"}}},
        {"EI", {{"highway", "residential"}, {"name", "Pine Avenue"}}},
        {"FJ", {{"highway", "residential"}, {"name", "Elm Avenue"}}},
        {"GK", {{"highway", "residential"}, {"name", "Maple Avenue"}}},
        {"HL", {{"highway", "residential"}, {"name", "Cedar Avenue"}}},
        {"IJ", {{"highway", "service"}, {"name", "South Street"}}},
        {"JK", {{"highway", "service"}, {"name", "South Street"}}},
        {"KL", {{"highway", "service"}, {"name", "South Street"}}},
    };

    // just an anchor from which to determine xy of the /tile request
    const gurka::nodes nodes = {
        {"x", {{"bla", "bla"}}},
    };

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
    const std::unordered_map<std::string, std::string> build_options = {
        {"loki.service_defaults.mvt_cache_dir", VALHALLA_BUILD_DIR "test/data/mvt_cache_dir"}};
    map = gurka::buildtiles(layout, ways, nodes, {},
                            VALHALLA_BUILD_DIR "test/data/gurka_vt_zoom_compare", build_options);

    const auto cache_dir = map.config.get<std::string>("loki.service_defaults.mvt_cache_dir");
    if (std::filesystem::exists(cache_dir)) {
      std::filesystem::remove_all(cache_dir);
    }
  }
};

gurka::map VectorTiles::map = {};

TEST_F(VectorTiles, TileRenderingDifferentZoomLevels) {

  auto test_tile = [&](const uint32_t z, const uint32_t exp_total_size, const uint32_t exp_edges,
                       const uint32_t exp_nodes, uint32_t& cache_count) {
    SCOPED_TRACE(std::format("Zoom {} failed", z));
    std::string tile_data;
    Api api = gurka::do_action(Options::tile, map, "x", z, "auto", {}, nullptr, &tile_data);
    const auto x = api.options().tile_xyz().x();
    const auto y = api.options().tile_xyz().y();

    // check that cache worked
    if (z >= map.config.get<uint32_t>("loki.service_defaults.mvt_cache_min_zoom")) {
      const auto cache_dir = map.config.get<std::string>("loki.service_defaults.mvt_cache_dir");
      const auto tile_path = loki::detail::mvt_local_path(z, x, y, cache_dir);
      EXPECT_TRUE(std::filesystem::exists(tile_path)) << "path doesn't exist: " + tile_path.string();
      cache_count++;
    }

    // with some buffer due to arch-dependent float interpretation
    uint32_t buffer = (exp_total_size * 0.02);
    EXPECT_LT(tile_data.size(), exp_total_size + buffer);
    EXPECT_GT(tile_data.size(), exp_total_size - buffer);

    vtzero::vector_tile tile{tile_data};
    uint32_t edges = 0;
    uint32_t nodes = 0;

    while (auto layer = tile.next_layer()) {
      std::string layer_name = std::string(layer.name());
      if (layer_name == "edges") {
        edges = layer.num_features();
        EXPECT_EQ(layer.version(), 2);
        EXPECT_EQ(layer.extent(), 4096);
      } else if (layer_name == "nodes") {
        nodes = layer.num_features();
      } else {
        FAIL() << "Unexpected layer: " << layer_name;
      }
    }

    EXPECT_EQ(edges, exp_edges);
    EXPECT_EQ(nodes, exp_nodes);
  };

  uint32_t cache_count = 0;
  test_tile(8, 3418, 3, 4, cache_count);    // only primary and upper
  test_tile(10, 5002, 7, 12, cache_count);  // adds secondary
  test_tile(11, 5717, 10, 12, cache_count); // adds tertiary & unclassified
  test_tile(12, 5717, 10, 12, cache_count); // same as 11
  test_tile(13, 7205, 14, 20, cache_count); // adds residential
  test_tile(14, 7903, 17, 20, cache_count); // adds service/other
  // per default we only cache from z11 on
  EXPECT_EQ(cache_count, 4);

  // execute the cache path
  test_tile(14, 7903, 17, 20, cache_count);

  // make sure we fail the request when z exceeds what the server supports
  EXPECT_THROW(
      {
        try {
          test_tile(16, 7903, 17, 20, cache_count);
        } catch (const valhalla_exception_t& e) {
          EXPECT_EQ(e.code, 175);
          EXPECT_STREQ(e.what(), "Exceeded max zoom level of: 14");
          throw;
        }
      },
      valhalla_exception_t);
}
