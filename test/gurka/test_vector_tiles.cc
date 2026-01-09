#include "baldr/attributes_controller.h"
#include "gurka.h"
#include "loki/tiles.h"
#include "loki/utils.h"
#include "loki/worker.h"
#include "midgard/constants.h"
#include "proto_conversions.h"
#include "test.h"

#include <valhalla/exceptions.h>

#include <gmock/gmock-matchers.h>
#include <gtest/gtest.h>
#include <vtzero/vector_tile.hpp>

#include <format>
#include <set>

using namespace valhalla;
using namespace valhalla::midgard;

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

  EXPECT_LT(tile_data.size(), 2800);
  EXPECT_GT(tile_data.size(), 2600);

  // expect a non-verbose request to have a lot less size
  std::string tile_data_slim;
  auto api_slim = gurka::do_action(Options::tile, map, "B", 14, "auto", {{"/verbose", "0"}}, nullptr,
                                   &tile_data_slim);
  EXPECT_LT(tile_data_slim.size(), tile_data.size() / 2);

  vtzero::vector_tile tile{tile_data};

  while (auto layer = tile.next_layer()) {
    EXPECT_TRUE(layer.num_features() > 0);

    std::string layer_name = std::string(layer.name());
    if (layer_name == "edges") {
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
      EXPECT_EQ(layer.version(), 2);
      EXPECT_EQ(layer.extent(), 4096);

      EXPECT_EQ(layer.num_features(), 2);

      auto feature = layer.next_feature();

      EXPECT_TRUE(feature.has_id()) << "Node feature should have an ID";
      EXPECT_GT(feature.id(), 0) << "Node feature ID should be positive";
      EXPECT_EQ(feature.geometry_type(), vtzero::GeomType::POINT) << "Node should be a point";

      std::set<std::string> expected_props = {"tile_level",        "tile_id",
                                              "node_id",           "type",
                                              "tagged_access",     "access:auto",
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
}

class VectorTiles : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetupTest() {
    const auto cache_dir = map.config.get<std::string>("loki.service_defaults.mvt_cache_dir");
    if (std::filesystem::exists(cache_dir)) {
      std::filesystem::remove_all(cache_dir);
    }
  }

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
  test_tile(8, 3297, 3, 4, cache_count);    // only primary and upper
  test_tile(10, 4787, 7, 12, cache_count);  // adds secondary
  test_tile(11, 5717, 11, 12, cache_count); // adds tertiary & unclassified
  test_tile(12, 5717, 11, 12, cache_count); // same as 11
  test_tile(13, 7205, 15, 20, cache_count); // adds residential
  test_tile(14, 7903, 18, 20, cache_count); // adds service/other
  // per default we only cache from z11 on
  EXPECT_EQ(cache_count, 4);

  // execute the cache path
  test_tile(14, 7903, 18, 20, cache_count);

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

TEST_F(VectorTiles, FilterIncludeExclude) {

  // verbose is set true in do_action
  std::string tile_data_full;
  Api api_full = gurka::do_action(Options::tile, map, "x", 10, "auto", {}, nullptr, &tile_data_full);

  // get a map with settings to not allow cache
  auto map_no_cache = map;
  auto current_config = map_no_cache.config.get<std::string>("loki.service_defaults.mvt_cache_dir");
  EXPECT_TRUE(current_config != "");
  map_no_cache.config.put("loki.service_defaults.mvt_cache_dir", "");

  auto test_tile_filter = [&](const uint32_t z, const std::string& filter_attribute,
                              const valhalla::FilterAction& filter_action,
                              const gurka::map& current_map, uint32_t& cache_count) {
    SCOPED_TRACE(std::format("Zoom {} failed, action: {}", z, FilterAction_Enum_Name(filter_action)));

    const std::unordered_map<std::string, std::string> options =
        {{"/verbose", "0"},
         {"/filters/attributes/0", filter_attribute},
         {"/filters/action", FilterAction_Enum_Name(filter_action)}};
    std::string tile_data_filter;
    Api api_filter = gurka::do_action(Options::tile, current_map, "x", z, "auto", options, nullptr,
                                      &tile_data_filter);

    EXPECT_LT(tile_data_filter.size(), tile_data_full.size());

    // check that cache worked
    if (const std::string cache_dir =
            current_map.config.get<std::string>("loki.service_defaults.mvt_cache_dir");
        !cache_dir.empty() &&
        z >= map.config.get<uint32_t>("loki.service_defaults.mvt_cache_min_zoom")) {
      const auto x = api_filter.options().tile_xyz().x();
      const auto y = api_filter.options().tile_xyz().y();
      const auto tile_path = loki::detail::mvt_local_path(z, x, y, cache_dir);
      EXPECT_TRUE(std::filesystem::exists(tile_path)) << "path doesn't exist: " + tile_path.string();
      cache_count++;
    }

    std::set<std::string_view> expected_props{"tile_level", "tile_id", "road_class"};
    // we need a separate size for edge_id, which is 2 attributes in the tiles and only one in the
    // controller
    auto expected_props_size = expected_props.size();
    if (filter_action == valhalla::include) {
      expected_props.insert(filter_attribute);
      expected_props_size++;
    } else {
      // need to go the other way around: from the full set we want to remove one
      auto add_to_expected = [&]<typename T, std::size_t N>(T(&arr)[N]) {
        for (size_t i = 0; i < N; ++i) {
          std::string_view attr = arr[i].attribute_flag;
          if (attr != filter_attribute) {
            expected_props.insert(attr);
            expected_props_size++;
          }
        };
      };

      add_to_expected(loki::detail::kSharedEdgeAttributes);
      add_to_expected(loki::detail::kForwardEdgeAttributes);
      add_to_expected(loki::detail::kReverseEdgeAttributes);
      add_to_expected(loki::detail::kForwardLiveSpeedAttributes);
      add_to_expected(loki::detail::kReverseLiveSpeedAttributes);
      if (filter_attribute != "edge.id") {
        expected_props.insert("edge.id");
        // in the tiles it's 2 attributes
        expected_props_size += 2;
      }
    }
    vtzero::vector_tile tile_filter{tile_data_filter};

    while (auto layer = tile_filter.next_layer()) {
      EXPECT_TRUE(layer.num_features() > 0);

      std::string layer_name = std::string(layer.name());
      if (layer_name == "edges") {
        auto feature = layer.next_feature();
        std::set<std::string_view> found_props;
        while (auto property = feature.next_property()) {
          std::string_view key{property.key().data(), property.key().size()};
          std::string_view loki_key;
          try {
            loki_key = loki::detail::kEdgePropToAttributeFlag.at(key);
          } catch (...) { continue; }

          EXPECT_TRUE(expected_props.count(loki_key) > 0) << "Edge should have property: " << key;
        }
        EXPECT_EQ(feature.num_properties(), expected_props_size);
      }
    }

    return tile_data_filter.size();
  };

  uint32_t cache_count = 0;
  // no cache allowed, filter include
  auto no_cache_size =
      test_tile_filter(8, "edge.speed_forward", valhalla::include, map_no_cache, cache_count);
  // no cache allowed, filter exclude
  test_tile_filter(8, "edge.speed_forward", valhalla::exclude, map_no_cache, cache_count);
  EXPECT_EQ(cache_count, 0);

  // cold cache
  // cache allowed, filter include
  auto cold_size = test_tile_filter(14, "edge.id", valhalla::include, map, cache_count);
  // cache allowed, filter exclude
  test_tile_filter(14, "edge.id", valhalla::exclude, map, cache_count);
  EXPECT_EQ(cache_count, 2);

  // warm cache
  cache_count = 0;
  // cache allowed, filter include
  auto warm_size = test_tile_filter(14, "edge.is_shortcut", valhalla::include, map, cache_count);
  // cache allowed, filter exclude
  test_tile_filter(14, "edge.id", valhalla::exclude, map, cache_count);
  EXPECT_EQ(cache_count, 2);

  EXPECT_EQ(cold_size, warm_size);
  // TODO: for some reason the tiles with no cache a magnitude smaller than the ones with cache
  // EXPECT_EQ(cold_size, no_cache_size);
}
