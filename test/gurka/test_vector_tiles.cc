#include "gurka.h"
#include "loki/worker.h"
#include "midgard/constants.h"

#include <valhalla/exceptions.h>

#include <gtest/gtest.h>
#include <vtzero/vector_tile.hpp>

#include <format>
#include <set>

using namespace valhalla;
using namespace valhalla::midgard;

TEST(VectorTiles, BasicTileRendering) {
  constexpr double gridsize = 100;

  const std::string ascii_map = R"(
      A---B---C
          |
          D
  )";

  const gurka::ways ways = {
      {"AB", {{"highway", "primary"}, {"name", "Main Street"}, {"osm_id", "10"}}},
      {"BC", {{"highway", "primary"}, {"name", "Main Street"}, {"osm_id", "11"}}},
      {"BD", {{"highway", "secondary"}, {"name", "Side Street"}, {"osm_id", "12"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize, {2.315260, 48.869168});
  auto map = gurka::buildtiles(layout, ways, {}, {}, VALHALLA_BUILD_DIR "test/data/gurka_vt_basic");

  std::string tile_data;
  auto api = gurka::do_action(Options::tile, map, "B", 14, "auto", {}, nullptr, &tile_data);

  EXPECT_GT(tile_data.size(), 3500);
  EXPECT_LT(tile_data.size(), 3600);

  vtzero::vector_tile tile{tile_data};

  bool has_edges = false;
  bool has_nodes = false;

  while (auto layer = tile.next_layer()) {
    std::string layer_name = std::string(layer.name());

    if (layer_name == "edges") {
      has_edges = true;

      EXPECT_EQ(layer.version(), 2);
      EXPECT_EQ(layer.extent(), 4096);

      EXPECT_EQ(layer.num_features(), 3);

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

      EXPECT_EQ(layer.num_features(), 5);

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

TEST(VectorTiles, BasicTileRenderingOnDifferentZoomLevels) {
  constexpr double gridsize = 500;

  const std::string ascii_map = R"(
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
      {"AE", {{"highway", "primary"}, {"name", "First Avenue"}}},
      {"BF", {{"highway", "primary"}, {"name", "Second Avenue"}}},
      {"CG", {{"highway", "primary"}, {"name", "Third Avenue"}}},
      {"DH", {{"highway", "primary"}, {"name", "Fourth Avenue"}}},
      {"EF", {{"highway", "primary"}, {"name", "Oak Street"}}},
      {"FG", {{"highway", "primary"}, {"name", "Oak Street"}}},
      {"GH", {{"highway", "primary"}, {"name", "Oak Street"}}},
      {"EI", {{"highway", "secondary"}, {"name", "Pine Avenue"}}},
      {"FJ", {{"highway", "secondary"}, {"name", "Elm Avenue"}}},
      {"GK", {{"highway", "secondary"}, {"name", "Maple Avenue"}}},
      {"HL", {{"highway", "secondary"}, {"name", "Cedar Avenue"}}},
      {"IJ", {{"highway", "secondary"}, {"name", "South Street"}}},
      {"JK", {{"highway", "secondary"}, {"name", "South Street"}}},
      {"KL", {{"highway", "secondary"}, {"name", "South Street"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize, {2.315260, 48.869168});
  auto map =
      gurka::buildtiles(layout, ways, {}, {}, VALHALLA_BUILD_DIR "test/data/gurka_vt_zoom_compare");

  auto test_tile = [&](const uint32_t z, const uint32_t exp_total_size, const uint32_t exp_edges,
                       const uint32_t exp_nodes) {
    SCOPED_TRACE(std::format("Zoom {} failed", z));
    std::string tile_data;
    std::ignore = gurka::do_action(Options::tile, map, "F", z, "auto", {}, nullptr, &tile_data);

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

  test_tile(8, 5369, 10, 8);
  test_tile(12, 6621, 14, 12);
}
