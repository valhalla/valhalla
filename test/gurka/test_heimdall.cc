#include "gurka.h"
#include "heimdall/worker.h"

#include <valhalla/exceptions.h>

#include <gtest/gtest.h>
#include <vtzero/vector_tile.hpp>

#include <set>

using namespace valhalla;

TEST(Heimdall, BasicTileRendering) {
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
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_heimdall_basic");

  auto graph_reader = std::make_shared<baldr::GraphReader>(map.config.get_child("mjolnir"));
  heimdall::heimdall_worker_t worker(map.config, graph_reader);

  const auto& node_b = layout.at("B");

  // Calculate which tile contains this point at zoom 14
  // Using standard slippy map tile formula
  uint32_t z = 14;
  double n = std::pow(2.0, z);
  uint32_t x = static_cast<uint32_t>((node_b.lng() + 180.0) / 360.0 * n);
  uint32_t y = static_cast<uint32_t>(
      (1.0 - std::asinh(std::tan(node_b.lat() * M_PI / 180.0)) / M_PI) / 2.0 * n);

  // Render the tile
  auto tile_data = worker.render_tile(z, x, y);

  // Verify we got data back
  ASSERT_FALSE(tile_data.empty()) << "Tile data should not be empty";
  ASSERT_GT(tile_data.size(), 100) << "Tile data should be at least 100 bytes";

  // Parse the MVT tile
  vtzero::vector_tile tile{tile_data};

  bool has_edges = false;
  bool has_nodes = false;
  uint32_t edge_feature_count = 0;
  uint32_t node_feature_count = 0;

  while (auto layer = tile.next_layer()) {
    std::string layer_name = std::string(layer.name());

    if (layer_name == "edges") {
      has_edges = true;
      edge_feature_count = layer.num_features();

      // Verify layer metadata
      EXPECT_EQ(layer.version(), 2) << "Layer version should be 2";
      EXPECT_EQ(layer.extent(), 4096) << "Layer extent should be 4096";

      // Verify edge features
      uint32_t feature_idx = 0;
      while (auto feature = layer.next_feature()) {
        if (feature_idx == 0) {
          // Check ID
          EXPECT_TRUE(feature.has_id()) << "Edge feature should have an ID";
          EXPECT_GT(feature.id(), 0) << "Edge feature ID should be positive";

          // Check geometry type
          EXPECT_EQ(feature.geometry_type(), vtzero::GeomType::LINESTRING)
              << "Edge should be a linestring";

          // Verify expected edge properties exist
          std::set<std::string> expected_props =
              {"tile_level",      "tile_id",         "road_class",         "use",
               "length",          "edge_id:forward", "speed:forward",      "access:auto:forward",
               "edge_id:reverse", "speed:reverse",   "access:auto:reverse"};
          std::set<std::string> found_props;

          while (auto property = feature.next_property()) {
            std::string key = std::string(property.key());
            found_props.insert(key);
          }

          // Check that all expected properties are present
          for (const auto& prop : expected_props) {
            EXPECT_TRUE(found_props.count(prop) > 0) << "Edge should have property: " << prop;
          }
        }
        feature_idx++;
      }

      // Expect at least 1 edge feature (depending on tile boundaries, not all edges may be visible)
      EXPECT_EQ(edge_feature_count, 1) << "Should have exactly 1 edge feature in this tile";

    } else if (layer_name == "nodes") {
      has_nodes = true;
      node_feature_count = layer.num_features();

      // Verify layer metadata
      EXPECT_EQ(layer.version(), 2) << "Layer version should be 2";
      EXPECT_EQ(layer.extent(), 4096) << "Layer extent should be 4096";

      // Verify node features
      uint32_t feature_idx = 0;
      while (auto feature = layer.next_feature()) {
        if (feature_idx == 0) {
          // Check ID
          EXPECT_TRUE(feature.has_id()) << "Node feature should have an ID";
          EXPECT_GT(feature.id(), 0) << "Node feature ID should be positive";

          // Check geometry type
          EXPECT_EQ(feature.geometry_type(), vtzero::GeomType::POINT) << "Node should be a point";

          // Verify expected node properties exist
          std::set<std::string> expected_props = {"tile_level",        "tile_id",
                                                  "node_id",           "type",
                                                  "edge_count",        "access:auto",
                                                  "access:pedestrian", "access:bicycle"};
          std::set<std::string> found_props;

          while (auto property = feature.next_property()) {
            std::string key = std::string(property.key());
            found_props.insert(key);
          }

          // Check that all expected properties are present
          for (const auto& prop : expected_props) {
            EXPECT_TRUE(found_props.count(prop) > 0) << "Node should have property: " << prop;
          }
        }
        feature_idx++;
      }

      // Expect exactly 1 node feature in this tile (only end nodes are included)
      EXPECT_EQ(node_feature_count, 1) << "Should have exactly 1 node feature in this tile";
    }
  }

  // Verify both expected layers exist
  EXPECT_TRUE(has_edges) << "Tile should contain 'edges' layer";
  EXPECT_TRUE(has_nodes) << "Tile should contain 'nodes' layer";
}

TEST(Heimdall, BasicTileRenderingOnDifferentZoomLevels) {
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

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_heimdall_zoom_compare");

  auto graph_reader = std::make_shared<baldr::GraphReader>(map.config.get_child("mjolnir"));
  heimdall::heimdall_worker_t worker(map.config, graph_reader);

  // Use center node to calculate tile coordinates
  const auto& node_f = layout.at("F"); // Center node

  // zoom 10
  uint32_t z10 = 10;
  double n10 = std::pow(2.0, z10);
  uint32_t x10 = static_cast<uint32_t>((node_f.lng() + 180.0) / 360.0 * n10);
  uint32_t y10 = static_cast<uint32_t>(
      (1.0 - std::asinh(std::tan(node_f.lat() * M_PI / 180.0)) / M_PI) / 2.0 * n10);

  // zoom 12
  uint32_t z12 = 12;
  double n12 = std::pow(2.0, z12);
  uint32_t x12 = static_cast<uint32_t>((node_f.lng() + 180.0) / 360.0 * n12);
  uint32_t y12 = static_cast<uint32_t>(
      (1.0 - std::asinh(std::tan(node_f.lat() * M_PI / 180.0)) / M_PI) / 2.0 * n12);

  // Render tile at zoom 10
  auto tile_data_z10 = worker.render_tile(z10, x10, y10);
  ASSERT_FALSE(tile_data_z10.empty()) << "Z10 tile data should not be empty";

  // Render tile at zoom 12
  auto tile_data_z12 = worker.render_tile(z12, x12, y12);
  ASSERT_FALSE(tile_data_z12.empty()) << "Z12 tile data should not be empty";

  // Parse Z10 tile
  vtzero::vector_tile tile_z10{tile_data_z10};
  uint32_t edges_z10 = 0;
  uint32_t nodes_z10 = 0;

  while (auto layer = tile_z10.next_layer()) {
    std::string layer_name = std::string(layer.name());
    if (layer_name == "edges") {
      edges_z10 = layer.num_features();
      EXPECT_EQ(layer.version(), 2) << "Z10 layer version should be 2";
      EXPECT_EQ(layer.extent(), 4096) << "Z10 layer extent should be 4096";
    } else if (layer_name == "nodes") {
      nodes_z10 = layer.num_features();
    }
  }

  // Parse Z12 tile
  vtzero::vector_tile tile_z12{tile_data_z12};
  uint32_t edges_z12 = 0;
  uint32_t nodes_z12 = 0;

  while (auto layer = tile_z12.next_layer()) {
    std::string layer_name = std::string(layer.name());
    if (layer_name == "edges") {
      edges_z12 = layer.num_features();
      EXPECT_EQ(layer.version(), 2) << "Z12 layer version should be 2";
      EXPECT_EQ(layer.extent(), 4096) << "Z12 layer extent should be 4096";
    } else if (layer_name == "nodes") {
      nodes_z12 = layer.num_features();
    }
  }

  EXPECT_EQ(edges_z10, 7);
  EXPECT_EQ(nodes_z10, 7);

  EXPECT_EQ(edges_z12, 14);
  EXPECT_EQ(nodes_z12, 11);
}
