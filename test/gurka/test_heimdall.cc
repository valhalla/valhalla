#include "gurka.h"
#include "heimdall/worker.h"
#include "midgard/constants.h"

#include <valhalla/exceptions.h>

#include <gtest/gtest.h>
#include <vtzero/vector_tile.hpp>

#include <set>

using namespace valhalla;
using namespace valhalla::midgard;

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
      (1.0 - std::asinh(std::tan(node_b.lat() * kPiDouble / 180.0)) / kPiDouble) / 2.0 * n);

  // Render the tile
  auto tile_data = worker.render_tile(z, x, y);

  ASSERT_EQ(tile_data.size(), 2768);

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
      (1.0 - std::asinh(std::tan(node_f.lat() * kPiDouble / 180.0)) / kPiDouble) / 2.0 * n10);

  // zoom 12
  uint32_t z12 = 12;
  double n12 = std::pow(2.0, z12);
  uint32_t x12 = static_cast<uint32_t>((node_f.lng() + 180.0) / 360.0 * n12);
  uint32_t y12 = static_cast<uint32_t>(
      (1.0 - std::asinh(std::tan(node_f.lat() * kPiDouble / 180.0)) / kPiDouble) / 2.0 * n12);

  // Render tile at zoom 10
  auto tile_data_z10 = worker.render_tile(z10, x10, y10);
  EXPECT_EQ(tile_data_z10.size(), 4626);

  // Render tile at zoom 12
  auto tile_data_z12 = worker.render_tile(z12, x12, y12);
  ASSERT_FALSE(tile_data_z12.empty());
  EXPECT_EQ(tile_data_z12.size(), 6833);

  // Parse Z10 tile
  vtzero::vector_tile tile_z10{tile_data_z10};
  uint32_t edges_z10 = 0;
  uint32_t nodes_z10 = 0;

  while (auto layer = tile_z10.next_layer()) {
    std::string layer_name = std::string(layer.name());
    if (layer_name == "edges") {
      edges_z10 = layer.num_features();
      EXPECT_EQ(layer.version(), 2);
      EXPECT_EQ(layer.extent(), 4096);
    } else if (layer_name == "nodes") {
      nodes_z10 = layer.num_features();
    } else {
      FAIL() << "Unexpected layer: " << layer_name;
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
      EXPECT_EQ(layer.version(), 2);
      EXPECT_EQ(layer.extent(), 4096);
    } else if (layer_name == "nodes") {
      nodes_z12 = layer.num_features();
    } else {
      FAIL() << "Unexpected layer: " << layer_name;
    }
  }

  EXPECT_EQ(edges_z10, 7);
  EXPECT_EQ(nodes_z10, 8);

  EXPECT_EQ(edges_z12, 14);
  EXPECT_EQ(nodes_z12, 16);
}
