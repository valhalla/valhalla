#include "baldr/custom_attributes_tile.h"
#include "baldr/graphreader.h"
#include "baldr/graphtile.h"
#include "gurka.h"
#include "microtar.h"
#include "mjolnir/custom_attributes_builder.h"

#include <gtest/gtest.h>
#include <vtzero/vector_tile.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <functional>
#include <limits>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

using namespace valhalla;
using namespace valhalla::baldr;

namespace {

const std::string kAttributeDefs = R"([{"name": "scenic", "type": "uint8", "scale": 0.01},
                                       {"name": "quality", "type": "uint8"}])";

// raw stored values of an edge, one per attribute in kAttributeDefs
using RowFunc = std::function<std::vector<uint8_t>(uint32_t edge_index)>;

std::vector<uint8_t> test_row(uint32_t edge_index) {
  return {static_cast<uint8_t>(edge_index % 100 + 1), static_cast<uint8_t>(200 - edge_index % 100)};
}

void write_tar_entry(mtar_t& tar, const std::string& name, const std::string& data) {
  if (mtar_write_file_header(&tar, name.c_str(), data.size()) != MTAR_ESUCCESS ||
      mtar_write_data(&tar, data.data(), data.size()) != MTAR_ESUCCESS) {
    throw std::runtime_error("Could not write " + name + " to the custom attributes tar");
  }
}

// writes a custom attributes tar for every graph tile of the map and points the config at it
void build_custom_attributes_extract(gurka::map& map,
                                     const std::string& attribute_defs,
                                     const RowFunc& row_func) {
  const auto defs = ParseCustomAttributeDefs(attribute_defs);
  const auto row_size = custom_attributes_row_size(defs);
  const auto extract_path = std::filesystem::path(map.config.get<std::string>("mjolnir.tile_dir")) /
                            "custom_attributes.tar";

  mtar_t tar;
  if (mtar_open(&tar, extract_path.string().c_str(), "w") != MTAR_ESUCCESS) {
    throw std::runtime_error("Could not create " + extract_path.string());
  }
  write_tar_entry(tar, "attributes.json", attribute_defs);

  // the tar being rewritten must not be mapped by this reader
  auto mjolnir_config = map.config.get_child("mjolnir");
  mjolnir_config.erase("custom_attributes_extract");
  GraphReader reader(mjolnir_config);
  for (const auto tile_id : reader.GetTileSet()) {
    const auto tile = reader.GetGraphTile(tile_id);
    CustomAttributesTileHeader header{};
    header.tile_id = tile_id;
    header.num_attributes = defs.size();
    header.row_size = row_size;
    header.directed_edge_count = tile->header()->directededgecount();
    header.custom_attributes_tile_version = CUSTOM_ATTRIBUTES_TILE_VERSION;

    std::string data(reinterpret_cast<const char*>(&header), sizeof(header));
    for (uint32_t i = 0; i < header.directed_edge_count; ++i) {
      const auto values = row_func(i);
      data.append(reinterpret_cast<const char*>(values.data()), values.size());
    }
    write_tar_entry(tar, GraphTile::FileSuffix(tile_id), data);
  }
  mtar_finalize(&tar);
  mtar_close(&tar);

  map.config.put("mjolnir.custom_attributes_extract", extract_path.string());
}

class CustomAttributes : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    const std::string ascii_map = R"(
      A-----B-----C
            |
            |
            D-----E
    )";
    const gurka::ways ways = {
        {"AB", {{"highway", "residential"}}},
        {"BC", {{"highway", "residential"}}},
        {"BD", {{"highway", "residential"}}},
        {"DE", {{"highway", "residential"}}},
    };
    const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100, {5.1079374, 52.0887254});
    const std::string workdir = VALHALLA_BUILD_DIR "test/data/gurka_custom_attributes";
    const std::string cache_dir = workdir + "/mvt_cache";
    std::filesystem::remove_all(cache_dir);
    map = gurka::buildtiles(layout, ways, {}, {}, workdir,
                            {{"mjolnir.concurrency", "1"},
                             {"loki.service_defaults.mvt_cache_dir", cache_dir}});
    build_custom_attributes_extract(map, kAttributeDefs, test_row);
  }
};
gurka::map CustomAttributes::map = {};

std::string tile_request(const gurka::map& map, const std::vector<std::string>& attributes) {
  std::unordered_map<std::string, std::string> options = {{"/verbose", "0"}};
  for (size_t i = 0; i < attributes.size(); ++i) {
    options["/filters/attributes/" + std::to_string(i)] = attributes[i];
  }
  if (!attributes.empty()) {
    options["/filters/action"] = "include";
  }
  std::string tile_data;
  gurka::do_action(Options::tile, map, "B", 14, "auto", options, nullptr, &tile_data);
  return tile_data;
}

} // namespace

TEST_F(CustomAttributes, MVTProperties) {
  const auto defs = ParseCustomAttributeDefs(kAttributeDefs);

  // the first request at a cacheable zoom builds and caches the full tile, the second one is
  // filtered from that cache and must drop the custom attributes again
  for (const bool with_custom_attributes : {true, false}) {
    SCOPED_TRACE(with_custom_attributes ? "with edge.custom_attribute"
                                        : "without edge.custom_attribute");
    std::vector<std::string> attributes{"edge.id"};
    if (with_custom_attributes) {
      attributes.emplace_back("edge.custom_attribute");
    }
    const std::string tile_data = tile_request(map, attributes);
    vtzero::vector_tile tile{tile_data};

    auto layer = tile.get_layer_by_name("edges");
    ASSERT_FALSE(layer.empty());
    uint32_t checked_features = 0;
    while (auto feature = layer.next_feature()) {
      std::unordered_map<std::string, vtzero::property_value> props;
      feature.for_each_property([&](const vtzero::property& prop) {
        props.emplace(std::string(prop.key().data(), prop.key().size()), prop.value());
        return true;
      });

      if (!with_custom_attributes) {
        for (const auto& [key, value] : props) {
          EXPECT_FALSE(key.starts_with("custom:")) << key;
        }
        ++checked_features;
        continue;
      }

      for (const auto& [suffix, edge_id_key] :
           {std::pair{":fwd", "edge_id:fwd"}, std::pair{":bwd", "edge_id:bwd"}}) {
        ASSERT_TRUE(props.contains(edge_id_key)) << edge_id_key;
        const auto edge_index = static_cast<uint32_t>(props.at(edge_id_key).uint_value());
        const auto raw = test_row(edge_index);
        for (size_t i = 0; i < defs.size(); ++i) {
          const std::string key = "custom:" + defs[i].name + suffix;
          ASSERT_TRUE(props.contains(key)) << key;
          EXPECT_FLOAT_EQ(props.at(key).float_value(), raw[i] * defs[i].scale) << key;
        }
      }
      ++checked_features;
    }
    EXPECT_GT(checked_features, 0u);
  }
}

TEST(AddCustomAttributes, LuaValuesPerDirection) {
  const std::string ascii_map = R"(
      A-----B-----C
            |
            |
            D-----E
  )";
  const gurka::ways ways = {
      {"AB", {{"highway", "residential"}, {"smoothness", "good"}}},
      {"BC", {{"highway", "residential"}, {"smoothness", "bad"}}},
      {"BD", {{"highway", "residential"}}},
      {"DE", {{"highway", "residential"}, {"smoothness", "excellent"}}},
  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100, {5.1079374, 52.0887254});
  const std::string workdir = VALHALLA_BUILD_DIR "test/data/gurka_add_custom_attributes";
  auto map = gurka::buildtiles(layout, ways, {}, {}, workdir, {{"mjolnir.concurrency", "1"}});

  // every edge starts out with scenic 0.07 and quality 9
  build_custom_attributes_extract(map, kAttributeDefs,
                                  [](uint32_t) { return std::vector<uint8_t>{7, 9}; });

  const std::string lua = R"(
    function custom_attributes(tags)
      if tags.smoothness == "good" then
        return {scenic = 0.5, quality = 3, unknown = 1}
      elseif tags.smoothness == "bad" then
        return {scenic = 0.2}, {scenic = 0.8}
      elseif tags.smoothness == "excellent" then
        return {quality = 999}
      end
    end
  )";
  // AB: 2 attributes on 2 edges, BC: 1 attribute per direction, DE: 1 attribute on 2 edges
  EXPECT_EQ(mjolnir::add_custom_attributes(map.config, {workdir + "/map.pbf"}, {"smoothness"}, lua,
                                           mjolnir::ShortcutStrategy::kNone),
            8u);

  GraphReader reader(map.config.get_child("mjolnir"));
  const auto& defs = reader.GetCustomAttributeDefs();
  ASSERT_EQ(defs.size(), 2u);
  auto expect_values = [&](const std::string& from, const std::string& to, float scenic,
                           float quality) {
    SCOPED_TRACE(from + to);
    const auto [edge_id, edge] = gurka::findEdgeByNodes(reader, map.nodes, from, to);
    const auto tile = reader.GetGraphTile(edge_id);
    ASSERT_NE(tile->custom_attributes_tile(), nullptr);
    EXPECT_NEAR(tile->custom_attributes_tile()->value(edge_id.id(), defs[0]), scenic, 1e-4);
    EXPECT_NEAR(tile->custom_attributes_tile()->value(edge_id.id(), defs[1]), quality, 1e-4);
  };

  expect_values("A", "B", 0.5f, 3.f);
  expect_values("B", "A", 0.5f, 3.f);
  expect_values("B", "C", 0.2f, 9.f);
  expect_values("C", "B", 0.8f, 9.f);

  // no smoothness tag
  expect_values("B", "D", 0.07f, 9.f);
  expect_values("D", "B", 0.07f, 9.f);

  // clamped to the maximum
  expect_values("D", "E", 0.07f, 255.f);
  expect_values("E", "D", 0.07f, 255.f);
}

class ShortcutStrategies : public ::testing::TestWithParam<mjolnir::ShortcutStrategy> {
protected:
  static gurka::map map;
  static std::string pbf;

  static void SetUpTestSuite() {
    const std::string ascii_map = R"(
      A-B-----C--D
    )";
    const gurka::ways ways = {
        {"AB", {{"highway", "motorway"}, {"name", "A1"}, {"test_scenic", "0.2"}}},
        {"BC", {{"highway", "motorway"}, {"name", "A1"}, {"test_scenic", "0.6"}}},
        {"CD", {{"highway", "motorway"}, {"name", "A1"}, {"test_scenic", "0.4"}}},
    };
    const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100, {5.1079374, 52.0887254});
    const std::string workdir = VALHALLA_BUILD_DIR "test/data/gurka_custom_attributes_shortcuts";
    map = gurka::buildtiles(layout, ways, {}, {}, workdir, {{"mjolnir.concurrency", "1"}});
    pbf = workdir + "/map.pbf";
  }

  // rebuilds the extract with scenic 0.07 and quality 9 everywhere, then runs the Lua script
  void add_attributes(mjolnir::ShortcutStrategy strategy) {
    build_custom_attributes_extract(map, kAttributeDefs,
                                    [](uint32_t) { return std::vector<uint8_t>{7, 9}; });
    const std::string lua = R"(
      function custom_attributes(tags)
        return {scenic = tonumber(tags.test_scenic)}
      end
    )";
    mjolnir::add_custom_attributes(map.config, {pbf}, {"test_scenic"}, lua, strategy);
  }
};
gurka::map ShortcutStrategies::map = {};
std::string ShortcutStrategies::pbf = {};

// scenic and quality of an edge
std::pair<float, float> edge_values(GraphReader& reader, const GraphId& edge_id) {
  const auto tile = reader.GetGraphTile(edge_id);
  const auto* ca_tile = tile->custom_attributes_tile();
  if (!ca_tile) {
    ADD_FAILURE() << "no custom attributes tile for " << edge_id;
    return {};
  }
  const auto& defs = reader.GetCustomAttributeDefs();
  return {ca_tile->value(edge_id.id(), defs[0]), ca_tile->value(edge_id.id(), defs[1])};
}

TEST_F(ShortcutStrategies, NoneKeepsShortcutDefaults) {
  add_attributes(mjolnir::ShortcutStrategy::kNone);

  GraphReader reader(map.config.get_child("mjolnir"));
  const auto [shortcut_id, shortcut] = gurka::findEdgeByNodes(reader, map.nodes, "A", "D");
  ASSERT_TRUE(shortcut && shortcut->is_shortcut());

  // the base edges got their Lua values, the shortcut kept the extract's defaults
  EXPECT_NEAR(edge_values(reader, std::get<0>(gurka::findEdgeByNodes(reader, map.nodes, "B", "C")))
                  .first,
              0.6f, 1e-4);
  const auto [scenic, quality] = edge_values(reader, shortcut_id);
  EXPECT_NEAR(scenic, 0.07f, 1e-4);
  EXPECT_NEAR(quality, 9.f, 1e-4);
}

TEST_P(ShortcutStrategies, ShortcutFromBaseEdges) {
  add_attributes(GetParam());

  GraphReader reader(map.config.get_child("mjolnir"));
  const auto [shortcut_id, shortcut] = gurka::findEdgeByNodes(reader, map.nodes, "A", "D");
  ASSERT_TRUE(shortcut && shortcut->is_shortcut());

  const auto edges = reader.RecoverShortcut(shortcut_id);
  ASSERT_EQ(edges.size(), 3u);
  float min = std::numeric_limits<float>::max(), max = 0.f;
  double weighted_sum = 0.0, total_length = 0.0;
  for (const auto& edge_id : edges) {
    const float scenic = edge_values(reader, edge_id).first;
    const auto length = reader.directededge(edge_id)->length();
    min = std::min(min, scenic);
    max = std::max(max, scenic);
    weighted_sum += scenic * length;
    total_length += length;
  }
  ASSERT_NEAR(min, 0.2f, 1e-4);
  ASSERT_NEAR(max, 0.6f, 1e-4);
  // the edges differ in length, so the weighted mean is not the plain mean of 0.4
  ASSERT_GT(std::abs(weighted_sum / total_length - 0.4), 0.02);

  float expected = 0.f;
  switch (GetParam()) {
    case mjolnir::ShortcutStrategy::kMin:
      expected = min;
      break;
    case mjolnir::ShortcutStrategy::kMax:
      expected = max;
      break;
    default:
      expected = static_cast<float>(weighted_sum / total_length);
      break;
  }
  const auto [scenic, quality] = edge_values(reader, shortcut_id);
  // the stored value is rounded to the attribute's scale of 0.01
  EXPECT_NEAR(scenic, expected, 0.005 + 1e-4);
  // Lua never set quality, so every base edge and the shortcut keep the default
  EXPECT_NEAR(quality, 9.f, 1e-4);
}

INSTANTIATE_TEST_SUITE_P(Strategies,
                         ShortcutStrategies,
                         ::testing::Values(mjolnir::ShortcutStrategy::kMin,
                                           mjolnir::ShortcutStrategy::kMax,
                                           mjolnir::ShortcutStrategy::kMean),
                         [](const auto& info) {
                           switch (info.param) {
                             case mjolnir::ShortcutStrategy::kMin:
                               return "Min";
                             case mjolnir::ShortcutStrategy::kMax:
                               return "Max";
                             default:
                               return "Mean";
                           }
                         });
