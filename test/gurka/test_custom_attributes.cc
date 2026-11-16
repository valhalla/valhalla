#include "baldr/custom_attributes_tile.h"
#include "baldr/graphreader.h"
#include "baldr/graphtile.h"
#include "gurka.h"
#include "microtar.h"

#include <gtest/gtest.h>
#include <vtzero/vector_tile.hpp>

#include <filesystem>
#include <functional>
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

  GraphReader reader(map.config.get_child("mjolnir"));
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
