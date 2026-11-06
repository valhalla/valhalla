#include "mjolnir/custom_attributes_builder.h"
#include "baldr/custom_attributes_tile.h"
#include "baldr/graphid.h"
#include "baldr/graphreader.h"
#include "midgard/logging.h"
#include "midgard/sequence.h"
#include "mjolnir/way_edges_processor.h"

#include <osmium/io/pbf_input.hpp>
#include <osmium/io/reader.hpp>
#include <osmium/osm/way.hpp>
#include <rapidjson/document.h>

#include <ctime>
#include <filesystem>
#include <format>
#include <stdexcept>
#include <string_view>
#include <unordered_map>
#include <unordered_set>

extern "C" {
#include <lauxlib.h>
#include <lua.h>
#include <lualib.h>
}

using namespace valhalla::baldr;

namespace valhalla {
namespace mjolnir {

namespace {

constexpr const char* kLuaFunction = "custom_attributes";

class CustomAttributesLua {
public:
  explicit CustomAttributesLua(const std::string& script) : state_(luaL_newstate()) {
    luaL_openlibs(state_);
    if (luaL_dostring(state_, script.c_str())) {
      std::string error = lua_tostring(state_, -1);
      lua_close(state_);
      throw std::runtime_error("Failed to load Lua script: " + error);
    }
    lua_getglobal(state_, kLuaFunction);
    const bool is_function = lua_isfunction(state_, -1);
    lua_pop(state_, 1);
    if (!is_function) {
      lua_close(state_);
      throw std::runtime_error(std::format("Lua script does not define {}(tags)", kLuaFunction));
    }
  }

  CustomAttributesLua(const CustomAttributesLua&) = delete;
  CustomAttributesLua& operator=(const CustomAttributesLua&) = delete;

  ~CustomAttributesLua() {
    lua_close(state_);
  }

  // Calls custom_attributes(tags) and hands every string -> number entry of the result to emit.
  // Returns the number of entries that were not string -> number.
  template <typename Emit> size_t call(const osmium::Way& way, Emit&& emit) {
    lua_getglobal(state_, kLuaFunction);
    lua_newtable(state_);
    for (const auto& tag : way.tags()) {
      lua_pushstring(state_, tag.key());
      lua_pushstring(state_, tag.value());
      lua_rawset(state_, -3);
    }

    if (lua_pcall(state_, 1, 1, 0)) {
      std::string error = lua_tostring(state_, -1);
      lua_pop(state_, 1);
      throw std::runtime_error(
          std::format("Lua {}() failed for way {}: {}", kLuaFunction, way.id(), error));
    }

    size_t invalid = 0;
    if (lua_istable(state_, -1)) {
      lua_pushnil(state_);
      while (lua_next(state_, -2) != 0) {
        // check the key type before lua_tostring, which would convert a number key in place
        if (lua_type(state_, -2) == LUA_TSTRING && lua_type(state_, -1) == LUA_TNUMBER) {
          emit(std::string_view(lua_tostring(state_, -2)),
               static_cast<float>(lua_tonumber(state_, -1)));
        } else {
          ++invalid;
        }
        lua_pop(state_, 1);
      }
    } else if (!lua_isnil(state_, -1)) {
      ++invalid;
    }
    lua_pop(state_, 1);
    return invalid;
  }

private:
  lua_State* state_;
};

struct WritableTile {
  CustomAttributesTileHeader* header;
  float* data;
};

// tiles of the tar keyed by tile base id, plus the attribute names from attributes.json
struct WriteableCustomAttributesExtract {
  midgard::tar archive;
  std::unordered_map<uint64_t, WritableTile> tiles;
  std::vector<std::string> names;

  explicit WriteableCustomAttributesExtract(const std::string& path) : archive(path, false) {
    archive.for_each([this](const std::string& name, const char* data, size_t size) {
      std::filesystem::path p{name};
      if (p.filename() == "attributes.json") {
        rapidjson::Document doc;
        doc.Parse(data, size);
        if (doc.IsArray()) {
          for (const auto& v : doc.GetArray()) {
            if (v.IsString()) {
              names.emplace_back(v.GetString(), v.GetStringLength());
            }
          }
        }
        return true;
      }
      if (p.extension() != ".gph") {
        return true;
      }
      if (size < sizeof(CustomAttributesTileHeader)) {
        LOG_WARN("Skipping {}: too small for header ({} bytes)", name, size);
        return true;
      }
      auto* header = reinterpret_cast<CustomAttributesTileHeader*>(const_cast<char*>(data));
      if (header->custom_attributes_tile_version != CUSTOM_ATTRIBUTES_TAR_VERSION) {
        LOG_WARN("Skipping {}: version {} != {}", name, header->custom_attributes_tile_version,
                 CUSTOM_ATTRIBUTES_TAR_VERSION);
        return true;
      }
      const size_t required = sizeof(CustomAttributesTileHeader) +
                              static_cast<size_t>(header->directed_edge_count) *
                                  static_cast<size_t>(header->num_attributes) * sizeof(float);
      if (size < required) {
        LOG_WARN("Skipping {}: {} bytes, expected {}", name, size, required);
        return true;
      }
      try {
        auto id = GraphId::FromTilePath(name);
        tiles[id.value] = {header, reinterpret_cast<float*>(const_cast<char*>(data) +
                                                            sizeof(CustomAttributesTileHeader))};
      } catch (...) { LOG_WARN("Skipping {}: not a tile path", name); }
      return true;
    });
  }
};

} // namespace

size_t add_custom_attributes(const boost::property_tree::ptree& config,
                             const std::vector<std::string>& input_files,
                             const std::vector<std::string>& tag_keys,
                             const std::string& lua_script) {
  const auto tar_path = config.get_optional<std::string>("mjolnir.custom_attributes_extract");
  if (!tar_path) {
    throw std::runtime_error("mjolnir.custom_attributes_extract is not set");
  }
  if (tag_keys.empty()) {
    throw std::runtime_error("At least one OSM tag key is required");
  }

  CustomAttributesLua lua(lua_script);

  WriteableCustomAttributesExtract ca_extract(*tar_path);
  if (ca_extract.names.empty() || ca_extract.tiles.empty()) {
    throw std::runtime_error(*tar_path + " has no attributes.json names or no usable tiles");
  }
  std::unordered_map<std::string, uint32_t> column;
  for (uint32_t i = 0; i < ca_extract.names.size(); ++i) {
    column.emplace(ca_extract.names[i], i);
  }
  for (const auto& [id, tile] : ca_extract.tiles) {
    if (tile.header->num_attributes != ca_extract.names.size()) {
      throw std::runtime_error(
          std::format("Tile {} has {} attributes, attributes.json lists {}", GraphId(id),
                      static_cast<uint32_t>(tile.header->num_attributes), ca_extract.names.size()));
    }
  }
  LOG_INFO("Custom attributes tar: {} tiles, {} attributes", ca_extract.tiles.size(),
           ca_extract.names.size());

  GraphReader reader(config.get_child("mjolnir"));
  const auto ways_edges = collect_way_edges(reader);
  LOG_INFO("Collected edges for {} ways", ways_edges.size());

  size_t ways_passed = 0, values_written = 0, invalid_entries = 0, edges_missing = 0;
  std::unordered_set<std::string> unknown_names;
  std::unordered_set<uint64_t> touched_tiles;

  for (const auto& file : input_files) {
    LOG_INFO("Reading ways from {}", file);
    osmium::io::Reader pbf(file, osmium::osm_entity_bits::way);
    while (osmium::memory::Buffer buffer = pbf.read()) {
      for (const auto& way : buffer.select<osmium::Way>()) {
        const auto& tags = way.tags();
        bool interesting = false;
        for (const auto& key : tag_keys) {
          if (tags.has_key(key.c_str())) {
            interesting = true;
            break;
          }
        }
        if (!interesting) {
          continue;
        }
        const auto way_edges = ways_edges.find(static_cast<uint64_t>(way.id()));
        if (way_edges == ways_edges.end()) {
          continue;
        }

        ++ways_passed;
        invalid_entries += lua.call(way, [&](std::string_view name, float value) {
          const auto col = column.find(std::string(name));
          if (col == column.end()) {
            if (unknown_names.emplace(name).second) {
              LOG_WARN("Lua returned unknown attribute '{}', ignoring it", name);
            }
            return;
          }
          for (const auto& edge : way_edges->second) {
            const auto tile_id = edge.edgeid.tile_base().value;
            const auto tile = ca_extract.tiles.find(tile_id);
            if (tile == ca_extract.tiles.end() ||
                edge.edgeid.id() >= tile->second.header->directed_edge_count) {
              ++edges_missing;
              continue;
            }
            const size_t n = tile->second.header->num_attributes;
            tile->second.data[static_cast<size_t>(edge.edgeid.id()) * n + col->second] = value;
            touched_tiles.insert(tile_id);
            ++values_written;
          }
        });
      }
    }
    pbf.close();
  }

  const auto now = static_cast<uint64_t>(std::time(nullptr));
  for (const auto tile_id : touched_tiles) {
    ca_extract.tiles[tile_id].header->last_update = now;
  }

  if (invalid_entries) {
    LOG_WARN("Lua returned {} entries that were not attribute name -> number", invalid_entries);
  }
  if (edges_missing) {
    LOG_WARN("{} edge values had no matching tile or edge index", edges_missing);
  }
  LOG_INFO("Passed {} ways to Lua, wrote {} values into {} tiles", ways_passed, values_written,
           touched_tiles.size());
  return values_written;
}

} // namespace mjolnir
} // namespace valhalla
