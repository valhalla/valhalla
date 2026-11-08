#include "mjolnir/custom_attributes_builder.h"
#include "baldr/custom_attributes_tile.h"
#include "baldr/graphid.h"
#include "baldr/graphreader.h"
#include "baldr/tilehierarchy.h"
#include "midgard/logging.h"
#include "midgard/sequence.h"
#include "mjolnir/way_edges_processor.h"

#include <osmium/io/pbf_input.hpp>
#include <osmium/io/reader.hpp>
#include <osmium/osm/way.hpp>

#include <algorithm>
#include <cmath>
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

// which edges of a way, relative to the OSM way direction, a value returned by Lua applies to
enum class edge_direction { kBoth, kForward, kBackward };

class custom_attributes_lua {
public:
  explicit custom_attributes_lua(const std::string& script) : state_(luaL_newstate()) {
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

  custom_attributes_lua(const custom_attributes_lua&) = delete;
  custom_attributes_lua& operator=(const custom_attributes_lua&) = delete;

  ~custom_attributes_lua() {
    lua_close(state_);
  }

  // Calls custom_attributes(tags) and hands every string -> number entry of the result to the
  // callback along with its direction. Lua returns either one table for both directions or a
  // forward and a backward table. Returns the number of entries that were not string -> number.
  template <typename attribute_callback_t>
  size_t call(const osmium::Way& way, attribute_callback_t&& callback) {
    lua_getglobal(state_, kLuaFunction);

    // prepare the tags as a table
    lua_newtable(state_);
    for (const auto& tag : way.tags()) {
      lua_pushstring(state_, tag.key());
      lua_pushstring(state_, tag.value());
      lua_rawset(state_, -3);
    }

    // call the user provided lua function
    if (lua_pcall(state_, 1, 2, 0)) {
      std::string error = lua_tostring(state_, -1);
      lua_pop(state_, 1);
      throw std::runtime_error(
          std::format("Lua {}() failed for way {}: {}", kLuaFunction, way.id(), error));
    }

    const int top = lua_gettop(state_);
    size_t invalid = 0;

    // if the second result is nil, only a single table was returned,
    // apply it to both forward and backward edges
    if (lua_isnil(state_, top)) {
      invalid += walk_table(top - 1, edge_direction::kBoth, callback);
    } else {
      invalid += walk_table(top - 1, edge_direction::kForward, callback);
      invalid += walk_table(top, edge_direction::kBackward, callback);
    }
    lua_pop(state_, 2);
    return invalid;
  }

private:
  // walks a table returned from the user provided lua function and passes each entry to the callback.
  // Returns the number of invalid entries, a non-table, non-nil value counts as one.
  template <typename attribute_callback_t>
  size_t walk_table(int index, edge_direction direction, attribute_callback_t& callback) {
    if (lua_isnil(state_, index)) {
      return 0;
    }
    if (!lua_istable(state_, index)) {
      return 1;
    }

    size_t invalid = 0;
    lua_pushnil(state_);
    while (lua_next(state_, index) != 0) {
      // check the types before converting and passing to the user callback
      if (lua_type(state_, -2) == LUA_TSTRING && lua_type(state_, -1) == LUA_TNUMBER) {
        callback(std::string_view(lua_tostring(state_, -2)),
                 static_cast<float>(lua_tonumber(state_, -1)), direction);
      } else {
        ++invalid;
      }
      lua_pop(state_, 1);
    }
    return invalid;
  }

  lua_State* state_;
};

struct writable_tile {
  CustomAttributesTileHeader* header;
  uint8_t* data;
};

// stores value / scale in the column, returns false if it had to be clamped to the type's range
bool encode(uint8_t* row, const CustomAttributeDef& def, float value) {
  const double raw = std::round(static_cast<double>(value) / def.scale);
  switch (def.type) {
    case CustomAttributeType::kUint8:
      row[def.offset / 8] = static_cast<uint8_t>(std::clamp(raw, 0.0, 255.0));
      return raw >= 0.0 && raw <= 255.0;
  }
  return false;
}

float decode(const uint8_t* row, const CustomAttributeDef& def) {
  switch (def.type) {
    case CustomAttributeType::kUint8:
      return row[def.offset / 8] * def.scale;
  }
  return 0.0f;
}

// tiles of the tar keyed by tile base id, plus the attribute definitions from attributes.json
struct writable_extract {
  midgard::tar archive;
  std::unordered_map<uint64_t, writable_tile> tiles;
  std::vector<CustomAttributeDef> defs;

  explicit writable_extract(const std::string& path) : archive(path, false) {
    archive.for_each([this](const std::string& name, const char* data, size_t size) {
      std::filesystem::path p{name};

      // TODO: use index.bin

      if (p.filename() == "attributes.json") {
        defs = ParseCustomAttributeDefs({data, size});
        return true;
      }

      // ignore unknown files
      if (p.extension() != ".gph") {
        return true;
      }

      if (size < sizeof(CustomAttributesTileHeader)) {
        LOG_WARN("Skipping {}: too small for header ({} bytes)", name, size);
        return true;
      }
      auto* header = reinterpret_cast<CustomAttributesTileHeader*>(const_cast<char*>(data));
      if (header->custom_attributes_tile_version != CUSTOM_ATTRIBUTES_TILE_VERSION) {
        LOG_WARN("Skipping {}: version {} != {}", name, header->custom_attributes_tile_version,
                 CUSTOM_ATTRIBUTES_TILE_VERSION);
        return true;
      }
      const size_t required =
          sizeof(CustomAttributesTileHeader) +
          static_cast<size_t>(header->directed_edge_count) * static_cast<size_t>(header->row_size);
      if (size < required) {
        LOG_WARN("Skipping {}: {} bytes, expected {}", name, size, required);
        return true;
      }
      try {
        auto id = GraphId::FromTilePath(name);
        tiles[id.value] = {header, reinterpret_cast<uint8_t*>(const_cast<char*>(data) +
                                                              sizeof(CustomAttributesTileHeader))};
      } catch (...) { LOG_WARN("Skipping {}: not a tile path", name); }
      return true;
    });
  }

  // the edge's row, or nullptr if the extract has no tile or row for it
  uint8_t* row(const GraphId& edgeid) {
    const auto tile = tiles.find(edgeid.tile_base().value);
    if (tile == tiles.end() || edgeid.id() >= tile->second.header->directed_edge_count) {
      return nullptr;
    }
    return tile->second.data + static_cast<size_t>(edgeid.id()) * tile->second.header->row_size;
  }
};

// Writes every shortcut's values from the edges it replaces. Returns the number of shortcuts
// written and adds their tiles to touched_tiles.
size_t write_shortcuts(GraphReader& reader,
                       writable_extract& ca_extract,
                       ShortcutStrategy strategy,
                       std::unordered_set<uint64_t>& touched_tiles) {
  size_t written = 0, skipped = 0;
  std::vector<float> values(ca_extract.defs.size());
  std::vector<std::pair<uint8_t*, uint32_t>> edge_rows; // row and length of each replaced edge

  // shortcuts only exist on the levels above the local level
  for (uint8_t level = 0; level < TileHierarchy::levels().back().level; ++level) {
    for (auto edge_id : reader.GetTileSet(level)) {
      if (reader.OverCommitted()) {
        reader.Trim();
      }
      graph_tile_ptr tile = reader.GetGraphTile(edge_id);
      for (uint32_t n = 0; n < tile->header()->directededgecount(); ++n, ++edge_id) {
        if (!tile->directededge(edge_id)->is_shortcut()) {
          continue;
        }
        uint8_t* shortcut_row = ca_extract.row(edge_id);
        if (!shortcut_row) {
          ++skipped;
          continue;
        }
        const auto edges = reader.RecoverShortcut(edge_id);

        // RecoverShortcut returns the shortcut itself if it fails
        if (edges.size() == 1 && edges.front() == edge_id) {
          ++skipped;
          continue;
        }

        edge_rows.clear();
        for (const auto& id : edges) {
          uint8_t* row = ca_extract.row(id);
          const auto* edge = reader.directededge(id);
          if (row && edge) {
            edge_rows.emplace_back(row, std::max(edge->length(), 1u));
          }
        }
        if (edge_rows.empty()) {
          ++skipped;
          continue;
        }

        for (size_t i = 0; i < ca_extract.defs.size(); ++i) {
          const auto& def = ca_extract.defs[i];
          float value = decode(edge_rows.front().first, def);
          double weighted_sum = 0.0, total_length = 0.0;
          for (const auto& [row, length] : edge_rows) {
            const float edge_value = decode(row, def);
            switch (strategy) {
              case ShortcutStrategy::kMin:
                value = std::min(value, edge_value);
                break;
              case ShortcutStrategy::kMax:
                value = std::max(value, edge_value);
                break;
              default: // kMean
                weighted_sum += static_cast<double>(edge_value) * length;
                total_length += length;
                break;
            }
          }
          if (strategy == ShortcutStrategy::kMean) {
            value = static_cast<float>(weighted_sum / total_length);
          }
          values[i] = value;
        }
        for (size_t i = 0; i < ca_extract.defs.size(); ++i) {
          encode(shortcut_row, ca_extract.defs[i], values[i]);
        }
        touched_tiles.insert(edge_id.tile_base().value);
        ++written;
      }
    }
  }

  if (skipped) {
    LOG_WARN("{} shortcuts could not be recovered or have no custom attributes tile", skipped);
  }
  return written;
}

} // namespace

size_t add_custom_attributes(const boost::property_tree::ptree& config,
                             const std::vector<std::string>& input_files,
                             const std::vector<std::string>& tag_keys,
                             const std::string& lua_script,
                             ShortcutStrategy shortcut_strategy) {
  const auto extract_path = config.get_optional<std::string>("mjolnir.custom_attributes_extract");
  if (!extract_path) {
    throw std::runtime_error("mjolnir.custom_attributes_extract is not set");
  }
  if (tag_keys.empty()) {
    throw std::runtime_error("At least one OSM tag key is required");
  }

  custom_attributes_lua lua(lua_script);

  writable_extract ca_extract(*extract_path);
  if (ca_extract.defs.empty() || ca_extract.tiles.empty()) {
    throw std::runtime_error(*extract_path +
                             " has no attributes.json definitions or no usable tiles");
  }

  std::unordered_map<std::string, const CustomAttributeDef*> column;
  for (const auto& def : ca_extract.defs) {
    column.emplace(def.name, &def);
  }

  const uint32_t row_size = custom_attributes_row_size(ca_extract.defs);
  for (const auto& [id, tile] : ca_extract.tiles) {
    if (tile.header->num_attributes != ca_extract.defs.size() || tile.header->row_size != row_size) {
      throw std::runtime_error(
          std::format("Tile {} has {} attributes in {} bytes, attributes.json lists {} in {} bytes",
                      GraphId(id), static_cast<uint32_t>(tile.header->num_attributes),
                      static_cast<uint32_t>(tile.header->row_size), ca_extract.defs.size(),
                      row_size));
    }
  }
  LOG_INFO("Custom attributes tar: {} tiles, {} attributes", ca_extract.tiles.size(),
           ca_extract.defs.size());

  GraphReader reader(config.get_child("mjolnir"));
  const auto ways_edges = collect_way_edges(reader, "", kAllAccess);
  LOG_INFO("Collected edges for {} ways", ways_edges.size());

  size_t ways_passed = 0, values_written = 0, values_clamped = 0, invalid_entries = 0,
         edges_missing = 0;
  std::unordered_set<std::string> unknown_names;
  std::unordered_set<uint64_t> touched_tiles;

  for (const auto& file : input_files) {
    LOG_INFO("Reading ways from {}", file);
    osmium::io::Reader pbf(file, osmium::osm_entity_bits::way);
    while (osmium::memory::Buffer buffer = pbf.read()) {
      for (const auto& way : buffer.select<osmium::Way>()) {
        const auto& tags = way.tags();

        const auto way_edges = ways_edges.find(static_cast<uint64_t>(way.id()));
        if (way_edges == ways_edges.end()) {
          continue;
        }

        // make the user pass a list of tag keys to quickly filter by
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

        ++ways_passed;
        invalid_entries +=
            lua.call(way, [&](std::string_view name, float value, edge_direction direction) {
              const auto col = column.find(std::string(name));
              if (col == column.end()) {
                if (unknown_names.emplace(name).second) {
                  LOG_WARN("Lua returned unknown attribute '{}', ignoring it", name);
                }
                return;
              }
              if (!std::isfinite(value)) {
                ++invalid_entries;
                return;
              }
              for (const auto& edge : way_edges->second) {
                if (direction != edge_direction::kBoth &&
                    edge.forward != (direction == edge_direction::kForward)) {
                  continue;
                }
                uint8_t* row = ca_extract.row(edge.edgeid);
                if (!row) {
                  ++edges_missing;
                  continue;
                }
                values_clamped += !encode(row, *col->second, value);
                touched_tiles.insert(edge.edgeid.tile_base().value);
                ++values_written;
              }
            });
      }
    }
    pbf.close();
  }

  size_t shortcuts_written = 0;
  if (shortcut_strategy != ShortcutStrategy::kNone) {
    shortcuts_written = write_shortcuts(reader, ca_extract, shortcut_strategy, touched_tiles);
  }

  const auto now = static_cast<uint64_t>(std::time(nullptr));
  for (const auto tile_id : touched_tiles) {
    ca_extract.tiles[tile_id].header->last_update = now;
  }

  if (invalid_entries) {
    LOG_WARN("Lua returned {} entries that were not attribute name -> finite number",
             invalid_entries);
  }
  if (values_clamped) {
    LOG_WARN("{} edge values were outside their attribute's range and got clamped", values_clamped);
  }
  if (edges_missing) {
    LOG_WARN("{} edge values had no matching tile or edge index", edges_missing);
  }
  LOG_INFO("Passed {} ways to Lua, wrote {} values and {} shortcuts into {} tiles", ways_passed,
           values_written, shortcuts_written, touched_tiles.size());
  return values_written;
}

} // namespace mjolnir
} // namespace valhalla
