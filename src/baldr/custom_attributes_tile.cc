#include "baldr/custom_attributes_tile.h"

#include <rapidjson/document.h>

#include <cmath>
#include <format>
#include <unordered_set>

namespace valhalla {
namespace baldr {

std::vector<CustomAttributeDef> ParseCustomAttributeDefs(std::string_view json) {
  rapidjson::Document doc;
  doc.Parse(json.data(), json.size());
  if (doc.HasParseError() || !doc.IsArray()) {
    throw std::runtime_error("Custom attribute definitions must be a JSON array");
  }

  std::vector<CustomAttributeDef> defs;
  std::unordered_set<std::string> names;
  uint32_t offset = 0;
  for (const auto& v : doc.GetArray()) {
    if (!v.IsObject() || !v.HasMember("name") || !v["name"].IsString() || !v.HasMember("type") ||
        !v["type"].IsString()) {
      throw std::runtime_error("Each custom attribute needs a string name and type");
    }

    CustomAttributeDef def;
    def.name = std::string(v["name"].GetString(), v["name"].GetStringLength());
    if (def.name.empty() || !names.insert(def.name).second) {
      throw std::runtime_error(
          std::format("Custom attribute name '{}' is empty or used twice", def.name));
    }

    const std::string_view type(v["type"].GetString(), v["type"].GetStringLength());
    if (type == "uint8") {
      def.type = CustomAttributeType::kUint8;
    } else {
      throw std::runtime_error(
          std::format("Custom attribute '{}' has unsupported type '{}'", def.name, type));
    }

    def.scale = 1.0f;
    if (v.HasMember("scale")) {
      if (!v["scale"].IsNumber() || !std::isfinite(v["scale"].GetFloat()) ||
          v["scale"].GetFloat() == 0.0f) {
        throw std::runtime_error(
            std::format("Custom attribute '{}' needs a finite, non-zero scale", def.name));
      }
      def.scale = v["scale"].GetFloat();
    }

    const uint32_t bits = custom_attribute_bits(def.type);
    if (bits % 8 == 0) {
      offset = (offset + 7) / 8 * 8;
    }

    // CustomAttributesTileHeader::row_size is 16 bits wide
    if (offset + bits > 0xFFFF * 8) {
      throw std::runtime_error("Custom attributes exceed 65535 bytes per edge");
    }

    def.offset = offset;
    offset += bits;
    defs.push_back(std::move(def));
  }
  return defs;
}

uint32_t custom_attributes_row_size(const std::vector<CustomAttributeDef>& defs) {
  return defs.empty() ? 0 : (defs.back().offset + custom_attribute_bits(defs.back().type) + 7) / 8;
}

} // namespace baldr
} // namespace valhalla
