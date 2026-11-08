#ifndef VALHALLA_BALDR_CUSTOM_ATTRIBUTES_TILE_H_
#define VALHALLA_BALDR_CUSTOM_ATTRIBUTES_TILE_H_

#include <valhalla/baldr/graphmemory.h>

#include <cstdint>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace valhalla {
namespace baldr {

const uint8_t CUSTOM_ATTRIBUTES_TILE_VERSION = 1;

struct CustomAttributesTileHeader {
  uint64_t tile_id;
  uint64_t last_update; // seconds since epoch
  uint64_t num_attributes : 16;
  uint64_t row_size : 16; // bytes per edge
  uint64_t spare : 32;
  uint32_t directed_edge_count;
  uint32_t custom_attributes_tile_version;
};

static_assert(sizeof(CustomAttributesTileHeader) == 32,
              "CustomAttributesTileHeader size is different than expected");

// values are stored as raw integers, decoded as raw * scale
enum class CustomAttributeType : uint8_t {
  kUint8 = 0,
};

constexpr uint32_t custom_attribute_bits(CustomAttributeType type) {
  switch (type) {
    case CustomAttributeType::kUint8:
      return 8;
  }
  return 0;
}

// one entry of the extract's attributes.json
struct CustomAttributeDef {
  std::string name;
  CustomAttributeType type;
  float scale;
  uint32_t offset; // bit offset within an edge's row, byte-sized types start on a byte boundary
};

// a column the costing reads, with the request weight already multiplied by the scale
struct WeightedCustomAttribute {
  uint32_t offset; // in bits
  CustomAttributeType type;
  float weight;
};

// raw value of the column at the bit offset of an edge's row, without the scale
inline float
decode_custom_attribute(const volatile uint8_t* row, uint32_t offset, CustomAttributeType type) {
  switch (type) {
    case CustomAttributeType::kUint8:
      return static_cast<float>(row[offset / 8]);
  }
  return 0.0f;
}

// stores round(value / scale) in the column, returns false if it had to be clamped to the type's
// range
bool encode_custom_attribute(uint8_t* row, const CustomAttributeDef& def, float value);

/**
 * Parse attributes.json: [{"name": "scenic", "type": "uint8", "scale": 0.01}, ...].
 * Columns are laid out in array order, padded only to put byte-sized types on a byte boundary;
 * scale defaults to 1.
 *
 * @throws std::runtime_error  if the JSON is not a valid attribute list
 */
std::vector<CustomAttributeDef> ParseCustomAttributeDefs(std::string_view json);

// bytes per edge for the given attribute list
uint32_t custom_attributes_row_size(const std::vector<CustomAttributeDef>& defs);

/**
 * Tiles with per-edge custom attribute data.
 *
 * The binary format:
 *   - 32-byte CustomAttributesTileHeader
 *   - edge_count rows of row_size bytes, the columns of a row as laid out by attributes.json
 */
class CustomAttributesTile {
public:
  CustomAttributesTile(const CustomAttributesTile&) = delete;
  CustomAttributesTile& operator=(const CustomAttributesTile&) = delete;

  CustomAttributesTile(CustomAttributesTile&&) = default;
  CustomAttributesTile& operator=(CustomAttributesTile&&) = default;

  CustomAttributesTile(std::nullptr_t) : memory_(nullptr), header_(nullptr), data_(nullptr) {
  }

  /**
   * Construct from raw memory, validating edge_count against the tile.
   *
   * @param memory               Unique pointer to raw data in memory to take ownership of
   * @param expected_edge_count  Expected number of directed edges in this tile
   *
   * @throws std::runtime_error  if header edge_count doesn't match or memory is too small
   */
  CustomAttributesTile(std::unique_ptr<const GraphMemory> memory, uint32_t expected_edge_count)
      : memory_(std::move(memory)), header_(nullptr), data_(nullptr) {
    if (!memory_) {
      return;
    }

    if (memory_->size < sizeof(CustomAttributesTileHeader)) {
      throw std::runtime_error("CustomAttributesTile: memory too small for header (size=" +
                               std::to_string(memory_->size) + ")");
    }

    header_ = reinterpret_cast<const volatile CustomAttributesTileHeader*>(memory_->data);

    if (header_->directed_edge_count != expected_edge_count) {
      throw std::runtime_error("CustomAttributesTile: edge_count mismatch (expected=" +
                               std::to_string(expected_edge_count) +
                               ", got=" + std::to_string(header_->directed_edge_count) + ")");
    }

    const size_t required_size =
        sizeof(CustomAttributesTileHeader) +
        static_cast<size_t>(header_->directed_edge_count) * static_cast<size_t>(header_->row_size);
    if (memory_->size < required_size) {
      throw std::runtime_error("CustomAttributesTile: memory too small for data (required=" +
                               std::to_string(required_size) +
                               ", got=" + std::to_string(memory_->size) + ")");
    }

    data_ =
        reinterpret_cast<const volatile uint8_t*>(memory_->data + sizeof(CustomAttributesTileHeader));
  }

  /**
   * Access to a single decoded attribute value.
   *
   * @param edge_index  Index of the directed edge
   * @param def         The attribute's definition from attributes.json
   * @return raw * scale, or 0.0f if uninitialized or out of bounds
   */
  float value(uint32_t edge_index, const CustomAttributeDef& def) const volatile {
    const volatile uint8_t* row = edge_row(edge_index);
    if (!row || def.offset + custom_attribute_bits(def.type) > header_->row_size * 8u) {
      return 0.0f;
    }
    return decode_custom_attribute(row, def.offset, def.type) * def.scale;
  }

  /**
   * Compute the weighted sum of the given attributes for an edge.
   *
   * @param  edge_index  Index of the directed edge (0-based)
   * @param  attributes  Columns to read, with weights already multiplied by their scale
   * @return sum of (raw_i * weight_i) weighted over the number of attributes, or 0.0f if no data
   */
  float weighted_sum(uint32_t edge_index, std::span<const WeightedCustomAttribute> attributes) const
      volatile {
    const volatile uint8_t* row = edge_row(edge_index);
    if (!row) {
      return 0.0f;
    }
    const uint32_t row_bits = header_->row_size * 8u;
    float sum = 0.0f;
    for (const auto& attribute : attributes) {
      if (attribute.offset + custom_attribute_bits(attribute.type) <= row_bits) {
        sum += decode_custom_attribute(row, attribute.offset, attribute.type) * attribute.weight;
      }
    }
    return sum;
  }

  uint32_t edge_count() const volatile {
    return header_ ? header_->directed_edge_count : 0u;
  }

  uint32_t num_attributes() const volatile {
    return header_ ? header_->num_attributes : 0u;
  }

  uint32_t row_size() const volatile {
    return header_ ? header_->row_size : 0u;
  }

  explicit operator bool() const volatile {
    return data_ != nullptr;
  }

protected:
  std::unique_ptr<const GraphMemory> memory_;

  const volatile uint8_t* edge_row(uint32_t edge_index) const volatile {
    if (!data_ || !header_ ||
        header_->custom_attributes_tile_version != CUSTOM_ATTRIBUTES_TILE_VERSION ||
        edge_index >= header_->directed_edge_count) {
      return nullptr;
    }
    return data_ + static_cast<size_t>(edge_index) * header_->row_size;
  }

private:
  // These are const pointers to data structures - once assigned, the pointer values won't change.
  // The pointer targets are marked as const volatile because they can be modified by code outside
  // our control (another process accessing a mmap'd file for example).
  const volatile CustomAttributesTileHeader* header_;
  const volatile uint8_t* data_;
};

} // namespace baldr
} // namespace valhalla

#endif // VALHALLA_BALDR_CUSTOM_ATTRIBUTES_TILE_H_
