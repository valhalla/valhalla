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

// allows us to store more than unsigned integers in the future
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

// a single entry of the extract's attributes.json
struct CustomAttributeDef {
  std::string name;
  CustomAttributeType type;
  float scale;
  uint32_t offset; // bit offset within an edge's row, byte-sized types start on a byte boundary
};

// the information stored in costing needed to retrieve the value and turn it into a factor
struct WeightedCustomAttribute {
  uint32_t offset; // in bits from the row start
  CustomAttributeType type;
  float scale;
  float weight;
};

/**
 * value of the column at the bit offset of an edge's row
 *
 * @param row    the row to read from
 * @param offset the column offset in the row
 * @param type   the column's data type
 * @param scale  the column's scale
 *
 * @return the stored value times the scale
 */
inline float decode_custom_attribute(const volatile uint8_t* row,
                                     uint32_t offset,
                                     CustomAttributeType type,
                                     float scale) {
  switch (type) {
    case CustomAttributeType::kUint8:
      return static_cast<float>(row[offset / 8]) * scale;
  }
  return 0.0f;
}

// stores round(value / scale) in the column, returns false if it had to be clamped to the type's
// range
bool encode_custom_attribute(uint8_t* row, const CustomAttributeDef& def, float value);

/**
 * Parse attributes.json: {"definitions": [{"name": "scenic", "type": "uint8", "scale": 0.01}, ...]}.
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

  // memory too small for its header's edges leaves the tile invalid
  CustomAttributesTile(std::unique_ptr<const GraphMemory> memory)
      : memory_(std::move(memory)), header_(nullptr), data_(nullptr) {
    if (!memory_ || memory_->size < sizeof(CustomAttributesTileHeader)) {
      return;
    }
    header_ = reinterpret_cast<const volatile CustomAttributesTileHeader*>(memory_->data);
    const size_t required_size =
        sizeof(CustomAttributesTileHeader) +
        static_cast<size_t>(header_->directed_edge_count) * static_cast<size_t>(header_->row_size);
    if (memory_->size < required_size) {
      header_ = nullptr;
      return;
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
    return decode_custom_attribute(row, def.offset, def.type, def.scale);
  }

  /**
   * Compute the weighted sum of the given attributes for an edge.
   *
   * @param  edge_index  Index of the directed edge (0-based)
   * @param  attributes  Columns to read, with their scale and weight
   * @return sum of (value_i * weight_i), or 0.0f if no data
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
        sum += decode_custom_attribute(row, attribute.offset, attribute.type, attribute.scale) *
               attribute.weight;
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
