#ifndef VALHALLA_BALDR_CUSTOM_ATTRIBUTES_TILE_H_
#define VALHALLA_BALDR_CUSTOM_ATTRIBUTES_TILE_H_

#include <valhalla/baldr/graphmemory.h>

#include <cstdint>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>

namespace valhalla {
namespace baldr {

// Binary format: 4-byte edge_count, 4-byte num_attributes, then edge_count * num_attributes floats
// laid out row-major: [edge0_attr0, edge0_attr1, ..., edge1_attr0, edge1_attr1, ...]
#pragma pack(push, 1)
struct CustomAttributesTileHeader {
  uint32_t edge_count;
  uint32_t num_attributes;
};
#pragma pack(pop)

static_assert(sizeof(CustomAttributesTileHeader) == 8,
              "CustomAttributesTileHeader must be exactly 8 bytes");

/**
 * Provides O(1) mmap'd access to per-edge multi-attribute float data.
 *
 * The .cab binary format:
 *   - 8-byte CustomAttributesTileHeader (edge_count u32, num_attributes u32)
 *   - edge_count * num_attributes floats, row-major by edge
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
   * @param memory               Ownership of the mmap'd .cab data
   * @param expected_edge_count  Expected number of directed edges in this tile
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

    if (header_->edge_count != expected_edge_count) {
      throw std::runtime_error("CustomAttributesTile: edge_count mismatch (expected=" +
                               std::to_string(expected_edge_count) +
                               ", got=" + std::to_string(header_->edge_count) + ")");
    }

    const size_t required_size = sizeof(CustomAttributesTileHeader) +
                                 static_cast<size_t>(header_->edge_count) *
                                     static_cast<size_t>(header_->num_attributes) * sizeof(float);
    if (memory_->size < required_size) {
      throw std::runtime_error("CustomAttributesTile: memory too small for data (required=" +
                               std::to_string(required_size) +
                               ", got=" + std::to_string(memory_->size) + ")");
    }

    data_ =
        reinterpret_cast<const volatile float*>(memory_->data + sizeof(CustomAttributesTileHeader));
  }

  /**
   * O(1) access to a single attribute by edge index and attribute index.
   *
   * @param edge_index  Index of the directed edge (0-based)
   * @param attr_index  Index of the attribute (0-based, < num_attributes)
   * @return The float value, or 0.0f if uninitialized or out of bounds
   */
  float value(uint32_t edge_index, uint32_t attr_index = 0) const volatile {
    if (!data_ || !header_ || edge_index >= header_->edge_count ||
        attr_index >= header_->num_attributes) {
      return 0.0f;
    }
    return data_[static_cast<size_t>(edge_index) * header_->num_attributes + attr_index];
  }

  /**
   * Compute the weighted sum across all attributes for a given edge.
   * weights must have exactly num_attributes() entries; weights[i] corresponds to attribute i.
   *
   * @param edge_index  Index of the directed edge (0-based)
   * @param weights     Per-attribute weights, indexed by attribute position
   * @return Σ (attr_i * weight_i), or 0.0f if no data
   */
  float weighted_sum(uint32_t edge_index, std::span<const float> weights) const volatile {
    if (!data_ || !header_ || edge_index >= header_->edge_count || weights.empty()) {
      return 0.0f;
    }
    const uint32_t n = header_->num_attributes;
    const size_t base = static_cast<size_t>(edge_index) * n;
    float sum = 0.0f;
    const uint32_t count =
        static_cast<uint32_t>(weights.size()) < n ? static_cast<uint32_t>(weights.size()) : n;
    for (uint32_t i = 0; i < count; ++i) {
      sum += data_[base + i] * weights[i];
    }
    return sum;
  }

  uint32_t edge_count() const volatile {
    return header_ ? header_->edge_count : 0u;
  }

  uint32_t num_attributes() const volatile {
    return header_ ? header_->num_attributes : 0u;
  }

  explicit operator bool() const volatile {
    return data_ != nullptr;
  }

protected:
  std::unique_ptr<const GraphMemory> memory_;

private:
  // These are const pointers to data structures - once assigned, the pointer values won't change.
  // The pointer targets are marked as const volatile because they can be modified by code outside
  // our control (another process accessing a mmap'd file for example).
  const volatile CustomAttributesTileHeader* header_;
  const volatile float* data_;
};

} // namespace baldr
} // namespace valhalla

#endif // VALHALLA_BALDR_CUSTOM_ATTRIBUTES_TILE_H_
