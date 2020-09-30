#include "baldr/util.h"

namespace valhalla {
namespace baldr {

uint32_t
OverwriteBits(const uint32_t dst, const uint32_t src, const uint32_t pos, const uint32_t len) {
  uint32_t shift = (pos * len);
  uint32_t mask = ((static_cast<uint32_t>(1) << len) - 1) << shift;
  return (dst & ~mask) | (src << shift);
}

uint32_t OverwriteBit(const uint32_t dst, const uint32_t src, const uint32_t pos) {
  uint32_t mask = (static_cast<uint32_t>(1) << pos);
  return (dst & ~mask) | (src << pos);
}

} // namespace baldr
} // namespace valhalla
