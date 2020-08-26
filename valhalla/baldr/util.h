#ifndef VALHALLA_BALDR_UTIL_H_
#define VALHALLA_BALDR_UTIL_H_

#include <cstdint>

namespace valhalla {
namespace baldr {

/**
 * Get the updated bit field.
 * @param dst  Data member to be updated.
 * @param src  Value to be updated.
 * @param pos  Position (pos element within the bit field).
 * @param len  Length of each element within the bit field.
 * @return  Returns an updated value for the bit field.
 */
uint32_t
OverwriteBits(const uint32_t dst, const uint32_t src, const uint32_t pos, const uint32_t len);

/**
 * Get the updated bit field.
 * @param dst  Data member to be updated.
 * @param src  Value to be updated.
 * @param pos  Position (pos element within the bit field).
 * @return  Returns an updated value for the bit field.
 */
uint32_t OverwriteBit(const uint32_t dst, const uint32_t src, const uint32_t pos);

} // namespace baldr
} // namespace valhalla

#endif // VALHALLA_BALDR_UTIL_H_
