#ifndef VALHALLA_BALDR_TRAFFICTILE_H_
#define VALHALLA_BALDR_TRAFFICTILE_H_

// This header supports exporting as a set of C++ minimal structs
// with namespaces, but with C-99 only types and headers, to make
// binding generation in other languages simpler.
// If you do `-DC_ONLY_INTERFACE`, you'll get a header with
// C99 stdint.h, and POD structs with no constructors
#ifndef C_ONLY_INTERFACE
#include <algorithm>
#include <cstdint>
#include <exception>
#include <string>
#include <type_traits>
#include <valhalla/baldr/graphconstants.h>
#include <vector>
#else
#include <stdint.h>
#endif

#ifndef C_ONLY_INTERFACE
namespace valhalla {
namespace baldr {
namespace traffic {

using std::uint16_t;
using std::uint32_t;
using std::uint64_t;
#endif

constexpr uint16_t INVALID_SPEED_AGE_BUCKET = 0;
constexpr uint16_t MAX_SPEED_AGE_BUCKET = 15;
constexpr uint16_t SPEED_AGE_BUCKET_SIZE = 2; // 2 minutes per bucket

// Traffic speeds are encoded as 8 bits in `Speed` below
constexpr uint32_t MAX_TRAFFIC_SPEED_KPH = 255;

/**
 * Helper function to return the approximate age in seconds of a record based
 * on the bucket it belongs to.  Because buckets have a 2 minute resolution,
 * we round to the median bucket time period valid (e.g. 1 min, 3 min, 5 min)
 * Non-class method to help with C-bindings interface
 *
 * @return Age in seconds of the start of the bucket, or -1 for INVALID_SPEED_AGE_BUCKET (0)
 */
inline int valhalla_traffic_age_bucket_to_seconds(const uint16_t age_bucket) {
  if (age_bucket == INVALID_SPEED_AGE_BUCKET)
    return -1;
  return (age_bucket - 1) * SPEED_AGE_BUCKET_SIZE * 60 + (SPEED_AGE_BUCKET_SIZE * 60 / 2);
}

/**
 * Helper function to determine which age bucket a seconds value
 * resides in.
 */
inline uint16_t valhalla_traffic_seconds_to_age_bucket(const int seconds) {
  // Simple integer division, should truncate as we want
  auto bucket = (seconds / 60) / SPEED_AGE_BUCKET_SIZE;
  if (bucket > MAX_SPEED_AGE_BUCKET)
    return INVALID_SPEED_AGE_BUCKET;
  return bucket + 1; // 0 seconds should go into bucket 1, as bucket 0 is for invalid records
}

struct Speed {
  uint16_t speed_kmh : 8;        // km/h - so max range is 0-255km/h
  uint16_t congestion_level : 3; // 0 - unknown, 1-6 - low-high, 7 - unused
  uint16_t age_bucket : 4;       // Age bucket for the speed record (see SPEED_AGE_BUCKET_SIZE)
  uint16_t spare : 1;            // TODO: reserved for later use
#ifndef C_ONLY_INTERFACE
  inline bool valid() const volatile {
    return age_bucket != INVALID_SPEED_AGE_BUCKET;
  }

  inline bool closed() const volatile {
    return valid() && speed_kmh == 0;
  }

  // Get age of record in seconds (based on the bucket
  // it belongs to)
  inline int age_secs() const volatile {
    return valhalla_traffic_age_bucket_to_seconds(age_bucket);
  }
#endif
};

// per-speed-tile header
struct TileHeader {
  uint64_t tile_id;
  uint64_t last_update; // seconds since epoch
  uint32_t directed_edge_count;
  uint32_t spare1;
  uint32_t spare2;
  uint32_t spare3;
};

#ifndef C_ONLY_INTERFACE
// Some checks to ensure that the interfaces don't get change accidentally.
// Modifying the sizes/layouts of these structs is a data-format breaking
// change and shouldn't be done lightly.
static_assert(sizeof(TileHeader) == sizeof(uint64_t) * 4,
              "traffic:TileHeader type size different than expected");
static_assert(sizeof(Speed) == sizeof(uint16_t),
              "traffic::Speed type size is different than expected");
#endif // C_ONLY_INTERFACE

/**
 * A tile of live traffic data.  The layout is:
 *
 * TileHeader (24 bytes)
 * n x Speed entries (n x 2 bytes)
 */
#ifndef C_ONLY_INTERFACE
namespace {
static constexpr volatile Speed INVALID_SPEED{0, 0, INVALID_SPEED_AGE_BUCKET, 0};

// Assert these constants are the same
// (We want to avoid including this file in graphconstants.h)
static_assert(MAX_TRAFFIC_SPEED_KPH == valhalla::baldr::kMaxTrafficSpeed,
              "Constants must be the same");
} // namespace
class Tile {
public:
  Tile(char* tile_ptr)
      : header{reinterpret_cast<volatile TileHeader*>(tile_ptr)},
        speeds{reinterpret_cast<volatile Speed*>(tile_ptr + sizeof(TileHeader))} {
  }

  const volatile Speed& getTrafficForDirectedEdge(const uint32_t directed_edge_offset) const {
    if (header == nullptr)
      return INVALID_SPEED;
    if (directed_edge_offset >= header->directed_edge_count)
      throw std::runtime_error("Speed requested for edgeid beyond bounds of tile (offset: " +
                               std::to_string(directed_edge_offset) +
                               ", edge count: " + std::to_string(header->directed_edge_count));

    return *(speeds + directed_edge_offset);
  }

  // Returns true if this tile is valid or not
  bool operator()() const {
    return header != nullptr;
  }

  // These are all const pointers to data structures - once assigned,
  // the pointer values won't change.  The pointer targets are marked
  // as const volatile because they can be modified by code outside
  // our control (another process accessing a mmap'd file for example)
  volatile TileHeader* header;
  volatile Speed* speeds;
};

} // namespace traffic
} // namespace baldr
} // namespace valhalla
#endif
#endif // VALHALLA_BALDR_TRAFFICTILE_H_
