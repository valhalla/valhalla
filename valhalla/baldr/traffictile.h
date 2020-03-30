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

struct Speed {
  uint16_t speed_kmh : 7;        // km/h - so max range is 0-127km/h
  uint16_t congestion_level : 3; // some value from 0 to 7 to report back
  uint16_t age : 4;              // Age in minutes, relative to the timestamp in tile header
  uint16_t spare : 2;            // TODO: reserved for later use
#ifndef C_ONLY_INTERFACE
  inline bool valid() const volatile {
    return speed_kmh > 0 || congestion_level >= 4;
  }

  inline bool closed() const volatile {
    return speed_kmh == 0 && congestion_level >= 4;
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
static constexpr volatile Speed INVALID_SPEED{0, 0, 15, 0};
}
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
