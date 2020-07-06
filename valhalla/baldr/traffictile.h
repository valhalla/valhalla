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

using std::uint16_t;
using std::uint32_t;
using std::uint64_t;
#endif

// This value _of bitfield_ (not in kph) signals that the live speed is not known (max value of 7 bit
// number)
constexpr uint32_t UNKNOWN_TRAFFIC_SPEED_RAW = (1 << 7) - 1;
// Traffic speeds are encoded as 7 bits in `TrafficSpeed` below with a 2kph multiplier
constexpr uint32_t MAX_TRAFFIC_SPEED_KPH = (UNKNOWN_TRAFFIC_SPEED_RAW - 1) << 1;
// This is the value in kph that signifies a traffic speed is unknown
constexpr uint32_t UNKNOWN_TRAFFIC_SPEED_KPH = UNKNOWN_TRAFFIC_SPEED_RAW << 1;

struct TrafficSpeed {
  uint64_t overall_speed : 7; // 0-255kph in 2kph resolution (access with `get_overall_speed()`)
  uint64_t speed1 : 7;        // 0-255kph in 2kph resolution (access with `get_speed(0)`)
  uint64_t speed2 : 7;
  uint64_t speed3 : 7;
  uint64_t breakpoint1 : 8; // position = length * breakpoint1 * 255
  uint64_t breakpoint2 : 8; // position = length * breakpoint2 * 255
  uint64_t congestion1 : 6; // Stores 0 (unknown), or 1->63 (no congestion->max congestion)
  uint64_t congestion2 : 6; //
  uint64_t congestion3 : 6; //
  uint64_t spare : 2;

#ifndef C_ONLY_INTERFACE
  inline bool valid() const volatile {
    return breakpoint1 != 0;
  }
  inline bool closed() const volatile {
    return breakpoint1 != 0 && overall_speed == 0;
  }

  inline bool closed(std::size_t subsegment) const volatile {
    if (!valid())
      return false;
    switch (subsegment) {
      case 0:
        return speed1 == 0;
      case 1:
        return breakpoint1 < 255 && speed2 == 0;
      case 2:
        return breakpoint2 < 255 && speed3 == 0;
      default:
        assert(false);
    }
  }
  /// Returns overall speed in kph across edge
  inline uint8_t get_overall_speed() const volatile {
    return overall_speed << 1;
  }

  /**
   * Returns speed in a certain subsegment in kph
   * If the speed is unknown UNKNOWN_TRAFFIC_SPEED_KPH is returned
   * @param subsegment   the index of the subsegment you want
   * @return returns the speed of the subsegment or UNKNOWN_TRAFFIC_SPEED_KPH if unknown
   */
  inline uint8_t get_speed(std::size_t subsegment) const volatile {
    if (!valid())
      return UNKNOWN_TRAFFIC_SPEED_KPH;
    switch (subsegment) {
      case 0:
        return speed1 << 1;
      case 1:
        return speed2 << 1;
      case 2:
        return speed3 << 1;
      default:
        assert(false);
    }
  }
#endif
};

// per-speed-tile header
struct TrafficTileHeader {
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
static_assert(sizeof(TrafficTileHeader) == sizeof(uint64_t) * 4,
              "TrafficTileHeader type size different than expected");
static_assert(sizeof(TrafficSpeed) == sizeof(uint64_t),
              "TrafficSpeed type size is different than expected");
#endif // C_ONLY_INTERFACE

/**
 * A tile of live traffic data.  The layout is:
 *
 * TrafficTileHeader (24 bytes)
 * n x TrafficSpeed entries (n x 2 bytes)
 */
#ifndef C_ONLY_INTERFACE
namespace {
static constexpr volatile TrafficSpeed INVALID_SPEED{0u,
                                                     UNKNOWN_TRAFFIC_SPEED_RAW,
                                                     UNKNOWN_TRAFFIC_SPEED_RAW,
                                                     UNKNOWN_TRAFFIC_SPEED_RAW,
                                                     0u,
                                                     0u,
                                                     0u,
                                                     0u,
                                                     0u,
                                                     0u};

// Assert these constants are the same
// (We want to avoid including this file in graphconstants.h)
static_assert(MAX_TRAFFIC_SPEED_KPH == valhalla::baldr::kMaxTrafficSpeed,
              "Constants must be the same");
} // namespace
class TrafficTile {
public:
  TrafficTile(char* tile_ptr)
      : header{reinterpret_cast<volatile TrafficTileHeader*>(tile_ptr)},
        speeds{reinterpret_cast<volatile TrafficSpeed*>(tile_ptr + sizeof(TrafficTileHeader))} {
  }

  const volatile TrafficSpeed& trafficspeed(const uint32_t directed_edge_offset) const {
    if (header == nullptr)
      return INVALID_SPEED;
    if (directed_edge_offset >= header->directed_edge_count)
      throw std::runtime_error("TrafficSpeed requested for edgeid beyond bounds of tile (offset: " +
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
  volatile TrafficTileHeader* header;
  volatile TrafficSpeed* speeds;
};

} // namespace baldr
} // namespace valhalla
#endif
#endif // VALHALLA_BALDR_TRAFFICTILE_H_
