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
#include <memory>
#include <string>
#include <type_traits>
#include <valhalla/baldr/graphconstants.h>
#include <valhalla/baldr/graphmemory.h>
#include <valhalla/baldr/json.h>
#else
#include <stdint.h>
#endif
#include <valhalla/valhalla.h>

#ifndef C_ONLY_INTERFACE
namespace valhalla {
namespace baldr {

using std::uint16_t;
using std::uint32_t;
using std::uint64_t;
#endif

// The version of traffic tile format
// This is not intended to allow for easy migration between in-compatible data-formats
// Changes are still expected to be applied in a backwards compatible way by using
// the spare bits available.
// This is more of a break-the-glass escape hatch for when the current format has
// reached its limits
const uint8_t TRAFFIC_TILE_VERSION = VALHALLA_VERSION_MAJOR;

// This value _of bitfield_ (not in kph) signals that the live speed is not known (max value of 7 bit
// number)
constexpr uint32_t UNKNOWN_TRAFFIC_SPEED_RAW = (1 << 7) - 1;
// Traffic speeds are encoded as 7 bits in `TrafficSpeed` below with a 2kph multiplier
constexpr uint32_t MAX_TRAFFIC_SPEED_KPH = (UNKNOWN_TRAFFIC_SPEED_RAW - 1) << 1;
// This is the value in kph that signifies a traffic speed is unknown
constexpr uint32_t UNKNOWN_TRAFFIC_SPEED_KPH = UNKNOWN_TRAFFIC_SPEED_RAW << 1;

constexpr uint8_t UNKNOWN_CONGESTION_VAL = 0;
constexpr uint8_t MAX_CONGESTION_VAL = 63;

struct TrafficSpeed {
  uint64_t
      overall_encoded_speed : 7; // 0-255kph in 2kph resolution (access with `get_overall_speed()`)
  uint64_t encoded_speed1 : 7;   // 0-255kph in 2kph resolution (access with `get_speed(0)`)
  uint64_t encoded_speed2 : 7;
  uint64_t encoded_speed3 : 7;
  uint64_t breakpoint1 : 8;   // position = length * (breakpoint1 / 255)
  uint64_t breakpoint2 : 8;   // position = length * (breakpoint2 / 255)
  uint64_t congestion1 : 6;   // Stores 0 (unknown), or 1->63 (no congestion->max congestion)
  uint64_t congestion2 : 6;   //
  uint64_t congestion3 : 6;   //
  uint64_t has_incidents : 1; // Are there incidents on this edge in the corresponding incident tile
  uint64_t spare : 1;

#ifndef C_ONLY_INTERFACE
  inline bool speed_valid() const volatile {
    return breakpoint1 != 0 && overall_encoded_speed != UNKNOWN_TRAFFIC_SPEED_RAW;
  }

  inline bool closed() const volatile {
    return breakpoint1 != 0 && overall_encoded_speed == 0;
  }

  inline bool closed(std::size_t subsegment) const volatile {
    if (!speed_valid())
      return false;
    switch (subsegment) {
      case 0:
        return encoded_speed1 == 0 || congestion1 == MAX_CONGESTION_VAL;
      case 1:
        return breakpoint1 < 255 && (encoded_speed2 == 0 || congestion2 == MAX_CONGESTION_VAL);
      case 2:
        return breakpoint2 < 255 && (encoded_speed3 == 0 || congestion3 == MAX_CONGESTION_VAL);
      default:
        assert(false);
        throw std::logic_error("Bad subsegment");
    }
  }

  /// Returns overall speed in kph across edge
  inline uint8_t get_overall_speed() const volatile {
    return overall_encoded_speed << 1;
  }

  /**
   * Returns speed in a certain subsegment in kph
   * If the speed is unknown UNKNOWN_TRAFFIC_SPEED_KPH is returned
   * @param subsegment   the index of the subsegment you want
   * @return returns the speed of the subsegment or UNKNOWN_TRAFFIC_SPEED_KPH if unknown
   */
  inline uint8_t get_speed(std::size_t subsegment) const volatile {
    if (!speed_valid())
      return UNKNOWN_TRAFFIC_SPEED_KPH;
    switch (subsegment) {
      case 0:
        return encoded_speed1 << 1;
      case 1:
        return encoded_speed2 << 1;
      case 2:
        return encoded_speed3 << 1;
      default:
        assert(false);
        throw std::logic_error("Bad subsegment");
    }
  }

  constexpr TrafficSpeed()
      : overall_encoded_speed{0}, encoded_speed1{0}, encoded_speed2{0}, encoded_speed3{0},
        breakpoint1{0}, breakpoint2{0}, congestion1{0}, congestion2{0}, congestion3{0},
        has_incidents{0}, spare{0} {
  }

  constexpr TrafficSpeed(const uint32_t overall_encoded_speed,
                         const uint32_t s1,
                         const uint32_t s2,
                         const uint32_t s3,
                         const uint32_t b1,
                         const uint32_t b2,
                         const uint32_t c1,
                         const uint32_t c2,
                         const uint32_t c3,
                         const bool incidents)
      : overall_encoded_speed{overall_encoded_speed}, encoded_speed1{s1}, encoded_speed2{s2},
        encoded_speed3{s3}, breakpoint1{b1}, breakpoint2{b2}, congestion1{c1}, congestion2{c2},
        congestion3{c3}, has_incidents{incidents}, spare{0} {
  }

  json::MapPtr json() const volatile {
    auto live_speed = json::map({});
    if (speed_valid()) {
      live_speed->emplace("overall_speed", static_cast<uint64_t>(get_overall_speed()));
      auto speed = static_cast<uint64_t>(get_speed(0));
      if (speed == UNKNOWN_TRAFFIC_SPEED_KPH)
        live_speed->emplace("speed_0", nullptr);
      else
        live_speed->emplace("speed_0", speed);
      auto congestion = (congestion1 - 1.0) / 62.0;
      if (congestion < 0)
        live_speed->emplace("congestion_0", nullptr);
      else
        live_speed->emplace("congestion_0", json::fixed_t{congestion, 2});
      live_speed->emplace("breakpoint_0", json::fixed_t{breakpoint1 / 255.0, 2});

      speed = static_cast<uint64_t>(get_speed(1));
      if (speed == UNKNOWN_TRAFFIC_SPEED_KPH)
        live_speed->emplace("speed_1", nullptr);
      else
        live_speed->emplace("speed_1", speed);
      congestion = (congestion2 - 1.0) / 62.0;
      if (congestion < 0)
        live_speed->emplace("congestion_1", nullptr);
      else
        live_speed->emplace("congestion_1", json::fixed_t{congestion, 2});
      live_speed->emplace("breakpoint_1", json::fixed_t{breakpoint2 / 255.0, 2});

      speed = static_cast<uint64_t>(get_speed(2));
      if (speed == UNKNOWN_TRAFFIC_SPEED_KPH)
        live_speed->emplace("speed_2", nullptr);
      else
        live_speed->emplace("speed_2", speed);
      congestion = (congestion3 - 1.0) / 62.0;
      if (congestion < 0)
        live_speed->emplace("congestion_2", nullptr);
      else
        live_speed->emplace("congestion_2", json::fixed_t{congestion, 2});
    }
    return live_speed;
  }
#endif
};

// per-speed-tile header
struct TrafficTileHeader {
  uint64_t tile_id;
  uint64_t last_update; // seconds since epoch
  uint32_t directed_edge_count;
  uint32_t traffic_tile_version;
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
static constexpr volatile TrafficSpeed INVALID_SPEED{
    UNKNOWN_TRAFFIC_SPEED_RAW,
    UNKNOWN_TRAFFIC_SPEED_RAW,
    UNKNOWN_TRAFFIC_SPEED_RAW,
    UNKNOWN_TRAFFIC_SPEED_RAW,
    0u,
    0u,
    0u,
    0u,
    0u,
    0u,
};

// Assert these constants are the same
// (We want to avoid including this file in graphconstants.h)
static_assert(MAX_TRAFFIC_SPEED_KPH == valhalla::baldr::kMaxTrafficSpeed,
              "Constants must be the same");
} // namespace
class TrafficTile {
public:
  // Disallow copying
  TrafficTile(const TrafficTile&) = delete;
  TrafficTile& operator=(const TrafficTile&) = delete;

  // Allow moving
  TrafficTile(TrafficTile&&) = default;
  TrafficTile& operator=(TrafficTile&&) = default;

  TrafficTile(std::unique_ptr<const GraphMemory> memory)
      : memory_(std::move(memory)),
        header(memory_ ? reinterpret_cast<volatile TrafficTileHeader*>(memory_->data) : nullptr),
        speeds(memory_ ? reinterpret_cast<volatile TrafficSpeed*>(memory_->data +
                                                                  sizeof(TrafficTileHeader))
                       : nullptr) {
  }

  const volatile TrafficSpeed& trafficspeed(const uint32_t directed_edge_offset) const {
    if (header == nullptr || header->traffic_tile_version != TRAFFIC_TILE_VERSION) {
      return INVALID_SPEED;
    }
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

private:
  std::unique_ptr<const GraphMemory> memory_;

public:
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
