#ifndef VALHALLA_BALDR_TRAFFICTILE_H_
#define VALHALLA_BALDR_TRAFFICTILE_H_

#include <cassert>
#include <cstdint>
#include <vector>

namespace valhalla {
namespace baldr {
namespace traffic {
struct Speed {
  std::uint16_t speed_kmh : 7;        // km/h - so max range is 0-127km/h
  std::uint16_t congestion_level : 3; // some value from 0 to 7 to report back
  std::uint16_t is_scale : 1;         // treat speed as a floating point multiplier to edge speed
  std::uint16_t age : 4;              //
  std::uint16_t has_incident : 1;     // TODO: reserved for whether edge has an incident or not
};

// single incident record
struct Incident {
  std::uint64_t edge_index : 21;
  std::uint64_t incident_type : 8;
  std::uint64_t start_location : 10;
  std::uint64_t length : 10;
  std::uint64_t spare : 15;
  Incident(const Incident& other) = default;
  Incident(const volatile Incident& other)
      : edge_index{other.edge_index}, incident_type{other.incident_type},
        start_location{other.start_location}, length{other.length}, spare{0} {
  }
  Incident& operator=(const Incident& other) = default;
};

// per-speed-tile header
struct TileHeader {
  std::uint64_t tile_id;
  std::uint32_t directededge_count;
  std::uint32_t incident_buffer_size;
  std::uint64_t active_incident_buffer : 1;
  std::uint64_t last_update : 63; // seconds since epoch
};

/**
 * A tile of live traffic data.  The layout is:
 *
 * TileHeader (24 bytes)
 * n x Speed entries (n x 4 bytes)
 * IncidentsHeader (8 bytes)
 * m x Incidents entries (buffer 0) (m x 8 bytes)
 * m x Incidents entries (buffer 1) (m x 8 bytes)
 *
 * The Incidents are sparse - the TileHeader->incident_buffer_size specifies how
 * much space is available in the tile.  The IncidentsHeader fields specify where
 * incidents begin/end in this buffer.  This allows for new incident lists to be
 * placed in the buffer, then the begin/end updated to atomicly move readers to
 * a new location.
 */
class Tile {
public:
  Tile(const char* tile_ptr)
      : header{reinterpret_cast<const TileHeader*>(tile_ptr)},
        speeds{reinterpret_cast<const volatile Speed*>(tile_ptr + offsetof(Tile, speeds))},
        incident_count_0{reinterpret_cast<const volatile std::uint32_t*>(
            tile_ptr + offsetof(Tile, incident_count_0))},
        incident_count_1{
            reinterpret_cast<const volatile std::uint32_t*>(
                tile_ptr + offsetof(Tile, incident_count_1))},
        incidents_0{reinterpret_cast<const volatile Incident*>(
            tile_ptr + offsetof(Tile, incidents_0))},
        incidents_1{reinterpret_cast<const volatile Incident*>(
            tile_ptr + offsetof(Tile, incidents_1))} {
  }

  Tile(const Tile& other) = default;
  Tile& operator=(const Tile& other) = default;

  const volatile Speed* getTrafficForDirectedEdge(const std::uint32_t directed_edge_offset) {
    assert(directed_edge_offset < header->directededge_count);
    return speeds + directed_edge_offset;
  }

  const std::vector<const Incident>
  getIncidentsForDirectedEdge(const std::uint32_t directed_edge_offset) {
    // Sanity check and exit early if false
    if (!getTrafficForDirectedEdge(directed_edge_offset)->has_incident)
      return {};

    // Copy the current active buffer so it won't change during our usage below
    auto active_buffer = header->active_incident_buffer;

    const auto& count = active_buffer == 0 ? *incident_count_0 : *incident_count_1;
    const auto& buffer = active_buffer == 0 ? incidents_0 : incidents_1;

    // Find the beginning and end of edges that match our directed_edge_offset
    // Note that equal_range uses a binary search when given a LegacyRandomAccessIterator,
    // which raw pointers satisfy.
    struct Comp {
      bool operator()(const volatile Incident& incident, std::uint32_t edge_index) const {
        return incident.edge_index < edge_index;
      }
      bool operator()(std::uint32_t edge_index, const volatile Incident& incident) const {
        return edge_index < incident.edge_index;
      }
    };
    const auto range = std::equal_range(buffer, buffer + count, directed_edge_offset, Comp{});

    // Copy the results so that they're non-volatile for our caller
    // TODO: race condition: make sure our range is still valid
    return std::vector<const Incident>(range.first, range.second);
  }

protected:
  // These are all const pointers to data structures - once assigned,
  // the pointer values won't change.  The pointer targets are marked
  // as const volatile because they can be modified by code outside
  // our control (another process accessing a mmap'd file for example)
  const volatile TileHeader* header;
  const volatile Speed* speeds;
  const volatile std::uint32_t* incident_count_0;
  const volatile std::uint32_t* incident_count_1;
  const volatile Incident* incidents_0;
  const volatile Incident* incidents_1;
};

} // namespace traffic
} // namespace baldr
} // namespace valhalla
#endif // VALHALLA_BALDR_TRAFFICTILE_H_
