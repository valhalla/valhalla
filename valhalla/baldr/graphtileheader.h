#ifndef VALHALLA_BALDR_GRAPHTILEHEADER_H_
#define VALHALLA_BALDR_GRAPHTILEHEADER_H_

#include <cstdlib>
#include <string>

#include <valhalla/baldr/graphid.h>

namespace valhalla {
namespace baldr {

// Maximum size of the version string (stored as a fixed size
// character array so the GraphTileHeader size remains fixed).
constexpr size_t kMaxVersionSize = 16;

/**
 * Summary information about the graph tile. Includes version
 * information and offsets to the various types of data.
 */
class GraphTileHeader {
 public:
  /**
   * Constructor
   */
  GraphTileHeader();

  /**
   * Gets the internal version
   * @return  Returns the internal version of this tile.
   */
  int64_t internal_version() const;

  /**
   * Gets the date when this tile was created. Returns a Unix timestamp
   * (seconds since 1/1/1970)
   * @return  Returns the date this tile was created.
   */
  uint64_t date_created() const;

  /**
   * Gets the version of this tile
   * @return  Returns the version of this tile.
   */
  std::string version() const;

  /**
   * Get the relative quality of name assignment for this tile.
   * @return  Returns relative name quality for this tile (0-15).
   */
  uint32_t name_quality() const;

  /**
   * Get the relative quality of speed assignment for this tile.
   * @return  Returns relative speed quality for this tile (0-15).
   */
  uint32_t speed_quality() const;

  /**
   * Get the relative quality of exit signs for this tile.
   * @return  Returns relative exit sign quality for this tile (0-15).
   */
  uint32_t exit_quality() const;

  /**
   * Get the GraphId (tileid and level) of this tile.
   * @return  Returns the graph Id.
   */
  const baldr::GraphId& graphid() const;

  /**
   * Gets the number of nodes in this tile.
   * @return  Returns the number of nodes.
   */
  uint32_t nodecount() const;

  /**
   * Gets the number of directed edges in this tile.
   * @return  Returns the number of directed edges.
   */
  uint32_t directededgecount() const;

  /**
   * Gets the number of signs in this tile.
   * @return  Returns the number of signs.
   */
  uint32_t signcount() const;

  /**
   * Gets the number of transit departures in this tile.
   * @return  Returns the number of transit departures.
   */
  uint32_t departurecount() const;

  /**
   * Gets the number of transit trips in this tile.
   * @return  Returns the number of transit trips.
   */
   uint32_t tripcount() const;

  /**
   * Gets the number of transit stops in this tile.
   * @return  Returns the number of transit stops.
   */
  uint32_t stopcount() const;

  /**
   * Gets the number of transit routes in this tile.
   * @return  Returns the number of transit routes.
   */
  uint32_t routecount() const;

  /**
   * Gets the number of transit transfers in this tile.
   * @return  Returns the number of transit transfers.
   */
  uint32_t transfercount() const;

  /**
   * Gets the number of transit calendar exceptions in this tile.
   * @return  Returns the number of transit calendar exceptions.
   */
  uint32_t calendarcount() const;

  /**
   * Gets the number of admins in this tile.
   * @return  Returns the number of admins.
   */
  uint32_t admincount() const;

  /**
   * Gets the offset to the edge info.
   * @return  Returns the number of bytes to offset to the edge information.
   */
  uint32_t edgeinfo_offset() const;

  /**
   * Gets the offset to the text list.
   * @return  Returns the number of bytes to offset to the text list.
   */
  uint32_t textlist_offset() const;

  /**
   * Get the offset to the administrative information.
   * @return  Returns the number of bytes to offset to the administrative
   *          information.
   */
  uint32_t admininfo_offset() const;

  /**
   * Get the offset to the Multi-Edge Restriction list. (TODO)
   * @return  Returns the number of bytes to offset to the the list of
   *          Multi-Edge Restrictions.
   */
  uint32_t merlist_offset() const;

  /**
   * Get the offset to the timed restriction list. (TODO)
   * @return  Returns the number of bytes to offset to the the list of
   *          timed restrictions.
   */
  uint32_t timedres_offset() const;

 protected:

  // Internal version info
  int64_t internal_version_;

  // Date the tile was created. Unix timestamp (seconds since 1/1/1970)
  uint64_t date_created_;

  // baldr version.
  char version_[kMaxVersionSize];

  // Quality metrics. These are 4 bit (0-15) relative quality indicators.
  struct TileQuality {
    uint64_t name           : 4;
    uint64_t speed          : 4;
    uint64_t exit           : 4;
    uint64_t spare          : 52;
  };
  TileQuality quality_;

  // GraphId (tileid and level) of this tile
  GraphId graphid_;

  // Number of nodes
  uint32_t nodecount_;

  // Number of directed edges
  uint32_t directededgecount_;

  // Number of signs
  uint32_t signcount_;

  // Number of transit departure records
  struct Transit1 {
    uint64_t departurecount : 24;
    uint64_t tripcount      : 24;
    uint64_t stopcount      : 16;
  };
  Transit1 transit1_;

  // Transit information for this tile.
  struct Transit2 {
    uint64_t routecount     : 16;
    uint64_t transfercount  : 16;
    uint64_t calendarcount  : 16;
    uint64_t spare          : 16;
  };
  Transit2 transit2_;

  // Number of admin records
  uint32_t admincount_;

  // Offset to edge info
  uint32_t edgeinfo_offset_;

  // Offset to text list
  uint32_t textlist_offset_;

  // Offset to the multi-edge restriction list
  uint32_t merlist_offset_;

  // Offset to the timed restriction list
  uint32_t timedres_offset_;
};

}
}

#endif  // VALHALLA_BALDR_GRAPHTILEHEADER_H_
