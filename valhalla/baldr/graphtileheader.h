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
   * Gets the offset to the edge info.
   * @return  Returns the number of bytes to offset to the edge information.
   */
  uint32_t edgeinfo_offset() const;

  /**
   * Gets the offset to the text list.
   * @return  Returns the number of bytes to offset to the text list.
   */
  uint32_t streetlist_offset() const;

  /**
   * Get the offset to the administrative information.
   * @return  Returns the number of bytes to offset to the administrative
   *          information.
   */
  uint32_t admininfo_offset() const;

  /**
   * Gets the offset to the name list.
   * @return  Returns the number of bytes to offset to the text list.
   */
  uint32_t namelist_offset() const;

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

  /**
   * Get the offset to the transit schedule list. (TODO)
   * @return  Returns the number of bytes to offset to the the list of
   *          transit departures.
   */
  uint32_t transit_offset() const;

 protected:

  // Internal version info
  int64_t internal_version_;

  // Date the tile was created. Unix timestamp (seconds since 1/1/1970)
  uint64_t date_created_;

  // baldr version.
  char version_[kMaxVersionSize];

  // GraphId (tileid and level) of this tile
  GraphId graphid_;

  // Number of nodes
  uint32_t nodecount_;

  // Number of directed edges
  uint32_t directededgecount_;

  // Number of signs
  uint32_t signcount_;

  // Offset to edge info
  uint32_t edgeinfo_offset_;

  // Offset to street list
  uint32_t streetlist_offset_;

  // Offset to the administrative information
  uint32_t admininfo_offset_;

  // Offset to name list
  uint32_t namelist_offset_;

  // Offset to the multi-edge restriction list
  uint32_t merlist_offset_;

  // Offset to the timed restriction list
  uint32_t timedres_offset_;

  // Offset to transit schedule list
  uint32_t transit_offset_;
};

}
}

#endif  // VALHALLA_BALDR_GRAPHTILEHEADER_H_
