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

// Total number of binned edge cells in the tile
constexpr size_t kGridDim = 5;
constexpr size_t kCellCount = kGridDim * kGridDim;

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
   * Get the GraphId (tileid and level) of this tile.
   * @return  Returns the graph Id.
   */
  const baldr::GraphId& graphid() const;

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
   * Get the relative road density within this tile.
   * @return  Returns the relative density for this tile (0-15).
   */
  uint32_t density() const;

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
   * Gets the number of access restrictions in this tile.
   * @return  Returns the number of restrictions.
   */
  uint32_t access_restriction_count() const;

  /**
   * Gets the number of admin records in this tile.
   * @return  Returns the number of admin records.
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
   * Get the offset to the Complex Restriction list.
   * @return  Returns the number of bytes to offset to the the list of
   *          complex restrictions.
   */
  uint32_t complex_restriction_offset() const;

  /**
   * Get the offset to the given cell in the 5x5 grid, the cells contain
   * graphids for all the edges that intersect the cell
   * @param  column of the grid
   * @param  row of the grid
   * @return the begin and end offset in the list of edge ids
   */
  std::pair<uint32_t, uint32_t> cell_offset(size_t column, size_t row) const;

 protected:
  // GraphId (tileid and level) of this tile
  GraphId graphid_;

  // Date the tile was created. Unix timestamp (seconds since 1/1/1970)
  uint64_t date_created_;

  // baldr version.
  char version_[kMaxVersionSize];

  // Quality metrics. These are 4 bit (0-15) relative quality indicators.
  uint64_t density_       : 4;
  uint64_t name_quality_  : 4;
  uint64_t speed_quality_ : 4;
  uint64_t exit_quality_  : 4;
  uint64_t spare1_        : 48;

  // Number of transit departure records
  uint64_t departurecount_ : 24;
  uint64_t stopcount_      : 16;
  uint64_t routecount_     : 12;
  uint64_t transfercount_  : 12;

  // Record counts (for fixed size records)
  uint32_t nodecount_;                  // Number of nodes
  uint32_t directededgecount_;          // Number of directed edges
  uint32_t signcount_;                  // Number of signs
  uint32_t access_restriction_count_;   // Number of access restriction records
  uint32_t admincount_;                 // Number of admin records

  // Offsets to beginning of data (for variable size records)
  uint32_t edgeinfo_offset_;            // Offset to edge info
  uint32_t textlist_offset_;            // Offset to text list
  uint32_t complex_restriction_offset_; // Offset to complex restriction list

  // Offsets for each cell of the 5x5 grid (for search/lookup)
  uint32_t cell_offsets_[kCellCount + 1];
};

}
}

#endif  // VALHALLA_BALDR_GRAPHTILEHEADER_H_
