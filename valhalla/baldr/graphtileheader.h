#ifndef VALHALLA_BALDR_GRAPHTILEHEADER_H_
#define VALHALLA_BALDR_GRAPHTILEHEADER_H_

#include <cstdint>
#include <cstdlib>
#include <string>

#include <valhalla/baldr/graphid.h>

namespace valhalla {
namespace baldr {

// Number of expansion slots remaining in this tile. If you want to add
// something to the tile simply subtract one from this number and add it
// just before the empty_slots_ array below. NOTE that it can ONLY be an
// offset in bytes and NOT a bitfield or union or anything of that sort
constexpr size_t kEmptySlots = 13;

// Maximum size of the version string (stored as a fixed size
// character array so the GraphTileHeader size remains fixed).
constexpr size_t kMaxVersionSize = 16;

// Total number of binned edge bins in the tile
constexpr size_t kBinsDim = 5;
constexpr size_t kBinCount = kBinsDim * kBinsDim;

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
  const baldr::GraphId& graphid() const{
    return graphid_;
  }

  /**
   * Set the graph Id of this tile.
   * @param  graphid  GraphId (tileid and level) of this tile.
   */
  void set_graphid(const baldr::GraphId& graphid);

  /**
   * Gets the date when this tile was created. Days since pivot date.
   * @return  Returns the date this tile was created.
   */
  uint32_t date_created() const {
    return date_created_;
  }

  /**
   * Set the date created.
   * @param  date  Days since pivot date.
   */
  void set_date_created(const uint32_t date);

  /**
   * Gets the version of this tile.
   * @return  Returns the version of this tile.
   */
  std::string version() const {
    return version_;
  }

  /**
   * Set the version string.
   * @param  version Version string.
   */
  void set_version(const std::string& version);

  /**
   * Returns the data set Id (latest OSM changeset Id).
   * @return  Returns the data set Id.
   */
  uint64_t dataset_id() const {
    return dataset_id_;
  }

  /**
   * Set the data set Id (latest OSM changeset Id).
   * @param  id  Data set Id.
   */
  void set_dataset_id(const uint64_t id);

  /**
   * Get the relative road density within this tile.
   * @return  Returns the relative density for this tile (0-15).
   */
  uint32_t density() const {
    return static_cast<uint32_t>(density_);
  }

  /**
   * Set the relative road density within this tile.
   * @param  density  Relative road density within this tile (0-15).
   */
  void set_density(const uint32_t density);

  /**
   * Get the relative quality of name assignment for this tile.
   * @return  Returns relative name quality for this tile (0-15).
   */
  uint32_t name_quality() const {
    return static_cast<uint32_t>(name_quality_);
  }

  /**
   * Set the relative quality of name assignment for this tile.
   * @param   name_quality  Relative name quality for this tile (0-15).
   */
  void set_name_quality(const uint32_t name_quality);

  /**
   * Get the relative quality of speed assignment for this tile.
   * @return  Returns relative speed quality for this tile (0-15).
   */
  uint32_t speed_quality() const {
    return static_cast<uint32_t>(speed_quality_);
  }

  /**
   * Set the relative quality of speed assignment for this tile.
   * @param  speed_quality   Relative speed quality for this tile (0-15).
   */
  void set_speed_quality(const uint32_t speed_quality);

  /**
   * Get the relative quality of exit signs for this tile.
   * @return  Returns relative exit sign quality for this tile (0-15).
   */
  uint32_t exit_quality() const {
    return static_cast<uint32_t>(exit_quality_);
  }

  /**
   * Set the relative quality of exit signs for this tile.
   * @param  exit_quality   Relative exit sign quality for this tile (0-15).
   */
  void set_exit_quality(const uint32_t exit_quality);

  /**
   * Gets the number of nodes in this tile.
   * @return  Returns the number of nodes.
   */
  uint32_t nodecount() const {
    return nodecount_;
  }

  /**
   * Sets the number of nodes in this tile.
   * @param  count  Number of nodes within the tile.
   */
  void set_nodecount(const uint32_t count);

  /**
   * Gets the number of directed edges in this tile.
   * @return  Returns the number of directed edges.
   */
  uint32_t directededgecount() const {
    return directededgecount_;
  }

  /**
   * Sets the number of directed edges in this tile.
   * @param  count  Number of directed edges within the tile.
   */
  void set_directededgecount(const uint32_t count);

  /**
   * Gets the number of signs in this tile.
   * @return  Returns the number of signs.
   */
  uint32_t signcount() const {
    return signcount_;
  }

  /**
   * Sets the number of signs within this tile.
   * @param count Number of signs within the tile.
   */
  void set_signcount(const uint32_t count);

  /**
   * Gets the number of transit departures in this tile.
   * @return  Returns the number of transit departures.
   */
  uint32_t departurecount() const {
    return departurecount_;
  }

  /**
   * Sets the number of transit departures in this tile.
   * @param departures  The number of transit departures.
   */
  void set_departurecount(const uint32_t departures);

  /**
   * Gets the number of transit stops in this tile.
   * @return  Returns the number of transit stops.
   */
  uint32_t stopcount() const {
    return stopcount_;
  }

  /**
   * Sets the number of transit stops in this tile.
   * @param  stops  The number of transit stops.
   */
  void set_stopcount(const uint32_t stops);

  /**
   * Gets the number of transit routes in this tile.
   * @return  Returns the number of transit routes.
   */
  uint32_t routecount() const {
    return routecount_;
  }

  /**
   * Sets the number of transit routes in this tile.
   * @param  routes  The number of transit routes.
   */
  void set_routecount(const uint32_t routes);

  /**
   * Gets the number of transit schedules in this tile.
   * @return  Returns the number of transit schedules.
   */
  uint32_t schedulecount() const {
    return schedulecount_;
  }

  /**
   * Sets the number of transit schedules in this tile.
   * @param  schedules   The number of transit schedules.
   */
  void set_schedulecount(const uint32_t schedules);

  /**
   * Gets the number of transit transfers in this tile.
   * @return  Returns the number of transit transfers.
   */
  uint32_t transfercount() const  {
    return transfercount_;
  }

  /**
   * Sets the number of transit transfers in this tile.
   * @param  transfers   The number of transit transfers.
   */
  void set_transfercount(const uint32_t transfers);

  /**
   * Gets the number of access restrictions in this tile.
   * @return  Returns the number of restrictions.
   */
  uint32_t access_restriction_count() const {
    return access_restriction_count_;
  }

  /**
   * Sets the number of access restrictions in this tile.
   * @param  restrictions   The number of access restrictions.
   */
  void set_access_restriction_count(const uint32_t restrictions);

  /**
   * Gets the number of admin records in this tile.
   * @return  Returns the number of admin records.
   */
  uint32_t admincount() const {
    return admincount_;
  }

  /**
   * Sets the number of admin records within this tile.
   * @param count Number of admin records within the tile.
   */
  void set_admincount(const uint32_t count);

  /**
   * Get the offset to the Complex Restriction list in the forward direction.
   * @return  Returns the number of bytes to offset to the the list of
   *          complex restrictions.
   */
  uint32_t complex_restriction_forward_offset() const {
    return complex_restriction_forward_offset_;
  }

  /**
   * Sets the offset to the list of complex restrictions in the forward direction.
   * @param offset Offset in bytes to the start of the complex restriction
   *               list.
   */
  void set_complex_restriction_forward_offset(const uint32_t offset);

  /**
   * Get the offset to the Complex Restriction list in the reverse direction.
   * @return  Returns the number of bytes to offset to the the list of
   *          complex restrictions.
   */
  uint32_t complex_restriction_reverse_offset() const {
    return complex_restriction_reverse_offset_;
  }

  /**
   * Sets the offset to the list of complex restrictions in the reverse direction.
   * @param offset Offset in bytes to the start of the complex restriction
   *               list.
   */
  void set_complex_restriction_reverse_offset(const uint32_t offset);

  /**
   * Gets the offset to the edge info.
   * @return  Returns the number of bytes to offset to the edge information.
   */
  uint32_t edgeinfo_offset() const {
    return edgeinfo_offset_;
  }

  /**
   * Sets the offset to the edge info.
   * @param offset Offset in bytes to the start of the edge information.
   */
  void set_edgeinfo_offset(const uint32_t offset);

  /**
   * Gets the offset to the text list.
   * @return  Returns the number of bytes to offset to the text list.
   */
  uint32_t textlist_offset() const {
    return textlist_offset_;
  }

  /**
   * Sets the offset to the text list.
   * @param offset Offset in bytes to the start of the text list.
   */
  void set_textlist_offset(const uint32_t offset);

  /**
   * Get the offset to the given bin in the 5x5 grid, the bins contain
   * graphids for all the edges that intersect the bin
   * @param  column of the grid
   * @param  row of the grid
   * @return the begin and end offset in the list of edge ids
   */
  std::pair<uint32_t, uint32_t> bin_offset(size_t column, size_t row) const;

  /**
   * Get the offset to the given bin in the 5x5 grid, the bins contain
   * graphids for all the edges that intersect the bin
   * @param  index of the bin within row major grid array
   * @return the begin and end offset in the list of edge ids
   */
  std::pair<uint32_t, uint32_t> bin_offset(size_t index) const;

  /**
   * Sets the edge bin offsets
   * @param offsets the offsets
   */
  void set_edge_bin_offsets(const uint32_t (&offsets)[baldr::kBinCount]);

  /**
   * Gets the number of traffic segment Ids in this tile.
   * @return  Returns the number of traffic segment Ids.
   */
  uint32_t traffic_id_count() const {
    return traffic_id_count_;
  }

  /**
   * Sets the number of traffic segment Ids in this tile.
   * @param  count   The number of traffic segment Ids.
   */
  void set_traffic_id_count(const uint32_t count);

  /**
   * Gets the flag indicating whether this tile includes edge elevation data.
   * @return  Returns true if this tile includes edge elevation data.
   */
  bool has_edge_elevation() const {
    return has_edge_elevation_;
  }

  /**
   * Sets flag indicating whether this tile includes edge elevation data.
   * @param  elev  True if this tile includes edge elevation data.
   */
  void set_has_edge_elevation(const bool elev) {
    has_edge_elevation_ = elev;
  }

  /**
   * Gets the offset to the traffic segment Ids.
   * @return  Returns the number of bytes to offset to the traffic segment Ids.
   */
  uint32_t traffic_segmentid_offset() const {
    return traffic_segmentid_offset_;
  }

  /**
   * Sets the offset to the traffic segment Ids.
   * @param offset Offset in bytes to the start of the traffic segment Ids.
   */
  void set_traffic_segmentid_offset(const uint32_t offset);

  /**
   * Gets the offset to the traffic chunks. Chunks occur when an edge
   * is associated to more than one traffic segment. Chunks store lists of
   * segment Ids and "weights".
   * @return  Returns the number of bytes to offset to the traffic chunks.
   */
  uint32_t traffic_chunk_offset() const  {
    return traffic_chunk_offset_;
  }

  /**
   * Sets the offset to the traffic chunks.
   * @param offset Offset in bytes to the start of the traffic chunks.
   */
  void set_traffic_chunk_offset(const uint32_t offset);

  /**
   * Gets the offset to the lane connectivity data.
   * @return  Returns the number of bytes to offset to the the lane connectivity data.
   */
  uint32_t lane_connectivity_offset() const {
    return lane_connectivity_offset_;
  }

  /**
   * Sets the offset to the lane connectivity data.
   * @param offset Offset in bytes to the start of the lane connectivity data.
   */
  void set_lane_connectivity_offset(const uint32_t offset);

  /**
   * Gets the offset to the edge elevation data.
   * @return  Returns the number of bytes to offset to the the edge elevation data.
   */
  uint32_t edge_elevation_offset() const {
    return edge_elevation_offset_;
  }

  /**
   * Sets the offset to the edge elevation data.
   * @param offset Offset in bytes to the start of the edge elevation data.
   */
  void set_edge_elevation_offset(const uint32_t offset);

  /**
   * Get the offset to the end of the tile
   * @return the number of bytes in the tile, unless the last slot is used
   */
  uint32_t end_offset() const;

  /**
   * Sets the offset to the end of the tile
   * @param offset the offset in bytes to the end of the tile
   */
  void set_end_offset(uint32_t offset);

 protected:
  // GraphId (tileid and level) of this tile
  GraphId graphid_;

  // baldr version.
  char version_[kMaxVersionSize];

  // Dataset Id
  uint64_t dataset_id_;

  // Quality metrics. These are 4 bit (0-15) relative quality indicators.
  uint64_t density_       : 4;
  uint64_t name_quality_  : 4;
  uint64_t speed_quality_ : 4;
  uint64_t exit_quality_  : 4;
  uint64_t spare1_        : 48;

  // Number of transit records
  uint64_t departurecount_ : 24;
  uint64_t stopcount_      : 16;
  uint64_t routecount_     : 12;
  uint64_t schedulecount_  : 12;

  // Number of transit transfers and number of traffic segment Ids (
  // generally the same as the number of directed edges but can be 0)
  uint64_t transfercount_      : 16;
  uint64_t traffic_id_count_   : 24;
  uint64_t has_edge_elevation_ : 1;
  uint64_t spare2_             : 23;

  // Date the tile was created. Days since pivot date.
  uint32_t date_created_;

  // Record counts (for fixed size records)
  uint32_t nodecount_;                  // Number of nodes
  uint32_t directededgecount_;          // Number of directed edges
  uint32_t signcount_;                  // Number of signs
  uint32_t access_restriction_count_;   // Number of access restriction records
  uint32_t admincount_;                 // Number of admin records

  // Offsets to beginning of data (for variable size records)
  uint32_t complex_restriction_forward_offset_; // Offset to complex restriction list
  uint32_t complex_restriction_reverse_offset_; // Offset to complex restriction list
  uint32_t edgeinfo_offset_;            // Offset to edge info
  uint32_t textlist_offset_;            // Offset to text list

  // Offsets for each bin of the 5x5 grid (for search/lookup)
  uint32_t bin_offsets_[kBinCount];

  // Offset to beginning of the traffic segment associations.
  uint32_t traffic_segmentid_offset_;

  // Offset to the beginning of traffic segment "chunks". Chunks occur when an
  // edge maps to more than one traffic segment.
  uint32_t traffic_chunk_offset_;

  // Offset to beginning of the lane connectivity data
  uint32_t lane_connectivity_offset_;

  // Offset to the beginning of the edge elevation data.
  uint32_t edge_elevation_offset_;

  // Marks the end of this version of the tile with the rest of the slots
  // being available for growth. If you want to use one of the empty slots,
  // simply add a uint32_t some_offset_; just above empty_slots_ and decrease
  // kEmptySlots by 1. Note that you can ONLY add an offset here and NOT a
  // bitfield or union or anything like that
  uint32_t empty_slots_[kEmptySlots];
};

}
}

#endif  // VALHALLA_BALDR_GRAPHTILEHEADER_H_
