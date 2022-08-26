#ifndef VALHALLA_BALDR_GRAPHTILEHEADER_H_
#define VALHALLA_BALDR_GRAPHTILEHEADER_H_

#include <cstdint>
#include <cstdlib>
#include <string>

#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/tilehierarchy.h>
#include <valhalla/midgard/logging.h>
#include <valhalla/midgard/pointll.h>

namespace valhalla {
namespace baldr {

// Number of expansion slots remaining in this tile. If you want to add
// something to the tile simply subtract one from this number and add it
// just before the empty_slots_ array below. NOTE that it can ONLY be an
// offset in bytes and NOT a bitfield or union or anything of that sort
constexpr size_t kEmptySlots = 11;

// Maximum size of the version string (stored as a fixed size
// character array so the GraphTileHeader size remains fixed).
constexpr size_t kMaxVersionSize = 16;

// Maximum value used for quality metrics
constexpr uint32_t kMaxQualityMeasure = 15;

// Maximum number of node transitions per tile (22 bits)
constexpr uint32_t kMaxNodeTransitions = 4194303;

// Maximum number of signs (24 bits).
constexpr uint32_t kMaxSigns = 16777215;

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
  GraphId graphid() const {
    return GraphId(graphid_);
  }

  /**
   * Set the graph Id of this tile.
   * @param  graphid  GraphId (tileid and level) of this tile.
   */
  void set_graphid(const GraphId& graphid) {
    graphid_ = graphid.value;
  }

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
  void set_density(const uint32_t density) {
    density_ = (density <= kMaxDensity) ? density : kMaxDensity;
  }

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
  void set_name_quality(const uint32_t name_quality) {
    name_quality_ = (name_quality <= kMaxQualityMeasure) ? name_quality : kMaxQualityMeasure;
  }

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
  void set_speed_quality(const uint32_t speed_quality) {
    speed_quality_ = (speed_quality <= kMaxQualityMeasure) ? speed_quality : kMaxQualityMeasure;
  }

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
  void set_exit_quality(const uint32_t exit_quality) {
    exit_quality_ = (exit_quality <= kMaxQualityMeasure) ? exit_quality : kMaxQualityMeasure;
  }

  /**
   * Gets the flag indicating whether this tile includes elevation data.
   * @return  Returns true if this tile includes edge elevation data.
   */
  bool has_elevation() const {
    return has_elevation_;
  }

  /**
   * Sets flag indicating whether this tile includes elevation data.
   * @param  elev  True if this tile includes edge elevation data.
   */
  void set_has_elevation(const bool elev) {
    has_elevation_ = elev;
  }

  /**
   * Gets the flag indicating whether this tile includes extended directed edge attributes.
   * @return  Returns true if this tile includes extended directed edge attributes.
   */
  bool has_ext_directededge() const {
    return has_ext_directededge_;
  }

  /**
   * Sets flag indicating whether this tile includes extended directed edge attributes.
   * @param  elev  True if this tile includes extended directed edge attributes.a.
   */
  void set_has_ext_directededge(const bool ext) {
    has_ext_directededge_ = ext;
  }

  /**
   * Get the base (SW corner) of the tile.
   * @return Returns the base lat,lon of the tile (degrees).
   */
  midgard::PointLL base_ll() const {
    GraphId id(graphid_);

    if (id.level() == TileHierarchy::GetTransitLevel().level) {
      return TileHierarchy::GetTransitLevel().tiles.Base(id.tileid());
    }
    return TileHierarchy::levels()[id.level()].tiles.Base(id.tileid());
    // return midgard::PointLL(base_ll_.first, base_ll_.second);
  }

  /**
   * Sets the base (SW corner) lat,lon of the tile.
   * @param ll  Base lat,lon of the tile.
   */
  void set_base_ll(const midgard::PointLL& ll) {
    base_ll_.first = ll.lng();
    base_ll_.second = ll.lat();
  }

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
  void set_dataset_id(const uint64_t id) {
    dataset_id_ = id;
  }

  /**
   * Gets the number of nodes in this tile.
   * @return  Returns the number of nodes.
   */
  uint32_t nodecount() const {
    return nodecount_;
  }

  /**
   * Sets the number of nodes in this tile. Error occurs if the count is greater than kMaxGraphId.
   * @param  count  Number of nodes within the tile.
   */
  void set_nodecount(const uint32_t count) {
    if (count > kMaxGraphId) {
      // Consider this a catastrophic error
      LOG_ERROR("Tile exceeded maximum node count: " + std::to_string(count));
    }
    nodecount_ = count;
  }

  /**
   * Gets the number of directed edges in this tile.
   * @return  Returns the number of directed edges.
   */
  uint32_t directededgecount() const {
    return directededgecount_;
  }

  /**
   * Sets the number of directed edges in this tile. Error occurs if the count is greater than
   * kMaxGraphId.
   * @param  count  Number of directed edges within the tile.
   */
  void set_directededgecount(const uint32_t count) {
    if (count > kMaxGraphId) {
      // Consider this a catastrophic error
      LOG_ERROR("Tile exceeded maximum directededge count: " + std::to_string(count));
    }
    directededgecount_ = count;
  }

  /**
   * Gets the count of predicted speed records.
   * @return  Returns the count of predicted speed records.
   */
  uint32_t predictedspeeds_count() const {
    return predictedspeeds_count_;
  }

  /**
   * Sets count of predicted speed records within the tile. Error occurs if this is greater than
   * kMaxGraphId.
   * @param offset Count of predicted speed records within the tile.
   */
  void set_predictedspeeds_count(const uint32_t count) {
    if (count > kMaxGraphId) {
      // Consider this a catastrophic error
      LOG_ERROR("Tile exceeded maximum predicted speed count: " + std::to_string(count));
    }
    predictedspeeds_count_ = count;
  }

  /**
   * Gets the number of node transitions in this tile.
   * @return  Returns the number of node transitions.
   */
  uint32_t transitioncount() const {
    return transitioncount_;
  }

  /**
   * Sets the number of node transitions in this tile. Error occurs if more than kMaxNodeTransitions.
   * @param  count  Number of node transitions within the tile.
   */
  void set_transitioncount(const uint32_t count) {
    if (count > kMaxNodeTransitions) {
      // Consider this a catastrophic error
      LOG_ERROR("Tile exceeded maximum node transition count: " + std::to_string(count));
    }
    transitioncount_ = count;
  }

  /**
   * Gets the number of signs in this tile.
   * @return  Returns the number of signs.
   */
  uint32_t signcount() const {
    return signcount_;
  }

  /**
   * Sets the number of signs within this tile. Error occurs if more than kMaxSigns.
   * @param count Number of signs within the tile.
   */
  void set_signcount(const uint32_t count) {
    if (count > kMaxSigns) {
      // Consider this a catastrophic error
      LOG_ERROR("Tile exceeded maximum node transition count: " + std::to_string(count));
    }
    signcount_ = count;
  }

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
  uint32_t transfercount() const {
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
   * @param  n  The number of access restrictions.
   */
  void set_access_restriction_count(const uint32_t n) {
    access_restriction_count_ = n;
  }

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
  void set_admincount(const uint32_t count) {
    admincount_ = count;
  }

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
  void set_complex_restriction_forward_offset(const uint32_t offset) {
    complex_restriction_forward_offset_ = offset;
  }

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
  void set_complex_restriction_reverse_offset(const uint32_t offset) {
    complex_restriction_reverse_offset_ = offset;
  }

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
  void set_edgeinfo_offset(const uint32_t offset) {
    edgeinfo_offset_ = offset;
  }

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
  void set_textlist_offset(const uint32_t offset) {
    textlist_offset_ = offset;
  }

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
  void set_date_created(const uint32_t date) {
    date_created_ = date;
  }

  /**
   * Get the offset to the given bin in the 5x5 grid, the bins contain
   * graphids for all the edges that intersect the bin
   * @param  column of the grid
   * @param  row of the grid
   * @return the begin and end offset in the list of edge ids
   */
  std::pair<uint32_t, uint32_t> bin_offset(size_t column, size_t row) const {
    return bin_offset(row * kBinsDim + column);
  }

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
  void set_lane_connectivity_offset(const uint32_t offset) {
    lane_connectivity_offset_ = offset;
  }

  /**
   * Gets the number of  turn lanes in this tile.
   * @return  Returns the number of  turn lanes.
   */
  uint32_t turnlane_count() const {
    return turnlane_count_;
  }

  /**
   * Sets the number of turn lanes within this tile.
   * @param count Number of turn lanes within the tile.
   */
  void set_turnlane_count(const uint32_t count) {
    turnlane_count_ = count;
  }

  /**
   * Gets the offset to predicted speeds.
   * @return  Returns the offset (bytes) to predicted speeds.
   */
  uint32_t predictedspeeds_offset() const {
    return predictedspeeds_offset_;
  }

  /**
   * Sets the offset to predicted speed data within the tile.
   * @param offset Offset to predicted speed data within the tile.
   */
  void set_predictedspeeds_offset(const uint32_t offset) {
    predictedspeeds_offset_ = offset;
  }

  /**
   * Get the offset to the end of the tile
   * @return the number of bytes in the tile, unless the last slot is used
   */
  uint32_t end_offset() const {
    return tile_size_;
  }

  /**
   * Sets the offset to the end of the tile
   * @param offset the offset in bytes to the end of the tile
   */
  void set_end_offset(uint32_t offset) {
    tile_size_ = offset;
  }

protected:
  // GraphId (tileid and level) of this tile. Data quality metrics.
  uint64_t graphid_ : 46;
  uint64_t density_ : 4;
  uint64_t name_quality_ : 4;
  uint64_t speed_quality_ : 4;
  uint64_t exit_quality_ : 4;
  uint64_t has_elevation_ : 1;        // Does this tile have elevation data
  uint64_t has_ext_directededge_ : 1; // Does this tile have extended directed edge data

  // TODO: in v4, don't store this its superfluous information, the graphid has all we need
  // Base lon, lat of the tile
  std::pair<float, float> base_ll_;

  // baldr version.
  char version_[kMaxVersionSize];

  // Dataset Id
  uint64_t dataset_id_;

  // Record counts (for fixed size records). Node and directed edge have a max of
  // kMaxGraphId which is 21 bits.
  uint64_t nodecount_ : 21;             // Number of nodes
  uint64_t directededgecount_ : 21;     // Number of directed edges
  uint64_t predictedspeeds_count_ : 21; // Number of predictive speed records
  uint64_t spare1_ : 1;

  // Currently there can only be twice as many transitions as there are nodes,
  // but in practice the number should be much less.
  uint32_t transitioncount_ : 22; // Number of node transitions
  uint32_t spare3_ : 10;          // TODO: DELETE ME IN V4
  uint32_t turnlane_count_ : 21;  // Number of turnlane records
  uint32_t spare4_ : 11;          // TODO: DELETE ME IN V4
  uint64_t transfercount_ : 16;   // Number of transit transfer records
  uint64_t spare2_ : 7;

  // Number of transit records
  uint64_t departurecount_ : 24;
  uint64_t stopcount_ : 16;
  uint64_t spare5_ : 1; // TODO: DELETE ME IN V4
  uint64_t routecount_ : 12;
  uint64_t schedulecount_ : 12;

  // Counts
  uint64_t signcount_ : 24;                // Number of signs
  uint64_t spare6_ : 16;                   // TODO: DELETE ME IN V4
  uint64_t access_restriction_count_ : 24; // Number of access restriction records
  uint64_t admincount_ : 16;               // Number of admin records
  uint64_t spare7_ : 24;                   // TODO: DELETE ME IN V4

  // Note all of the comments about deleting spare in v4
  // There are typos in the bitfield containing spare2_ above
  // They cause the fields to be spread across multiple words
  // The code should have been something like:
  /*
    uint64_t transitioncount_ : 22; // Number of node transitions
    uint64_t turnlane_count_ : 21;  // Number of turnlane records
    uint64_t transfercount_ : 16;   // Number of transit transfer records
    uint64_t spare2_ : 5;
  */
  // This causes the compiler to pad the last word in a platform dependent way
  // On x64 based systems the compiler will pad to 64bits
  // On x86 based systems the compiler will only pad to 32bits
  // This made tiles incompatibilte between x64 and x86 platforms
  // To fix it we insert spare in the right places so that its the same as what
  // the compiler automatically does on an 64bit system but makes 32bit sysems
  // see the layout of the structure that way as well. This way both platforms agree
  // that the x64 implementation, which is what we generally use to build tiles,
  // is the correct implementation

  // Spare 8=byte words that can be used for custom information. As long as the size of
  // the GraphTileHeader structure and order of data within the structure does not change
  // this should be backwards compatible. Make sure use of bits from spareword* does not
  // exceed 128 bits.
  uint64_t spareword0_;
  uint64_t spareword1_;

  // Offsets to beginning of data (for variable size records)
  uint32_t complex_restriction_forward_offset_; // Offset to complex restriction list
  uint32_t complex_restriction_reverse_offset_; // Offset to complex restriction list
  uint32_t edgeinfo_offset_;                    // Offset to edge info
  uint32_t textlist_offset_;                    // Offset to text list

  // Date the tile was created. Days since pivot date.
  uint32_t date_created_;

  // Offsets for each bin of the 5x5 grid (for search/lookup)
  uint32_t bin_offsets_[kBinCount];

  // Offset to beginning of the lane connectivity data
  uint32_t lane_connectivity_offset_;

  // Offset to the beginning of the predicted speed data
  uint32_t predictedspeeds_offset_;

  // GraphTile data size in bytes
  uint32_t tile_size_;

  // Marks the end of this version of the tile with the rest of the slots
  // being available for growth. If you want to use one of the empty slots,
  // simply add a uint32_t some_offset_; just above empty_slots_ and decrease
  // kEmptySlots by 1. Note that you can ONLY add an offset here and NOT a
  // bitfield or union or anything like that
  uint32_t empty_slots_[kEmptySlots];
};

} // namespace baldr
} // namespace valhalla

#endif // VALHALLA_BALDR_GRAPHTILEHEADER_H_
