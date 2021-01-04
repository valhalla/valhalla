#ifndef VALHALLA_BALDR_EDGEINFO_H_
#define VALHALLA_BALDR_EDGEINFO_H_

#include <cstdint>
#include <iostream>
#include <ostream>
#include <string>
#include <vector>

#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/json.h>
#include <valhalla/midgard/encoded.h>
#include <valhalla/midgard/pointll.h>
#include <valhalla/midgard/util.h>

namespace valhalla {
namespace baldr {

constexpr size_t kMaxNamesPerEdge = 15;
constexpr size_t kMaxEncodedShapeSize = 65535;

// Use elevation bins of 2 meters to store mean elevation. Clamp to a range
// from -500 meters to 7683 meters.
constexpr uint32_t kMaxStoredElevation = 4095; // 12 bits
constexpr float kElevationBinSize = 2.0f;
constexpr float kMinElevation = -500.0f;
constexpr float kMaxElevation = kMinElevation + (kElevationBinSize * kMaxStoredElevation);

// Name information. Information about names added to the names list within
// the tile. A name can have a textual representation followed by optional
// fields that provide additional information about the name.
struct NameInfo {
  uint32_t name_offset_ : 24;      // Offset to start of text string
  uint32_t additional_fields_ : 4; // Additional text fields following
                                   // the name. These can be used for
                                   // additional information like language
                                   // phonetic string, etc.
  uint32_t is_route_num_ : 1;      // Flag used to indicate if this is a route number
                                   // vs just a name.
  uint32_t tagged_ : 1;            // Future use - this indicates the text string is
                                   // specially tagged (for example uses the first char as
                                   // the tag type). To make this forward and backward
                                   // compatible, tagged text will not be read in GetNames
                                   // and GetNamesAndTags until code is ready to actually use it.
  uint32_t spare_ : 2;

  bool operator==(const NameInfo& other) const {
    return (name_offset_ == other.name_offset_);
  }

  // operator < for sorting
  bool operator<(const NameInfo& other) const {
    return (name_offset_ < other.name_offset_);
  }
};

/**
 * Edge information not required in shortest path algorithm and is
 * common among the 2 directions.
 */
class EdgeInfo {
public:
  EdgeInfo() = delete;
  EdgeInfo(const EdgeInfo& other) = delete;
  EdgeInfo& operator=(const EdgeInfo&) = delete;
  EdgeInfo(EdgeInfo&&) = default;
  EdgeInfo& operator=(EdgeInfo&&) = default;

  /**
   * Constructor
   * @param  ptr  Pointer to a bit of memory that has the info for this edge
   * @param  names_list  Pointer to the start of the text/names list.
   * @param  names_list_length  Length (bytes) of the text/names list.
   */
  EdgeInfo(char* ptr, const char* names_list, const size_t names_list_length);

  /**
   * Destructor
   */
  virtual ~EdgeInfo();

  /**
   * Gets the OSM way Id.
   * @return  Returns the OSM way Id.
   */
  uint64_t wayid() const {
    return (static_cast<uint64_t>(extended_wayid3_) << 56) |
           (static_cast<uint64_t>(extended_wayid2_) << 48) |
           (static_cast<uint64_t>(ei_.extended_wayid1_) << 40) |
           (static_cast<uint64_t>(ei_.extended_wayid0_) << 32) | static_cast<uint64_t>(ei_.wayid_);
  }

  /**
   * Get the mean elevation along the edge.
   * @return  Returns mean elevation in meters relative to sea level.
   */
  float mean_elevation() const {
    return kMinElevation + (ei_.mean_elevation_ * kElevationBinSize);
  }

  /**
   * Get the bike network mask for this directed edge.
   * @return  Returns the bike network mask for this directed edge.
   */
  uint32_t bike_network() const {
    return ei_.bike_network_;
  }

  /**
   * Gets the speed limit in KPH.
   * @return  Returns the speed limit in KPH.
   */
  uint32_t speed_limit() const {
    return ei_.speed_limit_;
  }

  /**
   * Get the number of names.
   * @return Returns the name count.
   */
  uint32_t name_count() const {
    return ei_.name_count_;
  }

  /**
   * Get the size of the encoded shape (number of bytes).
   * @return  Returns the shape size.
   */
  uint32_t encoded_shape_size() const {
    return ei_.encoded_shape_size_;
  }

  /**
   * Get the name info for the specified name index.
   * @param  index  Index into the name list.
   * @return  Returns the name info.
   */
  NameInfo GetNameInfo(uint8_t index) const;

  /**
   * Convenience method to get the names for an edge
   * @param  only_tagged_names  Bool indicating whether or not to return only the tagged names
   *
   * @return   Returns a list (vector) of names.
   */
  std::vector<std::string> GetNames(bool only_tagged_names = false) const;

  /**
   * Convenience method to get the names and route number flags for an edge.
   * @param  include_tagged_names  Bool indicating whether or not to return the tagged names too
   *
   * @return   Returns a list (vector) of name/route number pairs.
   */
  std::vector<std::pair<std::string, bool>> GetNamesAndTypes(bool include_tagged_names = false) const;

  /**
   * Convenience method to get the names and the tagged type for an edge.
   * @return   Returns a list (vector) of name/tagged type pairs.
   */
  std::vector<std::pair<std::string, uint8_t>> GetTaggedNamesAndTypes() const;

  /**
   * Convenience method to get the types for the names.
   * @return   Returns types - If a bit is set, it is a route number.
   */
  uint16_t GetTypes() const;

  /**
   * Get the shape of the edge.
   * @return  Returns the the list of lat,lng points describing the
   *          shape of the edge.
   */
  const std::vector<midgard::PointLL>& shape() const;

  midgard::Shape7Decoder<midgard::PointLL> lazy_shape() const {
    return midgard::Shape7Decoder<midgard::PointLL>(encoded_shape_, ei_.encoded_shape_size_);
  }

  /**
   * Returns the encoded shape string.
   * @return  Returns the encoded shape string.
   */
  std::string encoded_shape() const;

  /**
   * Returns json representing this object
   * @return json object
   */
  json::MapPtr json() const;

  // Operator EqualTo based on nodea and nodeb.
  bool operator==(const EdgeInfo& rhs) const;

  // Fixed size data within EdgeInfo
  struct EdgeInfoInner {
    uint32_t wayid_ : 32; // OSM way Id

    uint32_t mean_elevation_ : 12; // Mean elevation with 2 meter precision
    uint32_t bike_network_ : 4;    // Mask of bicycle network types (see graphconstants.h)
    uint32_t speed_limit_ : 8;     // Speed limit (kph)
    uint32_t extended_wayid0_ : 8; // Next byte of the way id

    uint32_t name_count_ : 4;          // How many name infos we expect
    uint32_t encoded_shape_size_ : 16; // How many bytes long the encoded shape is
    uint32_t extended_wayid1_ : 8;     // Next next byte of the way id
    uint32_t extended_wayid_size_ : 2; // How many more bytes the way id is stored in
    uint32_t spare0_ : 2;              // not used
  };

protected:
  // Fixed size information
  EdgeInfoInner ei_;

  // List of name information (offsets, etc.)
  const NameInfo* name_info_list_;

  // The encoded shape of the edge
  const char* encoded_shape_;

  // Where we optionally keep the last 2 bytes of a 64bit wayid
  uint8_t extended_wayid2_;
  uint8_t extended_wayid3_;

  // Lng, lat shape of the edge
  mutable std::vector<midgard::PointLL> shape_;

  // The list of names within the tile
  const char* names_list_;

  // The size of the names list
  size_t names_list_length_;
};

} // namespace baldr
} // namespace valhalla

#endif // VALHALLA_BALDR_EDGEINFO_H_
