#ifndef VALHALLA_BALDR_EDGEINFO_H_
#define VALHALLA_BALDR_EDGEINFO_H_

#include <cstdint>
#include <map>
#include <string>
#include <tuple>
#include <vector>

#include <valhalla/baldr/conditional_speed_limit.h>
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
  uint32_t tagged_ : 1;            // This indicates the text string is
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

// indexes of linguistic attributes in the linguistic map value-tuple
constexpr size_t kLinguisticMapTupleLanguageIndex = 0;
constexpr size_t kLinguisticMapTuplePhoneticAlphabetIndex = 1;
constexpr size_t kLinguisticMapTuplePronunciationIndex = 2;
constexpr size_t kLinguisticHeaderSize = 3;

// Unfortunately a bug was found where we were returning a blank phoneme (kNone = 0) for a linguistic
// record where it just contained a language and no phoneme.  This caused us to stop reading the
// header and in turn caused the name_index to be off.  This is why kNone is now equal to 5
struct linguistic_text_header_t {
  uint32_t language_ : 8; // this is just the language as we will derive locale by getting admin info
  uint32_t length_ : 8;   // pronunciation length
  uint32_t phonetic_alphabet_ : 3;
  uint32_t name_index_ : 4; // what name is this pronunciation for
  uint32_t spare_ : 1;
  uint32_t DO_NOT_USE_ : 8; // DONT EVER USE THIS WE DON'T ACTUALLY STORE IT IN THE TEXT LIST
};

/**
 * Decode the level information encoded as variable length, variable precision numbers.
 *
 * The first varint denotes the string size, to avoid the value 0 from being interpreted
 * as a null character. The second varint denotes the precision to apply to all values
 * except for the sentinel value used as a separator of continuous ranges.
 *
 *
 * The returned array includes level values and sentinel values. Ex.:
 *  "12;14" translates to {12, 1048575, 14}
 *  "0-12;14" translates to {0, 12, 1048575, 14}
 *
 * @param encoded the encoded varint array
 *
 * @return a tuple that contains
 *   a) the decoded levels array and
 *   b) the precision used
 */
std::pair<std::vector<std::pair<float, float>>, uint32_t> decode_levels(const std::string& encoded);

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
   * Does this EdgeInfo have elevation data.
   * @return Returns true if the EdgeInfo record has elevation along the edge.
   */
  bool has_elevation() const {
    return ei_.has_elevation_;
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
   * @return   Returns a list (vector) of names.
   */
  std::vector<std::string> GetNames() const;

  /** Convenience method to get the names and route number flags for an edge.
   *
   *  This one does not calculate the types
   *  Like GetNamesAndTypes but without using memory for the types
   *
   * @param  include_tagged_values  Bool indicating whether or not to return the tagged values too
   * @return Returns a list (vector) (name, route number flag) pairs
   */
  std::vector<std::pair<std::string, bool>> GetNames(bool include_tagged_values) const;

  /**
   * Convenience method to get the non linguistic, tagged values for an edge.
   *
   *
   * @return   Returns a list (vector) of tagged names.
   */
  std::vector<std::string> GetTaggedValues() const;

  /**
   * Convenience method to get the linguistic names for an edge
   * @param  type  type of linguistic names we are interested in obtaining.
   *
   * @return   Returns a list (vector) of linguistic names.
   */
  std::vector<std::string> GetLinguisticTaggedValues() const;

  /**
   * Convenience method to get the names, route number flags and tag value type for an edge.
   * @param  include_tagged_values  Bool indicating whether or not to return the tagged values too
   *
   * @return   Returns a list (vector) of name/route number flags/types tuples.
   */
  std::vector<std::tuple<std::string, bool, uint8_t>>
  GetNamesAndTypes(bool include_tagged_names = false) const;

  /**
   * Convenience method to get tags of the edge.
   * Key incidates tag, value may contain arbitrary blob of data.
   * @return   Returns a map of tags
   */
  const std::multimap<TaggedValue, std::string>& GetTags() const;

  /**
   * Convenience method to get a Linguistic map for an edge.
   * @return   Returns a unordered_map in which the key is a index into the name list from
   * GetNamesAndTypes and the tuple contains a pronunciation (w/wo a language) or no pronunciation and
   * just a language
   */
  std::unordered_map<uint8_t, std::tuple<uint8_t, uint8_t, std::string>> GetLinguisticMap() const;

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
   * Returns the encoded elevation along the edge. The sampling interval is uniform
   * (based on the length of the edge). The sampling interval is returned via argument.
   * @param  length  Length of the edge. Used to determine sampling interval.
   * @param  interval  Sampling interval (reference - value is returned).
   * @return Returns a vector holding delta encoded elevation along the edge.
   */
  std::vector<int8_t> encoded_elevation(const uint32_t length, double& interval) const;

  /**
   * Returns the list of conditional speed limits for the edge.
   * @return Conditional speed limits for the edge.
   */
  std::vector<ConditionalSpeedLimit> conditional_speed_limits() const;

  /**
   * Get layer index of the edge relatively to other edges(Z-level). Can be negative.
   * @see https://wiki.openstreetmap.org/wiki/Key:layer
   * @return layer index of the edge
   */
  int8_t layer() const;

  /**
   * Get levels of the edge.
   * @see https://wiki.openstreetmap.org/wiki/Key:level
   * @return a pair where the first member is a vector of contiguous level ranges (inclusive) and
   * the the second member is the max precision found on any of the level tokens.
   */
  std::pair<std::vector<std::pair<float, float>>, uint32_t> levels() const;

  /**
   * Convenience method that checks whether the edge connects the passed level.
   */
  bool includes_level(float lvl) const;

  /**
   * Get layer:ref of the edge.
   * @see https://wiki.openstreetmap.org/wiki/Key:level:ref
   * @return layer index of the edge
   */
  std::vector<std::string> level_ref() const;

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
    uint32_t has_elevation_ : 1;       // Does the edgeinfo have elevation?
    uint32_t spare0_ : 1;              // not used
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

  // Encoded elevation
  const int8_t* encoded_elevation_;

  // The list of names within the tile
  const char* names_list_;

  // The size of the names list
  size_t names_list_length_;

  // for fast access to tag values stored in names list
  mutable std::multimap<TaggedValue, std::string> tag_cache_;
  mutable bool tag_cache_ready_ = false;
};

} // namespace baldr
} // namespace valhalla

#endif // VALHALLA_BALDR_EDGEINFO_H_
