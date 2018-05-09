#ifndef VALHALLA_BALDR_EDGEINFO_H_
#define VALHALLA_BALDR_EDGEINFO_H_

#include <cstdint>
#include <iostream>
#include <ostream>
#include <string>
#include <vector>

#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/json.h>
#include <valhalla/midgard/pointll.h>
#include <valhalla/midgard/shape_decoder.h>
#include <valhalla/midgard/util.h>

using namespace valhalla::midgard;

namespace valhalla {
namespace baldr {

constexpr size_t kMaxNamesPerEdge = 15;
constexpr size_t kMaxEncodedShapeSize = 65535;

// Name information. Information about names added to the names list within
// the tile. A name can have a textual representation followed by optional
// fields that provide additional information about the name.
struct NameInfo {
  uint32_t name_offset_ : 24;      // Offset to start of text string
  uint32_t additional_fields_ : 4; // Additional text fields following
                                   // the name. These can be used for
                                   // additional information like language
                                   // phonetic string, etc.
  uint32_t is_ref_ : 1;            // Flag used to indicate if this is a ref
                                   // vs a name.
  uint32_t spare_ : 3;

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
    return wayid_;
  }

  /**
   * Get the number of names.
   * @return Returns the name count.
   */
  uint32_t name_count() const {
    return item_->name_count;
  }

  /**
   * Get the size of the encoded shape (number of bytes).
   * @return  Returns the shape size.
   */
  uint32_t encoded_shape_size() const {
    return item_->encoded_shape_size;
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

  /**
   * Get a list of names and NameInfo for each name of the edge.
   * @return  Returns a vector of string,NameInfo pairs.
   */
  std::vector<std::pair<std::string, NameInfo>> GetNamesAndInfo() const;

  /**
   * Convenience method to get the types for the names.
   * @return   Returns types - If a bit is set, it is a ref.
   */
  uint16_t GetTypes() const;

  /**
   * Get the shape of the edge.
   * @return  Returns the the list of lat,lng points describing the
   *          shape of the edge.
   */
  const std::vector<PointLL>& shape() const;

  midgard::Shape7Decoder<PointLL> lazy_shape() const {
    return midgard::Shape7Decoder<PointLL>(encoded_shape_, item_->encoded_shape_size);
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

  struct PackedItem {
    uint32_t name_count : 4;
    uint32_t encoded_shape_size : 16;
    uint32_t reserved : 5; // Reserved for use by forks of Valhalla
    uint32_t spare : 7;
  };

protected:
  // OSM way Id
  uint64_t wayid_;

  // Where we keep the statistics about how large the vectors below are
  const PackedItem* item_;

  // List of name information (offsets, etc.)
  const NameInfo* name_info_list_;

  // The encoded shape of the edge
  const char* encoded_shape_;

  // Lng, lat shape of the edge
  mutable std::vector<PointLL> shape_;

  // The list of names within the tile
  const char* names_list_;

  // The size of the names list
  size_t names_list_length_;
};

} // namespace baldr
} // namespace valhalla

#endif // VALHALLA_BALDR_EDGEINFO_H_
