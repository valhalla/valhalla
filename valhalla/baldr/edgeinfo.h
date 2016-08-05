#ifndef VALHALLA_BALDR_EDGEINFO_H_
#define VALHALLA_BALDR_EDGEINFO_H_

#include <vector>
#include <string>
#include <ostream>
#include <iostream>

#include <valhalla/midgard/pointll.h>
#include <valhalla/midgard/util.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/json.h>

using namespace valhalla::midgard;

namespace valhalla {
namespace baldr {

constexpr size_t kMaxNamesPerEdge = 15;
constexpr size_t kMaxEncodedShapeSize = 65535;

/**
 * Edge information not required in shortest path algorithm and is
 * common among the 2 directions.
 */
class EdgeInfo {
 public:
  EdgeInfo() = delete;
  EdgeInfo(const EdgeInfo& other) = delete;

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
  uint64_t wayid() const;

  /**
   * Get the number of names.
   * @return Returns the name count.
   */
  uint32_t name_count() const;

  /**
   * Get the size of the encoded shape (number of bytes).
   * @return  Returns the shape size.
   */
  uint32_t encoded_shape_size() const;

  /**
   * Get the name offset for the specified name index.
   * @param  index  Index into the name list.
   * @return  Returns the offset into the text/name list.
   */
  uint32_t GetStreetNameOffset(uint8_t index) const;

  /**
   * Convenience method to get the names for an edge
   * @return   Returns a list (vector) of names.
   */
  std::vector<std::string> GetNames() const;

  /**
   * Get the shape of the edge.
   * @return  Returns the the list of lat,lng points describing the
   *          shape of the edge.
   */
  const std::vector<PointLL>& shape() const;

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
  bool operator ==(const EdgeInfo& rhs) const;

  struct PackedItem {
    uint32_t name_count          :4;
    uint32_t encoded_shape_size  :16;
    uint32_t spare               :12;
  };

 protected:
  // OSM way Id
  uint64_t wayid_;

  // Where we keep the statistics about how large the vectors below are
  PackedItem* item_;

  // List of roadname indexes
  uint32_t* street_name_offset_list_;

  // The encoded shape of the edge
  mutable char* encoded_shape_;

  // Lng, lat shape of the edge
  mutable std::vector<PointLL> shape_;

  // The list of names within the tile
  const char* names_list_;

  // The size of the names list
  const size_t names_list_length_;

};

}
}

#endif  // VALHALLA_BALDR_EDGEINFO_H_
