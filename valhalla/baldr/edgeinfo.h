#ifndef VALHALLA_BALDR_EDGEINFO_H_
#define VALHALLA_BALDR_EDGEINFO_H_

#include <vector>
#include <string>
#include <ostream>
#include <iostream>

#include <valhalla/midgard/pointll.h>
#include <valhalla/midgard/util.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/exitsign.h>

using namespace valhalla::midgard;

namespace valhalla {
namespace baldr {

constexpr size_t kMaxEncodedShapeSize = 16383;

/**
 * Edge information not required in shortest path algorithm and is
 * common among the 2 directions.
 * @author  David W. Nesbitt
 */
class EdgeInfo {
 public:
  /**
   * Constructor
   *
   * @param pointer to a bit of memory that has the info for this edge
   */
  EdgeInfo(char* ptr);

  /**
   * Destructor
   *
   */
  virtual ~EdgeInfo();

  // Returns the name count
  const uint32_t name_count() const;

  // Returns the shape count
  const uint32_t encoded_shape_size() const;

  // Returns the exit sign count
  const uint32_t exit_sign_count() const;

  // Returns the name index at the specified index.
  const uint32_t GetStreetNameOffset(uint8_t index) const;

  /**
   * Get the shape of the edge.
   * @return  Returns the the list of lat,lng points describing the
   * *        shape of the edge.
   */
  const std::vector<PointLL>& shape() const;

  // Operator EqualTo based on nodea and nodeb.
  bool operator ==(const EdgeInfo& rhs) const;

  // Packed items: counts for names, shape, exit signs
  union PackedItem {
    struct Fields {

      uint32_t name_count                     :4;
      uint32_t encoded_shape_size             :14;
      uint32_t exit_sign_count                :4;
      uint32_t spare                          :10;
    } fields;
    uint32_t value;
  };


 protected:
  /**
   * Constructor
   */
  EdgeInfo();

 private:

  // Where we keep the statistics about how large the vectors below are
  PackedItem* item_;

  // List of roadname indexes
  uint32_t* street_name_offset_list_;

  // The encoded shape of the edge
  mutable char* encoded_shape_;

  // List of exit signs (type and index)
  ExitSign* exit_signs_;

  // Lng, lat shape of the edge
  mutable std::vector<PointLL> shape_;

};

}
}

#endif  // VALHALLA_BALDR_EDGEINFO_H_
