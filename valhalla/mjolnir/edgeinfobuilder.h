#ifndef VALHALLA_MJOLNIR_EDGEINFOBUILDER_H_
#define VALHALLA_MJOLNIR_EDGEINFOBUILDER_H_

#include <vector>
#include <string>
#include <iostream>

#include <valhalla/midgard/pointll.h>
#include <valhalla/midgard/util.h>
#include <valhalla/baldr/graphid.h>

using namespace valhalla::midgard;
using namespace valhalla::baldr;

namespace valhalla {
namespace mjolnir {

/**
 * Edge information not required in shortest path algorithm and is
 * common among the 2 directions.
  */
class EdgeInfoBuilder {
 public:
  /**
   * Set the OSM way Id.
   * @param wayid  Way Id.
   */
  void set_wayid(const uint64_t wayid);

  /**
   * Set the indexes to names used by this edge
   * @param  text_name_offset_list  a list of name indexes.
   */
  void set_text_name_offset_list(const std::vector<uint32_t>& text_name_offset_list);

  /**
   * Set the shape of the edge.
   * @param  shape  the the list of lat,lng points describing the
   * *        shape of the edge.
   */
  void set_shape(const std::vector<PointLL>& shape);

  // Returns the size in bytes of this object.
  std::size_t SizeOf() const;

 protected:

  // OSM Way Id
  uint64_t wayid_;

  // List of roadname indexes
  std::vector<uint32_t> text_name_offset_list_;

  // Lat,lng shape of the edge
  std::string encoded_shape_;

  friend std::ostream& operator<<(std::ostream& os, const EdgeInfoBuilder& id);

};

}
}

#endif  // VALHALLA_MJOLNIR_EDGEINFOBUILDER_H_
