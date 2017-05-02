#ifndef VALHALLA_MJOLNIR_EDGEINFOBUILDER_H_
#define VALHALLA_MJOLNIR_EDGEINFOBUILDER_H_

#include <cstdint>
#include <vector>
#include <list>
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
 * Edge information. Not required in shortest path algorithm and is
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
   * Set the name info for names used by this edge
   * @param  offsets  List of street name info.
   */
  void set_name_info_list(const std::vector<baldr::NameInfo>& name_info);

  /**
   * Add name info to the list.
   * @param  info  Adds name information to the list.
   */
  void AddNameInfo(const baldr::NameInfo& info);

  /**
   * Set the shape of the edge.
   * @param  shape  List of lat,lng points describing the
   *                shape of the edge.
   */
  template <class shape_container_t>
  void set_shape(const shape_container_t& shape);

  /**
   * Set the encoded shape string.
   * @param  encoded_shape  Encoded shape string
   */
  void set_encoded_shape(const std::string& encoded_shape);

  /**
   * Get the size of this edge info (without padding).
   * @return  Returns the size in bytes of this object.
   */
  std::size_t BaseSizeOf() const;

  /**
   * Get the size of this edge info. Includes padding to align to
   * 8-byte boundaries.
   * @return  Returns the size in bytes of this object.
   */
  std::size_t SizeOf() const;

 protected:

  // OSM Way Id
  uint64_t wayid_;

  // List of name info (offsets, etc.)
  std::vector<baldr::NameInfo> name_info_list_;

  // Lat,lng shape of the edge
  std::string encoded_shape_;

  friend std::ostream& operator<<(std::ostream& os, const EdgeInfoBuilder& id);
};

}
}

#endif  // VALHALLA_MJOLNIR_EDGEINFOBUILDER_H_
