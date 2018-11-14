#ifndef VALHALLA_MJOLNIR_EDGEINFOBUILDER_H_
#define VALHALLA_MJOLNIR_EDGEINFOBUILDER_H_

#include <cstdint>
#include <iostream>
#include <list>
#include <string>
#include <vector>

#include <valhalla/baldr/graphid.h>
#include <valhalla/midgard/pointll.h>
#include <valhalla/midgard/util.h>

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
  void set_wayid(const uint32_t wayid);

  /**
   * Get the mean elevation along the edge.
   * @return  Returns mean elevation in meters relative to sea level.
   */
  float mean_elevation() const {
    return kMinElevation + (w0_.mean_elevation_ * kElevationBinSize);
  }

  /**
   * Set the mean elevation.
   * @param  mean_elev  Mean elevation in meters.
   */
  void set_mean_elevation(const float mean_elev);

  /**
   * Sets the speed limit in KPH.
   * @param  speed_limit  Speed limit in KPH.
   */
  void set_speed_limit(const uint32_t speed_limit);

  /**
   * Get the bike network mask for this directed edge.
   * @return  Returns the bike network mask for this directed edge.
   */
  uint32_t bike_network() const {
    return w0_.bike_network_;
  }

  /**
   * Sets the bike network mask indicating which (if any) bicycle networks are
   * along this edge. See baldr/directededge.h for definitions.
   * @param  bike_network  Bicycle network mask.
   */
  void set_bike_network(const uint32_t bike_network);

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
  template <class shape_container_t> void set_shape(const shape_container_t& shape);

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
  // 1st 8-byte word
  union Word0 {
    struct {
      uint64_t wayid_ : 32;          // OSM way Id
      uint64_t mean_elevation_ : 12; // Mean elevation with 2 meter precision
      uint64_t bike_network_ : 4;    // Mask of bicycle network types (see graphconstants.h)
      uint64_t speed_limit_ : 8;     // Speed limit (kph)
      uint64_t spare0_ : 8;
    };
    uint64_t value_;
  };
  Word0 w0_;

  // List of name info (offsets, etc.)
  std::vector<baldr::NameInfo> name_info_list_;

  // Lat,lng shape of the edge
  std::string encoded_shape_;

  friend std::ostream& operator<<(std::ostream& os, const EdgeInfoBuilder& id);
};

} // namespace mjolnir
} // namespace valhalla

#endif // VALHALLA_MJOLNIR_EDGEINFOBUILDER_H_
