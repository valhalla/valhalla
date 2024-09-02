#ifndef VALHALLA_MJOLNIR_EDGEINFOBUILDER_H_
#define VALHALLA_MJOLNIR_EDGEINFOBUILDER_H_

#include <cstdint>
#include <string>
#include <vector>

#include <valhalla/baldr/edgeinfo.h>
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
  void set_wayid(const uint64_t wayid);

  /**
   * Get the mean elevation along the edge.
   * @return  Returns mean elevation in meters relative to sea level.
   */
  float mean_elevation() const {
    return kMinElevation + (ei_.mean_elevation_ * kElevationBinSize);
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
   * Sets the elevation flag.
   * @param  elevation  Flag indicating whether the edge has elevation data.
   */
  void set_has_elevation(const bool elevation);

  /**
   * Get the bike network mask for this directed edge.
   * @return  Returns the bike network mask for this directed edge.
   */
  uint32_t bike_network() const {
    return ei_.bike_network_;
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
   * Set encoded elevation.
   * @param  encoded_elevation  Encoded elevation
   */
  void set_encoded_elevation(const std::vector<int8_t>& encoded_elevation);

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

  /**
   * Check if the given name_info exists in this edge info.
   * @param  offset  The offset of the name info.
   * @return  Returns true if the name info exists, false otherwise.
   */
  bool has_name_info(uint32_t offset) const {
    for (const auto& name : name_info_list_) {
      if (name.name_offset_ == offset) {
        return true;
      }
    }
    return false;
  }

protected:
  // Fixed size information
  baldr::EdgeInfo::EdgeInfoInner ei_{};

  // Where we optionally keep the last 2 bytes of a 64bit wayid
  uint8_t extended_wayid2_;
  uint8_t extended_wayid3_;

  // List of name info (offsets, etc.)
  std::vector<baldr::NameInfo> name_info_list_;

  // Lat,lng shape of the edge
  std::string encoded_shape_;

  // Encoded elevation
  std::vector<int8_t> encoded_elevation_;

  friend std::ostream& operator<<(std::ostream& os, const EdgeInfoBuilder& id);
};

} // namespace mjolnir
} // namespace valhalla

#endif // VALHALLA_MJOLNIR_EDGEINFOBUILDER_H_
