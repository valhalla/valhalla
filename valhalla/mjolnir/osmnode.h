#ifndef VALHALLA_MJOLNIR_OSMNODE_H
#define VALHALLA_MJOLNIR_OSMNODE_H

#include <cstdint>
#include <string>
#include <vector>

namespace valhalla {
namespace mjolnir {

// Internal storage of lat,lng
using OSMLatLng = std::pair<float, float>;

/**
 * OSM node information. Result of parsing an OSM node.
 */
class OSMNode {
 public:
  /**
   * Constructor
   */
  OSMNode();

  /**
   * Constructor given a lng,lat
   */
  OSMNode(const float lng, const float lat);

  /**
   * Destructor.
   */
  ~OSMNode();

  /**
   * Sets the lat,lng.
   * @param  ll  Lat,lng of the node.
   */
  void set_latlng(const OSMLatLng& ll);

  /**
   * Gets the lat,lng.
   * @return   Returns the lat,lng of the node.
   */
  const OSMLatLng& latlng() const;

  /**
   * Set modes mask.
   */
  void set_modes_mask(const uint32_t modes_mask);

  /**
   * Get the modes mask.
   */
  uint32_t modes_mask() const;

  /**
   * Set gate flag.
   */
  void set_gate(const bool gate);

  /**
   * Get the gate flag.
   */
  bool gate() const;

  /**
   * Set bollard flag.
   */
  void set_bollard(const bool bollard);

  /**
   * Get the bollard flag.
   */
  bool bollard() const;

  /**
   * Set the exit to flag
   */
  void set_exit_to(const bool exit_to);

  /**
   * Get the exit to flag
   */
  bool exit_to() const;

  /**
   * Set the ref flag
   */
  void set_ref(const bool ref);

  /**
   * Get the ref flag
   */
  bool ref() const;

  /**
   * Set the name flag (for some exit information)
   */
  void set_name(const bool name);

  /**
   * Get the name flag
   */
  bool name() const;

  /**
   * Set the intersection flag.
   * @param  intersection  Is this node part of an intersection? True if
   *                       this node is an end node of more than 1 way.
   */
  void set_intersection(const bool intersection);

  /**
   * Get the intersection flag
   * @return  Returns true if the node is part of an intersection (end
   *          node of more than 1 way), false if not.
   */
  bool intersection() const;

  /**
   * Set traffic_signal flag.
   */
  void set_traffic_signal(const bool traffic_signal);

  /**
   * Get the traffic_signal flag.
   */
  bool traffic_signal() const;

  /**
   * Get the attributes value.
   * @return  Returns the attributes word.
   */
  uint32_t attributes() const;

 protected:
  // Lat,lng of the node
  OSMLatLng latlng_;

  // Node attributes
  union NodeAttributes {
    struct Fields {
      uint32_t modes_mask     : 8;
      uint32_t gate           : 1;
      uint32_t bollard        : 1;
      uint32_t exit_to        : 1;
      uint32_t ref            : 1;
      uint32_t name           : 1;
      uint32_t intersection   : 1;
      uint32_t traffic_signal : 1;
      uint32_t non_link_edge  : 1;   // Used in derived Node class.
      uint32_t link_edge      : 1;   // Used in derived Node class.
      uint32_t spare          : 15;
    } fields;
    uint32_t v;
  };
  NodeAttributes attributes_;
};

}
}

#endif  // VALHALLA_MJOLNIR_OSMNODE_H
