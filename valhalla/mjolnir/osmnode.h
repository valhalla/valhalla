#ifndef VALHALLA_MJOLNIR_OSMNODE_H
#define VALHALLA_MJOLNIR_OSMNODE_H

#include <cstdint>
#include <string>
#include <vector>

#include <valhalla/baldr/graphconstants.h>

namespace valhalla {
namespace mjolnir {

struct NodeAttributes {
  uint32_t access_mask      : 12;
  uint32_t type             : 4;
  uint32_t exit_to          : 1;
  uint32_t ref              : 1;
  uint32_t name             : 1;
  uint32_t intersection     : 1;
  uint32_t traffic_signal   : 1;
  uint32_t forward_signal   : 1;
  uint32_t backward_signal  : 1;
  uint32_t non_link_edge    : 1;
  uint32_t link_edge        : 1;
  uint32_t shortlink        : 1;  // Link edge < kMaxInternalLength
  uint32_t non_ferry_edge   : 1;
  uint32_t ferry_edge       : 1;
  uint32_t spare            : 4;
};

/**
 * OSM node information. Result of parsing an OSM node.
 */
struct OSMNode {

  // The osm id of the node
  uint64_t osmid;

  // Lat,lng of the node
  float lng, lat;

  // Node attributes. Shared by OSMNode and GraphBuilder Node.
  NodeAttributes attributes_;

  /**
   * Sets the lat,lng.
   * @param  ll  Lat,lng of the node.
   */
  void set_latlng(const std::pair<float, float>& ll);

  /**
   * Gets the lat,lng.
   * @return   Returns the lat,lng of the node.
   */
  std::pair<float, float> latlng() const;

  /**
   * Set access mask.
   */
  void set_access_mask(const uint32_t access_mask);

  /**
   * Get the access mask.
   */
  uint32_t access_mask() const;

  /**
    * Set payment mask.
    */
   // void set_payment_mask(const uint32_t payment_mask);

   /**
    * Get the payment mask.
    */
   // uint32_t payment_mask() const;

  /**
   * Sets the type.
   * @param  type
   */
  void set_type(const baldr::NodeType type);

  /**
   * Get the type.
   * @return  Returns the type of node.
   */
  baldr::NodeType type() const;

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
   * Set forward_signal flag.
   */
  void set_forward_signal(const bool forward_signal);

  /**
   * Get the forward_signal flag.
   */
  bool forward_signal() const;

  /**
   * Set backward_signal flag.
   */
  void set_backward_signal(const bool backward_signal);

  /**
   * Get the backward_signal flag.
   */
  bool backward_signal() const;

  /**
   * Get the attributes.
   * @return  Returns the attributes.
   */
  const NodeAttributes& attributes() const;
};

}
}

#endif  // VALHALLA_MJOLNIR_OSMNODE_H
