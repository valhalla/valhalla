#ifndef VALHALLA_MJOLNIR_PBFGRAPHBUILDER_OSMNODE_H
#define VALHALLA_MJOLNIR_PBFGRAPHBUILDER_OSMNODE_H

#include <cstdint>
#include <string>
#include <vector>

#include <valhalla/midgard/pointll.h>

namespace valhalla {
namespace mjolnir {

// OSM node
class OSMNode {
 public:
  /**
   * Constructor
   */
  OSMNode();

  /**
   * Constructor given a lat,lng
   */
  OSMNode(const float lat, const float lng);

  ~OSMNode();

  /**
   * Sets the lat,lng.
   */
  void set_latlng(const midgard::PointLL& ll);

  /**
   * Gets the lat,lng.
   */
  midgard::PointLL latlng() const;

  /**
   * Add an edge.
   */
  void AddEdge(const unsigned int edgeindex);

  /**
   * Get the number of outbound edges.
   */
  uint32_t edge_count() const;

  /**
   * Get the list of edges.
   */
  const std::vector<uint32_t>& edges() const;

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
   * Increment uses
   */
  void IncrementUses();

  /**
   * Get the number of uses
   */
  uint32_t uses() const;

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
   * Set modes mask.
   */
  void set_modes_mask(const uint32_t modes_mask);

  /**
   * Get the modes mask.
   */
  bool modes_mask() const;

 private:
  midgard::PointLL latlng_;
  std::vector<uint32_t>* edges_;

  // Node attributes
  union NodeAttributes {
    struct Fields {
      uint32_t uses       : 8;
      uint32_t gate       : 1;
      uint32_t bollard    : 1;
      uint32_t exit_to    : 1;
      uint32_t ref        : 1;
      uint32_t modes_mask : 8;
      uint32_t spare      : 14;
    } fields;
    uint32_t v;
  };
  NodeAttributes attributes_;
};


}
}

#endif  // VALHALLA_MJOLNIR_PBFGRAPHBUILDER_OSMNODE_H
