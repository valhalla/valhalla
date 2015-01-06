#ifndef VALHALLA_MJOLNIR_PBFGRAPHBUILDER_OSMNODE_H
#define VALHALLA_MJOLNIR_PBFGRAPHBUILDER_OSMNODE_H

#include <cstdint>
#include <string>
#include <vector>

#include <valhalla/midgard/pointll.h>
#include <valhalla/baldr/graphid.h>

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

  /**
   * Destructor.
   */
  ~OSMNode();

  /**
   * Sets the lat,lng.
   * @param  ll  Lat,lng of the node.
   */
  void set_latlng(const midgard::PointLL& ll);

  /**
   * Gets the lat,lng.
   * @return   Returns the lat,lng of the node.
   */
  const midgard::PointLL& latlng() const;

  /**
   * Set the graph Id
   * @param  graphid  Graph ID for this node
   */
  void set_graphid(const baldr::GraphId& graphid);

  /**
   * Get the graph Id of this node (after tiling).
   * @return  Returns the graph Id of this node.
   */
  const baldr::GraphId& graphid() const;

  /**
   * Add an edge.
   */
  void AddEdge(const uint32_t edgeindex);

  /**
   * Get the number of edges beginning or ending at the node.
   * @return  Returns the number of edges.
   */
  uint32_t edge_count() const;

  /**
   * Get the list of edges.
   * @return  Returns the list of edge indexes used by the node.
   */
 const std::vector<uint32_t>& edges() const;

 /**
  * Get a mutable list of edge indexes.
  * @return  Returns the list of edge indexes used by the node.
  */
 std::vector<uint32_t>& mutable_edges();

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
  // Lat,lng of the node
  midgard::PointLL latlng_;

  // GraphId of the node (after tiling)
  baldr::GraphId graphid_;

  // List of edges beginning or ending at the node
  std::vector<uint32_t> edges_;

  // Node attributes
  union NodeAttributes {
    struct Fields {
      uint16_t gate       : 1;
      uint16_t bollard    : 1;
      uint16_t exit_to    : 1;
      uint16_t ref        : 1;
      uint16_t modes_mask : 8;
      uint16_t spare      : 4;
    } fields;
    uint16_t v;
  };
  NodeAttributes attributes_;
};

}
}

#endif  // VALHALLA_MJOLNIR_PBFGRAPHBUILDER_OSMNODE_H
