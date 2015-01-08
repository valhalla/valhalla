#ifndef VALHALLA_MJOLNIR_GRAPHTILEBUILDER_H_
#define VALHALLA_MJOLNIR_GRAPHTILEBUILDER_H_

#include <fstream>
#include <iostream>
#include <list>

#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphtile.h>
#include <valhalla/mjolnir/graphtileheaderbuilder.h>
#include <valhalla/mjolnir/nodeinfobuilder.h>
#include <valhalla/mjolnir/directededgebuilder.h>
#include <valhalla/mjolnir/edgeinfobuilder.h>

namespace valhalla {
namespace mjolnir {

/**
 * Graph information for a tile within the Tiled Hierarchical Graph.
 * @author  David W. Nesbitt
 */
class GraphTileBuilder : public baldr::GraphTile {
 public:
  /**
   * Constructor
   */
  GraphTileBuilder();

  /**
   * Constructor given an existing tile. This is used to read in the tile
   * data and then add to it (e.g. adding node connections between hierarchy
   * levels.
   * @param  basedir  Base directory path
   * @param  graphid  GraphId used to determine the tileid and level
   */
  GraphTileBuilder(const std::string& basedir, const GraphId& graphid);

  /**
   * Output the tile to file. Stores as binary data.
   * @param  graphid  GraphID to store.
   * @param  basedirectory  Base data directory
   */
  void StoreTileData(const std::string& basedirectory,
                     const baldr::GraphId& graphid);

  /**
   * Update a graph tile with new header, nodes, and directed edges. This
   * is used to add directed edges connecting two hierarchy levels.
   * @param  basedirectory  Base data directory
   * @param  hdr            Update header
   * @param  nodes          Update list of nodes
   * @param  directededges  Updated list of edges.
   */
  void Update(const std::string& basedirectory,
              const GraphTileHeaderBuilder& hdr,
              const std::vector<NodeInfoBuilder>& nodes,
              const std::vector<DirectedEdgeBuilder> directededges);

  /**
   * Add a node and its outbound edges.
   */
  void AddNodeAndDirectedEdges(
      const NodeInfoBuilder& node,
      const std::vector<DirectedEdgeBuilder>& directededges);

  /**
   * Set the edge info and size.
   */
  void SetEdgeInfoAndSize(const std::list<EdgeInfoBuilder>& edges,
                          const std::size_t edgeinfo_size);

  /**
   * Set the text list and size.
   */
  void SetTextListAndSize(const std::list<std::string>& textlist,
                          const std::size_t textlist_size);

  /**
   * Gets a builder for a node from an existing tile.
   * @param  idx  Index of the node within the tile.
   */
  NodeInfoBuilder& node(const size_t idx);

  /**
   * Gets a builder for a directed edge from existing tile data.
   * @param  idx  Index of the directed edge within the tile.
   */
  DirectedEdgeBuilder& directededge(const size_t idx);

 protected:
  // Write all edgeinfo items to specified stream
  void SerializeEdgeInfosToOstream(std::ostream& out);

  // Write all textlist items to specified stream
  void SerializeTextListToOstream(std::ostream& out);

  // Header information for the tile
  GraphTileHeaderBuilder header_builder_;

  // List of nodes. This is a fixed size structure so it can be
  // indexed directly.
  std::vector<NodeInfoBuilder> nodes_builder_;

  // List of directed edges. This is a fixed size structure so it can be
  // indexed directly.
  std::vector<DirectedEdgeBuilder> directededges_builder_;

  // List of edge info structures. Since edgeinfo is not fixed size we
  // use offsets in directed edges.
  std::list<EdgeInfoBuilder> edgeinfos_builder_;

  // Names as sets of null-terminated char arrays. Edge info has offsets
  // into this array.
  std::list<std::string> textlist_builder_;
};

}
}

#endif  // VALHALLA_MJOLNIR_GRAPHTILEBUILDER_H_

