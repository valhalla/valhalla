#ifndef VALHALLA_MJOLNIR_GRAPHTILEBUILDER_H_
#define VALHALLA_MJOLNIR_GRAPHTILEBUILDER_H_

#include <fstream>
#include <iostream>

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
   * Output the tile to file. Stores as binary data.
   * @param  graphid  GraphID to store.
   * @param  basedirectory  Base data directory
   */
  void StoreTileData(const std::string& basedirectory,
                     const baldr::GraphId& graphid);


  /**
   * Add a node and its outbound edges.
   */
  void AddNodeAndDirectedEdges(
      const NodeInfoBuilder& node,
      const std::vector<DirectedEdgeBuilder>& directededges);

  /**
   * Set the edge info and size.
   */
  void SetEdgeInfoAndSize(const std::vector<EdgeInfoBuilder>& edges,
                          const std::size_t edgeinfo_size);

  /**
   * Set the text list and size.
   */
  void SetTextListAndSize(const std::vector<std::string>& textlist,
                          const std::size_t textlist_size);

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
  std::vector<EdgeInfoBuilder> edgeinfos_builder_;

  // Size of the edgeinfo data
  std::size_t edgeinfo_size_;

  // Names as sets of null-terminated char arrays. Edge info has offsets
  // into this array.
  std::vector<std::string> textlist_builder_;

  // Size of the textlist data
  std::size_t textlist_size_;
};

}
}

#endif  // VALHALLA_MJOLNIR_GRAPHTILEBUILDER_H_

