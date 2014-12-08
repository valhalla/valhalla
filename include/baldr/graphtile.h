#ifndef VALHALLA_BALDR_GRAPHTILE_H_
#define VALHALLA_BALDR_GRAPHTILE_H_

#include "graphid.h"
#include "graphtileheader.h"
#include "directededge.h"
#include "nodeinfo.h"
#include "edgeinfo.h"

namespace valhalla{
namespace baldr{

/**
 * Graph information for a tile within the Tiled Hierarchical Graph.
 * @author  David W. Nesbitt
 */
class GraphTile {
 public:
  /**
   * Constructor
   */
  GraphTile();

  // TODO - method/constructor to read from a file?

  /**
   * Gets the graph tile header.
   * @return  Returns the header for the graph tile.
   */
  const GraphTileHeader& header() const;

  /**
   * Get a pointer to a node.
   * @return  Returns a pointer to the node.
   */
  const NodeInfo* node(const GraphId& node) const;

  /**
   * Get a pointer to a node.
   * @return  Returns a pointer to the node.
   */
  const DirectedEdge* directededge(const GraphId& edge) const;

  /**
   * Get a pointer to edge info.
   * @return  Returns edge info.
   */
  EdgeInfo* edgeinfo() const;

 protected:
   // Header information for the tile
   GraphTileHeader header_;

   // List of nodes. This is a fixed size structure so it can be
   // indexed directly.
   NodeInfo* nodes_;

   // List of directed edges. This is a fixed size structure so it can be
   // indexed directly.
   DirectedEdge* directededges_;

   // List of edge info structures. Since edgeinfo is not fixed size we
   // use offsets in directed edges.
   void* edgeinfo_;

   // Names as sets of null-terminated char arrays. Edge info has offsets
   // into this array.
   void* namelist_;
};

}
}

#endif  // VALHALLA_BALDR_GRAPHTILE_H_
