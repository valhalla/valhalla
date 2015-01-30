#ifndef VALHALLA_BALDR_GRAPHTILE_H_
#define VALHALLA_BALDR_GRAPHTILE_H_

#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphtileheader.h>
#include <valhalla/baldr/directededge.h>
#include <valhalla/baldr/nodeinfo.h>
#include <valhalla/baldr/edgeinfo.h>
#include <valhalla/baldr/tilehierarchy.h>
#include <valhalla/baldr/exitsigninfo.h>

#include <boost/shared_array.hpp>
#include <memory>

namespace valhalla {
namespace baldr {

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

  /**
   * Constructor given a GraphId. Reads the graph tile from file
   * into memory.
   */
  GraphTile(const TileHierarchy& hierarchy, const GraphId& graphid);

  /**
   * Destructor
   */
  virtual ~GraphTile();

  /**
   * Gets the directory like filename suffix given the graphId
   * @param  graphid  Graph Id to construct filename.
   * @param  hierarchy The tile hierarchy structure to get info about how many tiles can exist at this level
   * @return  Returns a filename including directory path as a suffix to be appended to another uri
   */
  static std::string FileSuffix(const GraphId& graphid, const TileHierarchy& hierarchy);

  /**
   * Gets the size of the tile in bytes. A value of 0 indicates an empty tile. A value
   * of 0 indicates an error reading the tile data.
   * or unsuccessful read.
   *
   * @return size   the size of the tile in bytes
   */
  size_t size() const;

  /**
   * Gets the id of the *graph tile
   *
   * @return id     the graph id of the tile (pointing to the first node)
   */
  GraphId id() const;

  /**
   * Gets a pointer to the graph tile header.
   * @return  Returns the header for the graph tile.
   */
  const GraphTileHeader* header() const;

  /**
   * Get a pointer to a node.
   * @return  Returns a pointer to the node.
   */
  const NodeInfo* node(const GraphId& node) const;

  /**
   * Get a pointer to a node.
   * @param  idx  Index of the node within the current tile.
   * @return  Returns a pointer to the node.
   */
  const NodeInfo* node(const size_t idx) const;

  /**
   * Get a pointer to a edge.
   * @param  edge  GraphId of the directed edge.
   * @return  Returns a pointer to the edge.
   */
  const DirectedEdge* directededge(const GraphId& edge) const;

  /**
   * Get a pointer to a edge.
   * @param  idx  Index of the directed edge within the current tile.
   * @return  Returns a pointer to the edge.
   */
  const DirectedEdge* directededge(const size_t idx) const;

  /**
   * Get a pointer to edge info.
   * @return  Returns edge info.
   */
  std::unique_ptr<const EdgeInfo> edgeinfo(const size_t offset) const;

  /**
   * Convenience method to get the directed edges originating at a node.
   * @param  node_index  Node Id within this tile.
   * @param  count       (OUT) Number of outbound edges
   * @param  edge_index  (OUT) Index of the first outbound edge.
   * @return  Returns a pointer to the first outbound directed edge.
   */
  const DirectedEdge* GetDirectedEdges(const uint32_t node_index,
                                       uint32_t& count, uint32_t& edge_index) const;

  /**
   * Convenience method to get the names for an edge given the offset to the
   * edge information.
   * @param  edgeinfo_offset  Offset to the edge info.
   * @return  Returns a list (vector) of names.
   */
  std::vector<std::string> GetNames(const uint32_t edgeinfo_offset) const;

  /**
   * Convenience method to get the exit signs for an edge given the offset to the
   * edge information.
   * @param  edgeinfo_offset  Offset to the edge info.
   * @return  Returns a list (vector) of exit signs.
   */
  std::vector<ExitSignInfo> GetExitSigns(const uint32_t edgeinfo_offset) const;

 protected:
  // Size of the tile in bytes
  size_t size_;

  // Graph tile memory, this must be shared so that we can put it into cache
  // Aparently you can std::move a non-copyable
  boost::shared_array<char> graphtile_;

  // Header information for the tile
  GraphTileHeader* header_;

  // List of nodes. This is a fixed size structure so it can be
  // indexed directly.
  NodeInfo* nodes_;

  // List of directed edges. This is a fixed size structure so it can be
  // indexed directly.
  DirectedEdge* directededges_;

  // List of edge info structures. Since edgeinfo is not fixed size we
  // use offsets in directed edges.
  char* edgeinfo_;

  // Size of the edgeinfo data
  std::size_t edgeinfo_size_;

  // Street names and exit names/numbers as sets of null-terminated char arrays.
  // Edge info has offsets into this array.
  char* textlist_;

  // Number of bytes in the text/name list
  std::size_t textlist_size_;

  // The id of the tile for convenience
  const GraphId id_;
};

}
}

#endif  // VALHALLA_BALDR_GRAPHTILE_H_
