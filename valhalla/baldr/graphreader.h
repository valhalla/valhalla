#ifndef VALHALLA_BALDR_GRAPHREADER_H_
#define VALHALLA_BALDR_GRAPHREADER_H_

#include <unordered_map>

#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphtile.h>
#include <valhalla/baldr/tilehierarchy.h>
#include <boost/property_tree/ptree.hpp>

namespace valhalla {
namespace baldr {

/**
 * Class that manages access to GraphTiles. Reads new tiles where necessary
 * and manages a memory cache of active tiles.
 */
class GraphReader {
 public:
  /**
   * Constructor
   *
   * @param ptree  the configuration for the tilehierarchy
   */
  GraphReader(const boost::property_tree::ptree& pt);

  /**
   * Constructor
   *
   * @param tilehierarchy  the tilehierarchy
   */
  GraphReader(const TileHierarchy& th);

  /**
   * Get a pointer to a graph tile object given a GraphId.
   * @param graphid  the graphid of the tile
   * @return GraphTile* a pointer to the graph tile
   */
  const GraphTile* GetGraphTile(const GraphId& graphid);

  /**
   * Get a pointer to a graph tile object given a PointLL and a Level
   * @param pointll  the lat,lng that the tile covers
   * @param level    the hierarchy level to use when getting the tile
   * @return GraphTile* a pointer to the graph tile
   */
  const GraphTile* GetGraphTile(const PointLL& pointll, const uint8_t level);

  /**
   * Get a pointer to a graph tile object given a PointLL and using the highest level in the hierarchy
   * @param pointll  the lat,lng that the tile covers
   * @return GraphTile* a pointer to the graph tile
   */
  const GraphTile* GetGraphTile(const PointLL& pointll);

  /**
   * Get the tile hierarchy used in this graph reader
   * @return hierarchy
   */
  const TileHierarchy& GetTileHierarchy() const;

 protected:
  // Information about where the tiles are kept
  const TileHierarchy tile_hierarchy_;

  // The actual cached GraphTile objects
  std::unordered_map<GraphId, GraphTile> tilecache_;
};

}
}

#endif  // VALHALLA_BALDR_GRAPHREADER_H_
