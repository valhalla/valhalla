#ifndef VALHALLA_BALDR_GRAPHREADER_H_
#define VALHALLA_BALDR_GRAPHREADER_H_

#include <map>

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
  GraphTile* GetGraphTile(const GraphId& graphid);

  /**
   * Get a pointer to a graph tile object given a PointLL and a Level
   * @param pointll  the lat,lng that the tile covers
   * @param level    the hierarchy level to use when getting the tile
   * @return GraphTile* a pointer to the graph tile
   */
  GraphTile* GetGraphTile(const PointLL& pointll, const unsigned char level);

 protected:
  // Information about where the tiles are kept
  TileHierarchy tile_hierarchy_;

  /**
   * Get a tile object from cache. Checks if the tile given by the tileid and level
   * from the graphid is already in the cache.
   * @param   graphid   Graph Id.
   * @return  Returns a pointer to the GraphTile if it is already in the
   *          cache. Returns nullptr if not in the cache.
   */
  GraphTile* GetTileFromCache(const GraphId& graphid);

  /**
   * Read a tile object from disk and adds it to the cache.
   * @param   graphid   Graph Id.
   * @return  Returns a pointer to the GraphTile. Returns nullptr if the tile
   *          is not available.
   */
  GraphTile* ReadTile(const GraphId& graphid);

 private:
  std::map<unsigned int, GraphTile*> tilecache_;


  unsigned int GetKey(const unsigned int level, const unsigned int tileid) const;
};

}
}

#endif  // VALHALLA_BALDR_GRAPHREADER_H_
