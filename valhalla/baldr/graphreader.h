#ifndef VALHALLA_BALDR_GRAPHREADER_H_
#define VALHALLA_BALDR_GRAPHREADER_H_

#include <map>

#include "graphid.h"
#include "graphtile.h"

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
   */
  GraphReader(const std::string& datadirectory);

  /**
   * Get a pointer to a graph tile object given a GraphId.
   */
  GraphTile* GetGraphTile(const GraphId& graphid);

 protected:
  // Data directory
  std::string datadir_;

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

  unsigned int GetKey(const unsigned int level, const unsigned int tileid);
};

}
}

#endif  // VALHALLA_BALDR_GRAPHREADER_H_
