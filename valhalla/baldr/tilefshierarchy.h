#ifndef VALHALLA_MIDGARD_TILEFSHIERARCHY_H
#define VALHALLA_MIDGARD_TILEFSHIERARCHY_H

#include <valhalla/baldr/tilehierarchy.h>
#include <valhalla/baldr/graphtilefsstorage.h>

namespace valhalla {
namespace baldr {

/**
 * class used to get information about a given hierarchy of tiles, using GraphTileFsStorage as storage backend.
 */
class TileFsHierarchy : public TileHierarchy {
 public:
  /**
   * Constructor
   * @param tile_dir the tile directory
   */
  TileFsHierarchy(const std::string& tile_dir) : TileHierarchy(CreateTileStorage(tile_dir)) { }

  const std::string& tile_dir() const { return std::static_pointer_cast<GraphTileFsStorage>(tile_storage())->GetTileDir(); }

 private:
  static std::shared_ptr<GraphTileStorage> CreateTileStorage(const std::string& tile_dir) {
    boost::property_tree::ptree pt;
    pt.put("tile_dir", tile_dir);
    return std::make_shared<GraphTileFsStorage>(pt);
  }
};

}
}

#endif  // VALHALLA_MIDGARD_TILEFSHIERARCHY_H
