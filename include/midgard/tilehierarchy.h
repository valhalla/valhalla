#ifndef VALHALLA_MIDGARD_TILEHIERARCHY_H
#define VALHALLA_MIDGARD_TILEHIERARCHY_H

#include <set>
#include <string>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include "../midgard/tiles.h"

namespace valhalla {
namespace midgard {


/**
 * class used to get information about a given hierarchy of tiles
 */
class TileHierarchy {
 public:
  /**
   * Constructor
   */
  TileHierarchy(const boost::property_tree::ptree& pt);

  /**
   * Encapsulates a few types together to define a level in the hierarchy
   */
  struct TileLevel{
    TileLevel(const unsigned char level, const std::string& name, const Tiles& tiles);
    bool operator<(const TileLevel& other) const;
    unsigned char level;
    std::string name;
    Tiles tiles;
  };

  /**
   * Get the set of levels in this hierarchy
   *
   * @return set of TileLevel objects
   */
  const std::set<TileLevel>& levels() const;

  /**
   * Get the root tile directory where the tiles are stored
   *
   * @return string directory
   */
  const std::string& tile_dir() const;

 private:
  explicit TileHierarchy();

  // a place to keep each level of the hierarchy
  std::set<TileLevel> levels_;
  // the tiles are stored
  std::string tile_dir_;
};

}
}

#endif  // VALHALLA_MIDGARD_TILEHIERARCHY_H
