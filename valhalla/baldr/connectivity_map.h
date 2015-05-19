#ifndef VALHALLA_BALDR_CONNECTIVITY_MAP_H_
#define VALHALLA_BALDR_CONNECTIVITY_MAP_H_

#include "baldr/tilehierarchy.h"

#include <vector>
#include <unordered_map>
#include <cstdint>

namespace valhalla {
  namespace baldr {
    //TODO: maintain consistent coloring of regions despite the connectivity changing
    class connectivity_map_t {
     public:
      /**
       * Constructus the connectivity map
       *
       */
      connectivity_map_t(const TileHierarchy& tile_hierarchy);

      /**
       * Returns the color for the given graphid
       *
       * @param id      the graphid
       * @return color  the color
       */
      size_t get_color(const GraphId& id) const;

      /**
       * Returns the geojson representing the connectivity map
       *
       * @param hierarchy_level the hierarchy level whos connectivity you want to see
       * @return string         the geojson
       */
      std::string to_geojson(const uint32_t hierarchy_level) const;

      /**
       * Returns the vector of colors (one per tile) representing the connectivity map
       *
       * @param hierarchy_level the hierarchy level whos connectivity you want to see
       * @return vector         the vector of colors per tile
       */
      std::vector<size_t> to_image(const uint32_t hierarchy_level) const;

     private:
      std::unordered_map<uint32_t, std::unordered_map<uint32_t, size_t> > colors;
      TileHierarchy tile_hierarchy;
    };
  }
}

#endif //VALHALLA_BALDR_CONNECTIVITY_MAP_H_
