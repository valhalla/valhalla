#ifndef VALHALLA_MJOLNIR_GRAPHTILEHEADERBUILDER_H_
#define VALHALLA_MJOLNIR_GRAPHTILEHEADERBUILDER_H_

#include "baldr/graphtileheader.h"

namespace valhalla {
namespace mjolnir {

// Summary information about the graph tile.
class GraphTileHeaderBuilder : public baldr::GraphTileHeader {
 public:
  // Constructor
  GraphTileHeaderBuilder();

  // Sets the number of nodes in this tile.
  void set_nodecount(unsigned int nodecount);

  // Sets the number of directed edges in this tile.
  void set_directededgecount(unsigned int directededgecount);

  // Sets the offset to the edge info.
  void set_edgeinfo_offset(unsigned int edgeinfo_offset);

  // Sets the offset to the name list.
  void set_namelist_offset(unsigned int namelist_offset);

};

}
}

#endif  // VALHALLA_MJOLNIR_GRAPHTILEHEADERBUILDER_H_
