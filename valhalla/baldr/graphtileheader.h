#ifndef VALHALLA_BALDR_GRAPHTILEHEADER_H_
#define VALHALLA_BALDR_GRAPHTILEHEADER_H_

#include <cstdlib>

namespace valhalla{
namespace baldr{

/**
 * Summary information about the graph tile.
 * @author  David W. Nesbitt
 */
class GraphTileHeader {
 public:
  /**
   * Constructor
   */
  GraphTileHeader();

  /**
   * Gets the number of nodes in this tile.
   * @return  Returns the number of nodes.
   */
  size_t nodecount() const;

  /**
   * Gets the number of directed edges in this tile.
   * @return  Returns the number of directed edges.
   */
 size_t directededgecount() const;

  /**
   * Gets the offset to the edge info.
   * @return  Returns the number of bytes to offset to the edge information.
   */
  size_t edgeinfo_offset() const;

  /**
   * Gets the offset to the text list.
   * @return  Returns the number of bytes to offset to the text list.
   */
  size_t textlist_offset() const;

 protected:
  // TODO - need to add some sort of versioning / creation date?

  // Number of nodes
  size_t nodecount_;

  // Number of directed edges
  size_t directededgecount_;

  // Offset to edge info
  size_t edgeinfo_offset_;

  // Offset to name list
  size_t textlist_offset_;
};

}
}

#endif  // VALHALLA_BALDR_GRAPHTILEHEADER_H_
