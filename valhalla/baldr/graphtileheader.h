#ifndef VALHALLA_BALDR_GRAPHTILEHEADER_H_
#define VALHALLA_BALDR_GRAPHTILEHEADER_H_

#include <cstdlib>
#include <string>
#include <ctime>

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
   * Gets the internal version
   * @return  Returns the internal version of this tile.
   */
  int64_t internal_version() const;

  /**
   * Gets the date when this tile was created.
   * @return  Returns the date this tile was created.
   */
  std::time_t date_created() const;

  /**
   * Gets the version of this tile
   * @return  Returns the  version of this tile.
   */
  std::string version() const;

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

  // internal version info
  int64_t internal_version_;

  // date the tile was created.
  std::time_t date_created_;

  //balr version.
  std::string version_;

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
