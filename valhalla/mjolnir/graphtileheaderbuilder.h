#ifndef VALHALLA_MJOLNIR_GRAPHTILEHEADERBUILDER_H_
#define VALHALLA_MJOLNIR_GRAPHTILEHEADERBUILDER_H_

#include <valhalla/baldr/graphtileheader.h>

namespace valhalla {
namespace mjolnir {

/**
 * Builds the summary information about the graph tile.
 */
class GraphTileHeaderBuilder : public baldr::GraphTileHeader {
 public:
  /**
   * Constructor
   */
  GraphTileHeaderBuilder();

  /**
   * Set the internal version
   * @param  version  Internal version. TODO - describe?
   */
  void set_internal_version(const int64_t version);

  /**
   * Set the date created.
   * @param  date  Unix timestamp. Time in seconds since 1/1/1970.
   */
  void set_date_created(const uint64_t date);

  /**
   * Set the version string.
   * @param  version Version string.
   */
  void set_version(const std::string& version);

  /**
   * Set the graph Id of this tile.
   * @param  graphid  GraphId (tileid and level) of this tile.
   */
  void set_graphid(const baldr::GraphId& graphid);

  /**
   * Sets the number of nodes in this tile.
   * @param  count  Number of nodes within the tile.
   */
  void set_nodecount(const uint32_t count);

  /**
   * Sets the number of directed edges in this tile.
   * @param  count  Number of directed edges within the tile.
   */
  void set_directededgecount(const uint32_t count);

  /**
   * Sets the number of signs within this tile.
   * @param count Number of signs within the tile.
   */
  void set_signcount(const uint32_t count);

  /**
   * Sets the number of simple turn restrictions within this tile.
   * @param count Number of simple turn restrictions within the tile.
   */
  void set_turnrestriction_count(const uint32_t count);

  /**
   * Sets the offset to the edge info.
   * @param offset Offset in bytes to the start of the edge information.
   */
  void set_edgeinfo_offset(const uint32_t offset);

  /**
   * Sets the offset to the name list.
   * @param offset Offset in bytes to the start of the text/name list.
   */
  void set_textlist_offset(const uint32_t offset);

  /**
   * Sets the offset to the administrative information.
   * @param offset Offset in bytes to the start of the  administrative
   *               information.
   */
  void set_admin_offset(const uint32_t offset);

  /**
   * Sets the offset to the list of Multi-Edge Restrictions.
   * @param offset Offset in bytes to the start of the Multi-Edge Restriction
   *               list.
   */
  void set_merlist_offset(const uint32_t offset);

  /**
   * Sets the offset to the list of timed restrictions.
   * @param offset Offset in bytes to the start of the Multi-Edge Restriction
   *               list.
   */
  void set_timedres_offset(const uint32_t offset);

  /**
   * Sets the offset to the list of transit departures / schedule.
   * @param offset Offset in bytes to the start of the transit schedules.
   */
  void set_transit_offset(const uint32_t offset);
};

}
}

#endif  // VALHALLA_MJOLNIR_GRAPHTILEHEADERBUILDER_H_
