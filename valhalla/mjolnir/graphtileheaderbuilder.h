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
   * Set the relative road density within this tile.
   * @param  density  Relative road density within this tile (0-15).
   */
  void set_density(const uint32_t density);

  /**
   * Set the relative quality of name assignment for this tile.
   * @param   name_quality  Relative name quality for this tile (0-15).
   */
  void set_name_quality(const uint32_t name_quality);

  /**
   * Set the relative quality of speed assignment for this tile.
   * @param  speed_quality   Relative speed quality for this tile (0-15).
   */
  void set_speed_quality(const uint32_t speed_quality);

  /**
   * Set the relative quality of exit signs for this tile.
   * @param  exit_quality   Relative exit sign quality for this tile (0-15).
   */
  void set_exit_quality(const uint32_t exit_quality);

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
   * Sets the number of transit departures in this tile.
   * @param departures  The number of transit departures.
   */
  void set_departurecount(const uint32_t departures);

  /**
   * Sets the number of transit stops in this tile.
   * @param  stops  The number of transit stops.
   */
  void set_stopcount(const uint32_t stops);

  /**
   * Sets the number of transit routes in this tile.
   * @param  routes  The number of transit routes.
   */
  void set_routecount(const uint32_t routes);

  /**
   * Sets the number of transit transfers in this tile.
   * @param  transfers   The number of transit transfers.
   */
  void set_transfercount(const uint32_t transfers);

  /**
   * Gets the number of transit calendar exceptions in this tile.
   * @param  calendars  The number of transit calendar exceptions.
   */
  void set_calendarcount(const uint32_t calendars);

  /**
   * Sets the number of admins within this tile.
   * @param count Number of admins within the tile.
   */
  void set_admincount(const uint32_t count);

  /**
   * Sets the offset to the edge info.
   * @param offset Offset in bytes to the start of the edge information.
   */
  void set_edgeinfo_offset(const uint32_t offset);

  /**
   * Sets the offset to the text list.
   * @param offset Offset in bytes to the start of the text list.
   */
  void set_textlist_offset(const uint32_t offset);

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
};

}
}

#endif  // VALHALLA_MJOLNIR_GRAPHTILEHEADERBUILDER_H_
