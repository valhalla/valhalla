#ifndef VALHALLA_BALDR_GRAPHTILE_H_
#define VALHALLA_BALDR_GRAPHTILE_H_

#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphtileheader.h>
#include <valhalla/baldr/directededge.h>
#include <valhalla/baldr/nodeinfo.h>
#include <valhalla/baldr/transitcalendar.h>
#include <valhalla/baldr/transitdeparture.h>
#include <valhalla/baldr/transitroute.h>
#include <valhalla/baldr/transitstop.h>
#include <valhalla/baldr/transittransfer.h>
#include <valhalla/baldr/transittrip.h>
#include <valhalla/baldr/sign.h>
#include <valhalla/baldr/edgeinfo.h>
#include <valhalla/baldr/admininfo.h>
#include <valhalla/baldr/tilehierarchy.h>
#include <boost/shared_array.hpp>
#include <memory>
#include "signinfo.h"

namespace valhalla {
namespace baldr {

/**
 * Graph information for a tile within the Tiled Hierarchical Graph.
 */
class GraphTile {
 public:
  /**
   * Constructor
   */
  GraphTile();

  /**
   * Constructor given a GraphId. Reads the graph tile from file
   * into memory.
   * @param  hierarchy  Data describing the tiling and hierarchy system.
   * @param  graphid    GraphId (tileid and level)
   */
  GraphTile(const TileHierarchy& hierarchy, const GraphId& graphid);

  /**
   * Destructor
   */
  virtual ~GraphTile();

  /**
   * Gets the directory like filename suffix given the graphId
   * @param  graphid  Graph Id to construct filename.
   * @param  hierarchy The tile hierarchy structure to get info about how many tiles can exist at this level
   * @return  Returns a filename including directory path as a suffix to be appended to another uri
   */
  static std::string FileSuffix(const GraphId& graphid, const TileHierarchy& hierarchy);

  /**
   * Get the tile Id given the full path to the file.
   * @param  fname  Filename with complete path.
   * @return  Returns the tile Id.
   */
  static GraphId GetTileId(const std::string& fname);

  /**
   * Gets the size of the tile in bytes. A value of 0 indicates an empty tile. A value
   * of 0 indicates an error reading the tile data.
   * or unsuccessful read.
   * @return  Returns the size of the tile in bytes.
   */
  size_t size() const;

  /**
   * Gets the id of the graph tile
   * @return  Returns the graph id of the tile (pointing to the first node)
   */
  GraphId id() const;

  /**
   * Gets a pointer to the graph tile header.
   * @return  Returns the header for the graph tile.
   */
  const GraphTileHeader* header() const;

  /**
   * Get a pointer to a node.
   * @return  Returns a pointer to the node.
   */
  const NodeInfo* node(const GraphId& node) const;

  /**
   * Get a pointer to a node.
   * @param  idx  Index of the node within the current tile.
   * @return  Returns a pointer to the node.
   */
  const NodeInfo* node(const size_t idx) const;

  /**
   * Get a pointer to a edge.
   * @param  edge  GraphId of the directed edge.
   * @return  Returns a pointer to the edge.
   */
  const DirectedEdge* directededge(const GraphId& edge) const;

  /**
   * Get a pointer to a edge.
   * @param  idx  Index of the directed edge within the current tile.
   * @return  Returns a pointer to the edge.
   */
  const DirectedEdge* directededge(const size_t idx) const;

  /**
   * Get a pointer to edge info.
   * @return  Returns edge info.
   */
  std::unique_ptr<const EdgeInfo> edgeinfo(const size_t offset) const;

  /**
   * Convenience method to get the directed edges originating at a node.
   * @param  node_index  Node Id within this tile.
   * @param  count       (OUT) Number of outbound edges
   * @param  edge_index  (OUT) Index of the first outbound edge.
   * @return  Returns a pointer to the first outbound directed edge.
   */
  const DirectedEdge* GetDirectedEdges(const uint32_t node_index,
                                       uint32_t& count, uint32_t& edge_index) const;

  /**
   * Convenience method to get the names for an edge given the offset to the
   * edge information.
   * @param  edgeinfo_offset  Offset to the edge info.
   * @return  Returns a list (vector) of names.
   */
  std::vector<std::string> GetNames(const uint32_t edgeinfo_offset) const;

  /**
   * Get the admininfo at the specified index. Populates the state name and
   * country name from the text/name list.
   * @param  idx  Index into the admin list.
   * @return  Returns the admin information.
   */
  AdminInfo admininfo(const size_t idx) const;

  /**
   * Get the admin at the specified index.
   * @param  idx  Index into the admin list.
   * @return  Returns a pointer to the admin structure.
   */
  const Admin* admin(const size_t idx) const;

  /**
   * Convenience method to get the signs for an edge given the directed
   * edge index.
   * @param  idx  Directed edge index. Used to lookup list of signs.
   * @return  Returns a list (vector) of signs.
   */
  std::vector<SignInfo> GetSigns(const uint32_t idx) const;

  /**
   * Get the next departure given the directed edge Id and the current
   * time (seconds from midnight). TODO - what if crosses midnight?
   * @param   edgeid  Directed edge Id.
   * @param   current_time  Current time (seconds from midnight).
   * @param   date    Date (TODO - comment on standard used).
   * @param   dow     Day of week (TODO - comment on DOW mask)
   * @return  Returns a pointer to the transit departure information.
   *          Returns nullptr if no departures are found.
   */
  const TransitDeparture* GetNextDeparture(const uint32_t edgeid,
              const uint32_t current_time, const uint32_t date,
              const uint32_t dow) const;

  /**
   * Get the transit trip given its trip Id.
   * @param   tripid  Trip Id.
   * @return  Returns a pointer to the transit trip information. Returns
   *          nullptr if the trip is not found.
   */
  const TransitTrip* GetTransitTrip(const uint32_t tripid) const;

  /**
   * Get the transit stop given its stop Id.
   * @param   stopid  Stop Id.
   * @return  Returns a pointer to the transit stop information. Returns
   *          nullptr if the stop is not found.
   */
  const TransitStop* GetTransitStop(const uint32_t stopid) const;

  /**
   * Get the transit route given its route Id.
   * @param   routeid  Route Id.
   * @return  Returns a pointer to the transit route information. Returns
   *          nullptr if the route is not found.
   */
  const TransitRoute* GetTransitRoute(const uint32_t routeid) const;

  /**
   * Get a pointer to the first transfer record given the stop Id and
   * compute the number of transfer records for the stop.
   * @param   stopid  Stop Id.
   * @return  Returns a pair with a pointer to the initial transfer record
   *          and a count of transfer records from the given stop Id.
   */
  std::pair<TransitTransfer*, uint32_t> GetTransfers(
                const uint32_t stopid) const;

  /**
   * Get a pointer to the first calendar exception record given the service
   * Id and compute the number of calendar exception records.
   * @param   serviceid  Service Id.
   * @return  Returns a pair with a pointer to the initial calendar exception
   *          record and a count of calendar exception records for this
   *          service Id.
   */
  std::pair<TransitCalendar*, uint32_t> GetCalendarExceptions(
                const uint32_t serviceid) const;

 protected:

  // Size of the tile in bytes
  size_t size_;

  // Graph tile memory, this must be shared so that we can put it into cache
  // Apparently you can std::move a non-copyable
  boost::shared_array<char> graphtile_;

  // Header information for the tile
  GraphTileHeader* header_;

  // List of nodes. This is a fixed size structure so it can be
  // indexed directly.
  NodeInfo* nodes_;

  // List of directed edges. This is a fixed size structure so it can be
  // indexed directly.
  DirectedEdge* directededges_;

  // Transit departures, many per index (indexed by directed edge index and
  // sorted by departure time)
  TransitDeparture* departures_;

  // Transit trips (indexed by trip Id - unique)
  TransitTrip* transit_trips_;

  // Transit stops (indexed by stop Id - unique)
  TransitStop* transit_stops_;

  // Transit route (indexed by route Id - unique)
  TransitRoute* transit_routes_;

  // Transit transfers, 1 or more per index (indexed by from stop Id)
  TransitTransfer* transit_transfers_;

  // Transit calendar exceptions, 1 or more per index (indexed by service Id)
  TransitCalendar* transit_exceptions_;

  // Signs (indexed by directed edge index)
  Sign* signs_;

  // List of admins. This is a fixed size structure so it can be
  // indexed directly.
  Admin* admins_;

  // List of edge info structures. Since edgeinfo is not fixed size we
  // use offsets in directed edges.
  char* edgeinfo_;

  // Size of the edgeinfo data
  std::size_t edgeinfo_size_;

  // Street names as sets of null-terminated char arrays. Edge info has
  // offsets into this array.
  char* textlist_;

  // Number of bytes in the text/name list
  std::size_t textlist_size_;

};

}
}

#endif  // VALHALLA_BALDR_GRAPHTILE_H_
