#ifndef VALHALLA_BALDR_GRAPHTILE_H_
#define VALHALLA_BALDR_GRAPHTILE_H_

#include <valhalla/baldr/accessrestriction.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphtileheader.h>
#include <valhalla/baldr/complexrestriction.h>
#include <valhalla/baldr/directededge.h>
#include <valhalla/baldr/nodeinfo.h>
#include <valhalla/baldr/trafficassociation.h>
#include <valhalla/baldr/transitdeparture.h>
#include <valhalla/baldr/transitroute.h>
#include <valhalla/baldr/transitstop.h>
#include <valhalla/baldr/transitschedule.h>
#include <valhalla/baldr/transittransfer.h>
#include <valhalla/baldr/sign.h>
#include <valhalla/baldr/edgeinfo.h>
#include <valhalla/baldr/admininfo.h>
#include <valhalla/baldr/tilehierarchy.h>

#include <valhalla/midgard/util.h>

#include <boost/shared_array.hpp>
#include <memory>
#include "signinfo.h"

namespace valhalla {
namespace baldr {

using tile_index_pair = std::pair<uint32_t, uint32_t>;

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
   * Constructor given the graph Id ... used for mmap
   */
  GraphTile(const GraphId& graphid, char* ptr, size_t size);

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
   * @param  fname    Filename with complete path.
   * @param  tile_dir Base tile directory.
   * @return  Returns the tile Id.
   */
  static GraphId GetTileId(const std::string& fname);

  /**
   * Get the bounding box of this graph tile.
   * @param  hierarchy the tile hierarchy this tile is under.
   * @return Returns the bounding box of the tile.
   */
  midgard::AABB2<PointLL> BoundingBox(const TileHierarchy& hierarchy) const;

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
   * Convenience method to get opposing edge Id given a directed edge.
   * The end node of the directed edge must be in this tile.
   * @param  edge  Directed edge.
   * @return Returns the GraphId of hte opposing directed edge.
   */
  GraphId GetOpposingEdgeId(const DirectedEdge* edge) const;

  /**
   * Get a pointer to edge info.
   * @return  Returns edge info.
   */
  EdgeInfo edgeinfo(const size_t offset) const;

  /**
   * Get the complex restrictions in the forward or reverse order.
   * @param   forward - do we want the restrictions in reverse order?
   * @param   id - edge id
   * @param   modes - access modes
   * @return  Returns the vector of complex restrictions in the order requested
   *          based on the id and modes.
   */
  std::vector<ComplexRestriction> GetRestrictions(const bool forward,
                                                  const GraphId id,
                                                  const uint64_t modes) const;

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
   * Convenience method to get the text/name for a given offset to the textlist
   * @param   textlist_offset  offset into the text list.
   * @return  Returns the desired string
   */
  std::string GetName(const uint32_t textlist_offset) const;

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
   * @param   lineid            Transit Line Id
   * @param   current_time      Current time (seconds from midnight).
   * @param   day               Days since the tile creation date.
   * @param   dow               Day of week (see graphconstants.h)
   * @param   date_before_tile  Is the date that was inputed before
   *                            the tile creation date?
   * @param   wheelchair        Only find departures with wheelchair access if true
   * @param   bicyle            Only find departures with bicycle access if true
   * @return  Returns a pointer to the transit departure information.
   *          Returns nullptr if no departures are found.
   */
  const TransitDeparture* GetNextDeparture(const uint32_t lineid,
                                           const uint32_t current_time,
                                           const uint32_t day,
                                           const uint32_t dow,
                                           bool  date_before_tile,
                                           bool wheelchair,
                                           bool bicycle) const;

  /**
   * Get the departure given the directed edge Id and tripid
   * @param   lineid  Transit Line Id
   * @param   tripid  Trip Id.
   * @return  Returns a pointer to the transit departure information.
   *          Returns nullptr if no departure is found.
   */
  const TransitDeparture* GetTransitDeparture(const uint32_t lineid,
                                              const uint32_t tripid) const;

  /**
   * Get the departures based on the line Id
   * @return  Returns a map of lineids to departures.
   */
  std::unordered_map<uint32_t,TransitDeparture*> GetTransitDepartures() const;

  /**
   * Get the stop onestops in this tile
   * @return  Returns a map of onestops
   */
  std::unordered_map<std::string, tile_index_pair>
    GetStopOneStops() const;

  /**
   * Get the route onestops in this tile
   * @return  Returns a map of onestops
   */
  std::unordered_map<std::string, std::list<tile_index_pair>>
    GetRouteOneStops() const;

  /**
   * Get the operator onestops in this tile
   * @return  Returns a map of onestops
   */
  std::unordered_map<std::string, std::list<tile_index_pair>>
    GetOperatorOneStops() const;

  /**
   * Get the transit stop given its index
   * @param   idx  stop index.
   * @return  Returns a pointer to the transit stop information.
   */
  const TransitStop* GetTransitStop(const uint32_t idx) const;

  /**
   * Get the transit route given its route Id.
   * @param   idx     Route index within the tile.
   * @return  Returns a pointer to the transit route information. Returns
   *          nullptr if the route is not found.
   */
  const TransitRoute* GetTransitRoute(const uint32_t idx) const;

  /**
   * Get the transit schedule given its schedule index.
   * @param   idx     Schedule index within the tile.
   * @return  Returns a pointer to the transit schedule information. Returns
   *          nullptr if the schedule is not found.
   */
  const TransitSchedule* GetTransitSchedule(const uint32_t idx) const;

  /**
   * Convenience method to get the access restrictions for an edge given the
   * edge Id.
   * @param   edgeid  Directed edge Id.
   * @param   access  Access.  Used to obtain the restrictions for the access
   *                   that we are interested in (see graphconstants.h)
   * @return  Returns a list (vector) of AccessRestrictions.
   */
  std::vector<AccessRestriction> GetAccessRestrictions(const uint32_t edgeid,
                                                       const uint32_t access) const;

  /**
   * Get an iteratable list of GraphIds given a bin in the tile
   * @param  column the bin's column
   * @param  row the bin's row
   * @return iterable container of graphids contained in the bin
   */
  midgard::iterable_t<GraphId> GetBin(size_t column, size_t row) const;

  /**
   * Get an iteratable list of GraphIds given a bin in the tile
   * @param  index the bin's index in the row major array
   * @return iterable container of graphids contained in the bin
   */
  midgard::iterable_t<GraphId> GetBin(size_t index) const;

  /**
   * Get traffic segment(s) associated to this edge.
   * @param  edge  GraphId of the directed edge.
   * @return  Returns a list of traffic segment Ids and weights that associate
   *          to this edge.
   */
  std::vector<std::pair<TrafficAssociation, float>> GetTrafficSegments(const GraphId& edge) const;

  /**
   * Get traffic segment(s) associated to this edge.
   * @param  edge  GraphId of the directed edge.
   * @return  Returns a list of traffic segment Ids and weights that associate
   *          to this edge.
   */
  std::vector<std::pair<TrafficAssociation, float>> GetTrafficSegments(const size_t idx) const;


 protected:

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

  // Transit stops (indexed by stop index within the tile)
  TransitStop* transit_stops_;

  // Transit route (indexed by route index within the tile)
  TransitRoute* transit_routes_;

  // Transit schedules (index by schedule index within the tile)
  TransitSchedule* transit_schedules_;

  // Transit transfer records.
  TransitTransfer* transit_transfers_;

  // Access restrictions, 1 or more per edge id
  AccessRestriction* access_restrictions_;

  // Signs (indexed by directed edge index)
  Sign* signs_;

  // List of admins. This is a fixed size structure so it can be
  // indexed directly.
  Admin* admins_;

  // List of complex_restrictions in the forward direction.
  char* complex_restriction_forward_;

  // Size of the complex restrictions in the forward direction
  std::size_t complex_restriction_forward_size_;

  // List of complex_restrictions in the reverse direction.
  char* complex_restriction_reverse_;

  // Size of the complex restrictions in the reverse direction
  std::size_t complex_restriction_reverse_size_;

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

  // List of edge graph ids. The list is broken up in bins which have
  // indices in the tile header.
  GraphId* edge_bins_;

  // Traffic segment association. Count is the same as the directed edge count.
  TrafficAssociation* traffic_segments_;

  // Traffic chunks. Chunks are an array of uint64_t which combines a traffic
  // segment Id (GraphId) and weight (combined int a single uint64_t).
  uint64_t* traffic_chunks_;

  // Number of bytes in the traffic chunk list
  std::size_t traffic_chunk_size_;

  // Map of stop one stops in this tile.
  std::unordered_map<std::string, tile_index_pair> stop_one_stops;

  // Map of route one stops in this tile.
  std::unordered_map<std::string, std::list<tile_index_pair>> route_one_stops;

  // Map of operator one stops in this tile.
  std::unordered_map<std::string, std::list<tile_index_pair>> oper_one_stops;

  /**
   * Set pointers to internal tile data structures.
   * @param  graphid    Graph Id for the tile.
   * @param  tile_ptr   Pointer to the start of the tile.
   * @param  tile_size  Tile size in bytes.
   */
  void Initialize(const GraphId& graphid, char* tile_ptr,
                  const size_t tile_size);

  void AssociateOneStopIds(const GraphId& graphid);
};

}
}

#endif  // VALHALLA_BALDR_GRAPHTILE_H_
