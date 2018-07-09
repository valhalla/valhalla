#ifndef VALHALLA_BALDR_GRAPHTILE_H_
#define VALHALLA_BALDR_GRAPHTILE_H_

#include <cstdint>
#include <valhalla/baldr/accessrestriction.h>
#include <valhalla/baldr/admininfo.h>
#include <valhalla/baldr/complexrestriction.h>
#include <valhalla/baldr/curler.h>
#include <valhalla/baldr/datetime.h>
#include <valhalla/baldr/directededge.h>
#include <valhalla/baldr/edge_elevation.h>
#include <valhalla/baldr/edgeinfo.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphtileheader.h>
#include <valhalla/baldr/laneconnectivity.h>
#include <valhalla/baldr/nodeinfo.h>
#include <valhalla/baldr/sign.h>
#include <valhalla/baldr/trafficassociation.h>
#include <valhalla/baldr/transitdeparture.h>
#include <valhalla/baldr/transitroute.h>
#include <valhalla/baldr/transitschedule.h>
#include <valhalla/baldr/transitstop.h>
#include <valhalla/baldr/transittransfer.h>
#include <valhalla/baldr/turnlanes.h>

#include <valhalla/midgard/aabb2.h>
#include <valhalla/midgard/util.h>

#include <memory>
#include <valhalla/baldr/signinfo.h>

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
   * @param  tile_dir   Tile directory.
   * @param  graphid    GraphId (tileid and level)
   */
  GraphTile(const std::string& tile_dir, const GraphId& graphid);

  /**
   * Constructor given the graph Id, pointer to the tile data, and the
   * size of the tile data. This is used for memory mapped (mmap) tiles.
   * @param  graphid  Tile Id.
   * @param  ptr      Pointer to the start of the tile's data.
   * @param  size     Size in bytes of the tile data.
   */
  GraphTile(const GraphId& graphid, char* ptr, size_t size);

  /**
   * Constructor given the graph Id, in memory tile data
   * @param  tile_url URL of tile
   * @param  graphid Tile Id
   * @param  curler curler that will handle tile downloading
   */
  GraphTile(const std::string& tile_url, const GraphId& graphid, curler_t& curler);

  /**
   * Destructor
   */
  virtual ~GraphTile();

  /**
   * Gets the directory like filename suffix given the graphId
   * @param  graphid  Graph Id to construct filename.
   * @return  Returns a filename including directory path as a suffix to be appended to another uri
   */
  static std::string FileSuffix(const GraphId& graphid);

  /**
   * Get the tile Id given the full path to the file.
   * @param  fname    Filename with complete path.
   * @return  Returns the tile Id.
   */
  static GraphId GetTileId(const std::string& fname);

  /**
   * Get the bounding box of this graph tile.
   * @return Returns the bounding box of the tile.
   */
  midgard::AABB2<PointLL> BoundingBox() const;

  /**
   * Gets the id of the graph tile
   * @return  Returns the graph id of the tile (pointing to the first node)
   */
  GraphId id() const {
    return header_->graphid();
  }

  /**
   * Gets a pointer to the graph tile header.
   * @return  Returns the header for the graph tile.
   */
  const GraphTileHeader* header() const {
    return header_;
  }

  /**
   * Get a pointer to a node.
   * @return  Returns a pointer to the node.
   */
  const NodeInfo* node(const GraphId& node) const {
    if (node.id() < header_->nodecount()) {
      return &nodes_[node.id()];
    }
    throw std::runtime_error(
        "GraphTile NodeInfo index out of bounds: " + std::to_string(node.tileid()) + "," +
        std::to_string(node.level()) + "," + std::to_string(node.id()) +
        " nodecount= " + std::to_string(header_->nodecount()));
  }

  /**
   * Get a pointer to a node.
   * @param  idx  Index of the node within the current tile.
   * @return  Returns a pointer to the node.
   */
  const NodeInfo* node(const size_t idx) const {
    if (idx < header_->nodecount()) {
      return &nodes_[idx];
    }
    throw std::runtime_error(
        "GraphTile NodeInfo index out of bounds: " + std::to_string(header_->graphid().tileid()) +
        "," + std::to_string(header_->graphid().level()) + "," + std::to_string(idx) +
        " nodecount= " + std::to_string(header_->nodecount()));
  }

  /**
   * Get a pointer to a edge.
   * @param  edge  GraphId of the directed edge.
   * @return  Returns a pointer to the edge.
   */
  const DirectedEdge* directededge(const GraphId& edge) const {
    if (edge.id() < header_->directededgecount()) {
      return &directededges_[edge.id()];
    }
    throw std::runtime_error(
        "GraphTile DirectedEdge index out of bounds: " + std::to_string(header_->graphid().tileid()) +
        "," + std::to_string(header_->graphid().level()) + "," + std::to_string(edge.id()) +
        " directededgecount= " + std::to_string(header_->directededgecount()));
  }

  /**
   * Get a pointer to a edge.
   * @param  idx  Index of the directed edge within the current tile.
   * @return  Returns a pointer to the edge.
   */
  const DirectedEdge* directededge(const size_t idx) const {
    if (idx < header_->directededgecount()) {
      return &directededges_[idx];
    }
    throw std::runtime_error(
        "GraphTile DirectedEdge index out of bounds: " + std::to_string(header_->graphid().tileid()) +
        "," + std::to_string(header_->graphid().level()) + "," + std::to_string(idx) +
        " directededgecount= " + std::to_string(header_->directededgecount()));
  }

  /**
   * Get an iterable set of directed edges from a node in this tile
   * @param  node  GraphId of the node from which the edges leave
   * @return returns an iterable collection of directed edges
   */
  iterable_t<const DirectedEdge> GetDirectedEdges(const GraphId& node) const;

  /**
   * Get an iterable set of directed edges from a node in this tile
   * @param  idx  Index of the node within the current tile
   * @return returns an iterable collection of directed edges
   */
  iterable_t<const DirectedEdge> GetDirectedEdges(const size_t idx) const;

  /**
   * Convenience method to get opposing edge Id given a directed edge.
   * The end node of the directed edge must be in this tile.
   * @param  edge  Directed edge.
   * @return Returns the GraphId of the opposing directed edge.
   */
  GraphId GetOpposingEdgeId(const DirectedEdge* edge) const {
    GraphId endnode = edge->endnode();
    return {endnode.tileid(), endnode.level(), node(endnode.id())->edge_index() + edge->opp_index()};
  }

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
  std::vector<ComplexRestriction*>
  GetRestrictions(const bool forward, const GraphId id, const uint64_t modes) const;

  /**
   * Convenience method to get the directed edges originating at a node.
   * @param  node_index  Node Id within this tile.
   * @param  count       (OUT) Number of outbound edges
   * @param  edge_index  (OUT) Index of the first outbound edge.
   * @return  Returns a pointer to the first outbound directed edge.
   */
  const DirectedEdge*
  GetDirectedEdges(const uint32_t node_index, uint32_t& count, uint32_t& edge_index) const;

  /**
   * Convenience method to get the names for an edge given the offset to the
   * edge information.
   * @param  edgeinfo_offset  Offset to the edge info.
   * @return  Returns a list (vector) of names.
   */
  std::vector<std::string> GetNames(const uint32_t edgeinfo_offset) const;

  /**
   * Convenience method to get the types for the names given the offset to the
   * edge information.
   * @param  edgeinfo_offset  Offset to the edge info.
   * @return  Returns unit16_t.  If a bit is set, then it is a ref.
   */
  uint16_t GetTypes(const uint32_t edgeinfo_offset) const;

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
   * @param   bicycle           Only find departures with bicycle access if true
   * @return  Returns a pointer to the transit departure information.
   *          Returns nullptr if no departures are found.
   */
  const TransitDeparture* GetNextDeparture(const uint32_t lineid,
                                           const uint32_t current_time,
                                           const uint32_t day,
                                           const uint32_t dow,
                                           bool date_before_tile,
                                           bool wheelchair,
                                           bool bicycle) const;

  /**
   * Get the departure given the directed edge Id and tripid
   * @param   lineid  Transit Line Id
   * @param   tripid  Trip Id.
   * @param   current_time      Current time (seconds from midnight).
   * @return  Returns a pointer to the transit departure information.
   *          Returns nullptr if no departure is found.
   */
  const TransitDeparture* GetTransitDeparture(const uint32_t lineid,
                                              const uint32_t tripid,
                                              const uint32_t current_time) const;

  /**
   * Get the departures based on the line Id
   * @return  Returns a map of lineids to departures.
   */
  std::unordered_map<uint32_t, TransitDeparture*> GetTransitDepartures() const;

  /**
   * Get the stop onestop Ids in this tile.
   * @return  Returns a map of transit stops with onestop Ids as the key and
   *          a GraphId (transit tile plus the stop Id) as the value.
   */
  const std::unordered_map<std::string, GraphId>& GetStopOneStops() const;

  /**
   * Get the route onestop Ids in this tile.
   * @return  Returns a map with the route onestop Id as the key and a list
   *          of GraphIds (transit tile plus route line Id) as the value.
   */
  const std::unordered_map<std::string, std::list<GraphId>>& GetRouteOneStops() const;

  /**
   * Get the operator onestops in this tile.
   * @return  Returns a map with onestop Id as the key with a list of GraphIds
   *          as the value.
   */
  const std::unordered_map<std::string, std::list<GraphId>>& GetOperatorOneStops() const;

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
   * @param   edge  GraphId of the directed edge.
   * @return  Returns a list of traffic segment Ids and weights that associate
   *          to this edge.
   */
  std::vector<TrafficSegment> GetTrafficSegments(const GraphId& edge) const;

  /**
   * Get traffic segment(s) associated to this edge.
   * @param   idx  index of the directed edge within the tile.
   * @return  Returns a list of traffic segment Ids and weights that associate
   *          to this edge.
   */
  std::vector<TrafficSegment> GetTrafficSegments(const uint32_t idx) const;

  /**
   * Get lane connections ending on this edge.
   * @param  idx  GraphId of the directed edge.
   * @return  Returns a list of lane connections ending on this edge.
   */
  std::vector<LaneConnectivity> GetLaneConnectivity(const uint32_t idx) const;

  /**
   * Convenience method to get the speed for an edge given the directed
   * edge index.
   * @param  de           Directed edge index. Used to lookup list of signs.
   * @param  current_time Current time (seconds since epoch). A value of 0
   *                      indicates the route is not time dependent.
   * @param  tz_index     timezone index for the node
   * @return  Returns the speed for the edge.
   */
  uint32_t GetSpeed(const DirectedEdge* de,
                    const uint64_t current_time = 0,
                    const uint32_t tz_index = 0) const {
    // if time dependent route and we are routing between 7 AM and 7 PM local time.
    if (current_time && DateTime::is_restricted(false, 7, 0, 19, 0, 0, 0, 0, 0, 0, 0, 0, current_time,
                                                baldr::DateTime::get_tz_db().from_index(tz_index))) {
      return (de->constrained_flow_speed() > 0) ? de->constrained_flow_speed() : de->speed();
    }
    return (de->free_flow_speed() > 0) ? de->free_flow_speed() : de->speed();
  }

  /**
   * Get a pointer to a edge elevation data for the specified edge.
   * @param  edge  GraphId of the directed edge.
   * @return  Returns a pointer to the edge elevation data for the edge.
   *          Returns nullptr if no elevation data exists.
   */
  const EdgeElevation* edge_elevation(const GraphId& edge) const {
    if (header_->has_edge_elevation() && edge.id() < header_->directededgecount()) {
      return &edge_elevation_[edge.id()];
    } else {
      return nullptr;
    }
  }

  /**
   * Convenience method to get the turn lanes for an edge given the directed edge index.
   * @param  idx  Directed edge index. Used to lookup turn lanes.
   * @return  Returns a list (vector) of signs.
   */
  std::vector<uint16_t> turnlanes(const uint32_t idx) const {
    uint32_t offset = turnlanes_offset(idx);
    return (offset > 0) ? TurnLanes::lanemasks(textlist_ + offset) : std::vector<uint16_t>();
  }

  /**
   * Convenience method to get the offset into the text table for the turn lanes
   * for the specified directed edge.
   * @param  idx  Directed edge index. Used to lookup turn lanes.
   * @return  Returns offset into the text table.
   */
  uint32_t turnlanes_offset(const uint32_t idx) const;

protected:
  // Graph tile memory, this must be shared so that we can put it into cache
  std::shared_ptr<std::vector<char>> graphtile_;

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
  TrafficChunk* traffic_chunks_;

  // Number of bytes in the traffic chunk list
  std::size_t traffic_chunk_size_;

  // Lane connectivity data.
  LaneConnectivity* lane_connectivity_;

  // Number of bytes in lane connectivity data.
  std::size_t lane_connectivity_size_;

  // Edge elevation data
  EdgeElevation* edge_elevation_;

  // Turn lanes (indexed by directed edge index)
  TurnLanes* turnlanes_;

  // Map of stop one stops in this tile.
  std::unordered_map<std::string, GraphId> stop_one_stops;

  // Map of route one stops in this tile.
  std::unordered_map<std::string, std::list<GraphId>> route_one_stops;

  // Map of operator one stops in this tile.
  std::unordered_map<std::string, std::list<GraphId>> oper_one_stops;

  /**
   * Set pointers to internal tile data structures.
   * @param  graphid    Graph Id for the tile.
   * @param  tile_ptr   Pointer to the start of the tile.
   * @param  tile_size  Tile size in bytes.
   */
  void Initialize(const GraphId& graphid, char* tile_ptr, const size_t tile_size);

  /**
   * For transit tiles, save off the pair<tileid,lineid> lookup via
   * onestop_ids.  This will be used for including or excluding transit lines
   * for transit routes.  Save 2 maps because operators contain all of their
   * route's tile_line pairs and it is used to include or exclude the operator
   * as a whole. Also associates stops.
   * @param  graphid  Tile Id.
   */
  void AssociateOneStopIds(const GraphId& graphid);
};

} // namespace baldr
} // namespace valhalla

#endif // VALHALLA_BALDR_GRAPHTILE_H_
