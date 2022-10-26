#pragma once

#include <valhalla/baldr/accessrestriction.h>
#include <valhalla/baldr/admininfo.h>
#include <valhalla/baldr/complexrestriction.h>
#include <valhalla/baldr/curler.h>
#include <valhalla/baldr/datetime.h>
#include <valhalla/baldr/directededge.h>
#include <valhalla/baldr/edgeinfo.h>
#include <valhalla/baldr/graphconstants.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphmemory.h>
#include <valhalla/baldr/graphtileheader.h>
#include <valhalla/baldr/graphtileptr.h>
#include <valhalla/baldr/laneconnectivity.h>
#include <valhalla/baldr/nodeinfo.h>
#include <valhalla/baldr/nodetransition.h>
#include <valhalla/baldr/predictedspeeds.h>
#include <valhalla/baldr/sign.h>
#include <valhalla/baldr/signinfo.h>
#include <valhalla/baldr/traffictile.h>
#include <valhalla/baldr/transitdeparture.h>
#include <valhalla/baldr/transitroute.h>
#include <valhalla/baldr/transitschedule.h>
#include <valhalla/baldr/transitstop.h>
#include <valhalla/baldr/transittransfer.h>
#include <valhalla/baldr/turnlanes.h>

#include <valhalla/midgard/aabb2.h>
#include <valhalla/midgard/logging.h>
#include <valhalla/midgard/util.h>

#include <valhalla/filesystem.h>

#include <cstdint>
#include <iterator>
#include <memory>

namespace valhalla {
namespace baldr {

const std::string SUFFIX_NON_COMPRESSED = ".gph";
const std::string SUFFIX_COMPRESSED = ".gph.gz";

class tile_getter_t;
/**
 * Graph information for a tile within the Tiled Hierarchical Graph.
 */
#ifndef ENABLE_THREAD_SAFE_TILE_REF_COUNT
class GraphTile : public boost::intrusive_ref_counter<GraphTile, boost::thread_unsafe_counter> {
#else
class GraphTile {
#endif // ENABLE_THREAD_SAFE_TILE_REF_COUNT
public:
  static const constexpr char* kTilePathPattern = "{tilePath}";

  /**
   * Constructs with a given GraphId. Reads the graph tile from file
   * into memory.
   * @param  tile_dir   Tile directory.
   * @param  graphid    GraphId (tileid and level)
   * @return nullptr if the tile could not be loaded. may throw
   */
  static graph_tile_ptr Create(const std::string& tile_dir,
                               const GraphId& graphid,
                               std::unique_ptr<const GraphMemory>&& traffic_memory = nullptr);

  /**
   * Constructs with a given the graph Id, pointer to the tile data, and the
   * size of the tile data. This is used for memory mapped (mmap) tiles.
   * @param  graphid  Tile Id.
   * @param  ptr      Pointer to the start of the tile's data.
   */
  static graph_tile_ptr Create(const GraphId& graphid, std::vector<char>&& memory);

  /**
   * Constructs given the graph Id, pointer to the tile data, and the
   * size of the tile data. This is used for memory mapped (mmap) tiles.
   * @param  graphid  Tile Id.
   * @param  ptr      Pointer to the start of the tile's data.
   * @param  size     Size in bytes of the tile data.
   */
  static graph_tile_ptr Create(const GraphId& graphid,
                               std::unique_ptr<const GraphMemory>&& memory,
                               std::unique_ptr<const GraphMemory>&& traffic_memory = nullptr);

  /**
   * Constructs a tile given a url for the tile using curl
   * @param  tile_url URL of tile
   * @param  graphid Tile Id
   * @param  tile_getter object that will handle tile downloading
   * @return whether or not the tile could be cached to disk
   */

  static graph_tile_ptr CacheTileURL(const std::string& tile_url,
                                     const GraphId& graphid,
                                     tile_getter_t* tile_getter,
                                     const std::string& cache_location);

  /**
   * Construct a tile given a url for the tile using curl
   * @param  tile_data graph tile raw bytes
   * @param  disk_location tile filesystem path
   */
  static void SaveTileToFile(const std::vector<char>& tile_data, const std::string& disk_location);

  /**
   * Destructor
   */
  virtual ~GraphTile();

  /**
   * Gets the directory like filename suffix given the graphId
   * @param  graphid      Graph Id to construct filename.
   * @param  gzipped      Modifies the suffix if you expect gzipped file names
   * @param  is_file_path Determines the 1000 separator to be used for file or URL access
   * @param  tiles        Allows passing a custom tile definition rather than pulling from static
   *                      hierarchy, which is useful for testing
   * @return  Returns a filename including directory path as a suffix to be appended to another uri
   */
  static std::string FileSuffix(const GraphId& graphid,
                                const std::string& suffix = valhalla::baldr::SUFFIX_NON_COMPRESSED,
                                bool is_file_path = true,
                                const TileLevel* tiles = nullptr);

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
  midgard::AABB2<midgard::PointLL> BoundingBox() const;

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
    assert(node.Tile_Base() == header_->graphid().Tile_Base());
    if (node.id() < header_->nodecount()) {
      return &nodes_[node.id()];
    }
    throw std::runtime_error(
        std::string(__FILE__) + ":" + std::to_string(__LINE__) +
        " GraphTile NodeInfo index out of bounds: " + std::to_string(node.tileid()) + "," +
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
        std::string(__FILE__) + ":" + std::to_string(__LINE__) +
        " GraphTile NodeInfo index out of bounds: " + std::to_string(header_->graphid().tileid()) +
        "," + std::to_string(header_->graphid().level()) + "," + std::to_string(idx) +
        " nodecount= " + std::to_string(header_->nodecount()));
  }

  /**
   * Convenience method to get the lat,lon of a node.
   * @param  nodeid  GraphId of the node.
   */
  midgard::PointLL get_node_ll(const GraphId& nodeid) const {
    assert(nodeid.Tile_Base() == header_->graphid().Tile_Base());
    return node(nodeid)->latlng(header()->base_ll());
  }

  /**
   * Get a pointer to a edge.
   * @param  edge  GraphId of the directed edge.
   * @return  Returns a pointer to the edge.
   */
  const DirectedEdge* directededge(const GraphId& edge) const {
    assert(edge.Tile_Base() == header_->graphid().Tile_Base());
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
   * Get a pointer to an edge extension .
   * @param  edge  GraphId of the directed edge.
   * @return  Returns a pointer to the edge extension.
   */
  const DirectedEdgeExt* ext_directededge(const GraphId& edge) const {
    assert(edge.Tile_Base() == header_->graphid().Tile_Base());

    // Testing against directededgecount since the number of directed edges
    // should be the same as the number of directed edge extensions
    if (edge.id() < header_->directededgecount()) {
      return &ext_directededges_[edge.id()];
    }
    throw std::runtime_error("GraphTile DirectedEdgeExt index out of bounds: " +
                             std::to_string(header_->graphid().tileid()) + "," +
                             std::to_string(header_->graphid().level()) + "," +
                             std::to_string(edge.id()) +
                             " directededgecount= " + std::to_string(header_->directededgecount()));
  }

  /**
   * Get a pointer to an edge extension.
   * @param  idx  Index of the directed edge within the current tile.
   * @return  Returns a pointer to the edge.
   */
  const DirectedEdgeExt* ext_directededge(const size_t idx) const {
    // Testing against directededgecount since the number of directed edges
    // should be the same as the number of directed edge extensions
    if (idx < header_->directededgecount()) {
      return &ext_directededges_[idx];
    }
    throw std::runtime_error("GraphTile DirectedEdgeExt index out of bounds: " +
                             std::to_string(header_->graphid().tileid()) + "," +
                             std::to_string(header_->graphid().level()) + "," + std::to_string(idx) +
                             " directededgecount= " + std::to_string(header_->directededgecount()));
  }

  /**
   * Get an iterable set of directed edges from a node in this tile
   * @param  node  Node from which the edges leave
   * @return returns an iterable collection of directed edges
   */
  midgard::iterable_t<const DirectedEdge> GetDirectedEdges(const NodeInfo* node) const;

  /**
   * Get an iterable set of directed edges from a node in this tile
   * @param  node  GraphId of the node from which the edges leave
   * @return returns an iterable collection of directed edges
   */
  midgard::iterable_t<const DirectedEdge> GetDirectedEdges(const GraphId& node) const;

  /**
   * Get an iterable set of directed edges from a node in this tile
   * WARNING: this only returns edges in this tile, edges at this node on another level
   *          will not be returned by this method, node transitions must be used
   *
   * @param  idx  Index of the node within the current tile
   * @return returns an iterable collection of directed edges
   */
  midgard::iterable_t<const DirectedEdge> GetDirectedEdges(const size_t idx) const;

  /**
   * Get an iterable set of directed edges extensions from a node in this tile
   * @param  node  Node from which the edges leave
   * @return returns an iterable collection of directed edges extensions
   */
  midgard::iterable_t<const DirectedEdgeExt> GetDirectedEdgeExts(const NodeInfo* node) const;

  /**
   * Get an iterable set of directed edges extensions from a node in this tile
   * @param  node  GraphId of the node from which the edges leave
   * @return returns an iterable collection of directed edges extensions
   */
  midgard::iterable_t<const DirectedEdgeExt> GetDirectedEdgeExts(const GraphId& node) const;

  /**
   * Get an iterable set of directed edges extensions from a node in this tile
   * WARNING: this only returns edge extensions in this tile, edges at this node on another level
   *          will not be returned by this method, node transitions must be used
   *
   * @param  idx  Index of the node within the current tile
   * @return returns an iterable collection of directed edges extensions
   */
  midgard::iterable_t<const DirectedEdgeExt> GetDirectedEdgeExts(const size_t idx) const;

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
   * Get a pointer to a node transition.
   * @param  idx  Index of the directed edge within the current tile.
   * @return  Returns a pointer to the edge.
   */
  const NodeTransition* transition(const uint32_t idx) const {
    if (idx < header_->transitioncount())
      return &transitions_[idx];
    throw std::runtime_error("GraphTile NodeTransition index out of bounds: " +
                             std::to_string(header_->graphid().tileid()) + "," +
                             std::to_string(header_->graphid().level()) + "," + std::to_string(idx) +
                             " transitioncount= " + std::to_string(header_->transitioncount()));
  }

  /**
   * Get an iterable set of transitions from a node in this tile
   * @param  node  Node from which the transitions leave
   * @return returns an iterable collection of node transitions
   */
  midgard::iterable_t<const NodeTransition> GetNodeTransitions(const NodeInfo* node) const {
    if (node < nodes_ || node >= nodes_ + header_->nodecount()) {
      throw std::logic_error(
          std::string(__FILE__) + ":" + std::to_string(__LINE__) +
          " GraphTile NodeInfo out of bounds: " + std::to_string(header_->graphid()));
    }
    const auto* trans = transitions_ + node->transition_index();
    return midgard::iterable_t<const NodeTransition>{trans, node->transition_count()};
  }

  /**
   * Get an iterable set of transitions from a node in this tile
   * @param  node  GraphId of the node from which the transitions leave
   * @return returns an iterable collection of node transitions
   */
  midgard::iterable_t<const NodeTransition> GetNodeTransitions(const GraphId& node) const {
    if (node.id() >= header_->nodecount()) {
      throw std::logic_error(
          std::string(__FILE__) + ":" + std::to_string(__LINE__) +
          " GraphTile NodeInfo index out of bounds: " + std::to_string(node.tileid()) + "," +
          std::to_string(node.level()) + "," + std::to_string(node.id()) +
          " nodecount= " + std::to_string(header_->nodecount()));
    }
    const auto* nodeinfo = nodes_ + node.id();
    return GetNodeTransitions(nodeinfo);
  }

  /**
   * Get an iterable set of nodes in this tile
   * @return returns an iterable collection of nodes
   */
  midgard::iterable_t<const NodeInfo> GetNodes() const {
    return midgard::iterable_t<const NodeInfo>{nodes_, header_->nodecount()};
  }

  /**
   * Get an iterable set of edges in this tile
   * @return returns an iterable collection of edges
   */
  midgard::iterable_t<const DirectedEdge> GetDirectedEdges() const {
    return midgard::iterable_t<const DirectedEdge>{directededges_, header_->directededgecount()};
  }

  /**
   * Get an iterable set of edge extensions in this tile
   * @return returns an iterable collection of edge extensions
   */
  midgard::iterable_t<const DirectedEdgeExt> GetDirectedEdgeExts() const {
    return midgard::iterable_t<const DirectedEdgeExt>{ext_directededges_,
                                                      header_->directededgecount()};
  }

  /**
   * Get a pointer to edge info.
   * @return  Returns edge info.
   */
  EdgeInfo edgeinfo(const DirectedEdge* edge) const;

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
   * Convenience method to get the directed edge extensions originating at a node.
   * @param  node_index  Node Id within this tile.
   * @param  count       (OUT) Number of outbound edges
   * @param  edge_index  (OUT) Index of the first outbound edge.
   * @return  Returns a pointer to the first outbound directed edge extension.
   */
  const DirectedEdgeExt*
  GetDirectedEdgeExts(const uint32_t node_index, uint32_t& count, uint32_t& edge_index) const;

  /**
   * Convenience method to get the names for an edge
   * @param  edge  Directed edge
   *
   * @return  Returns a list (vector) of names.
   */
  std::vector<std::string> GetNames(const DirectedEdge* edge) const;

  /**
   * Convenience method to get the types for the names given the edge
   * @param  edge  Directed edge
   * @return  Returns unit16_t.  If a bit is set, then it is a ref.
   */
  uint16_t GetTypes(const DirectedEdge* edge) const;

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
   * @param  idx  Directed edge or node index. Used to lookup list of signs.
   * @param  signs_on_node Are we looking for signs at the node?  These are the
   *                       intersection names.
   * @return  Returns a list (vector) of signs.
   */
  std::vector<SignInfo> GetSigns(const uint32_t idx, bool signs_on_node = false) const;

  /**
   * Convenience method to get the signs for an edge given the directed
   * edge index.
   * @param  idx  Directed edge or node index. Used to lookup list of signs.
   * @param  signs_on_node Are we looking for signs at the node?  These are the
   *                       intersection names.
   * @return  Returns a list (vector) of signs.
   */
  std::vector<SignInfo>
  GetSigns(const uint32_t idx,
           std::unordered_map<uint32_t, std::pair<uint8_t, std::string>>& index_pronunciation_map,
           bool signs_on_node = false) const;

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
   * Get lane connections ending on this edge.
   * @param  idx  GraphId of the directed edge.
   * @return  Returns a list of lane connections ending on this edge.
   */
  std::vector<LaneConnectivity> GetLaneConnectivity(const uint32_t idx) const;

  /**
   * Convenience method for use with costing to get the speed for an edge given the directed
   * edge and a time (seconds since start of the week). If the current speed of the edge
   * is 0 then the current speed is ignore and other speed sources are used to prevent
   * issues with costing
   *
   * @param  de            Directed edge information.
   * @param  traffic_mask  A mask denoting which types of traffic data should be used to get the speed
   * @param  seconds       Seconds of the week since midnight (ie Monday morning). Defaults to noon
   *                       Monday. Note that for free and constrained flow there is no concept of a
   *                       week so we modulus the time to day based seconds
   * @param  flow_sources  Which speed sources were used in this speed calculation. Optional pointer,
   *                       if nullptr is passed in flow_sources does nothing.
   * @param  seconds_from_now   Absolute number of seconds from now till the moment the edge is
   * passed. Be careful when setting the value in reverse direction algorithms to use proper value. It
   * affects the percentage of live-traffic usage on the edge. The bigger seconds_from_now is set the
   * less percentage is taken. Currently this parameter is set to 0 when building a route with reverse
   * and bidirectional a*.
   * @return Returns the speed for the edge.
   */
  inline uint32_t GetSpeed(const DirectedEdge* de,
                           uint8_t flow_mask = kConstrainedFlowMask,
                           uint64_t seconds = kInvalidSecondsOfWeek,
                           bool is_truck = false,
                           uint8_t* flow_sources = nullptr,
                           const uint64_t seconds_from_now = 0) const {
    // if they dont want source info we bind it to a temp and no one will miss it
    uint8_t temp_sources;
    if (!flow_sources)
      flow_sources = &temp_sources;
    *flow_sources = kNoFlowMask;

    // TODO(danpat): for short-ish durations along the route, we should fade live
    //               speeds into any historic/predictive/average value we'd normally use

    constexpr double LIVE_SPEED_FADE = 1. / 3600.;
    // This parameter describes the weight of live-traffic on a specific edge. In the beginning of the
    // route live-traffic gives more information about current congestion situation. But the further
    // we go the less consistent this traffic is. We prioritize predicted traffic in this case.
    // Want to have a smooth decrease function.
    float live_traffic_multiplier = 1. - std::min(seconds_from_now * LIVE_SPEED_FADE, 1.);
    uint32_t partial_live_speed = 0;
    float partial_live_pct = 0;
    if ((flow_mask & kCurrentFlowMask) && traffic_tile() && live_traffic_multiplier != 0.) {
      auto directed_edge_index = std::distance(const_cast<const DirectedEdge*>(directededges_), de);
      auto volatile& live_speed = traffic_tile.trafficspeed(directed_edge_index);
      // only use current speed if its valid and non zero, a speed of 0 makes costing values crazy
      if (live_speed.speed_valid() && (partial_live_speed = live_speed.get_overall_speed()) > 0) {
        *flow_sources |= kCurrentFlowMask;
        if (live_speed.breakpoint1 == 255) {
          partial_live_pct = 1.;
        } else {

          // Since live speed didn't cover the entire edge, lets calculate the coverage
          // to facilitate blending with other sources for uncovered part
          partial_live_pct =
              (
                  // First section
                  (live_speed.encoded_speed1 != UNKNOWN_TRAFFIC_SPEED_RAW ? live_speed.breakpoint1
                                                                          : 0)
                  // Second section
                  + (live_speed.encoded_speed2 != UNKNOWN_TRAFFIC_SPEED_RAW
                         ? (live_speed.breakpoint2 - live_speed.breakpoint1)
                         : 0)
                  // Third section
                  + (live_speed.encoded_speed3 != baldr::UNKNOWN_TRAFFIC_SPEED_RAW
                         ? (255 - live_speed.breakpoint2)
                         : 0)) /
              255.0;
        }
        partial_live_pct *= live_traffic_multiplier;
        if (partial_live_pct == 1.) {
          return partial_live_speed;
        }
      }
    }

    // use predicted speed if a time was passed in, the predicted speed layer was requested, and if
    // the edge has predicted speed
    auto invalid_time = seconds == kInvalidSecondsOfWeek;
    if (!invalid_time && (flow_mask & kPredictedFlowMask) && de->has_predicted_speed()) {
      seconds %= midgard::kSecondsPerWeek;
      uint32_t idx = de - directededges_;
      float speed = predictedspeeds_.speed(idx, seconds);
      if (valid_speed(speed)) {
        *flow_sources |= kPredictedFlowMask;
        return static_cast<uint32_t>(partial_live_speed * partial_live_pct +
                                     (1 - partial_live_pct) * (speed + 0.5f));
      }
#ifdef LOGGING_LEVEL_TRACE
      else
        LOG_TRACE("Predicted speed = " + std::to_string(speed) + " for edge index: " +
                  std::to_string(idx) + " of tile: " + std::to_string(header_->graphid()));
#endif
    }

    // fallback to constrained if time of week is within 7am to 7pm (or if no time was passed in) and
    // if the edge has constrained speed
    // kInvalidSecondsOfWeek %= midgard::kSecondsPerDay = 12.1
    seconds %= midgard::kSecondsPerDay;
    auto is_daytime = (25200 < seconds && seconds < 68400);
    if ((invalid_time || is_daytime) && (flow_mask & kConstrainedFlowMask) &&
        valid_speed(de->constrained_flow_speed())) {
      *flow_sources |= kConstrainedFlowMask;
      return static_cast<uint32_t>(partial_live_speed * partial_live_pct +
                                   (1 - partial_live_pct) * de->constrained_flow_speed());
    }
#ifdef LOGGING_LEVEL_TRACE
    else if (de->constrained_flow_speed() != 0)
      LOG_TRACE("Constrained flow speed = " + std::to_string(de->constrained_flow_speed()) +
                " for edge index: " + std::to_string(de - directededges_) +
                " of tile: " + std::to_string(header_->graphid()));
#endif

    // fallback to freeflow if time of week is not within 7am to 7pm (or if no time was passed in) and
    // the edge has freeflow speed
    if ((invalid_time || !is_daytime) && (flow_mask & kFreeFlowMask) &&
        valid_speed(de->free_flow_speed())) {
      *flow_sources |= kFreeFlowMask;
      return static_cast<uint32_t>(partial_live_speed * partial_live_pct +
                                   (1 - partial_live_pct) * de->free_flow_speed());
    }
#ifdef LOGGING_LEVEL_TRACE
    else if (de->free_flow_speed() != 0)
      LOG_TRACE("Freeflow speed = " + std::to_string(de->constrained_flow_speed()) +
                " for edge index: " + std::to_string(de - directededges_) +
                " of tile: " + std::to_string(header_->graphid()));
#endif

    // Fallback further to specified or derived speed
    auto const speed = static_cast<uint32_t>(partial_live_speed * partial_live_pct +
                                             (1 - partial_live_pct) * de->speed());
    return (is_truck && (de->truck_speed() > 0)) ? std::min(de->truck_speed(), speed) : speed;
  }

  inline const volatile TrafficSpeed& trafficspeed(const DirectedEdge* de) const {
    auto directed_edge_index = std::distance(const_cast<const DirectedEdge*>(directededges_), de);
    return traffic_tile.trafficspeed(directed_edge_index);
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

  /**
   * Convenience method to determine whether an edge is currently closed
   * due to traffic.  Roads are considered closed when the following are true
   *   a) have traffic data for that tile
   *   b) we have a valid record for that edge
   *   b) the speed is zero
   *
   * @param edge  the directed edge for which we need to know if its closed
   * @return      whether or not its closed
   */
  inline bool IsClosed(const DirectedEdge* edge) const {
    auto volatile& live_speed =
        traffic_tile.trafficspeed(static_cast<uint32_t>(edge - directededges_));
    return live_speed.closed();
  }

  const TrafficTile& get_traffic_tile() const {
    return traffic_tile;
  }

protected:
  // Graph tile memory. A Graph tile owns its memory.
  std::unique_ptr<const GraphMemory> memory_;

  // Header information for the tile
  GraphTileHeader* header_{};

  // List of nodes. Fixed size structure, indexed by Id within the tile.
  NodeInfo* nodes_{};

  // List of transitions between nodes on different levels. NodeInfo contains
  // an index and count of transitions.
  NodeTransition* transitions_{};

  // List of directed edges. Fixed size structure indexed by Id within the tile.
  DirectedEdge* directededges_{};

  // Extended directed edge records. For expansion. These are indexed by the same
  // Id as the directed edge.
  DirectedEdgeExt* ext_directededges_{};

  // Access restrictions, 1 or more per edge id
  AccessRestriction* access_restrictions_{};

  // Transit departures, many per index (indexed by directed edge index and
  // sorted by departure time)
  TransitDeparture* departures_{};

  // Transit stops (indexed by stop index within the tile)
  TransitStop* transit_stops_{};

  // Transit route (indexed by route index within the tile)
  TransitRoute* transit_routes_{};

  // Transit schedules (index by schedule index within the tile)
  TransitSchedule* transit_schedules_{};

  // Transit transfer records.
  TransitTransfer* transit_transfers_{};

  // Signs (indexed by directed edge index)
  Sign* signs_{};

  // Turn lanes (indexed by directed edge index)
  TurnLanes* turnlanes_{};

  // List of admins. This is a fixed size structure so it can be
  // indexed directly.
  Admin* admins_{};

  // List of complex_restrictions in the forward direction.
  char* complex_restriction_forward_{};

  // Size of the complex restrictions in the forward direction
  std::size_t complex_restriction_forward_size_{};

  // List of complex_restrictions in the reverse direction.
  char* complex_restriction_reverse_{};

  // Size of the complex restrictions in the reverse direction
  std::size_t complex_restriction_reverse_size_{};

  // List of edge info structures. Since edgeinfo is not fixed size we
  // use offsets in directed edges.
  char* edgeinfo_{};

  // Size of the edgeinfo data
  std::size_t edgeinfo_size_{};

  // Street names as sets of null-terminated char arrays. Edge info has
  // offsets into this array.
  char* textlist_{};

  // Number of bytes in the text/name list
  std::size_t textlist_size_{};

  // List of edge graph ids. The list is broken up in bins which have
  // indices in the tile header.
  GraphId* edge_bins_{};

  // Lane connectivity data.
  LaneConnectivity* lane_connectivity_{};

  // Number of bytes in lane connectivity data.
  std::size_t lane_connectivity_size_{};

  // Predicted speeds
  PredictedSpeeds predictedspeeds_;

  // Map of stop one stops in this tile.
  std::unordered_map<std::string, GraphId> stop_one_stops;

  // Map of route one stops in this tile.
  std::unordered_map<std::string, std::list<GraphId>> route_one_stops;

  // Map of operator one stops in this tile.
  std::unordered_map<std::string, std::list<GraphId>> oper_one_stops;

  // Pointer to live traffic data (can be nullptr if not active)
  TrafficTile traffic_tile{nullptr};

  // GraphTiles are noncopyable.
  GraphTile(const GraphTile&) = delete;
  GraphTile& operator=(const GraphTile&) = delete;

  // They are, however, moveable.
  GraphTile(GraphTile&&) = default;
  GraphTile& operator=(GraphTile&&) = default;

  // constructs empty tile
  GraphTile();

  /**
   * Constructor given the graph Id, pointer to the tile data, and the
   * size of the tile data. This is used for memory mapped (mmap) tiles.
   * @param  graphid  Tile Id.
   * @param  ptr      Pointer to the start of the tile's data.
   * @param  size     Size in bytes of the tile data.
   */
  GraphTile(const GraphId& graphid,
            std::unique_ptr<const GraphMemory> memory,
            std::unique_ptr<const GraphMemory> traffic_memory = nullptr);

  /**
   * Constructor given the graph Id, pointer to the tile data, and the
   * size of the tile data. This is used for memory mapped (mmap) tiles.
   * @param  graphid  Tile Id.
   * @param  ptr      Pointer to the start of the tile's data.
   * @param  size     Size in bytes of the tile data.
   */
  GraphTile(const std::string& tile_dir,
            const GraphId& graphid,
            std::unique_ptr<const GraphMemory>&& traffic_memory = nullptr);

  /**
   * Initialize pointers to internal tile data structures.
   * @param  graphid    Graph Id for the tile.
   */
  void Initialize(const GraphId& graphid);

  /**
   * For transit tiles, save off the pair<tileid,lineid> lookup via
   * onestop_ids.  This will be used for including or excluding transit lines
   * for transit routes.  Save 2 maps because operators contain all of their
   * route's tile_line pairs and it is used to include or exclude the operator
   * as a whole. Also associates stops.
   * @param  graphid  Tile Id.
   */
  void AssociateOneStopIds(const GraphId& graphid);

  /** Decrompresses tile bytes into the internal graphtile byte buffer
   * @param  graphid     the id of the tile to be decompressed
   * @param  compressed  the compressed bytes
   * @return a pointer to a graphtile if it  has been successfully initialized with
   *         the uncompressed data, or nullptr
   */
  static graph_tile_ptr DecompressTile(const GraphId& graphid, const std::vector<char>& compressed);
};

} // namespace baldr
} // namespace valhalla
