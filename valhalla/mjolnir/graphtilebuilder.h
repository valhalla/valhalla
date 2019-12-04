#ifndef VALHALLA_MJOLNIR_GRAPHTILEBUILDER_H_
#define VALHALLA_MJOLNIR_GRAPHTILEBUILDER_H_

#include <algorithm>
#include <boost/functional/hash.hpp>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <list>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <valhalla/baldr/admin.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphtile.h>
#include <valhalla/baldr/graphtileheader.h>
#include <valhalla/baldr/nodetransition.h>
#include <valhalla/baldr/sign.h>
#include <valhalla/baldr/signinfo.h>
#include <valhalla/baldr/transitdeparture.h>
#include <valhalla/baldr/transitroute.h>
#include <valhalla/baldr/transitschedule.h>
#include <valhalla/baldr/transitstop.h>
#include <valhalla/baldr/turnlanes.h>

#include <valhalla/mjolnir/complexrestrictionbuilder.h>
#include <valhalla/mjolnir/directededgebuilder.h>
#include <valhalla/mjolnir/edgeinfobuilder.h>

namespace valhalla {
namespace mjolnir {

using edge_tuple = std::tuple<uint32_t, baldr::GraphId, baldr::GraphId>;

/**
 * Graph information for a tile within the Tiled Hierarchical Graph.
 */
class GraphTileBuilder : public baldr::GraphTile {
public:
  /**
   * Constructor given an existing tile. This is used to read in the tile
   * data and then add to it (e.g. adding node connections between hierarchy
   * levels. If the deserialize flag is set then all objects are serialized
   * from memory into builders that can be added to and then stored using
   * StoreTileData.
   * @param  tile_dir               Base directory path
   * @param  graphid                GraphId used to determine the tileid and level
   * @param  deserialize            If true the existing objects in the tile are
   *                                converted into builders so they can be added to.
   * @param  serialize_turn_lanes   If true, the offsets are truely text offsets.
   *                                If false, the offsets are indexes into unique name file
   */
  GraphTileBuilder(const std::string& tile_dir,
                   const GraphId& graphid,
                   const bool deserialize,
                   bool serialize_turn_lanes = true);

  /**
   * Output the tile to file. Stores as binary data.
   * @param  graphid  GraphID to store.
   * @param  hierarchy  Gives info about number of tiles per level
   */
  void StoreTileData();

  /**
   * Update a graph tile with new nodes and directed edges. Assumes no new
   * nodes or edges are added. Attributes within existing nodes and edges
   * are updated. This is used in GraphValidator to update directed edge
   * information.
   * @param nodes Updated list of nodes
   * @param directededges Updated list of edges.
   */
  void Update(const std::vector<NodeInfo>& nodes, const std::vector<DirectedEdge>& directededges);

  /**
   * Get the current list of node builders.
   * @return  Returns the node info builders.
   */
  std::vector<NodeInfo>& nodes();

  /**
   * Gets the current list of directed edge (builders).
   * @return  Returns the directed edge builders.
   */
  std::vector<DirectedEdge>& directededges();

  /**
   * Gets the current list of node transition (builders).
   * @return  Returns a reference to node transition builders.
   */
  std::vector<NodeTransition>& transitions() {
    return transitions_builder_;
  }

  /**
   * Add a transit departure.
   * @param  departure  Transit departure record.
   */
  void AddTransitDeparture(const baldr::TransitDeparture& departure);

  /**
   * Add a transit stop.
   * @param  stop  Transit stop record.
   */
  void AddTransitStop(const baldr::TransitStop& stop);

  /**
   * Add a transit route.
   * @param  route  Transit route record.
   */
  void AddTransitRoute(const baldr::TransitRoute& route);

  /**
   * Add a transit schedule.
   * @param  schedule  Transit schedule record.
   */
  void AddTransitSchedule(const baldr::TransitSchedule& schedule);

  /**
   * Add an access restriction.
   * @param  access_restriction  Access Restriction record.
   */
  void AddAccessRestriction(const baldr::AccessRestriction& access_restriction);

  /**
   * Add restriction.
   * @param  restrictions Access restrictions
   */
  void AddAccessRestrictions(const std::vector<AccessRestriction>& restrictions);

  /**
   * Add sign information.
   * @param  idx  Directed edge index.
   * @param  signs  Sign information.
   */
  void AddSigns(const uint32_t idx, const std::vector<baldr::SignInfo>& signs);

  /**
   * Add lane connectivity information.
   * @param  idx  Directed edge index.
   * @param  lc  Lane connectivity information.
   */
  void AddLaneConnectivity(const std::vector<baldr::LaneConnectivity>& lc);

  /**
   * Add forward complex restriction.
   * @param  res  Complex restriction.
   */
  void AddForwardComplexRestriction(const ComplexRestrictionBuilder& res);

  /**
   * Add reverse complex restriction.
   * @param  res  Complex restriction.
   */
  void AddReverseComplexRestriction(const ComplexRestrictionBuilder& res);

  /**
   *
   * @param  edgeindex         The index of the edge - used with nodea and nodeb to
   *                           form tuple that uniquely identifies the edge info since
   *                           there are two directed edges per edge info.
   * @param  nodea             One of two nodes - used with edgeindex and nodeb to
   *                           form tuple that uniquely identifies the edge info since
   *                           there are two directed edges per edge info.
   * @param  nodeb             One of two nodes - used with edgeindex and nodea to
   *                           form tuple that uniquely identifies the edge info since
   *                           there are two directed edges per edge info.
   * @param edge_info_offset   the index of the edge info within the tile
   *
   * @return            The edge info offset that will be stored in the directed edge.
   */
  bool HasEdgeInfo(const uint32_t edgeindex,
                   const baldr::GraphId& nodea,
                   const baldr::GraphId& nodeb,
                   uint32_t& edge_info_offset);

  /**
   * Add the edge info to the tile.
   *
   * @param  edgeindex  The index of the edge - used with nodea and nodeb to
   *                    form tuple that uniquely identifies the edge info since
   *                    there are two directed edges per edge info.
   * @param  nodea  One of two nodes - used with edgeindex and nodeb to
   *                form tuple that uniquely identifies the edge info since
   *                there are two directed edges per edge info.
   * @param  nodeb  One of two nodes - used with edgeindex and nodea to
   *                form tuple that uniquely identifies the edge info since
   *                there are two directed edges per edge info.
   * @param  wayid  The target edge is part of this the way id.
   * @param  elev   Mean elevation.
   * @param  bn     Bike network.
   * @param  spd    Speed limit.
   * @param  lls    The shape of the target edge.
   * @param  names  The names of the target edge.
   * @param  types  Bits indicating if the name is a ref vs a name.
   * @param  added  Set to true if the target edge was newly added to the list,
   *                set to false if the target edge was already in the list.
   * @param  diff_names Indicates the opposing direction has different names.
   *                    If true a new EdgeInfo is always added.
   * @return  The edge info offset that will be stored in the directed edge.
   */
  template <class shape_container_t>
  uint32_t AddEdgeInfo(const uint32_t edgeindex,
                       const baldr::GraphId& nodea,
                       const baldr::GraphId& nodeb,
                       const uint32_t wayid,
                       const float elev,
                       const uint32_t bn,
                       const uint32_t spd,
                       const shape_container_t& lls,
                       const std::vector<std::string>& names,
                       const uint16_t types,
                       bool& added,
                       bool diff_names = false);

  /**
   * Add the edge info to the tile. This method accepts an encoded shape string.
   * @param  edgeindex  The index of the edge - used with nodea and nodeb to
   *                    form tuple that uniquely identifies the edge info since
   *                    there are two directed edges per edge info.
   * @param  nodea  One of two nodes - used with edgeindex and nodeb to
   *                form tuple that uniquely identifies the edge info since
   *                there are two directed edges per edge info.
   * @param  nodeb  One of two nodes - used with edgeindex and nodea to
   *                form tuple that uniquely identifies the edge info since
   *                there are two directed edges per edge info.
   * @param  wayid  The target edge is part of this the way id.
   * @param  elev   Mean elevation.
   * @param  bn     Bike network.
   * @param  spd    Speed limit.
   * @param  llstr  The shape of the target edge as an encoded string.
   * @param  names  The names of the target edge.
   * @param  types  Bits indicating if the name is a ref vs a name.
   * @param  added  Set to true if the target edge was newly added to the list,
   *                set to false if the target edge was already in the list.
   * @param  diff_names Indicates the opposing direction has different names.
   *                    If true a new EdgeInfo is always added.
   * @return  The edge info offset that will be stored in the directed edge.
   */
  uint32_t AddEdgeInfo(const uint32_t edgeindex,
                       const baldr::GraphId& nodea,
                       const baldr::GraphId& nodeb,
                       const uint32_t wayid,
                       const float elev,
                       const uint32_t bn,
                       const uint32_t spd,
                       const std::string& llstr,
                       const std::vector<std::string>& names,
                       const uint16_t types,
                       bool& added,
                       bool diff_names = false);

  /**
   * Set the mean elevation in the most recently added EdgeInfo.
   * @param elev Mean elevation.
   */
  void set_mean_elevation(const float elev);

  /**
   * Set the mean elevation to the EdgeInfo given the edge info offset. This requires
   * a serialized tile builder.
   * @param offset Edge info offset.
   * @param elev Mean elevation.
   */
  void set_mean_elevation(const uint32_t offset, const float elev);

  /**
   * Add a name to the text list.
   * @param  name  Name/text to add.
   * @return  Returns offset (bytes) to the name.
   */
  uint32_t AddName(const std::string& name);

  /**
   * Add admin info to the tile.
   * @param  country_name   Country name of the admin
   * @param  state_name     State name of the admin
   * @param  country_iso    Country ISO Code.  ISO3166-1
   * @param  state_iso      State ISO Code.  ISO3166-2  Example://
   *                        Prince Edward Island = PE
   *                        Country ISO + dash + state ISO will give
   *                        you ISO3166-2 for state.
   * @return  The admin offset that will be stored on the node.
   */
  uint32_t AddAdmin(const std::string& country_name,
                    const std::string& state_name,
                    const std::string& country_iso,
                    const std::string& state_iso);

  /**
   * Gets a reference to the header builder.
   * @return  Returns a reference to the header builder.
   */
  GraphTileHeader& header_builder();

  /**
   * Gets a node from an existing tile.
   * @param  idx  Index of the node within the tile.
   * @return  Returns a reference to the node builder.
   */
  NodeInfo& node(const size_t idx);

  /**
   * Get the node at the specified index.
   * @param  idx  Index of the node builder.
   * @return  Returns a reference to the node builder.
   */
  NodeInfo& node_builder(const size_t idx);

  /**
   * Gets a directed edge from existing tile data.
   * @param  idx  Index of the directed edge within the tile.
   * @return  Returns a reference to the directed edge.
   */
  DirectedEdge& directededge(const size_t idx);

  /**
   * Gets a pointer to directed edges within the list being built.
   * @param  idx  Index of the directed edge within the tile.
   * @return  Returns a pointer to the directed edge builder (allows
   *          accessing all directed edges from a node).
   */
  const DirectedEdge* directededges(const size_t idx);

  /**
   * Get the directed edge builder at the specified index.
   * @param  idx  Index of the directed edge builder.
   * @return  Returns a reference to the directed edge builder.
   */
  DirectedEdge& directededge_builder(const size_t idx);

  // TODO - add access method to directededge_ext_builder if extended directed edge
  // attributes are needed.

  /**
   * Gets a non-const access restriction from existing tile data.
   * @param  idx  Index of the restriction (index in the array, not the
   *              directed edge index) within the tile.
   * @return  Returns a reference to the access restriction.
   */
  AccessRestriction& accessrestriction(const size_t idx);

  /**
   * Gets an access restriction builder at the specified index.
   * @param  idx  Index of the restriction (index in the array, not the
   *              directed edge index) within the tile.
   * @return  Returns a reference to the access restriction (builder).
   */
  AccessRestriction& accessrestriction_builder(const size_t idx);

  /**
   * Gets a non-const sign (builder) from existing tile data.
   * @param  idx  Index of the sign (index in the array, not the
   *              directed edge index) within the tile.
   * @return  Returns a reference to the sign builder.
   */
  Sign& sign(const size_t idx);

  /**
   * Get the sign builder at the specified index.
   * @param  idx  Index of the sign builder.
   * @return  Returns a reference to the sign builder.
   */
  Sign& sign_builder(const size_t idx);

  /**
   * Gets a const admin builder at specified index.
   * @param  idx  Index of the admin builder in the list.
   */
  const Admin& admins_builder(size_t idx);

  /**
   * Sets the tile creation date.
   * @param  tile_creation_date  number of days from pivot date
   */
  void AddTileCreationDate(const uint32_t tile_creation_date);

  /**
   * Generates bin information for the edges in the provided tile
   * @param hierarchy  to perform the intersection with the bins' geoms
   * @param tile       the tile whose edges need the binned
   * @param tweeners   the additional bins in other tiles that intersect this tiles edges
   */
  using tweeners_t = std::unordered_map<GraphId, std::array<std::vector<GraphId>, kBinCount>>;
  static std::array<std::vector<GraphId>, kBinCount> BinEdges(const GraphTile* tile,
                                                              tweeners_t& tweeners);

  /**
   * Adds to the bins the tile already has, only modifies the header to reflect the new counts
   * and the bins themselves, everything else is copied directly without ever looking at it
   * @param tile_dir   Base tile directory
   * @param tile       the tile that needs the bins added
   * @param more_bins  the extra bin data to append to the tile
   */
  static void AddBins(const std::string& tile_dir,
                      const GraphTile* tile,
                      const std::array<std::vector<GraphId>, kBinCount>& more_bins);

  /**
   * Get the turn lane builder at the specified index.
   * @param  idx  Index of the turn lane builder.
   * @return  Returns a reference to the turn lane builder.
   */
  TurnLanes& turnlane_builder(const size_t idx);

  /**
   * Add turn lane information for a directed edge.
   * @param  idx  Directed edge index.
   * @param  str  Turn lane information.
   */
  void AddTurnLanes(const uint32_t idx, const std::string& str);

  /**
   * Add turn lane information for a directed edge.
   * @param  idx      Directed edge index.
   * @param  tl_idx   Turn lane index into the OSMData name_offset map
   */
  void AddTurnLanes(const uint32_t idx, const uint32_t tl_idx);

  /**
   * Add turn lanes
   * @param  turn_lanes vector of turn lanes
   */
  void AddTurnLanes(const std::vector<TurnLanes>& turn_lanes);

  /**
   * Add a predicted speed profile for a directed edge.
   * @param  idx  Edge Id within the tile.
   * @param  profile  Compressed profile (200 short int)
   * @param  predicted_count_hint  How many predicted speeds should we expect to add
   */
  void AddPredictedSpeed(const uint32_t idx,
                         const std::vector<int16_t>& profile,
                         const size_t predicted_count_hint = 256);

  /**
   * Updates a tile with predictive speed data. Also updates directed edges with
   * free flow and constrained flow speeds and the predicted traffic flag. The
   * predicted traffic is written after turn lane data.
   * @param  directededges  Updated directed edge information.
   */
  void UpdatePredictedSpeeds(const std::vector<DirectedEdge>& directededges);

protected:
  struct EdgeTupleHasher {
    std::size_t operator()(const edge_tuple& k) const {
      std::size_t seed = 13;
      boost::hash_combine(seed, index_hasher(std::get<0>(k)));
      boost::hash_combine(seed, id_hasher(std::get<1>(k)));
      boost::hash_combine(seed, id_hasher(std::get<2>(k)));
      return seed;
    }
    // function to hash each id
    std::hash<uint32_t> index_hasher;
    std::hash<valhalla::baldr::GraphId> id_hasher;
  };

  // Edge tuple for sharing edges that have common nodes and edgeindex
  static edge_tuple EdgeTuple(const uint32_t edgeindex,
                              const valhalla::baldr::GraphId& nodea,
                              const valhalla::baldr::GraphId& nodeb) {
    return (nodea < nodeb) ? std::make_tuple(edgeindex, nodea, nodeb)
                           : std::make_tuple(edgeindex, nodeb, nodea);
  }

  // Write all edgeinfo items to specified stream
  void SerializeEdgeInfosToOstream(std::ostream& out) const;

  // Write all textlist items to specified stream
  void SerializeTextListToOstream(std::ostream& out) const;

  // Base tile directory
  std::string tile_dir_;

  // Header information for the tile
  GraphTileHeader header_builder_;

  // List of nodes. This is a fixed size structure so it can be
  // indexed directly.
  std::vector<NodeInfo> nodes_builder_;

  // List of directed edges. This is a fixed size structure so it can be
  // indexed directly.
  std::vector<DirectedEdge> directededges_builder_;

  // Optional list of directed edge extended attributes. If this is used it must be the same size
  // as the directededges_builder.
  std::vector<DirectedEdgeExt> directededges_ext_builder_;

  // List of node transitions. This is a fixed size structure so it can be
  // indexed directly.
  std::vector<NodeTransition> transitions_builder_;

  // List of transit departures. Sorted by directed edge Id and
  // departure time
  std::vector<baldr::TransitDeparture> departure_builder_;

  // Transit stops.
  std::vector<baldr::TransitStop> stop_builder_;

  // Transit route.
  std::vector<baldr::TransitRoute> route_builder_;

  // Transit schedules.
  std::vector<baldr::TransitSchedule> schedule_builder_;

  // List of restrictions. Sorted by directed edge Id
  std::vector<baldr::AccessRestriction> access_restriction_builder_;

  // List of signs. This is a fixed size structure so it can be
  // indexed directly.
  std::vector<Sign> signs_builder_;

  // List of admins. This is a fixed size structure so it can be
  // indexed directly.
  std::vector<Admin> admins_builder_;

  // Admin info offset
  std::unordered_map<std::string, size_t> admin_info_offset_map_;

  // The forward complex restriction list
  std::vector<ComplexRestrictionBuilder> complex_restriction_forward_builder_;

  // The reverse complex restriction list
  std::vector<ComplexRestrictionBuilder> complex_restriction_reverse_builder_;

  // Edge info offset and map
  size_t edge_info_offset_ = 0;
  std::unordered_map<edge_tuple, size_t, EdgeTupleHasher> edge_offset_map_;
  std::unordered_map<uint32_t, EdgeInfoBuilder*> edgeinfo_offset_map_;

  // The edgeinfo list
  std::list<EdgeInfoBuilder> edgeinfo_list_;

  // Text list offset and map
  uint32_t text_list_offset_ = 0;
  std::unordered_map<std::string, uint32_t> text_offset_map_;

  // Text list. List of names used within this tile
  std::list<std::string> textlistbuilder_;

  // List of lane connectivity records.
  std::vector<LaneConnectivity> lane_connectivity_builder_;

  // List of turn lanes.
  std::vector<TurnLanes> turnlanes_builder_;

  // Offsets into predicted speed profiles for each directed edge.
  std::vector<uint32_t> speed_profile_offset_builder_;

  // Predicted speed profiles. 200 short int for each directed edge which has predicted speed.
  std::vector<int16_t> speed_profile_builder_;

  // lane connectivity list offset
  uint32_t lane_connectivity_offset_ = 0;
};

} // namespace mjolnir
} // namespace valhalla

#endif // VALHALLA_MJOLNIR_GRAPHTILEBUILDER_H_
