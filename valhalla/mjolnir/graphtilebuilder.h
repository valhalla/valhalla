#ifndef VALHALLA_MJOLNIR_GRAPHTILEBUILDER_H_
#define VALHALLA_MJOLNIR_GRAPHTILEBUILDER_H_

#include <boost/functional/hash.hpp>
#include <fstream>
#include <iostream>
#include <list>
#include <utility>
#include <algorithm>
#include <string>
#include <memory>
#include <list>
#include <unordered_set>
#include <unordered_map>

#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphtile.h>
#include <valhalla/baldr/signinfo.h>
#include <valhalla/baldr/transitcalendar.h>
#include <valhalla/baldr/transitdeparture.h>
#include <valhalla/baldr/transitroute.h>
#include <valhalla/baldr/transitstop.h>
#include <valhalla/baldr/transittransfer.h>
#include <valhalla/baldr/transittrip.h>

#include <valhalla/mjolnir/graphtileheaderbuilder.h>
#include <valhalla/mjolnir/nodeinfobuilder.h>
#include <valhalla/mjolnir/directededgebuilder.h>
#include <valhalla/mjolnir/edgeinfobuilder.h>
#include <valhalla/mjolnir/admininfobuilder.h>
#include <valhalla/baldr/tilehierarchy.h>
#include "signbuilder.h"

namespace valhalla {
namespace mjolnir {

using edge_tuple = std::tuple<uint32_t, baldr::GraphId, baldr::GraphId>;

/**
 * Graph information for a tile within the Tiled Hierarchical Graph.
 */
class GraphTileBuilder : public baldr::GraphTile {
 public:
  /**
   * Constructor
   */
  GraphTileBuilder();

  /**
   * Constructor given an existing tile. This is used to read in the tile
   * data and then add to it (e.g. adding node connections between hierarchy
   * levels. If the deserialize flag is set then all objects are serialized
   * from memory into builders that can be added to and then stored using
   * StoreTileData.
   * @param  basedir  Base directory path
   * @param  graphid  GraphId used to determine the tileid and level
   * @param  deserialize  If true the existing objects in the tile are
   *                      converted into builders so they can be added to.
   */
  GraphTileBuilder(const baldr::TileHierarchy& hierarchy,
                   const GraphId& graphid,
                   const bool deserialize);

  /**
   * Output the tile to file. Stores as binary data.
   * @param  graphid  GraphID to store.
   * @param  hierarchy  Gives info about number of tiles per level
   */
  void StoreTileData(const baldr::TileHierarchy& hierarchy,
                     const baldr::GraphId& graphid);

  /**
   * Update a graph tile with new header, nodes, and directed edges. Used
   * in GraphValidator to update directed edge information.
   * @param hierarchy How the tiles are setup on disk
   * @param hdr Updated header
   * @param nodes Updated list of nodes
   * @param directededges Updated list of edges.
   */
  void Update(const baldr::TileHierarchy& hierarchy,
            const GraphTileHeaderBuilder& hdr,
            const std::vector<NodeInfoBuilder>& nodes,
            const std::vector<DirectedEdgeBuilder>& directededges);

  /**
   * Update a graph tile with new header, nodes, directed edges, signs,
   * and turn restrictions.
   * This is used to add directed edges connecting two hierarchy levels.
   * @param  hierarchy      How the tiles are setup on disk
   * @param  hdr            Update header
   * @param  nodes          Update list of nodes
   * @param  directededges  Updated list of edges.
   * @param  signs          Updated list of signs.
   * @param  trs            Updated list of turn restrictions.
   */
  void Update(const baldr::TileHierarchy& hierarchy,
              const GraphTileHeaderBuilder& hdr,
              const std::vector<NodeInfoBuilder>& nodes,
              const std::vector<DirectedEdgeBuilder>& directededges,
              const std::vector<SignBuilder>& signs);

  /**
   * Add a node and its outbound edges. Sets the node's edge index
   * and edge count.
   * @param  node   Node information builder.
   * @param  directededges  List of directed edges (builders) outbound
   *                        from the node.
   */
  void AddNodeAndDirectedEdges(
      NodeInfoBuilder& node,
      const std::vector<DirectedEdgeBuilder>& directededges);

  /**
   * Get the current list of node builders.
   * @return  Returns the node info builders.
   */
  const std::vector<NodeInfoBuilder>& nodes() const;

  /**
   * Gets the current list of directed edge (builders).
   * @return  Returns the directed edge builders.
   */
  const std::vector<DirectedEdgeBuilder>& directededges() const;

  /**
   * Clear the current list of nodes (builders).
   */
  void ClearNodes();

  /**
   * Clear the current list of directed edges (builders).
   */
  void ClearDirectedEdges();

  /**
   * Add a transit departure.
   * @param  departure  Transit departure record.
   */
  void AddTransitDeparture(const baldr::TransitDeparture& departure);

  /**
   * Add a transit trip.
   * @param  trip  Transit trip record.
   */
  void AddTransitTrip(const baldr::TransitTrip& trip);

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
   * Add a transit transfer.
   * @param  transfer  Transit transfer record.
   */
  void AddTransitTransfer(const baldr::TransitTransfer& transfer);

  /**
   * Add a transit calendar exception.
   * @param  exception  Transit calendar exception record.
   */
  void AddTransitCalendar(const baldr::TransitCalendar& exception);

  /**
   * Add sign information.
   * @param  idx  Directed edge index.
   * @param  signs  Sign information.
   */
  void AddSigns(const uint32_t idx,
                const std::vector<baldr::SignInfo>& signs);

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
  bool HasEdgeInfo(const uint32_t edgeindex, const baldr::GraphId& nodea,
                       const baldr::GraphId& nodeb, uint32_t& edge_info_offset);

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
   * @param  lls  The shape of the target edge.
   * @param  names  The names of the target edge.
   * @param  added  Set to true if the target edge was newly added to the list,
   *                set to false if the target edge was already in the list.
   *
   * @return  The edge info offset that will be stored in the directed edge.
   */
  uint32_t AddEdgeInfo(const uint32_t edgeindex, const baldr::GraphId& nodea,
                       const baldr::GraphId& nodeb,
                       const uint64_t wayid,
                       const std::vector<PointLL>& lls,
                       const std::vector<std::string>& names,
                       bool& added);

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
   * @param  start_dst      Date when daylight savings time starts.
   * @param  end_dst        Date when daylight savings time ends.
   *
   * @return  The admin offset that will be stored on the node.
   */
  uint32_t AddAdmin(const std::string& country_name, const std::string& state_name,
                    const std::string& country_iso, const std::string& state_iso,
                    const std::string& start_dst, const std::string& end_dst);

  /**
   * Gets a builder for a node from an existing tile.
   * @param  idx  Index of the node within the tile.
   * @return  Returns a reference to the node builder.
   */
  NodeInfoBuilder& node(const size_t idx);

  /**
   * Get the node builder at the specified index.
   * @param  idx  Index of the node builder.
   * @return  Returns a reference to the node builder.
   */
  NodeInfoBuilder& node_builder(const size_t idx);

  /**
   * Gets a builder for a directed edge from existing tile data.
   * @param  idx  Index of the directed edge within the tile.
   * @return  Returns a reference to the directed edge builder.
   */
  DirectedEdgeBuilder& directededge(const size_t idx);

  /**
   * Get the directed edge builder at the specified index.
   * @param  idx  Index of the directed edge builder.
   * @return  Returns a reference to the directed edge builder.
   */
  DirectedEdgeBuilder& directededge_builder(const size_t idx);

  /**
   * Gets a non-const sign (builder) from existing tile data.
   * @param  idx  Index of the sign (index in the array, not the
   *              directed edge index) within the tile.
   * @return  Returns a reference to the sign builder.
   */
  SignBuilder& sign(const size_t idx);

  /**
   * Get the sign builder at the specified index.
   * @param  idx  Index of the sign builder.
   * @return  Returns a reference to the sign builder.
   */
  SignBuilder& sign_builder(const size_t idx);

  /**
   * Gets a const admin builder at specified index.
   * @param  idx  Index of the admin builder in the list.
   */
  const AdminInfoBuilder& admins_builder(size_t idx);

 protected:

  struct EdgeTupleHasher {
    std::size_t operator()(const edge_tuple& k) const {
      std::size_t seed = 13;
      boost::hash_combine(seed, index_hasher(std::get<0>(k)));
      boost::hash_combine(seed, id_hasher(std::get<1>(k)));
      boost::hash_combine(seed, id_hasher(std::get<2>(k)));
      return seed;
    }
    //function to hash each id
    std::hash<uint32_t> index_hasher;
    std::hash<valhalla::baldr::GraphId> id_hasher;
  };

  // Edge tuple for sharing edges that have common nodes and edgeindex
  static edge_tuple EdgeTuple(const uint32_t edgeindex,
                       const valhalla::baldr::GraphId& nodea,
                       const valhalla::baldr::GraphId& nodeb) {
    return (nodea < nodeb) ? std::make_tuple(edgeindex, nodea, nodeb):
        std::make_tuple(edgeindex, nodeb, nodea);
  }

  // Write all edgeinfo items to specified stream
  void SerializeEdgeInfosToOstream(std::ostream& out);

  // Write all textlist items to specified stream
  void SerializeTextListToOstream(std::ostream& out);

  // Header information for the tile
  GraphTileHeaderBuilder header_builder_;

  // List of nodes. This is a fixed size structure so it can be
  // indexed directly.
  std::vector<NodeInfoBuilder> nodes_builder_;

  // List of directed edges. This is a fixed size structure so it can be
  // indexed directly.
  std::vector<DirectedEdgeBuilder> directededges_builder_;

  // List of transit departures. Sorted by directed edge Id and
  // departure time
  std::vector<baldr::TransitDeparture> departure_builder_;

  // Transit trips. Sorted by trip Id.
  std::vector<baldr::TransitTrip> trip_builder_;

  // Transit stops. Sorted by stop Id.
  std::vector<baldr::TransitStop> stop_builder_;

  // Transit route. Sorted by route Id.
  std::vector<baldr::TransitRoute> route_builder_;

  // Transit transfers. Sorted by from stop Id.
  std::vector<baldr::TransitTransfer> transfer_builder_;

  // Transit calendar exceptions. Sorted by service Id.
  std::vector<baldr::TransitCalendar> exception_builder_;

  // List of signs. This is a fixed size structure so it can be
  // indexed directly.
  std::vector<SignBuilder> signs_builder_;

  // List of admins. This is a fixed size structure so it can be
  // indexed directly.
  std::vector<AdminInfoBuilder> admins_builder_;

  // Admin info offset
  std::unordered_map<std::string,size_t> admin_info_offset_map_;

  // Edge info offset and map
  size_t edge_info_offset_ = 0;
  std::unordered_map<edge_tuple, size_t, EdgeTupleHasher> edge_offset_map_;

  // The edgeinfo list
  std::list<EdgeInfoBuilder> edgeinfo_list_;

  // Text list offset and map
  uint32_t text_list_offset_ = 0;
  std::unordered_map<std::string, uint32_t> text_offset_map_;

  // Text list. List of names used within this tile
  std::list<std::string> textlistbuilder_;
};

}
}

#endif  // VALHALLA_MJOLNIR_GRAPHTILEBUILDER_H_

