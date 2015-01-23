#include "mjolnir/graphoptimizer.h"
#include <valhalla/midgard/logging.h>

#include <ostream>
#include <set>
#include <boost/format.hpp>

using namespace valhalla::midgard;
using namespace valhalla::baldr;

namespace valhalla {
namespace mjolnir {

GraphOptimizer::GraphOptimizer(const boost::property_tree::ptree& pt)
      : dupcount_(0),
        tile_hierarchy_(pt),
        graphreader_(tile_hierarchy_) {

  // Make sure there are at least 2 levels!
  if (tile_hierarchy_.levels().size() < 2)
    throw std::runtime_error("Bad tile hierarchy - need 2 levels");
}

void GraphOptimizer::Optimize() {
  // Iterate through all levels and all tiles.
  // TODO - concurrency
  for (auto tile_level :  tile_hierarchy_.levels()) {
    dupcount_ = 0;
    uint32_t level = (uint32_t)tile_level.second.level;
    uint32_t ntiles = tile_level.second.tiles.TileCount();
    for (uint32_t tileid = 0; tileid < ntiles; tileid++) {
      // Get the graph tile. Skip if no tile exists (common case)
      GraphTileBuilder tilebuilder(tile_hierarchy_, GraphId(tileid, level, 0));
      if (tilebuilder.size() == 0) {
        continue;
      }

      // Update nodes and directed edges as needed
      const GraphTileHeader* existinghdr = tilebuilder.header();
      GraphTileHeaderBuilder hdrbuilder;
      hdrbuilder.set_nodecount(existinghdr->nodecount());
      hdrbuilder.set_directededgecount(existinghdr->directededgecount());
      hdrbuilder.set_edgeinfo_offset(existinghdr->edgeinfo_offset());
      hdrbuilder.set_textlist_offset(existinghdr->textlist_offset());

      std::vector<NodeInfoBuilder> nodes;
      std::vector<DirectedEdgeBuilder> directededges;

      // Iterate through the nodes and the directed edges
      uint32_t nodecount = tilebuilder.header()->nodecount();
      GraphId node(tileid, level, 0);
      for (uint32_t i = 0; i < nodecount; i++, node++) {

        NodeInfoBuilder nodeinfo = tilebuilder.node(i);

        // Go through directed edges and update data
        for (uint32_t j = 0, n = nodeinfo.edge_count(); j < n; j++) {
          DirectedEdgeBuilder& directededge = tilebuilder.directededge(
                                  nodeinfo.edge_index() + j);

          // Set the opposing edge index
          directededge.set_opp_index(GetOpposingEdgeIndex(node, directededge));
          directededges.emplace_back(std::move(directededge));
        }

        // Add the node to the list
        nodes.emplace_back(std::move(nodeinfo));
      }

      // Write the new file
      tilebuilder.Update(tile_hierarchy_, hdrbuilder, nodes, directededges);
    }

    LOG_WARN((boost::format("Possible duplicates at level: %1% = %2%")
        % level % dupcount_).str());
  }
}

// Get the GraphId of the opposing edge.
uint32_t GraphOptimizer::GetOpposingEdgeIndex(const GraphId& startnode,
                                              DirectedEdge& edge) {
  // Get the tile at the end node and get the node info
  GraphId endnode = edge.endnode();
  GraphTile* tile = graphreader_.GetGraphTile(endnode);
  const NodeInfo* nodeinfo = tile->node(endnode.id());

  // TODO - check if more than 1 edge has matching startnode and
  // distance!

  // Get the directed edges and return when the end node matches
  // the specified node and length matches
  uint32_t opp_index = 777777;
  const DirectedEdge* directededge = tile->directededge(
              nodeinfo->edge_index());
  for (uint32_t i = 0; i < nodeinfo->edge_count(); i++, directededge++) {
    // End node must match the start node, shortcut flag must match
    // and lengths must be close...
    if (directededge->endnode() == startnode &&
        edge.shortcut() == directededge->shortcut() &&
        directededge->length() == edge.length()) {
      if (opp_index != 7777777) {
   //     LOG_WARN("More than 1 edge matches nodes and length");
        dupcount_++;
      }
      opp_index = i;
    }
  }

  if (opp_index == 777777) {
    LOG_ERROR("Opposing edge not found");
    LOG_WARN((boost::format("Opposing edge not found at LL=%1%,%2%")
      % nodeinfo->latlng().lat() % nodeinfo->latlng().lng()).str());
    LOG_WARN((boost::format("Length = %1% Startnode %2% EndNode %3% sc %4%")
      % edge.length() % startnode % edge.endnode() % edge.shortcut()).str());
    directededge =  tile->directededge(nodeinfo->edge_index());
    for (uint32_t i = 0; i < nodeinfo->edge_count(); i++, directededge++) {
      LOG_WARN((boost::format("Length = %1% Endnode: %2% sc %3%")
        % directededge->length() % directededge->endnode()
        % directededge->shortcut()).str());
    }
    return 31;  // TODO - what value to return that will not impact routes?
  }
  return opp_index;
}

}
}
