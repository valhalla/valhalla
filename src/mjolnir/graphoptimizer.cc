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
      : tile_hierarchy_(pt),
        graphreader_(tile_hierarchy_) {

  // Make sure there are at least 2 levels!
  if (tile_hierarchy_.levels().size() < 2)
    throw std::runtime_error("Bad tile hierarchy - need 2 levels");
}

void GraphOptimizer::Optimize() {
  // Iterate through all levels and all tiles.
  // TODO - concurrency
  for (auto tile_level :  tile_hierarchy_.levels()) {
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
          // NOTE: shortcut edges do not always have opposing shortcuts
          // may need to fix this!
          if (directededge.shortcut()) {
            directededge.set_opp_index(31);
          } else {
            directededge.set_opp_index(GetOpposingEdgeIndex(node, directededge));
          }
          directededges.emplace_back(std::move(directededge));
        }

        // Add the node to the list
        nodes.emplace_back(std::move(nodeinfo));
      }

      // Write the new file
      tilebuilder.Update(tile_hierarchy_, hdrbuilder, nodes, directededges);
    }
  }
}

// Get the GraphId of the opposing edge.
uint32_t GraphOptimizer::GetOpposingEdgeIndex(const GraphId& node,
                                              DirectedEdge& edge) {
  // Get the tile at the end node
  GraphId endnode = edge.endnode();
  GraphTile* tile = graphreader_.GetGraphTile(endnode);

  // Get the node info
  const NodeInfo* nodeinfo = tile->node(endnode.id());
  uint32_t n = nodeinfo->edge_count();

  // Get the directed edges and return when the end node matches
  // the specified node and length matches
  const DirectedEdge* directededge =  tile->directededge(
              nodeinfo->edge_index());
  for (uint32_t i = 0; i < n; i++, directededge++) {
    if (directededge->endnode() == node &&
        fabs(directededge->length() - edge.length()) < 0.0001f) {
      return i;
    }
  }

  LOG_WARN((boost::format("Opposing edge not found at LL=%1%,%2% edges at end node= %3%")
    % nodeinfo->latlng().lat() % nodeinfo->latlng().lng() % n).str());
  LOG_WARN((boost::format("Length = %1% Basenode %2% EndNode %3% sc,td,tu %4%,%5%,%6%")
    % edge.length() % node % edge.endnode() % edge.shortcut() % edge.trans_down() % edge.trans_up()).str());
  directededge =  tile->directededge(nodeinfo->edge_index());
  for (uint32_t i = 0; i < n; i++, directededge++) {
    LOG_WARN((boost::format("Length = %1% Endnode: %2%") % directededge->length() % directededge->endnode()).str());
  }
  return 0;
}

}
}
