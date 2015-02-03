
#include "thor/pathalgorithm.h"

#include <valhalla/midgard/logging.h>

using namespace valhalla::baldr;

namespace valhalla {
namespace thor {

// TODO - this should be a config value (or get from config!)
constexpr uint32_t kLocalLevel = 2;

GraphId PathAlgorithm::GetStartNode(GraphReader& graphreader,
                     const DirectedEdge* directededge) {
  // Get the end node of directed edge
  const GraphTile* tile = graphreader.GetGraphTile(directededge->endnode());
  const NodeInfo* nodeinfo = tile->node(directededge->endnode());

  // Get the opposing edge and then get its endnode
  const DirectedEdge* edge = tile->directededge(nodeinfo->edge_index() +
                       directededge->opp_index());
  GraphId endnode = edge->endnode();

  // Transition down to the local level
  while (endnode.level() != kLocalLevel) {
    // The transition down edge will be one of the last edges (transition up
    // edges are last if they exist)
    tile = graphreader.GetGraphTile(endnode);
    nodeinfo = tile->node(endnode);
    edge = tile->directededge(nodeinfo->edge_index() +
                              nodeinfo->edge_count() - 1);
    while (!edge->trans_down()) {
      // TODO - validate we don't go too far!
      edge--;
    }
    endnode = edge->endnode();
  }
  return endnode;
}

// Form the path from the adjacency list.
// TODO - support partial distances at origin/destination
std::vector<GraphId> PathAlgorithm::FormLocalPath(const uint32_t dest,
               GraphReader& graphreader) {
  // Add the destination edge - it should be on the local level
  std::vector<GraphId> edgesonpath;
  if (edgelabels_[dest].edgeid().level() != kLocalLevel) {
    LOG_ERROR("Destination edge is not on the local level");
  }
  edgesonpath.push_back(edgelabels_[dest].edgeid());

  uint32_t edgelabel_index = dest;
  GraphId prior_local_node;
  while ((edgelabel_index = edgelabels_[edgelabel_index].predecessor()) !=
              kInvalidLabel) {
    // Get the GraphId of the directed edge and and get the directed edge info
    GraphId edgeid = edgelabels_[edgelabel_index].edgeid();
    const GraphTile* tile = graphreader.GetGraphTile(edgeid);
    const DirectedEdge* directededge = tile->directededge(edgeid);

    // Store the end node if a downward transition to the local level
    if (directededge->trans_down() &&
        directededge->endnode().level() == kLocalLevel) {
      prior_local_node = directededge->endnode();
    }

    // Skip transition edges
    if (directededge->trans_down() || directededge->trans_up()) {
      continue;
    }

    // If the directed edge is not on the local level we recover the edges
    // on the local level that make up this edgeid
    if (edgeid.level() != kLocalLevel) {
      // Get the start of this directed edge
      GraphId startnode = GetStartNode(graphreader, directededge);

      // Recover the path on the local level between this start node and
      // the prior local node
      if (directededge->shortcut()) {
        // TODO!!!!
        // Need to recover a set of edges that make up the shortcut

      } else {
        // Get the directed edge on the local level that ends at the
        // prior local node
        uint32_t n = 0;
        const GraphTile* tile = graphreader.GetGraphTile(startnode);
        const NodeInfo* nodeinfo = tile->node(startnode);
        uint32_t edgeindex = nodeinfo->edge_index();
        directededge = tile->directededge(edgeindex);
        while (!(directededge->endnode() == prior_local_node &&
              n < nodeinfo->edge_count())) {
          directededge++;
          edgeindex++;
          n++;
        }
if (!(directededge->endnode() == prior_local_node)) {
  LOG_ERROR("Prior end not found...");
}

        // Set the prior end node on the local level to startnode
        prior_local_node = startnode;

        // Add the directed edge graph Id to the path
        edgesonpath.emplace_back(GraphId(startnode.tileid(), startnode.level(),
                                edgeindex));
      }
    } else {
      // Add the edges on path if not a transition up or down
      edgesonpath.emplace_back(edgelabels_[edgelabel_index].edgeid());
    }
  }

  // Reverse the list and return
  std::reverse(edgesonpath.begin(), edgesonpath.end());
  return edgesonpath;
}

}
}
