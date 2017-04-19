#include <cstdint>

#include "thor/pathalgorithm.h"

#include "midgard/logging.h"

using namespace valhalla::baldr;
using namespace valhalla::sif;

namespace valhalla {
namespace thor {

// TODO - this should be a config value (or get from config!)
constexpr uint32_t kLocalLevel = 2;

/**
 * Get the begin node of a directed edge - transition down to the local
 * level if it is not already on that level.
 */
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
    uint32_t n = 0;
    while (!edge->trans_down()) {
      // TODO - validate we don't go too far!
      n++;
      edge--;
      if (n > 100) {
        LOG_ERROR("n > 100!!");
        break;
      }
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
        // Recover a set of edges on the local level that make up the shortcut
        prior_local_node = RecoverShortcut(graphreader, startnode,
                                           prior_local_node,
                                           directededge, edgesonpath);
        if (!prior_local_node.Is_Valid()) {
          LOG_ERROR("Could not recover shortcut edge - store the shortcut");
          prior_local_node = startnode;
          edgesonpath.emplace_back(edgelabels_[edgelabel_index].edgeid());
        }
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

        // TODO - do we still need this?
        if (!(directededge->endnode() == prior_local_node)) {
          LOG_ERROR("Prior end not found...");
        }

        // Set the prior end node on the local level to startnode
        prior_local_node = startnode;

        // Add the directed edge graph Id to the path
        edgesonpath.emplace_back(startnode.tileid(), startnode.level(),
                                edgeindex);
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

/**
 * Recover a shortcut path on the local level. Follows rules of shortcuts -
 * edges must be same importance, use, and be driveable in the forward
 * direction at each node.
 */
GraphId PathAlgorithm::RecoverShortcut(GraphReader& graphreader,
                                       const GraphId& startnode,
                                       const GraphId& endnode,
                                       const DirectedEdge* shortcutedge,
                                       std::vector<GraphId>& edgesonpath) {
  // Start and end nodes are on the local level
  GraphId invalid_graph_id;
  if (startnode.level() != kLocalLevel || endnode.level() != kLocalLevel) {
    LOG_ERROR("RecoverShortcut - end nodes are not on the local level");
    return invalid_graph_id;
  }

  // Get the end node lat,lng
  const GraphTile* tile = graphreader.GetGraphTile(endnode);
  const NodeInfo* nodeinfo = tile->node(endnode);
  const PointLL& endll = nodeinfo->latlng();

  // Expand from the start node
  tile = graphreader.GetGraphTile(startnode);
  nodeinfo = tile->node(startnode);
  GraphId edgeid(startnode.tileid(), startnode.level(),
                 nodeinfo->edge_index());
  const DirectedEdge* directededge = tile->directededge(nodeinfo->edge_index());
  for (uint32_t i = 0; i < nodeinfo->edge_count(); i++,
            directededge++, ++edgeid) {
    if (directededge->trans_up() ||
        directededge->use() != shortcutedge->use() ||
        directededge->classification() != shortcutedge->classification() ||
        !(directededge->forwardaccess() & kAutoAccess)) {
      continue;
    }

    // Clear the list of directed edges and add this edge
    std::vector<GraphId> edges;
    edges.push_back(edgeid);

    // Continue to get connected edges (only 1 should match) until we
    // reach the end node or the length + remaining distance exceeds
    // the distance to the end plus some tolerance
    bool found = true;
    const DirectedEdge* connectededge = directededge;
    while (found) {
      // Get the end node - success if we found the target endnode
      // of the shortcut edge
      if (connectededge->endnode() == endnode) {
        // Found it - add the edges (in reverse order)
        for (auto it = edges.rbegin(); it != edges.rend(); it++) {
          edgesonpath.push_back(*it);
        }
        return startnode;
      }

      found = false;

      // Get the nodeinfo and check if we might be on the wrong initial path
      tile = graphreader.GetGraphTile(connectededge->endnode());
      nodeinfo = tile->node(connectededge->endnode());
      if (nodeinfo->latlng().Distance(endll) > (shortcutedge->length())) {
        break;
      }

      // Expand from end of the prior directed edge
      uint32_t opp_index = connectededge->opp_index();
      GraphId connedgeid(connectededge->endnode().tileid(), startnode.level(),
                       nodeinfo->edge_index());
      connectededge = tile->directededge(nodeinfo->edge_index());
      for (uint32_t j = 0; j < nodeinfo->edge_count(); j++,
                connectededge++, ++connedgeid) {
        // Skip opposing directed edge, any transition edges. Skip any
        // non-matching directed edges (use, importance) or non-driveable.
        if (j == opp_index || connectededge->trans_up() ||
            connectededge->use() != shortcutedge->use() ||
            connectededge->classification() != shortcutedge->classification() ||
            !(connectededge->forwardaccess() & kAutoAccess)) {
          continue;
        }

        // Found a matching edge - should be the only one!
        edges.push_back(connedgeid);
        found = true;
        break;
      }
    }
  }

  // Error condition - return an invalid GraphId
  return invalid_graph_id;
}

}
}
