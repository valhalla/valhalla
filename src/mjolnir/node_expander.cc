#include "mjolnir/node_expander.h"

namespace valhalla {
namespace mjolnir {

node_bundle collect_node_edges(const sequence<Node>::iterator& node_itr,
                               sequence<Node>& nodes,
                               sequence<Edge>& edges) {
  //copy out the first nodes attributes (as they are the correctly merged one)
  auto itr = node_itr;
  node_bundle bundle(*itr);
  Node node;
  //for each node with the same id (duplicate)
  for(; itr != nodes.end() && (node = *itr).node.osmid == bundle.node.osmid; ++itr) {
    ++bundle.node_count;
    if(node.is_start()) {
      auto edge_itr = edges[node.start_of];
      auto edge = *edge_itr;
      // Set driveforward - this edge is traversed in forward direction
      edge.attributes.driveforward = edge.attributes.driveableforward;
      bundle.node_edges.emplace(std::make_pair(edge, node.start_of));
      bundle.node.attributes_.link_edge = bundle.node.attributes_.link_edge || edge.attributes.link;
      bundle.node.attributes_.ferry_edge = bundle.node.attributes_.ferry_edge || edge.attributes.driveable_ferry;
      bundle.node.attributes_.shortlink |= edge.attributes.shortlink;
      // Do not count non-driveable (e.g. emergency service roads) as a
      // non-link edge or non-ferry edge
      if (edge.attributes.driveableforward || edge.attributes.driveablereverse) {
        bundle.node.attributes_.non_link_edge = bundle.node.attributes_.non_link_edge || !edge.attributes.link;
        bundle.node.attributes_.non_ferry_edge = bundle.node.attributes_.non_ferry_edge || !edge.attributes.driveable_ferry;
      }
      if (edge.attributes.link) {
        bundle.link_count++;
      } else {
        bundle.non_link_count++;
      }
      if (edge.attributes.driveforward) {
        bundle.driveforward_count++;
      }
    }
    if(node.is_end()) {
      auto edge_itr = edges[node.end_of];
      auto edge = *edge_itr;
      // Set driveforward - this edge is traversed in reverse direction
      edge.attributes.driveforward = edge.attributes.driveablereverse;
      bundle.node_edges.emplace(std::make_pair(edge, node.end_of));
      bundle.node.attributes_.link_edge = bundle.node.attributes_.link_edge || edge.attributes.link;
      bundle.node.attributes_.ferry_edge = bundle.node.attributes_.ferry_edge || edge.attributes.driveable_ferry;
      bundle.node.attributes_.shortlink |= edge.attributes.shortlink;
      // Do not count non-driveable (e.g. emergency service roads) as a non-link edge
      if (edge.attributes.driveableforward || edge.attributes.driveablereverse) {
        bundle.node.attributes_.non_link_edge = bundle.node.attributes_.non_link_edge || !edge.attributes.link;
        bundle.node.attributes_.non_ferry_edge = bundle.node.attributes_.non_ferry_edge || !edge.attributes.driveable_ferry;
      }
      if (edge.attributes.link) {
        bundle.link_count++;
      } else {
        bundle.non_link_count++;
      }
      if (edge.attributes.driveforward) {
        bundle.driveforward_count++;
      }
    }
  }
  return bundle;
}

}
}
