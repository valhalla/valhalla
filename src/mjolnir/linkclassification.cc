#include "mjolnir/ferry_connections.h"

#include <list>
#include <set>
#include <unordered_map>

#include <valhalla/baldr/graphid.h>

using namespace valhalla::baldr;

namespace valhalla {
namespace mjolnir {

// Get the best classification for any driveable non-link edges from a node.
uint32_t GetBestNonLinkClass(const std::map<Edge, size_t>& edges) {
  uint32_t bestrc = kAbsurdRoadClass;
  for (const auto& edge : edges) {
    if (!edge.first.attributes.link &&
        (edge.first.attributes.driveableforward ||
         edge.first.attributes.driveablereverse)) {
      if (edge.first.attributes.importance < bestrc) {
        bestrc = edge.first.attributes.importance;
      }
    }
  }
  return bestrc;
}

// Reclassify links (ramps and turn channels). OSM usually classifies links as
// the best classification, while to more effectively create shortcuts it is
// better to "downgrade" link edges to the lower classification.
void ReclassifyLinks(const std::string& ways_file,
                     const std::string& nodes_file,
                     const std::string& edges_file,
                     DataQuality& stats) {
  LOG_INFO("Reclassifying link graph edges...")

  uint32_t count = 0;
  std::unordered_set<size_t> visitedset;  // Set of visited nodes
  std::unordered_set<size_t> expandset;   // Set of nodes to expand
  std::list<size_t> linkedgeindexes;     // Edge indexes to reclassify
  std::multiset<uint32_t> endrc;           //
  sequence<OSMWay> ways(ways_file, false);
  sequence<Edge> edges(edges_file, false);
  sequence<Node> nodes(nodes_file, false);

  //try to expand
  auto expand = [&expandset, &endrc, &nodes, &edges, &visitedset] (const Edge& edge, const sequence<Node>::iterator& node_itr) {
    auto end_node_itr = (edge.sourcenode_ == node_itr.position() ? nodes[edge.targetnode_] : node_itr);
    auto end_node_bundle = collect_node_edges(end_node_itr, nodes, edges);

    // Add the classification if there is a driveable non-link edge
    if (end_node_bundle.node.attributes_.non_link_edge) {
      endrc.insert(GetBestNonLinkClass(end_node_bundle.node_edges));

      // Expand if the link count > 1 and a "short link" is present (could be
      // an internal intersection link). Do not want to expand in cases where
      // an exit ramp crosses onto an entrance back onto the same highway
      // that was exited. But there are cases with internal intersection links
      // that we do need to expand.
      // Also expand if only one non-link edge exists and more than 1 link
      // exists(common case - service road off a ramp)
      if ( end_node_bundle.link_count > 1 &&
          (end_node_bundle.node.attributes_.shortlink ||
           end_node_bundle.non_link_count == 1)   &&
          visitedset.find(end_node_itr.position()) == visitedset.end()) {
       expandset.insert(end_node_itr.position());
      }
    } else if (visitedset.find(end_node_itr.position()) == visitedset.end()) {
      expandset.insert(end_node_itr.position());
    }
  };

  //for each node
  sequence<Node>::iterator node_itr = nodes.begin();
  while (node_itr != nodes.end()) {
    // If the node has a both links and non links at it
    auto bundle = collect_node_edges(node_itr, nodes, edges);
    if (bundle.node.attributes_.link_edge &&
        bundle.node.attributes_.non_link_edge) {
      // Get the highest classification of non-link edges at this node
      endrc = { GetBestNonLinkClass(bundle.node_edges) };

      // Expand from all link edges
      for (const auto& startedge : bundle.node_edges) {
        // Get the edge information. Skip non-link edges and link edges
        // already reclassified
        if (!startedge.first.attributes.link ||
             startedge.first.attributes.reclass_link) {
          continue;
        }

        // Clear the visited set and start expanding at the end of this edge
        visitedset = {};
        expandset = {};
        linkedgeindexes = { startedge.second };
        expand(startedge.first, node_itr);

        // Expand edges until all paths reach a node that has a non-link and
        // only one link edge
        while (!expandset.empty()) {
          // Expand all edges from this node and pop from expand set
          auto expand_node_itr = nodes[*expandset.begin()];
          auto expanded = collect_node_edges(expand_node_itr, nodes, edges);
          visitedset.insert(expand_node_itr.position());
          expandset.erase(expandset.begin());
          for (const auto& expandededge : expanded.node_edges) {
            // Do not allow use of the start edge or any non-link edge
            if (expandededge.second == startedge.second ||
                !expandededge.first.attributes.link) {
              continue;
            }
            // Expand from end node of this link edge
            expand(expandededge.first, expand_node_itr);
          }
        }

        // Once expand list is empty - mark all link edges encountered
        // with the specified classification / importance and break out
        // of this loop (can still expand other ramp edges
        // from the starting node
        // Make sure this connects...
        if (endrc.size() < 2) {
          stats.AddIssue(kUnconnectedLinkEdge, GraphId(),
                         (*ways[startedge.first.wayindex_]).way_id(), 0);
        }
        else {
          // Set to the value of the 2nd best road class of all
          // connections. This protects against downgrading links
          // when branches occur.
          uint32_t rc = *(++endrc.cbegin());
          for (auto idx : linkedgeindexes) {
            sequence<Edge>::iterator element = edges[idx];
            auto edge = *element;
            edge.attributes.reclass_link = true;
            if (rc > edge.attributes.importance) {
              edge.attributes.importance = rc;
              element = edge;
              count++;
            }
          }
        }
      }
    }

    //go to the next node
    node_itr += bundle.node_count;
  }
  LOG_INFO("Finished with " + std::to_string(count) + " reclassified.");
}

}
}
