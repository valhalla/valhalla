#include "mjolnir/ferry_connections.h"

#include <list>
#include <set>
#include <unordered_map>

#include "baldr/graphid.h"
#include "midgard/util.h"

using namespace valhalla::baldr;

namespace valhalla {
namespace mjolnir {

// Test if the set of edges can be classified as a turn channel. Total length
// must be less than kMaxTurnChannelLength and there cannot be any exit signs.
bool IsTurnChannel(const uint32_t count, sequence<OSMWay>& ways,
                   sequence<Edge>& edges,
                   sequence<OSMWayNode>& way_nodes,
                   std::unordered_set<size_t>& linkedgeindexes,
                   const uint32_t rc) {
  // Method to get the shape for an edge - since LL is stored as a pair of
  // floats we need to change into PointLL to get length of an edge
  const auto EdgeShape = [&way_nodes](size_t idx, const size_t count) {
    std::list<PointLL> shape;
    for (size_t i = 0; i < count; ++i) {
      auto node = (*way_nodes[idx++]).node;
      shape.emplace_back(node.lng, node.lat);
    }
    return shape;
  };

  // If there are more than 2 end node connections or road classes are
  // not motorway or trunk then this is not a turn channel
  if (count > 2 || rc <= static_cast<uint32_t>(RoadClass::kTrunk)) {
    return false;
  }

  // Iterate through the link edges
  bool exit_sign = false;
  float total_length = 0.0f;
  for (auto idx : linkedgeindexes) {
    sequence<Edge>::iterator element = edges[idx];
    auto edge = *element;
    auto shape = EdgeShape(edge.llindex_, edge.attributes.llcount);
    total_length += valhalla::midgard::length(shape);
    if (total_length > kMaxTurnChannelLength) {
      return false;
    }
    OSMWay way = *ways[edge.wayindex_];
    if (way.junction_ref_index() != 0 ||
        way.destination_ref_index() != 0 ||
        way.destination_street_index() != 0 ||
        way.destination_ref_to_index() != 0 ||
        way.destination_street_to_index() != 0 ||
        way.destination_index() != 0) {
     return false;
    }
  }
  return true;
}

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
                     const std::string& way_nodes_file,
                     DataQuality& stats) {
  LOG_INFO("Reclassifying link graph edges...")

  uint32_t count = 0;
  std::unordered_set<size_t> visitedset;      // Set of visited nodes
  std::unordered_set<size_t> expandset;       // Set of nodes to expand
  std::unordered_set<size_t> linkedgeindexes; // Edge indexes to reclassify
  std::multiset<uint32_t> endrc;              // Classifications at end nodes
  sequence<OSMWay> ways(ways_file, false);
  sequence<Edge> edges(edges_file, false);
  sequence<Node> nodes(nodes_file, false);
  sequence<OSMWayNode> way_nodes(way_nodes_file, false);

  // Lambda to expand from the end node of an edge
  auto expand = [&expandset, &endrc, &nodes, &edges, &visitedset] (const Edge& edge, const sequence<Node>::iterator& node_itr) {
    auto end_node_itr =  edge.sourcenode_ == node_itr.position() ?
                          nodes[edge.targetnode_] : nodes[edge.sourcenode_];
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
    bool has_exit = false;
    auto bundle = collect_node_edges(node_itr, nodes, edges);
    if (bundle.node.ref() || bundle.node.exit_to()) {
      has_exit = true;
    }
    if (bundle.node.attributes_.link_edge &&
        bundle.node.attributes_.non_link_edge) {
      // Get the highest classification of non-link edges at this node
      endrc = { GetBestNonLinkClass(bundle.node_edges) };

      // Expand from all link edges
      for (const auto& startedge : bundle.node_edges) {
        // Get the edge information. Skip non-link edges and link edges
        // already tested for reclassification
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
          if (expanded.node.ref() || expanded.node.exit_to()) {
            has_exit = true;
          }
          visitedset.insert(expand_node_itr.position());
          expandset.erase(expandset.begin());
          for (const auto& expandededge : expanded.node_edges) {
            // Do not allow use of the start edge, any non-link edge, or any
            // edge already considered for reclassification
            if (expandededge.second == startedge.second ||
               !expandededge.first.attributes.link ||
                expandededge.first.attributes.reclass_link) {
              continue;
            }

            // Add the link to the set of edges to reclassify
            linkedgeindexes.insert(expandededge.second);

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
          if (rc == kAbsurdRoadClass) {
            continue;
          }

          // If there are only 2 end road classes test if this should be
          // a turn channel - must be no more than the max turn cost length
          // and have no exit signs
          bool turn_channel = (has_exit) ? false :
                  IsTurnChannel(endrc.size(), ways, edges,
                      way_nodes, linkedgeindexes, rc);

          // Reclassify link edges
          for (auto idx : linkedgeindexes) {
            sequence<Edge>::iterator element = edges[idx];
            auto edge = *element;
            if (rc > edge.attributes.importance) {
              edge.attributes.importance = rc;
              count++;
            }
            if (turn_channel) {
              edge.attributes.turn_channel = true;
            }

            // Mark the edge so we don't try to reclassify it again
            edge.attributes.reclass_link = true;
            element = edge;
          }
        }
      }
    }

    // Go to the next node
    node_itr += bundle.node_count;
  }
  LOG_INFO("Finished with " + std::to_string(count) + " reclassified.");
}

}
}
