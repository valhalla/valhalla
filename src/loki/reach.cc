#include "loki/reach.h"

using namespace valhalla::baldr;

namespace valhalla {
namespace loki {

directed_reach SimpleReach(const DirectedEdge* edge,
                           uint32_t max_reach,
                           GraphReader& reader,
                           uint8_t direction,
                           const sif::EdgeFilter& edge_filter,
                           const sif::NodeFilter& node_filter) {
  // TODO: harden against incomplete tile sets

  // no reach to start with
  directed_reach reach{};

  // TODO: throw these vectors into a cache that we reuse
  // we keep a queue of nodes to expand from, to prevent duplicate expansion we use a set
  // each node we pop from the set will increase the reach and be added to the done set
  // the done set is used to avoid duplicate expansion of already dequeued nodes
  // we also track how many nodes were added as transitions from other levels
  // this allows us to have "duplicate" nodes but not do any trickery with the expansion
  std::unordered_set<uint64_t> queue, done;
  queue.reserve(max_reach);
  done.reserve(max_reach);
  size_t transitions = 0;

  // helper lambda to enqueue a node
  const GraphTile* tile = nullptr;
  auto enqueue = [&](const GraphId& node_id) {
    // skip nodes which are done or invalid
    if (!node_id.Is_Valid() || done.find(node_id) != done.cend())
      return;
    // if the node isnt accessable bail
    if (!reader.GetGraphTile(node_id, tile))
      return;
    const auto* node = tile->node(node_id);
    if (node_filter(node))
      return;
    // otherwise we enqueue it
    queue.insert(node_id);
    // and we enqueue it on the other levels
    for (auto& transition : tile->GetNodeTransitions(node))
      queue.insert(transition.endnode());
    // and we remember how many duplicates we enqueued
    transitions += node->transition_count();
  };

  // seed the expansion with a place to start expanding from
  if (edge_filter(edge) > 0)
    enqueue(edge->endnode());

  // get outbound reach by doing a simple forward expansion until you either hit the max_reach
  // or you can no longer expand
  while (direction & kOutbound && queue.size() + done.size() - transitions < max_reach &&
         !queue.empty()) {
    // increase the reach and get the nodes id
    auto node_id = GraphId(*done.insert(*queue.begin()).first);
    // pop the node from the queue
    queue.erase(queue.begin());
    // expand from the node
    if (!reader.GetGraphTile(node_id, tile))
      continue;
    for (const auto& edge : tile->GetDirectedEdges(node_id)) {
      // if this edge is traversable we enqueue its end node
      if (edge_filter(&edge) > 0)
        enqueue(edge.endnode());
    }
  }
  reach.outbound = queue.size() + done.size() - transitions;

  // TODO: move to graphreader
  // helper lambdas to get the begin node of an edge by using its opposing edges end node
  auto begin_node = [&](const DirectedEdge* edge) -> GraphId {
    // grab the node
    if (!reader.GetGraphTile(edge->endnode(), tile))
      return {};
    const auto* node = tile->node(edge->endnode());
    // grab the opp edges end node
    const auto* opp_edge = tile->directededge(node->edge_index() + edge->opp_index());
    return opp_edge->endnode();
  };

  // seed the expansion with a place to start expanding from
  done.clear();
  queue.clear();
  transitions = 0;
  if (edge_filter(edge) > 0)
    enqueue(begin_node(edge));

  // get inbound reach by doing a simple reverse expansion until you either hit the max_reach
  // or you can no longer expand
  while (direction & kInbound && queue.size() + done.size() - transitions < max_reach &&
         !queue.empty()) {
    // increase the reach and get the nodes id
    auto node_id = GraphId(*done.insert(*queue.begin()).first);
    // pop the node from the queue
    queue.erase(queue.begin());
    // expand from the node
    if (!reader.GetGraphTile(node_id, tile))
      continue;
    for (const auto& edge : tile->GetDirectedEdges(node_id)) {
      // get the opposing edge
      if (!reader.GetGraphTile(edge.endnode(), tile))
        continue;
      const auto* node = tile->node(edge.endnode());
      const auto* opp_edge = tile->directededge(node->edge_index() + edge.opp_index());
      // if this edge is traversable we enqueue its end node
      if (edge_filter(opp_edge) > 0)
        enqueue(opp_edge->endnode());
    }
  }
  reach.inbound = queue.size() + done.size() - transitions;

  return reach;
}

/*
directed_reach Reach(const DirectedEdge* edge,
                     uint32_t max_reach,
                     GraphReader& reader,
                     uint8_t direction,
                     const sif::EdgeFilter& edge_filter,
                     const sif::NodeFilter& node_filter,
                     reach_cache* cache) {
  // TODO: if edge reach exists in cache for both directions, return it

  directed_reach reach{};

  // TODO: expand bidirectional island cache
  // if start edge, end node and opp edge are not filtered, push them both into the bidir queue
  // while the bidir queue is not empty and reach is < max in both directions
  //    pop edge off the bidir queue
  //    increment in and outbound reach
  //    go to end node and if filtered
  //      continue
  //    for each edge from node
  //      if edge and node and op edge is not filtered
  //        add the edges to the bidir queue
  //      else if edge is not filtered
  //        add to outbound queue
  //      else if opp edge is not filtered
  //        add to inbound queue

  // while outbound queue is not empty and outbound reach < max
  //   pop edge off outbound queue
  //   increment outbound reach
  //   go to end node and if filtered
  //     continue
  //   for each edge from node
  //     if edge is not filtered
  //       push edge into outbound queue

  // while inbound queue is not empty and inbound reach < max
  //   pop edge off inbound queue
  //   increment inbound reach
  //   go to end node and if filtered
  //     continue
  //   for each edge from node
  //     if opp edge is not filtered
  //       push opp edge into inbound queue

  // TODO: in the above we may push the same edges into queues at several places we need to avoid that
  // by keeping track maybe of the nodes we visit or maybe of the edges we visit

  // TODO: we also when searching only in one direction could end up back to a section of the graph
  // that is bidirectional. in this case we should start tracking this other bidirectional island or
  // maybe its not worth it unless a candidate shows up in there

  // TODO: when the single direction search hits a bidirectional island we can immediately increment
  // that search to include that islands size

  throw std::logic_error("Optimized reach is not yet implemented");
  return reach;
}*/

} // namespace loki
} // namespace valhalla