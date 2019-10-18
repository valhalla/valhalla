#include "loki/reach.h"

using namespace valhalla::baldr;

namespace valhalla {
namespace loki {

directed_reach SimpleReach(const DirectedEdge* edge,
                           uint32_t max_reach,
                           GraphReader& reader,
                           const sif::EdgeFilter& edge_filter,
                           const sif::NodeFilter& node_filter,
                           uint8_t direction) {

  // no reach to start with
  directed_reach reach{};
  if (max_reach == 0)
    return reach;

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
    for (const auto& transition : tile->GetNodeTransitions(node))
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
  // settled nodes + will be settled nodes - duplicated transitions nodes
  reach.outbound =
      std::min(static_cast<uint32_t>(queue.size() + done.size() - transitions), max_reach);

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
      // if this opposing edge is traversable we enqueue its begin node
      if (edge_filter(opp_edge) > 0)
        enqueue(edge.endnode());
    }
  }
  // settled nodes + will be settled nodes - duplicated transitions nodes
  reach.inbound =
      std::min(static_cast<uint32_t>(queue.size() + done.size() - transitions), max_reach);

  return reach;
}

} // namespace loki
} // namespace valhalla