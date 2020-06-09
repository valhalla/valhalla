#include "loki/reach.h"

using namespace valhalla::baldr;

namespace valhalla {
namespace loki {

Reach::Reach() : Dijkstras() {
  // Mock up the Location struct with the important stuff missing
  auto* path_edge = locations_.Add()->add_path_edges();
  path_edge->set_distance(0);
  path_edge->set_begin_node(false);
  path_edge->set_end_node(false);
}

directed_reach Reach::operator()(const DirectedEdge* edge,
                                 const baldr::GraphId edge_id,
                                 uint32_t max_reach,
                                 GraphReader& reader,
                                 const std::shared_ptr<sif::DynamicCost>& costing,
                                 uint8_t direction) {

  // no reach is needed
  directed_reach reach{};
  if (max_reach == 0)
    return reach;
  max_reach_ = max_reach;

  auto node_filter = costing->GetNodeFilter();
  auto edge_filter = costing->GetEdgeFilter();

  // we keep a queue of nodes to expand from, to prevent duplicate expansion we use a set
  // each node we pop from the set will increase the reach and be added to the done set
  // the done set is used to avoid duplicate expansion of already dequeued nodes
  // we also track how many nodes were added as transitions from other levels
  // this allows us to have "duplicate" nodes but not do any trickery with the expansion

  // helper lambda to enqueue a node
  Clear();
  const GraphTile* tile = nullptr;
  auto enqueue = [&](const GraphId& node_id) {
    // skip nodes which are done or invalid
    if (!node_id.Is_Valid() || done_.find(node_id) != done_.cend())
      return;
    // if the node isnt accessable bail
    if (!reader.GetGraphTile(node_id, tile))
      return;
    const auto* node = tile->node(node_id);
    if (node_filter(node))
      return;
    // otherwise we enqueue it
    queue_.insert(node_id);
    // and we enqueue it on the other levels
    for (const auto& transition : tile->GetNodeTransitions(node))
      queue_.insert(transition.endnode());
    // and we remember how many duplicates we enqueued
    transitions_ += node->transition_count();
  };

  // TODO: here we stay extra conservative by avoiding starting on a simple restriction because we
  // TODO: dont have predecessor information in the simple reach expansion so we bail 1 edge earlier
  // NOTE: we can expand from the end of a complex restriction because it cant possibly be on our path
  // and we can expand from the start of a complex restriction because only the end would mark a
  // potential stopping point (maybe a path followed the restriction)

  // seed the expansion with a place to start expanding from
  if (edge_filter(edge) > 0 && !edge->restrictions())
    enqueue(edge->endnode());

  // get outbound reach by doing a simple forward expansion until you either hit the max_reach
  // or you can no longer expand
  while (direction & kOutbound && queue_.size() + done_.size() - transitions_ < max_reach &&
         !queue_.empty()) {
    // increase the reach and get the nodes id
    auto node_id = GraphId(*done_.insert(*queue_.begin()).first);
    // pop the node from the queue
    queue_.erase(queue_.begin());
    // expand from the node
    if (!reader.GetGraphTile(node_id, tile))
      continue;
    for (const auto& edge : tile->GetDirectedEdges(node_id)) {
      // TODO: we'd rather say !edge.end_simple_restriction() and not !edge.restrictions()
      // TODO: but we'd need the predecessor information to do that so we punt 1 edge earlier
      // NOTE: we can go through the start of the restriction because only the end would mark a
      // potential stopping point (maybe a path followed the restriction)

      // if this edge is traversable we enqueue its end node
      if (edge_filter(&edge) > 0 && !edge.end_restriction() && !edge.restrictions())
        enqueue(edge.endnode());
    }
  }
  // settled nodes + will be settled nodes - duplicated transitions nodes
  reach.outbound =
      std::min(static_cast<uint32_t>(queue_.size() + done_.size() - transitions_), max_reach);

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

  // NOTE: we can expand from the start of a complex or regular restriction because it cant possibly
  // be on our path and we can expand from the end of a complex restriction because only the start
  // would mark a potential stopping point (maybe a path followed the restriction)

  // seed the expansion with a place to start expanding from
  Clear();
  if (edge_filter(edge) > 0)
    enqueue(begin_node(edge));

  // get inbound reach by doing a simple reverse expansion until you either hit the max_reach
  // or you can no longer expand
  while (direction & kInbound && queue_.size() + done_.size() - transitions_ < max_reach &&
         !queue_.empty()) {
    // increase the reach and get the nodes id
    auto node_id = GraphId(*done_.insert(*queue_.begin()).first);
    // pop the node from the queue
    queue_.erase(queue_.begin());
    // expand from the node
    if (!reader.GetGraphTile(node_id, tile))
      continue;
    for (const auto& edge : tile->GetDirectedEdges(node_id)) {
      // get the opposing edge
      if (!reader.GetGraphTile(edge.endnode(), tile))
        continue;
      const auto* node = tile->node(edge.endnode());
      const auto* opp_edge = tile->directededge(node->edge_index() + edge.opp_index());

      // NOTE: we can go through the end of the restriction because only the start would mark a
      // potential stopping point (maybe a path followed the restriction). We also have to stop
      // at the start of a simple restriction because it could have been on our path

      // if this opposing edge is traversable we enqueue its begin node
      if (edge_filter(opp_edge) > 0 && !opp_edge->start_restriction() && !opp_edge->restrictions())
        enqueue(edge.endnode());
    }
  }
  // settled nodes + will be settled nodes - duplicated transitions nodes
  reach.inbound =
      std::min(static_cast<uint32_t>(queue_.size() + done_.size() - transitions_), max_reach);

  // if we didnt make the limit we should do an exact check for whichever directions we didnt meet
  auto retry_direction = (direction & kOutbound && reach.outbound < max_reach ? kOutbound : 0) |
                         (direction & kInbound && reach.inbound < max_reach ? kInbound : 0);
  if (retry_direction) {
    auto retry_reach = exact(edge, edge_id, max_reach, reader, costing, retry_direction);
    reach.outbound = std::max(reach.outbound, retry_reach.outbound);
    reach.inbound = std::max(reach.inbound, retry_reach.inbound);
  }

  return reach;
}

directed_reach Reach::exact(const valhalla::baldr::DirectedEdge* edge,
                            const GraphId edge_id,
                            uint32_t max_reach,
                            valhalla::baldr::GraphReader& reader,
                            const std::shared_ptr<sif::DynamicCost>& costing,
                            uint8_t direction) {
  // can we even try to expand?
  directed_reach reach{};
  if (costing->GetEdgeFilter()(edge) == 0) {
    return reach;
  }

  // fake up the input location
  const baldr::GraphTile* tile = nullptr;
  const auto* node = reader.GetEndNode(edge, tile);
  auto ll = node->latlng(tile->header()->base_ll());
  locations_.Mutable(0)->mutable_ll()->set_lng(ll.first);
  locations_.Mutable(0)->mutable_ll()->set_lat(ll.second);
  locations_.Mutable(0)->mutable_path_edges(0)->set_graph_id(edge_id);
  locations_.Mutable(0)->mutable_path_edges(0)->mutable_ll()->set_lng(ll.first);
  locations_.Mutable(0)->mutable_path_edges(0)->mutable_ll()->set_lat(ll.second);

  // fake up the costing array
  std::shared_ptr<sif::DynamicCost> costings[static_cast<int>(sif::TravelMode::kMaxTravelMode)];
  costings[static_cast<int>(costing->travel_mode())] = costing;

  // expand in the forward direction
  if (direction | kOutbound) {
    Clear();
    Compute(locations_, reader, costings, costing->travel_mode());
    reach.outbound = std::min(static_cast<uint32_t>(done_.size() - transitions_), max_reach);
  }

  // expand in the reverse direction
  if (direction | kInbound) {
    Clear();
    ComputeReverse(locations_, reader, costings, costing->travel_mode());
    reach.inbound = std::min(static_cast<uint32_t>(done_.size() - transitions_), max_reach);
  }

  return reach;
}

// callback fired when a node is expanded from, the node will be the end node of the previous label
void Reach::ExpandingNode(baldr::GraphReader& graphreader,
                          const baldr::GraphTile* tile,
                          const baldr::NodeInfo* node,
                          const sif::EdgeLabel& current,
                          const sif::EdgeLabel* previous) {
  // compute the nodes id
  GraphId node_id = tile->header()->graphid();
  node_id.set_id(node - tile->node(0));
  // mark the node and if its a new one then mark its doppelgänger on the other levels
  if (done_.insert(node_id).second) {
    for (const auto& trans : tile->GetNodeTransitions(node)) {
      done_.insert(trans.endnode());
      ++transitions_;
    }
  }
}

// when the main loop is looking to continue expanding we tell it to terminate here
thor::ExpansionRecommendation Reach::ShouldExpand(baldr::GraphReader& graphreader,
                                                  const sif::EdgeLabel& pred,
                                                  const thor::InfoRoutingType route_type) {
  if ((done_.size() - transitions_) < max_reach_)
    return thor::ExpansionRecommendation::continue_expansion;
  return thor::ExpansionRecommendation::prune_expansion;
}

// tell the expansion how many labels to expect and how many buckets to use
void Reach::GetExpansionHints(uint32_t& bucket_count, uint32_t& edge_label_reservation) const {
  // TODO: tweak these for performance
  bucket_count = max_reach_ * 50;
  edge_label_reservation = max_reach_ * 10;
}

void Reach::Clear() {
  queue_.clear();
  queue_.reserve(max_reach_);
  done_.clear();
  done_.reserve(max_reach_);
  transitions_ = 0;
  Dijkstras::Clear();
}

} // namespace loki
} // namespace valhalla
