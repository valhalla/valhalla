#include "loki/reach.h"

namespace valhalla {
namespace loki {

directed_reach SimpleReach(const valhalla::baldr::DirectedEdge* edge,
                           uint32_t max_reach,
                           valhalla::baldr::GraphReader& reader,
                           uint8_t direction,
                           const sif::EdgeFilter& edge_filter,
                           const sif::NodeFilter& node_filter,
                           reach_cache* cache) {
  directed_reach reach{};

  // TODO: get outbound reach by doing a simple forward expansion until you either hit the max_reach
  // or you can no longer expand. Use a vector as a queue for upcoming edges to expand from
  if (direction & kOutbound) {}

  // TODO: get inbound reach by doing a simple reverse expansion until you either hit the max_reach
  // or you can no longer expand. Use a vector as a queue for upcoming edges to expand from
  if (direction & kInbound) {}

  return reach;
}

directed_reach Reach(const valhalla::baldr::DirectedEdge* edge,
                     uint32_t max_reach,
                     valhalla::baldr::GraphReader& reader,
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

  return reach;
}

} // namespace loki
}