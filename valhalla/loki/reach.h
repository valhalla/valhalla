#pragma once
#include <cstdint>

#include <valhalla/baldr/directededge.h>
#include <valhalla/loki/search.h>
#include <valhalla/thor/dijkstras.h>

constexpr uint8_t kInbound = 1;
constexpr uint8_t kOutbound = 2;

namespace valhalla {
namespace loki {

// NOTE: another approach is possible which would still allow for one-at-a-time look up. In this case
// we could actually keep the tree from the previous expansion and as soon as the tree from the next
// expansion intersects it we could merge the two and continue. To make that work we'd need to remove
// the part of the expansion that isn't relevant to the current expansion and re-sort the edge set.
// That would not be an easy task. Instead we could just use the intersection as a short circuit to
// terminate the expansion if the threshold has been met. The problem here is one of diminishing
// returns. Which expansion do you keep around for performing the intersections. Surely not all of
// them, so the question is which ones. The first one may not be relevant for the second one but may
// be for the 3rd one. The trick is we dont keep any of them around, instead we just remember the
// the outcomes for any edges that we touched. Specifically we keep a set of edge ids we found to be
// reachable. We can use this information to shortcut the current expansion when it intersects these
// edges. If you are searching outbound and hit an edge that is reachabile outbound then you can stop
// and claim reachable outbound. The same works for the inbound direction. Sadly you cannot use
// previously found unreachable edges to shortcut a search as they could be local minima (just a
// branch) of your current search tree

struct directed_reach {
  uint32_t outbound : 16;
  uint32_t inbound : 16;
};

class Reach : public thor::Dijkstras {
public:
  Reach();
  // TODO: currently this interface has no place for time, we need to both add it and handle
  // TODO: the problem of guessing what time to use at the other end of the route depending on
  // TODO: whether its depart_at or arrive_by
  /**
   * Returns the in and outbound reach for a given edge in the graph and a given costing model
   * @param edge        the directed edge in the graph for which we want to know the reach
   * @param edge_id     the id of the directed edge
   * @param max_reach   the maximum reach to check
   * @param reader      a graph reader so we can do an expansion
   * @param costing     the costing model to apply during the expansion
   * @param direction   a mask of which directions we care about in or out or both
   * @return the reach in both directions for the given edge
   */
  directed_reach operator()(const baldr::DirectedEdge* edge,
                            const baldr::GraphId edge_id,
                            uint32_t max_reach,
                            baldr::GraphReader& reader,
                            const std::shared_ptr<sif::DynamicCost>& costing,
                            uint8_t direction = kInbound | kOutbound);

protected:
  // the main method above will do a conservative reach estimate stopping the expansion at any
  // edges which the costing could decide to skip (because of restrictions and possibly more?)
  // when that happens and the maximum reach is not found, this is then validated with a more
  // accurate exact expansion performed by the method below
  directed_reach exact(const baldr::DirectedEdge* edge,
                       const baldr::GraphId edge_id,
                       uint32_t max_reach,
                       baldr::GraphReader& reader,
                       const std::shared_ptr<sif::DynamicCost>& costing,
                       uint8_t direction = kInbound | kOutbound);

  // we keep a queue of nodes to expand from, to prevent duplicate expansion we use a set
  // each node we pop from the set will increase the reach and be added to the done set
  // the done set is used to avoid duplicate expansion of already dequeued nodes
  // we also track how many nodes were added as transitions from other levels
  // this allows us to have "duplicate" nodes but not do any trickery with the expansion
  void enqueue(const baldr::GraphId& node_id,
               baldr::GraphReader& reader,
               const std::shared_ptr<sif::DynamicCost>& costing,
               graph_tile_ptr tile);

  // callback fired when a node is expanded from, the node will be the end node of the previous label
  virtual void ExpandingNode(baldr::GraphReader& graphreader,
                             graph_tile_ptr tile,
                             const baldr::NodeInfo* node,
                             const sif::EdgeLabel& current,
                             const sif::EdgeLabel* previous) override;

  // when the main loop is looking to continue expanding we tell it to terminate here
  virtual thor::ExpansionRecommendation ShouldExpand(baldr::GraphReader& graphreader,
                                                     const sif::EdgeLabel& pred,
                                                     const thor::ExpansionType route_type) override;

  // tell the expansion how many labels to expect and how many buckets to use
  virtual void GetExpansionHints(uint32_t& bucket_count,
                                 uint32_t& edge_label_reservation) const override;

  // need to reset the queues
  virtual void Clear() override;

  google::protobuf::RepeatedPtrField<Location> locations_;
  std::unordered_set<uint64_t> queue_, done_;
  uint32_t max_reach_{};
  size_t transitions_{};
};

} // namespace loki
} // namespace valhalla
