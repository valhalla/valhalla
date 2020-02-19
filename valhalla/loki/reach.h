#pragma once
#include <cstdint>

#include <valhalla/baldr/directededge.h>
#include <valhalla/loki/search.h>
#include <valhalla/thor/dijkstras.h>

constexpr uint8_t kInbound = 1;
constexpr uint8_t kOutbound = 2;

namespace valhalla {
namespace loki {

struct directed_reach {
  uint32_t outbound : 16;
  uint32_t inbound : 16;
};

directed_reach SimpleReach(const valhalla::baldr::DirectedEdge* edge,
                           uint32_t max_reach,
                           valhalla::baldr::GraphReader& reader,
                           const std::shared_ptr<sif::DynamicCost>& costing,
                           uint8_t direction = kInbound | kOutbound);

// NOTE: at the moment this checks one edge at a time. That works well with loki's current search
// implementation in that it expects to check one at a time. The performance of such a solution is
// not really optimal though. Instead what we can do is initialize dijkstras with a large batch of
// of edges (all that loki finds within the radius). Then when we get to the ShouldExpand call we
// can prune edges in the search that have already been shown to be reachable and we can keep
// expanding those labels which still need to be looked at. To actually do that though, we need more
// information in the edge label. Specifically we need an index that tells what location the chain
// leading to the label started at and we need to keep track of the length of the chain. Also because
// paths converge (when we update a label with a shorter path) we need to keep a map of locations
// whose paths take over the expansion chain of another location. This could get tricky because a
// chain can swap ownership multiple times. More thought is required to see if we could do something
// more efficiently in batch.

// NOTE: another approach is possible which would still allow for one-at-a-time look up. In this case
// we could actually keep the tree from the previous expansion and as soon as the tree from the next
// expansion intersects it we could merge the two and continue. To make that work we'd need to remove
// the part of the expansion that isn't relevant to the current expansion and re-sort the edge set.
// That would not be an easy task. Instead we could just use the intersection as a short circuit to
// terminate the expansion if the threshold has been met. The problem here is one of diminishing
// returns. Which expansion do you keep around for performing the intersections. Surely not all of
// them, so the question is which ones. The first one may not be relevant for the second one but may
// be for the 3rd one.

class Reach : public thor::Dijkstras {
public:
  directed_reach operator()(const valhalla::baldr::DirectedEdge* edge,
                            const baldr::GraphId edge_id,
                            uint32_t max_reach,
                            valhalla::baldr::GraphReader& reader,
                            const std::shared_ptr<sif::DynamicCost>& costing,
                            uint8_t direction = kInbound | kOutbound);

  /**
   * cleanup
   */
  virtual void Clear() override;

protected:
  // when the main loop is looking to continue expanding we tell it to terminate here
  virtual thor::ExpansionRecommendation ShouldExpand(baldr::GraphReader& graphreader,
                                                     const sif::EdgeLabel& pred,
                                                     const thor::InfoRoutingType route_type) override;

  // tell the expansion how many labels to expect and how many buckets to use
  virtual void GetExpansionHints(uint32_t& bucket_count,
                                 uint32_t& edge_label_reservation) const override;

  uint32_t max_reach_;
};

} // namespace loki
} // namespace valhalla
