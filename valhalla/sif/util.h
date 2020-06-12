#pragma once
#include <valhalla/baldr/double_bucket_queue.h>
#include <valhalla/proto/api.pb.h>
#include <valhalla/sif/dynamiccost.h>
#include <valhalla/sif/edgelabel.h>

#include <functional>

namespace valhalla {
namespace sif {

using PathEdge = std::pair<baldr::GraphId, TravelMode>;
using PathEdgeCallback = std::function<PathEdge(void)>;
using EdgeLabelCallback = std::function<void(const EdgeLabel& label)>;
void recost(const valhalla::Location& origin,
            const valhalla::Location& destination,
            const std::shared_ptr<DynamicCost>* mode_costing,
            const PathEdgeCallback& edge_cb,
            const EdgeLabelCallback& label_cb) {
  // keep grabbing edges while we get valid ids
  PathEdge edge;
  EdgeLabel label;
  while ((edge = edge_cb()).first.Is_Valid()) {
    // is this the first edge
    auto prev_label = label;
    if (prev_label.predecessor() == baldr::kInvalidLabel) {
      // edge cost * partial distance
    } // this is not the first edge
    else {
      // time update
      // turn cost
      // edge cost * partial distance if its the last edge by id assuming no loops?
    }
    label_cb(label);
  }
}

} // namespace sif
} // namespace valhalla