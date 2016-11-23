#include <vector>
#include <algorithm>
#include <exception>
#include <valhalla/midgard/logging.h>
#include <valhalla/baldr/errorcode_util.h>

#include "thor/expandfromnode.h"
#include "thor/service.h"

using namespace valhalla::baldr;
using namespace valhalla::sif;
using namespace valhalla::thor;


namespace valhalla {
namespace thor {

/*
 * An “edge-walking” method for use when the input shape is exact
 * shape from a prior Valhalla route. This will walk the input shape
 * and compare to Valhalla edge’s end node positions to form the list of edges.
 *
 */
bool ExpandFromNode::FormPath(const std::shared_ptr<sif::DynamicCost>* mode_costing,
                              const TravelMode& mode,
                              GraphReader& reader,
                              const std::vector<midgard::PointLL>& shape,
                              size_t& correlated_index,
                              const GraphTile* tile, const GraphId& node,
                              const GraphId& stop_node,
                              EdgeLabel& prev_edge_label,
                              float& elapsed_time,
                              std::vector<PathInfo>& path_infos,
                              const bool from_transition) {

  // If node equals stop node then when are done expanding
  if (node == stop_node) {
    return true;
  }

  const NodeInfo* node_info = tile->node(node);
  GraphId edge_id(node.tileid(), node.level(), node_info->edge_index());
  const DirectedEdge* de = tile->directededge(node_info->edge_index());
  for (uint32_t i = 0; i < node_info->edge_count(); i++, de++, edge_id++) {
    // Skip shortcuts
    if (de->is_shortcut()) {
      continue;
    }

    // Process transition edge if previous edge was not from a transition
    if (de->trans_down() || de->trans_up()) {
      if (from_transition) {
        continue;
      } else {
        const GraphTile* end_node_tile = reader.GetGraphTile(de->endnode());
        if (end_node_tile == nullptr) {
          continue;
        }
        if (FormPath(mode_costing, mode, reader, shape, correlated_index, end_node_tile, de->endnode(),
                             stop_node, prev_edge_label, elapsed_time, path_infos, true)) {
          return true;
        } else {
          continue;
        }
      }
    }

    // Initialize index and shape
    size_t index = correlated_index;
    uint32_t shape_length = 0;
    uint32_t de_length = de->length() + 50; // TODO make constant

    const GraphTile* end_node_tile = reader.GetGraphTile(de->endnode());
    if (end_node_tile == nullptr) {
      continue;
    }
    PointLL de_end_ll = end_node_tile->node(de->endnode())->latlng();

    // Process current edge until shape matches end node
    // or shape length is longer than the current edge
    while (index < shape.size()
        && (std::round(shape.at(correlated_index).Distance(shape.at(index)))
            < de_length)) {
      if (shape.at(index).ApproximatelyEqual(de_end_ll)) {
        // Update the elapsed time based on transition cost
        elapsed_time += mode_costing[static_cast<int>(mode)]->TransitionCost(
            de, node_info, prev_edge_label).secs;

        // Update the elapsed time based on edge cost
        elapsed_time += mode_costing[static_cast<int>(mode)]->EdgeCost(de).secs;

        // Add edge and update correlated index
        path_infos.emplace_back(mode, std::round(elapsed_time), edge_id, 0);

        // Set previous edge label
        prev_edge_label = {kInvalidLabel, edge_id, de, {}, 0, 0, mode, 0};

        // Continue walking shape to find the end edge...
        return (FormPath(mode_costing, mode, reader, shape, index, end_node_tile, de->endnode(),
                                 stop_node, prev_edge_label, elapsed_time, path_infos, false));

      }
      index++;
    }
  }
  return false;
}

}
}
