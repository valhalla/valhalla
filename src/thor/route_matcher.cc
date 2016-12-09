#include <vector>
#include <algorithm>
#include <exception>
#include <valhalla/midgard/logging.h>
#include <valhalla/baldr/errorcode_util.h>

#include "thor/route_matcher.h"
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

namespace {

// Minimum tolerance for edge length
constexpr float kMinLengthTolerance = 10.0f;

// Get the length to compare to the edge length
float length_comparison(const float length, const bool exact_match) {
  // Alter tolerance based on exact_match flag
  float t, max_t;
  if (exact_match) {
    t = length * 1.05f;
    max_t = 25.0f;
  } else {
    t = length * 1.1f;
    max_t = 100.0f;
  }
  float tolerance = (t < kMinLengthTolerance) ? kMinLengthTolerance :
                      (t > max_t) ? max_t : t;
  return length + tolerance;
}

const PathLocation::PathEdge* find_begin_edge(
    const std::vector<baldr::PathLocation>& correlated) {
  // Iterate through start edges
  for (const auto& edge : correlated.front().edges) {
    // If origin is at a node - skip any inbound edge
    if (edge.end_node()) {
      continue;
    }
    return &edge;  //TODO special case
  }
  return nullptr;
}

const PathLocation::PathEdge* find_end_edge(
    const std::vector<baldr::PathLocation>& correlated) {
  // Iterate through end edges
  for (const auto& edge : correlated.back().edges) {
    // If destination is at a node - skip any outbound edge
    if (edge.begin_node()) {
      continue;
    }
    return &edge;  //TODO special case
  }
  return nullptr;
}

const GraphId find_start_node(GraphReader& reader, const GraphId& edge_id) {
  const GraphTile* tile = reader.GetGraphTile(edge_id);
  if (tile == nullptr) {
    throw std::runtime_error("Tile is null");
  }
  const DirectedEdge* de = tile->directededge(edge_id);

  GraphId opp_edge_id = tile->GetOpposingEdgeId(de);
  const DirectedEdge* opp_de = tile->directededge(opp_edge_id);

  return opp_de->endnode();
}

bool expand_from_node(const std::shared_ptr<sif::DynamicCost>* mode_costing,
                      const TravelMode& mode, GraphReader& reader,
                      const std::vector<midgard::PointLL>& shape,
                      const std::vector<float>& distances,
                      size_t& correlated_index, const GraphTile* tile,
                      const GraphId& node, const GraphId& stop_node,
                      EdgeLabel& prev_edge_label, float& elapsed_time,
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
    // Skip shortcuts and transit connection edges
    // TODO - later might allow transit connections for multi-modal
    if (de->is_shortcut() || de->use() == Use::kTransitConnection) {
      continue;
    }

    // Look back in path_infos by 1-2 edges to make sure we aren't in a loop.
    // A loop can occur if we have edges shorter than the lat,lng tolerance.
    uint32_t n = path_infos.size();
    if (n > 1 &&  (edge_id == path_infos[n-2].edgeid ||
        edge_id == path_infos[n-1].edgeid)) {
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
        if (expand_from_node(mode_costing, mode, reader, shape, distances,
                             correlated_index, end_node_tile, de->endnode(),
                             stop_node, prev_edge_label, elapsed_time,
                             path_infos, true)) {
          return true;
        } else {
          continue;
        }
      }
    }

    // Get the end node LL and set up the length comparison
    const GraphTile* end_node_tile = reader.GetGraphTile(de->endnode());
    if (end_node_tile == nullptr) {
      continue;
    }
    PointLL de_end_ll = end_node_tile->node(de->endnode())->latlng();
    float de_length = length_comparison(de->length(), true);

    // Process current edge until shape matches end node
    // or shape length is longer than the current edge. Increment to the
    // next shape point after the correlated index.
    size_t index = correlated_index + 1;
    float length = 0.0f;
    while (index < shape.size()) {
      // Exclude edge if length along shape is longer than the edge length
      length += distances.at(index);
      if (length > de_length) {
        break;
      }

      // Found a match if shape equals directed edge LL within tolerance
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
        if (expand_from_node(mode_costing, mode, reader, shape, distances,
                                 index, end_node_tile, de->endnode(),
                                 stop_node, prev_edge_label, elapsed_time,
                                 path_infos, false)) {
          return true;
        } else {
          // Match failed along this edge, pop the last entry off path_infos
          // and try the next edge
          path_infos.pop_back();
          break;
        }
      }
      index++;
    }
  }
  return false;
}

}

bool RouteMatcher::FormPath(
    const std::shared_ptr<sif::DynamicCost>* mode_costing,
    const sif::TravelMode& mode, baldr::GraphReader& reader,
    const std::vector<midgard::PointLL>& shape,
    const std::vector<baldr::PathLocation>& correlated,
    std::vector<PathInfo>& path_infos) {
  float elapsed_time = 0.f;

  // Process and validate begin edge
  const PathLocation::PathEdge* begin_path_edge = find_begin_edge(correlated);
  if ((begin_path_edge == nullptr) || !(begin_path_edge->id.Is_Valid())) {
    throw std::runtime_error("Invalid begin edge id");
  }
  const GraphTile* begin_edge_tile = reader.GetGraphTile(begin_path_edge->id);
  if (begin_edge_tile == nullptr) {
    throw std::runtime_error("Begin tile is null");
  }

  // Process and validate end edge
  const PathLocation::PathEdge* end_path_edge = find_end_edge(correlated);
  if ((end_path_edge == nullptr) || !(end_path_edge->id.Is_Valid())) {
    throw std::runtime_error("Invalid end edge id");
  }
  const GraphTile* end_edge_tile = reader.GetGraphTile(end_path_edge->id);
  if (end_edge_tile == nullptr) {
    throw std::runtime_error("End tile is null");
  }

  // Assign the end edge start node
  const GraphId end_edge_start_node = find_start_node(reader,
                                                      end_path_edge->id);

  // Process directed edge and info
  const DirectedEdge* de = begin_edge_tile->directededge(begin_path_edge->id);
  const GraphTile* end_node_tile = reader.GetGraphTile(de->endnode());
  if (begin_edge_tile == nullptr) {
    throw std::runtime_error("End node tile is null");
  }
  PointLL de_end_ll = end_node_tile->node(de->endnode())->latlng();

  // Form distances between shape points
  std::vector<float> distances;
  distances.push_back(0.0f);
  for (size_t i = 1; i < shape.size(); i++) {
    distances.push_back(shape[i].Distance(shape[i-1]));
  }

  // Initialize indexes and shape
  size_t index = 0;
  float length = 0.0f;
  float de_length = length_comparison(de->length() * (1 - begin_path_edge->dist), true);
  EdgeLabel prev_edge_label;
  // Loop over shape to form path from matching edges
  while (index < shape.size()) {
    if (index > 0) {
      length += distances.at(index);
    }
    if (length > de_length) {
      break;
    }

    // Check if shape is within tolerance at the end node
    if (shape.at(index).ApproximatelyEqual(de_end_ll)) {

      // Update the elapsed time edge cost at begin edge
      elapsed_time += mode_costing[static_cast<int>(mode)]->EdgeCost(de).secs
          * (1 - begin_path_edge->dist);

      // Add begin edge
      path_infos.emplace_back(mode, std::round(elapsed_time),
                              begin_path_edge->id, 0);

      // Set previous edge label
      prev_edge_label = {kInvalidLabel, begin_path_edge->id, de, {}, 0, 0, mode, 0};

      // Continue walking shape to find the end edge...
      if (expand_from_node(mode_costing, mode, reader, shape, distances, index,
                           end_node_tile, de->endnode(), end_edge_start_node,
                           prev_edge_label, elapsed_time, path_infos, false)) {
        // Update the elapsed time based on transition cost
        elapsed_time += mode_costing[static_cast<int>(mode)]->TransitionCost(
            de, end_edge_tile->node(end_edge_start_node), prev_edge_label).secs;

        // Update the elapsed time based on edge cost
        elapsed_time += mode_costing[static_cast<int>(mode)]->EdgeCost(de).secs
            * end_path_edge->dist;

        // Add end edge
        path_infos.emplace_back(mode, std::round(elapsed_time),
                                end_path_edge->id, 0);

        return true;
      } else {
        // Did not find end edge - so get out
        return false;
      }
    }
    index++;
  }
  return false;
}

}
}
