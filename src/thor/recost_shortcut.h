#pragma once

#include "baldr/graphreader.h"
#include "sif/dynamiccost.h"
#include "sif/recost.h"
#include "thor/pathinfo.h"

using namespace valhalla::baldr;
using namespace valhalla::sif;
using namespace valhalla::thor;

namespace {

/**
 * Recover shortcut edge and recost it
 * @param  graphreader  Graph tile reader.
 * @param  shortcut_id  Edge id of the shortcut to be recosted
 * @param  costing      Costing function
 * @param  start_transition_cost  Transition cost for the shortcut edge
 * @param  previous_cost    Total cost from the previous edge
 * @param time_info         Time tracking information about the start of the route
 * @param invariant         Static date_time, dont offset the time as the path lengthens
 * @return  Returns array of PathInfo objects for each supersede edges
 *          respectively. Just append these object to your current path.
 *          In case of fail (shortcut recovering/recosting fail) returns empty array.
 */
inline std::vector<PathInfo> recost_shortcut_forward(GraphReader& graphreader,
                                                     const GraphId& shortcut_id,
                                                     const DynamicCost& costing,
                                                     Cost start_transition_cost,
                                                     Cost previous_cost,
                                                     const TimeInfo& time_info,
                                                     const bool invariant) {
  auto edges = graphreader.RecoverShortcut(shortcut_id);
  if (edges.size() == 1) {
    // failed to recover shortcut edge: it's not necessary to continue
    return {};
  }

  std::vector<PathInfo> path;
  // As soon as recosting function doesn't know anything about the previous edge
  // we should define start cost here and add it to each superseded edge.
  Cost start_cost = previous_cost + start_transition_cost;

  const auto on_new_label = [&path, &start_cost, &start_transition_cost](const EdgeLabel& label) {
    // If it's the first edge -> use already known transition cost between
    // the previous edge and the shortcut.
    Cost transition_cost = path.empty() ? start_transition_cost : label.transition_cost();
    path.emplace_back(label.mode(), start_cost + label.cost(), label.edgeid(), 0,
                      label.restriction_idx(), transition_cost);
  };

  auto edge_iter = edges.begin();
  const auto get_next_edge = [&edge_iter, &edges]() -> GraphId {
    if (edge_iter == edges.end())
      return {};

    return *edge_iter++;
  };

  const GraphTile* tile = nullptr;
  const GraphId node_id = graphreader.edge_startnode(edges.front(), tile);
  if (!node_id.Is_Valid()) {
    LOG_ERROR("Failed to recost shortcut edge: node cannot be found");
    return {};
  }
  const auto* node = graphreader.nodeinfo(node_id, tile);
  if (!node) {
    LOG_ERROR("Failed to recost shortcut edge: node cannot be found");
    return {};
  }

  // calculate time when we enter shortcut edge considering 'invariant' option;
  // this time is a start time for recost function
  const auto seconds_offset = invariant ? 0.f : start_cost.secs;
  const auto offset_time = time_info.forward(seconds_offset, static_cast<int>(node->timezone()));

  try {
    // recost all superseded edges
    recost_forward(graphreader, costing, get_next_edge, on_new_label, 0.f, 1.f, offset_time,
                   invariant);
  } catch (const std::exception& e) {
    LOG_ERROR(std::string("Failed to recost shortcut edge: ") + e.what());
    return {};
  } catch (...) {
    LOG_ERROR("Failed to recost shortcut edge: unknown exception");
    return {};
  }

  return path;
}

} // namespace
