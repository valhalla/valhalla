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
 * TODO: add time information
 * @return  Returns array of PathInfo objects for each supersede edges
 *          respectively. Just append these object to your current path.
 *          In case of fail (shortcut recovering/recosting fail) returns empty array.
 */
inline std::vector<PathInfo> recost_shortcut_forward(GraphReader& graphreader,
                                                     const GraphId& shortcut_id,
                                                     const DynamicCost& costing,
                                                     Cost start_transition_cost,
                                                     Cost previous_cost) {
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

  try {
    // recost all superseded edges
    recost_forward(graphreader, costing, get_next_edge, on_new_label);
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
