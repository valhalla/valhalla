#include "sif/recost.h"
#include "baldr/time_info.h"

namespace valhalla {
namespace sif {

// what this function calls to get the next edge
using EdgeCallback = std::function<baldr::GraphId(void)>;
// what this function calls to emit the next label
using LabelCallback = std::function<void(const EdgeLabel& label)>;

/**
 * Will take a sequence of edges and create the set of edge labels that would represent it
 * Allows for the caller to essentially re-compute the costing of a given path
 *
 * @param reader            used to get access to graph data. modifyable because its got a cache
 * @param costing           single costing object to be used for costing/access computations
 * @param edge_cb           the callback used to get each edge in the path
 * @param label_cb          the callback used to emit each label in the path
 * @param source_pct        the percent along the initial edge the source location is
 * @param target_pct        the percent along the final edge the target location is
 * @param date_time         the time string representing the local time before traversing the first
 *                          edge. of the format YYYY-MM-DDTHH:mm
 */
void recost_forward(baldr::GraphReader& reader,
                    const sif::DynamicCost& costing,
                    const EdgeCallback& edge_cb,
                    const LabelCallback& label_cb,
                    float source_pct,
                    float target_pct,
                    const std::string& date_time) {
  // out of bounds edge scaling
  if (source_pct < 0.f || source_pct > 1.f || target_pct < 0.f || target_pct > 1.f) {
    throw std::logic_error("Source and target percentages must be between 0 and 1 inclusive");
  }

  // grab the first path edge
  baldr::GraphId edge_id = edge_cb();
  if (!edge_id.Is_Valid()) {
    return;
  }

  // fetch the graph objects
  const baldr::GraphTile* tile = nullptr;
  const baldr::DirectedEdge* edge = reader.directededge(edge_id, tile);
  const baldr::NodeInfo* node = edge ? reader.nodeinfo(edge->endnode(), tile) : nullptr;

  // first edge is bogus
  if (!edge) {
    throw std::runtime_error("Edge cannot be found");
  }

  // fail if the first edge is filtered
  if (costing.Filter(edge, edge_id, tile) == 0.f) {
    throw std::runtime_error("This path requires different edge access than this costing allows");
  }

  // setup the time tracking
  baldr::DateTime::tz_sys_info_cache_t tz_cache;
  std::string dt = date_time;
  baldr::TimeInfo time_info = baldr::TimeInfo::make(dt, node ? node->timezone() : 0, &tz_cache);
  edge = nullptr;
  node = nullptr;

  // keep grabbing edges while we get valid ids
  EdgeLabel label;
  uint32_t predecessor = baldr::kInvalidLabel;
  Cost cost{};
  double length = 0;

  while (edge_id.Is_Valid()) {
    // get the previous edges node
    node = edge ? reader.nodeinfo(edge->endnode(), tile) : nullptr;
    if (edge && !node) {
      throw std::runtime_error("Node cannot be found");
    }

    // this node is not allowed
    if (node && !costing.Allowed(node)) {
      throw std::runtime_error("This path requires different node access than this costing allows");
    }

    // grab the edge
    edge = reader.directededge(edge_id, tile);
    if (!edge) {
      throw std::runtime_error("Edge cannot be found");
    }

    // update the time
    auto ti = node ? time_info.forward(cost.secs, node->timezone()) : time_info;

    // TODO: if this edge begins a restriction, we need to start popping off edges into queue
    // so that we can find if we reach the end of the restriction. then we need to replay the
    // queued edges as normal
    int time_restrictions_TODO = -1;

    // this edge is not allowed
    if (predecessor != baldr::kInvalidLabel &&
        !costing.Allowed(edge, label, tile, edge_id, ti.local_time, node->timezone(),
                         time_restrictions_TODO)) {
      throw std::runtime_error("This path requires different edge access than this costing allows");
    }

    // how much of the edge will we use, trim if its the first or last edge
    float edge_pct = 1.f;
    if (source_pct != -1) {
      edge_pct -= source_pct;
      source_pct = -1;
    }
    auto next_id = edge_cb();
    if (!next_id.Is_Valid()) {
      edge_pct -= 1.f - target_pct;
    }

    // the cost for traversing this intersection
    Cost transition_cost = node ? costing.TransitionCost(edge, node, label) : Cost{};
    // update the cost to the end of this edge
    cost += transition_cost + costing.EdgeCost(edge, tile, ti.second_of_week) * edge_pct;
    // update the length to the end of this edge
    length += edge->length() * edge_pct;
    // construct the label
    label = EdgeLabel(predecessor++, edge_id, edge, cost, cost.cost, 0, costing.travel_mode(), length,
                      transition_cost, time_restrictions_TODO);
    // hand back the label
    label_cb(label);
    // next edge
    edge_id = next_id;
  }
}

} // namespace sif
} // namespace valhalla
