#pragma once
#include <valhalla/baldr/double_bucket_queue.h>
#include <valhalla/baldr/time_info.h>
#include <valhalla/proto/api.pb.h>
#include <valhalla/sif/dynamiccost.h>
#include <valhalla/sif/edgelabel.h>

#include <functional>
#include <utility>

namespace valhalla {
namespace sif {

// edge id, mode of travel, how much of the edge is used as a percentage from 0 to 1
using PathEdge = std::tuple<baldr::GraphId, TravelMode, float>;
// what this function calls to get the next edge
using EdgeCallback = std::function<PathEdge(void)>;
// what this function calls to emit the next label
using LabelCallback = std::function<void(const EdgeLabel& label)>;

/**
 * Will take a sequence of edges and create the set of edge labels that would represent it
 * Allows for the caller to essentially re-compute the costing of a given path
 *
 * @param reader            used to get access to graph data. modifyable because its got a cache
 * @param origin            the location on the first edge of the path. modifyable because current
 *                          time recosting requires the setting of a date and time
 * @param destination       the location on the last edge of the path
 * @param mode_costing      per travel mode costing objects to be used for costing computations
 * @param edge_cb           the callback used to get each edge in the path
 * @param label_cb          the callback used to emit each label in the path
 */
void recost_forward(baldr::GraphReader& reader,
                    const std::shared_ptr<DynamicCost>* mode_costing,
                    std::string date_time,
                    const EdgeCallback& edge_cb,
                    const LabelCallback& label_cb) {
  // graph the first path edge
  baldr::GraphId edge_id;
  TravelMode mode;
  float edge_pct;
  std::tie(edge_id, mode, edge_pct) = edge_cb();

  // fetch the graph objects
  const baldr::GraphTile* tile = nullptr;
  const baldr::DirectedEdge* edge = reader.directededge(edge_id, tile);
  const baldr::NodeInfo* node = edge ? reader.nodeinfo(edge->endnode(), tile) : nullptr;

  // setup the time tracking
  baldr::DateTime::tz_sys_info_cache_t tz_cache;
  baldr::TimeInfo time_info =
      baldr::TimeInfo::make(date_time, node ? node->timezone() : 0, &tz_cache);
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
    if (edge && !node)
      throw std::runtime_error("Cannot recost path, node cannot be found");
    // grab the edge
    edge = reader.directededge(edge_id, tile);
    if (!edge)
      throw std::runtime_error("Cannot recost path, edge cannot be found");

    // TODO: if this edge begins a restriction, we need to start popping off edges into queue
    // so that we can find if we reach the end of the restriction. then we need to replay the
    // queued edges as normal
    bool time_restrictions_TODO = false;

    // update the time
    auto ti = node ? time_info.forward(cost.secs, node->timezone()) : time_info;
    // grab the costing for this mode
    auto costing = mode_costing[static_cast<uint8_t>(mode)];
    // if we've never set up the time information before we need to do it now
    Cost transition_cost = node ? costing->TransitionCost(edge, node, label) : Cost{};
    // TODO: we use a different costing when its transit
    // update the cost to the end of this edge
    cost += transition_cost + costing->EdgeCost(edge, tile, ti.second_of_week) * edge_pct;
    // update the length to the end of this edge
    length += edge->length() * edge_pct;
    // construct the label
    label = EdgeLabel(predecessor++, edge_id, edge, cost, cost.cost, 0, mode, length, transition_cost,
                      time_restrictions_TODO);
    // hand back the label
    label_cb(label);
    // next edge
    std::tie(edge_id, mode, edge_pct) = edge_cb();
  }
}

} // namespace sif
} // namespace valhalla