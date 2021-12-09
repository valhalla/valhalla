#pragma once
#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/time_info.h>
#include <valhalla/sif/dynamiccost.h>
#include <valhalla/sif/edgelabel.h>

#include <functional>

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
 * @param time_info         the time tracking information representing the local time before
 *                          traversing the first edge
 * @param invariant         static date_time, dont offset the time as the path lengthens
 * @param ignore_access     ignore access restrictions for edges and nodes if it's true
 */
void recost_forward(baldr::GraphReader& reader,
                    const sif::DynamicCost& costing,
                    const EdgeCallback& edge_cb,
                    const LabelCallback& label_cb,
                    float source_pct = 0.f,
                    float target_pct = 1.f,
                    const baldr::TimeInfo& time_info = baldr::TimeInfo::invalid(),
                    const bool invariant = false,
                    const bool ignore_access = false);
} // namespace sif
} // namespace valhalla
