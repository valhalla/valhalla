#ifndef VALHALLA_LOKI_LINEAR_COST_FACTORS_H_
#define VALHALLA_LOKI_LINEAR_COST_FACTORS_H_

#include <valhalla/baldr/graphreader.h>
#include <valhalla/proto/options.pb.h>
#include <valhalla/sif/dynamiccost.h>

namespace valhalla {
namespace loki {

/**
 * Resolve linear cost features into edge ID with a range and a cost factor by edge
 * walking the graph.
 *
 * @param mode_costing        costing used to edge walk the lines
 * @param mode                travel mode
 * @param reader              GraphReader instance
 * @param options             the request options
 * @param min_allowed_factor  the smallest factor the config admits
 * @param max_allowed_edges   the max number of edges the config allows
 */
void add_cost_factor_edges(const sif::mode_costing_t& mode_costing,
                           const sif::TravelMode& mode,
                           baldr::GraphReader& reader,
                           valhalla::Options& options,
                           double min_allowed_factor,
                           uint64_t max_allowed_edges);

} // namespace loki
} // namespace valhalla

#endif // VALHALLA_LOKI_LINEAR_COST_FACTORS_H_
