#ifndef VALHALLA_SIF_TRANSITCOST_H_
#define VALHALLA_SIF_TRANSITCOST_H_

#include <cstdint>
#include <valhalla/baldr/rapidjson_utils.h>
#include <valhalla/proto/options.pb.h>
#include <valhalla/sif/dynamiccost.h>

namespace valhalla {
namespace sif {

/**
 * Parses the transit cost options from json and stores values in pbf.
 * @param doc The json request represented as a DOM tree.
 * @param costing_options_key A string representing the location in the DOM tree where the costing
 *                            options are stored.
 * @param pbf_costing_options A mutable protocol buffer where the parsed json values will be stored.
 */
void ParseTransitCostOptions(const rapidjson::Document& doc,
                             const std::string& costing_options_key,
                             CostingOptions* pbf_costing_options);

/**
 * Create a transit cost object.
 * @param  costing specified costing type.
 * @param  options pbf with request options.
 */
cost_ptr_t CreateTransitCost(const Costing costing, const Options& options);

} // namespace sif
} // namespace valhalla

#endif // VALHALLA_SIF_TRANSITCOST_H_
