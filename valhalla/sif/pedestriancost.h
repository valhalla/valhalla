#ifndef VALHALLA_SIF_PEDESTRIANCOST_H_
#define VALHALLA_SIF_PEDESTRIANCOST_H_

#include <memory>

#include <valhalla/baldr/rapidjson_fwd.h>

namespace valhalla {
class Costing;
namespace sif {
class DynamicCost;
using cost_ptr_t = std::shared_ptr<DynamicCost>;

/**
 * Parses the pedestrian cost options from json and stores values in pbf.
 * @param doc The json request represented as a DOM tree.
 * @param costing_options_key A string representing the location in the DOM tree where the costing
 *                            options are stored.
 * @param pbf_costing A mutable protocol buffer where the parsed json values will be stored.
 */
void ParsePedestrianCostOptions(const rapidjson::Document& doc,
                                const std::string& costing_options_key,
                                Costing* pbf_costing);

/**
 * Create a pedestriancost
 * @param  options pbf with request options.
 */
cost_ptr_t CreatePedestrianCost(const Costing& costing);

cost_ptr_t CreateBikeShareCost(const Costing& costing);

} // namespace sif
} // namespace valhalla

#endif // VALHALLA_SIF_PEDESTRIANCOST_H_
