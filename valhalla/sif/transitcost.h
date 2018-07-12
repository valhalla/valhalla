#ifndef VALHALLA_SIF_TRANSITCOST_H_
#define VALHALLA_SIF_TRANSITCOST_H_

#include <cstdint>
#include <valhalla/baldr/rapidjson_utils.h>
#include <valhalla/proto/directions_options.pb.h>
#include <valhalla/sif/dynamiccost.h>

namespace valhalla {
namespace sif {

/**
 * Parses the transit cost options from json and stores values in pbf.
 */
void ParseTransitCostOptions(const rapidjson::Document& doc,
                             const std::string& costing_options_key,
                             odin::CostingOptions* pbf_costing_options);

/**
 * Create a transit cost object.
 * @param  config  Property tree with configuration / options.
 */
cost_ptr_t CreateTransitCost(const boost::property_tree::ptree& config);

} // namespace sif
} // namespace valhalla

#endif // VALHALLA_SIF_TRANSITCOST_H_
