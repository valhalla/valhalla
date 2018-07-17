#ifndef VALHALLA_SIF_BICYCLECOST_H_
#define VALHALLA_SIF_BICYCLECOST_H_

#include <cstdint>
#include <valhalla/baldr/rapidjson_utils.h>
#include <valhalla/proto/directions_options.pb.h>
#include <valhalla/sif/dynamiccost.h>

namespace valhalla {
namespace sif {

/**
 * Parses the bicycle cost options from json and stores values in pbf.
 * @param doc The json request represented as a DOM tree.
 * @param costing_options_key A string representing the location in the DOM tree where the costing
 *                            options are stored.
 * @param pbf_costing_options A mutable protocol buffer where the parsed json values will be stored.
 */
void ParseBicycleCostOptions(const rapidjson::Document& doc,
                             const std::string& costing_options_key,
                             odin::CostingOptions* pbf_costing_options);

/**
 * Create a bicyclecost
 * @param  config  Property tree with configuration / options.
 */
cost_ptr_t CreateBicycleCost(const boost::property_tree::ptree& config);

} // namespace sif
} // namespace valhalla

#endif // VALHALLA_SIF_BICYCLECOST_H_
