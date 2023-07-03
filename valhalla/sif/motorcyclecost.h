#ifndef VALHALLA_SIF_MOTORCYCLECOST_H_
#define VALHALLA_SIF_MOTORCYCLECOST_H_

#include <cstdint>

#include <boost/property_tree/ptree.hpp>
#include <valhalla/baldr/rapidjson_utils.h>
#include <valhalla/proto/options.pb.h>
#include <valhalla/sif/dynamiccost.h>

namespace valhalla {
namespace sif {

/**
 * Parses the motorcycle cost options from json and stores values in pbf.
 * @param doc The json request represented as a DOM tree.
 * @param costing_options_key A string representing the location in the DOM tree where the costing
 *                            options are stored.
 * @param pbf_costing         A mutable protocol buffer where the parsed json values will be stored.
 */
void ParseMotorcycleCostOptions(const rapidjson::Document& doc,
                                const std::string& costing_options_key,
                                Costing* pbf_costing);

/**
 * Create motorcycle cost method. This is derived from auto costing and
 * uses the same rules except for some different access restrictions
 * and the tendency to avoid hills
 * @param  costing pbf with request options.
 */
cost_ptr_t CreateMotorcycleCost(const Costing& costing);

} // namespace sif
} // namespace valhalla

#endif // VALHALLA_SIF_MOTORCYCLECOST_H_
