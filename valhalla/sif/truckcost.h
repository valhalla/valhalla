#ifndef VALHALLA_SIF_TRUCKCOST_H_
#define VALHALLA_SIF_TRUCKCOST_H_

#include <cstdint>
#include <valhalla/baldr/rapidjson_utils.h>
#include <valhalla/proto/directions_options.pb.h>
#include <valhalla/sif/dynamiccost.h>

namespace valhalla {
namespace sif {

/**
 * Parses the truck cost options from json and stores values in pbf.
 */
void ParseTruckCostOptions(const rapidjson::Document& doc,
                           const std::string& costing_options_key,
                           odin::CostingOptions* pbf_costing_options);

/**
 * Create a truckcost
 * @param  config  Property tree with configuration / options.
 */
cost_ptr_t CreateTruckCost(const boost::property_tree::ptree& config);

} // namespace sif
} // namespace valhalla

#endif // VALHALLA_SIF_TRUCKCOST_H_
