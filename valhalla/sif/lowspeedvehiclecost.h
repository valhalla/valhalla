#ifndef VALHALLA_LOWSPEEDVEHICLECOST_H
#define VALHALLA_LOWSPEEDVEHICLECOST_H

#include <cstdint>

#include <boost/property_tree/ptree.hpp>
#include <valhalla/baldr/rapidjson_utils.h>
#include <valhalla/proto/options.pb.h>
#include <valhalla/sif/dynamiccost.h>

namespace valhalla {
namespace sif {

/**
 * Parses the low_speed_vehicle cost options from json and stores values in pbf.
 * @param doc The json request represented as a DOM tree.
 * @param costing_options_key A string representing the location in the DOM tree where the costing
 *                            options are stored.
 * @param pbf_costing A mutable protocol buffer where the parsed json values will be stored.
 */
void ParseLowSpeedVehicleCostOptions(const rapidjson::Document& doc,
                                     const std::string& costing_options_key,
                                     Costing* pbf_costing);

/**
 * Create low-speed vehicle costing method.
 * @param  options pbf with request options.
 */
cost_ptr_t CreateLowSpeedVehicleCost(const Costing& costing);

} // namespace sif
} // namespace valhalla


#endif // VALHALLA_LOWSPEEDVEHICLECOST_H
