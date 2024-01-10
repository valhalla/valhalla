#ifndef VALHALLA_GOLFCARTCOST_H
#define VALHALLA_GOLFCARTCOST_H

#include <cstdint>

#include <boost/property_tree/ptree.hpp>
#include <valhalla/baldr/rapidjson_utils.h>
#include <valhalla/proto/options.pb.h>
#include <valhalla/sif/dynamiccost.h>

namespace valhalla {
namespace sif {

/**
 * Parses the golf_cart cost options from json and stores values in pbf.
 * @param doc The json request represented as a DOM tree.
 * @param costing_options_key A string representing the location in the DOM tree where the costing
 *                            options are stored.
 * @param pbf_costing A mutable protocol buffer where the parsed json values will be stored.
 */
void ParseGolfCartCostOptions(const rapidjson::Document& doc,
                              const std::string& costing_options_key,
                              Costing* pbf_costing);

/**
 * Create golf cart cost method.
 * @param  options pbf with request options.
 */
cost_ptr_t CreateGolfCartCost(const Costing& costing);

} // namespace sif
} // namespace valhalla


#endif // VALHALLA_GOLFCARTCOST_H
