#ifndef VALHALLA_SIF_AUTOCOST_H_
#define VALHALLA_SIF_AUTOCOST_H_

#include <cstdint>

#include <boost/property_tree/ptree.hpp>
#include <valhalla/baldr/rapidjson_utils.h>
#include <valhalla/proto/options.pb.h>
#include <valhalla/sif/dynamiccost.h>

namespace valhalla {
namespace sif {

/**
 * Parses the auto cost options from json and stores values in pbf.
 * @param doc The json request represented as a DOM tree.
 * @param costing_options_key A string representing the location in the DOM tree where the costing
 *                            options are stored.
 * @param co A mutable protocol buffer where the parsed json values will be stored.
 */
void ParseAutoCostOptions(const rapidjson::Document& doc,
                          const std::string& costing_options_key,
                          Costing* pbf_costing);

/**
 * Create an auto route cost method. This is generally shortest time but uses
 * hierarchies and can avoid "shortcuts" through residential areas.
 * @param  costing pbf with request options.
 */
cost_ptr_t CreateAutoCost(const Costing& costing);

/**
 * Parses the bus cost options from json and stores values in pbf.
 * @param doc The json request represented as a DOM tree.
 * @param costing_options_key A string representing the location in the DOM tree where the costing
 *                            options are stored.
 * @param pbf_costing_options A mutable protocol buffer where the parsed json values will be stored.
 */
void ParseBusCostOptions(const rapidjson::Document& doc,
                         const std::string& costing_options_key,
                         Costing* pbf_costing);

/**
 * Create a bus cost method. This is derived from auto costing and
 * uses the same rules except for using the bus access flag instead
 * of the auto access flag.
 * @param  costing pbf with request options.
 */
cost_ptr_t CreateBusCost(const Costing& costing);

/**
 * Parses the taxi cost options from json and stores values in pbf.
 * @param doc The json request represented as a DOM tree.
 * @param costing_options_key A string representing the location in the DOM tree where the costing
 *                            options are stored.
 * @param pbf_costing         A mutable protocol buffer where the parsed json values will be stored.
 */
void ParseTaxiCostOptions(const rapidjson::Document& doc,
                          const std::string& costing_options_key,
                          Costing* pbf_costing);

/**
 * Create a taxi cost method. This is derived from auto costing and
 * uses the same rules except for favoring taxi roads
 * @param  costing pbf with request options.
 */
cost_ptr_t CreateTaxiCost(const Costing& costing);

} // namespace sif
} // namespace valhalla

#endif // VALHALLA_SIF_AUTOCOST_H_
