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
 * @param pbf_costing_options A mutable protocol buffer where the parsed json values will be stored.
 */
void ParseAutoCostOptions(const rapidjson::Document& doc,
                          const std::string& costing_options_key,
                          CostingOptions* pbf_costing_options);

/**
 * Create an auto route cost method. This is generally shortest time but uses
 * hierarchies and can avoid "shortcuts" through residential areas.
 * @param  options pbf with request options.
 */
cost_ptr_t CreateAutoCost(const CostingOptions& options);

/**
 * Parses the auto_shorter cost options from json and stores values in pbf.
 * @param doc The json request represented as a DOM tree.
 * @param costing_options_key A string representing the location in the DOM tree where the costing
 *                            options are stored.
 * @param pbf_costing_options A mutable protocol buffer where the parsed json values will be stored.
 */
void ParseAutoShorterCostOptions(const rapidjson::Document& doc,
                                 const std::string& costing_options_key,
                                 CostingOptions* pbf_costing_options);

/**
 * Create an auto shorter cost method. This is derived from auto costing and
 * uses the same rules except the edge cost uses an adjusted speed that
 * (non-linearly) reduces the importance of edge speed.
 * @param  options pbf with request options.
 */
cost_ptr_t CreateAutoShorterCost(const CostingOptions& options);

/**
 * Parses the auto_data_fix cost options from json and stores values in pbf.
 * @param doc The json request represented as a DOM tree.
 * @param costing_options_key A string representing the location in the DOM tree where the costing
 *                            options are stored.
 * @param pbf_costing_options A mutable protocol buffer where the parsed json values will be stored.
 */
void ParseAutoDataFixCostOptions(const rapidjson::Document& doc,
                                 const std::string& costing_options_key,
                                 CostingOptions* pbf_costing_options);

/**
 * Create an auto costing method for data fixing. This is derived from auto
 * costing but overrides Allowed rules to allow driving against oneway and
 * it ignores turn restrictions. his can be useful for map-matching traces
 * when trying data that may have incorrect restrictions or oneway information.
 * @param  options pbf with request options.
 */
cost_ptr_t CreateAutoDataFixCost(const CostingOptions& options);

/**
 * Parses the bus cost options from json and stores values in pbf.
 * @param doc The json request represented as a DOM tree.
 * @param costing_options_key A string representing the location in the DOM tree where the costing
 *                            options are stored.
 * @param pbf_costing_options A mutable protocol buffer where the parsed json values will be stored.
 */
void ParseBusCostOptions(const rapidjson::Document& doc,
                         const std::string& costing_options_key,
                         CostingOptions* pbf_costing_options);

/**
 * Create a bus cost method. This is derived from auto costing and
 * uses the same rules except for using the bus access flag instead
 * of the auto access flag.
 * @param  options pbf with request options.
 */
cost_ptr_t CreateBusCost(const CostingOptions& options);

/**
 * Parses the taxi cost options from json and stores values in pbf.
 * @param doc The json request represented as a DOM tree.
 * @param costing_options_key A string representing the location in the DOM tree where the costing
 *                            options are stored.
 * @param pbf_costing_options A mutable protocol buffer where the parsed json values will be stored.
 */
void ParseTaxiCostOptions(const rapidjson::Document& doc,
                          const std::string& costing_options_key,
                          CostingOptions* pbf_costing_options);

/**
 * Create a taxi cost method. This is derived from auto costing and
 * uses the same rules except for favoring taxi roads
 * @param  options pbf with request options.
 */
cost_ptr_t CreateTaxiCost(const CostingOptions& options);

} // namespace sif
} // namespace valhalla

#endif // VALHALLA_SIF_AUTOCOST_H_
