#ifndef VALHALLA_SIF_AUTOCOST_H_
#define VALHALLA_SIF_AUTOCOST_H_

#include <cstdint>

#include <boost/property_tree/ptree.hpp>
#include <valhalla/baldr/rapidjson_utils.h>
#include <valhalla/proto/directions_options.pb.h>
#include <valhalla/sif/dynamiccost.h>

namespace valhalla {
namespace sif {

/**
 * Parses the auto cost options from json and stores values in pbf.
 */
void ParseAutoCostOptions(const rapidjson::Document& doc,
                          const std::string& costing_options_key,
                          odin::CostingOptions* pbf_costing_options);

/**
 * Create an auto route cost method. This is generally shortest time but uses
 * hierarchies and can avoid "shortcuts" through residential areas.
 */
cost_ptr_t CreateAutoCost(const boost::property_tree::ptree& config);

/**
 * Parses the auto_shorter cost options from json and stores values in pbf.
 */
void ParseAutoShorterCostOptions(const rapidjson::Document& doc,
                                 const std::string& costing_options_key,
                                 odin::CostingOptions* pbf_costing_options);

/**
 * Create an auto shorter cost method. This is derived from auto costing and
 * uses the same rules except the edge cost uses an adjusted speed that
 * (non-linearly) reduces the importance of edge speed.
 */
cost_ptr_t CreateAutoShorterCost(const boost::property_tree::ptree& config);

/**
 * Parses the auto_data_fix cost options from json and stores values in pbf.
 */
void ParseAutoDataFixCostOptions(const rapidjson::Document& doc,
                                 const std::string& costing_options_key,
                                 odin::CostingOptions* pbf_costing_options);

/**
 * Create an auto costing method for data fixing. This is derived from auto
 * costing but overrides Allowed rules to allow driving against oneway and
 * it ignores turn restrictions. his can be useful for map-matching traces
 * when trying data that may have incorrect restrictions or oneway information.
 */
cost_ptr_t CreateAutoDataFixCost(const boost::property_tree::ptree& config);

/**
 * Parses the bus cost options from json and stores values in pbf.
 */
void ParseBusCostOptions(const rapidjson::Document& doc,
                         const std::string& costing_options_key,
                         odin::CostingOptions* pbf_costing_options);

/**
 * Create a bus cost method. This is derived from auto costing and
 * uses the same rules except for using the bus access flag instead
 * of the auto access flag.
 */
cost_ptr_t CreateBusCost(const boost::property_tree::ptree& config);

/**
 * Parses the hov cost options from json and stores values in pbf.
 */
void ParseHOVCostOptions(const rapidjson::Document& doc,
                         const std::string& costing_options_key,
                         odin::CostingOptions* pbf_costing_options);

/**
 * Create a hov cost method. This is derived from auto costing and
 * uses the same rules except for favoring hov roads
 */
cost_ptr_t CreateHOVCost(const boost::property_tree::ptree& config);

} // namespace sif
} // namespace valhalla

#endif // VALHALLA_SIF_AUTOCOST_H_
