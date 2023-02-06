#ifndef VALHALLA_SIF_CUSTOMCOST_H_
#define VALHALLA_SIF_CUSTOMCOST_H_

#include <cstdint>

#include <boost/property_tree/ptree.hpp>
#include <valhalla/baldr/rapidjson_utils.h>
#include <valhalla/proto/options.pb.h>
#include <valhalla/sif/dynamiccost.h>

namespace valhalla {
namespace sif {

void ParseCustomCostOptions(const rapidjson::Document& doc,
                            const std::string& costing_options_key,
                            Costing* pbf_costing);

cost_ptr_t CreateCustomCost(const Costing& costing);

} // namespace sif
} // namespace valhalla

#endif // VALHALLA_SIF_CUSTOMCOST_H_
