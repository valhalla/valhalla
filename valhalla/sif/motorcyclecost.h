#ifndef VALHALLA_SIF_MOTORCYCLECOST_H_
#define VALHALLA_SIF_MOTORCYCLECOST_H_

#include <cstdint>

#include <boost/property_tree/ptree.hpp>
#include <valhalla/sif/dynamiccost.h>

namespace valhalla {
namespace sif {

/**
 * Create motorcycle cost method. This is derived from auto costing and
 * uses the same rules except for some different access restrictions
 * and the tendency to avoid hills
 */
cost_ptr_t CreateMotorcycleCost(const boost::property_tree::ptree& config);
} // namespace sif
} // namespace valhalla

#endif // VALHALLA_SIF_MOTORCYCLECOST_H_
