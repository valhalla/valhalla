#ifndef VALHALLA_SIF_AUTOCOST_H_
#define VALHALLA_SIF_AUTOCOST_H_

#include <cstdint>

#include <valhalla/sif/dynamiccost.h>
#include <boost/property_tree/ptree.hpp>

namespace valhalla {
namespace sif {

/**
 * Create an auto route cost method. This is generally shortest time but uses
 * hierarchies and can avoid "shortcuts" through residential areas.
 */
cost_ptr_t CreateAutoCost(const boost::property_tree::ptree& config);

/**
 * Create an auto shorter cost method. This is derived from auto costing and
 * uses the same rules except the edge cost uses an adjusted speed that
 * (non-linearly) reduces the importance of edge speed.
 */
cost_ptr_t CreateAutoShorterCost(const boost::property_tree::ptree& config);

/**
 * Create a bus cost method. This is derived from auto costing and
 * uses the same rules except for using the bus access flag instead
 * of the auto access flag.
 */
cost_ptr_t CreateBusCost(const boost::property_tree::ptree& config);

/**
 * Create a hov cost method. This is derived from auto costing and
 * uses the same rules except for favoring hov roads
 */
cost_ptr_t CreateHOVCost(const boost::property_tree::ptree& config);

}
}

#endif  // VALHALLA_SIF_AUTOCOST_H_
