#ifndef VALHALLA_THOR_AUTOCOST_H_
#define VALHALLA_THOR_AUTOCOST_H_

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <valhalla/sif/dynamiccost.h>

namespace valhalla {
namespace thor {

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

}
}

#endif  // VALHALLA_THOR_AUTOCOST_H_
