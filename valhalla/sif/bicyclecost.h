#ifndef VALHALLA_THOR_BICYCLECOST_H_
#define VALHALLA_THOR_BICYCLECOST_H_

#include <valhalla/thor/dynamiccost.h>

namespace valhalla {
namespace thor {

/**
 * Create a bicyclecost
 * @param  config  Property tree with configuration / options.
 */
cost_ptr_t CreateBicycleCost(const boost::property_tree::ptree& config);

}
}

#endif  // VALHALLA_THOR_BICYCLECOST_H_
