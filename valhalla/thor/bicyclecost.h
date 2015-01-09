#ifndef VALHALLA_THOR_BICYCLECOST_H_
#define VALHALLA_THOR_BICYCLECOST_H_

#include <valhalla/thor/dynamiccost.h>

namespace valhalla {
namespace thor {

/**
 * Create a bicyclecost
 *
 */
cost_ptr_t CreateBicycleCost(/*pt::ptree const& config*/);

}
}

#endif  // VALHALLA_THOR_BICYCLECOST_H_
