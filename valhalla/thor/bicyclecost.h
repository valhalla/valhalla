#ifndef VALHALLA_THOR_BICYCLECOST_H_
#define VALHALLA_THOR_BICYCLECOST_H_

#include <valhalla/thor/dynamiccost.h>

namespace valhalla {
namespace thor {

/**
 * Create an bicyclecost heuristic
 *
 */
cost_ptr_t CreateBicycleHeuristic(/*pt::ptree const& config*/);

}
}

#endif  // VALHALLA_THOR_BICYCLECOST_H_
