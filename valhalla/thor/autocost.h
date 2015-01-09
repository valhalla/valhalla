#ifndef VALHALLA_THOR_AUTOCOST_H_
#define VALHALLA_THOR_AUTOCOST_H_

#include <valhalla/thor/dynamiccost.h>

namespace valhalla {
namespace thor {

/**
 * Create an autocost heuristic
 *
 */
cost_ptr_t CreateAutoHeuristic(/*pt::ptree const& config*/);

}
}

#endif  // VALHALLA_THOR_AUTOCOST_H_
