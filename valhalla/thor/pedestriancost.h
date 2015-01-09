#ifndef VALHALLA_THOR_PEDESTRIANCOST_H_
#define VALHALLA_THOR_PEDESTRIANCOST_H_

#include <valhalla/baldr/directededge.h>
#include <valhalla/baldr/nodeinfo.h>
#include <valhalla/thor/dynamiccost.h>

namespace valhalla {
namespace thor {

/**
 * Create an pedestriancost heuristic
 *
 */
cost_ptr_t CreatePedestrianHeuristic(/*pt::ptree const& config*/);

}
}

#endif  // VALHALLA_THOR_PEDESTRIANCOST_H_
