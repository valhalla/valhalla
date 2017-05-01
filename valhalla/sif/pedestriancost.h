#ifndef VALHALLA_SIF_PEDESTRIANCOST_H_
#define VALHALLA_SIF_PEDESTRIANCOST_H_

#include <cstdint>
#include <valhalla/baldr/directededge.h>
#include <valhalla/baldr/nodeinfo.h>
#include <valhalla/sif/dynamiccost.h>

namespace valhalla {
namespace sif {

/**
 * Create a pedestriancost
 *
 */
cost_ptr_t CreatePedestrianCost(const boost::property_tree::ptree& config);

}
}

#endif  // VALHALLA_SIF_PEDESTRIANCOST_H_
