#ifndef VALHALLA_SIF_BICYCLECOST_H_
#define VALHALLA_SIF_BICYCLECOST_H_

#include <valhalla/sif/dynamiccost.h>

namespace valhalla {
namespace sif {

/**
 * Create a bicyclecost
 * @param  config  Property tree with configuration / options.
 */
cost_ptr_t CreateBicycleCost(const boost::property_tree::ptree& config);

}
}

#endif  // VALHALLA_SIF_BICYCLECOST_H_
