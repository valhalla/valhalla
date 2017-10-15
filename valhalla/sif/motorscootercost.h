#ifndef VALHALLA_SIF_MOTORSCOOTERCOST_H_
#define VALHALLA_SIF_MOTORSCOOTERCOST_H_

#include <cstdint>

#include <valhalla/sif/dynamiccost.h>
#include <boost/property_tree/ptree.hpp>

namespace valhalla {
namespace sif {

/**
 * Create motor scooter cost method. This is derived from auto costing and
 * uses the same rules except for some different access restrictions
 * and the tendency to avoid hills
 */
cost_ptr_t CreateMotorScooterCost(const boost::property_tree::ptree& config);

}
}

#endif  // VALHALLA_SIF_MOTORSCOOTERCOST_H_
