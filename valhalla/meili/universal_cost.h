// -*- mode: c++ -*-
#ifndef MMP_UNIVERSAL_COST_H__
#define MMP_UNIVERSAL_COST_H__
#include <cstdint>

#include <boost/property_tree/ptree.hpp>

#include <valhalla/sif/costconstants.h>
#include <valhalla/sif/dynamiccost.h>

namespace valhalla {

namespace meili {

constexpr valhalla::sif::TravelMode kUniversalTravelMode = static_cast<valhalla::sif::TravelMode>(4);

valhalla::sif::cost_ptr_t CreateUniversalCost(const boost::property_tree::ptree& config);

} // namespace meili

} // namespace valhalla

#endif // MMP_UNIVERSAL_COST_H__
