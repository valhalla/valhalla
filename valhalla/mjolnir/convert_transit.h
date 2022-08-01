#pragma once

#include <boost/property_tree/ptree.hpp>
#include <unordered_set>

#include <valhalla/baldr/graphid.h>

namespace valhalla {
namespace mjolnir {

std::unordered_set<baldr::GraphId> convert_transit(const boost::property_tree::ptree& pt);
}
} // namespace valhalla