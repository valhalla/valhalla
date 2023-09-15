#pragma once

#include <boost/property_tree/ptree.hpp>

#include <cstdint>
#include <iostream>
#include <list>
#include <string>
#include <vector>

namespace valhalla {
namespace mjolnir {

bool BuildAdminFromPBF(const boost::property_tree::ptree& pt,
                       const std::vector<std::string>& input_files);
}
} // namespace valhalla
