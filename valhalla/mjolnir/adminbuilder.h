#pragma once

#include <boost/property_tree/ptree_fwd.hpp>

#include <string>
#include <vector>

namespace valhalla {
namespace mjolnir {

bool BuildAdminFromPBF(const boost::property_tree::ptree& pt,
                       const std::vector<std::string>& input_files);
}
} // namespace valhalla
