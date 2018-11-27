#ifndef VALHALLA_MJOLNIR_RESTRICTIONBUILDER_H
#define VALHALLA_MJOLNIR_RESTRICTIONBUILDER_H

#include <boost/property_tree/ptree.hpp>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>

namespace valhalla {
namespace mjolnir {

/**
 * Class used to enhance graph tile information at the local level.
 */
class RestrictionBuilder {
public:
  /**
   * Enhance the local level graph tile information.
   * @param pt                             property tree containing the hierarchy configuration
   * @param complex_from_restriction_file  where to grab the complex from restrictions
   * @param complex_to_restriction_file    where to grab the complex to restrictions
   */
  static void Build(const boost::property_tree::ptree& pt,
                    const std::string& complex_from_restriction_file,
                    const std::string& complex_to_restriction_file);
};

} // namespace mjolnir
} // namespace valhalla

#endif // VALHALLA_MJOLNIR_RESTRICTIONBUILDER_H
