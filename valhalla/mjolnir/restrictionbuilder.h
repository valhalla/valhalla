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
   * @param pt                        property tree containing the hierarchy configuration
   * @param complex_restriction_file  where to grab the complex restrictions
   * @param end_map                   to --> from multimap of complex restriction ids
   */
  static void Build(const boost::property_tree::ptree& pt,
                    const std::string& complex_restriction_file,
                    const std::unordered_multimap<uint64_t, uint64_t>& end_map);
};

} // namespace mjolnir
} // namespace valhalla

#endif // VALHALLA_MJOLNIR_RESTRICTIONBUILDER_H
