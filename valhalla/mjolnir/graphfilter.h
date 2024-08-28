#ifndef VALHALLA_MJOLNIR_GRAPHFILTER_H
#define VALHALLA_MJOLNIR_GRAPHFILTER_H

#include <boost/property_tree/ptree.hpp>

#include <valhalla/baldr/graphreader.h>

namespace valhalla {
namespace mjolnir {

/**
 * Class used to optionally filter edges based on access.
 */
class GraphFilter {
public:
  /**
   * Update the tiles based on filtering logic in place.
   * @param pt Configuration file
   */
  static void Filter(const boost::property_tree::ptree& pt);
};

} // namespace mjolnir
} // namespace valhalla

#endif // VALHALLA_MJOLNIR_GRAPHFILTER_H
