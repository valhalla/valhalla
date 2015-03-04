#ifndef VALHALLA_MJOLNIR_GRAPHENHANCER_H
#define VALHALLA_MJOLNIR_GRAPHENHANCER_H

#include <boost/property_tree/ptree.hpp>

namespace valhalla {
namespace mjolnir {

/**
 * Class used to enhance graph tile information at the local level.
 */
class GraphEnhancer {
 public:

  /**
   * Enhance the local level graph tile information.
   * @param pt   property tree containing the heirarchy configuration
   */
  static void Enhance(const boost::property_tree::ptree& pt);

};

}
}

#endif  // VALHALLA_MJOLNIR_GRAPHENHANCER_H
