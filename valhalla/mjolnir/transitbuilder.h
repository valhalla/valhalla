#ifndef VALHALLA_MJOLNIR_TRANSITBUILDER_H
#define VALHALLA_MJOLNIR_TRANSITBUILDER_H

#include <boost/property_tree/ptree.hpp>

namespace valhalla {
namespace mjolnir {

/**
 * Class used to build transit data within the graph tiles.
 */
class TransitBuilder {
public:
  /**
   * Add transit information to the graph tiles. At present this is called during the larger tileset
   * build. This somewhat simplifies the updates that need to be done on the existing tiles to make
   * the connection between the two graphs. Mainly because there are no hierarchies, no spatial
   * index, no restrictions.
   *
   * TODO: rework the tile update methods to be sensitive to hierarchy (transitions), binned edges and
   *  restrictions. All of these refer to edge or node ids directly, so adding the transit connections
   *  means we have to update these as well. The easiest approach would be to move the nodes and edges
   *  to the ends of their respective arrays and update the stuff that references them. This will
   *  produce small unused/unreferenced holes in those arrays but avoids the problem of having to
   *  offset all the ids. We could also avoid wasting space by offsetting all the things, we'll have
   *  to see which is easier than the other
   * @param pt   Property tree containing the hierarchy configuration
   *             and other configuration needed to build transit.
   */
  static void Build(const boost::property_tree::ptree& pt);
};

} // namespace mjolnir
} // namespace valhalla

#endif // VALHALLA_MJOLNIR_TRANSITBUILDER_H
