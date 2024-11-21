#ifndef VALHALLA_MJOLNIR_EDGEBOUNDSBUILDER_H
#define VALHALLA_MJOLNIR_EDGEBOUNDSBUILDER_H

#include <deque>

#include <boost/property_tree/ptree.hpp>

#include "baldr/graphid.h"

namespace valhalla {
namespace mjolnir {

/**
 * Class used to add bounding circle data for each edge in the Valhalla graph tiles.
 */
class EdgeBoundsBuilder {
public:
  /**
   * @brief Add edge bounds information to the graph tiles.
   * param[in] config Config file to set EdgeBoundsBuilder properties
   * param[in] tile_ids Sequence of valhalla tile ids to build elevation tiles for.
   * @attention It is considered that tiles are from the directory specified in config file.
   */
  static void Build(const boost::property_tree::ptree& config,
                    std::deque<baldr::GraphId> tile_ids = {});
};

} // namespace mjolnir
} // namespace valhalla

#endif // VALHALLA_MJOLNIR_EDGEBOUNDSBUILDER_H
