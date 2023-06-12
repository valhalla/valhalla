#ifndef VALHALLA_MJOLNIR_ELEVATIONBUILDER_H
#define VALHALLA_MJOLNIR_ELEVATIONBUILDER_H

#include <deque>

#include <boost/property_tree/ptree.hpp>

#include "baldr/graphid.h"

namespace valhalla {
namespace mjolnir {

/**
 * Class used to add elevation data to the Valhalla graph tiles.
 */
class ElevationBuilder {
public:
  /**
   * @brief Add elevation information to the graph tiles.
   * param[in] config Config file to set ElevationBuilder properties
   * param[in] threads Amount of threads.
   * param[in] tile_ids Sequence of valhalla tile ids to build elevation tiles for.
   * @attention It is considered that tiles are from the directory specified in config file.
   */
  static void Build(const boost::property_tree::ptree& config,
                    const std::deque<baldr::GraphId> tile_ids = {},
                    const uint32_t num_threads = 0U);
};

} // namespace mjolnir
} // namespace valhalla

#endif // VALHALLA_MJOLNIR_ELEVATIONBUILDER_H
