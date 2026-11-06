#pragma once

#include <boost/property_tree/ptree.hpp>

namespace valhalla {
namespace mjolnir {

/**
 * Enriches unnamed pedestrian edges with the name of the nearest
 * road edge. For each unnamed pedestrian edge, finds the best matching
 * named road using spatial proximity (R-tree) refined by average polyline distance.
 *
 * Processes all local-level tiles in parallel using work-stealing (atomic tile counter)
 * with largest tiles scheduled first for load balancing.
 *
 * @param pt  Full configuration property tree (reads mjolnir.tile_dir, mjolnir.concurrency).
 */
void EnrichPedestrianEdgeNames(const boost::property_tree::ptree& pt);

} // namespace mjolnir
} // namespace valhalla
