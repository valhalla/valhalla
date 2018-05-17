#ifndef VALHALLA_LOKI_NODE_SEARCH_H_
#define VALHALLA_LOKI_NODE_SEARCH_H_

#include <cstdint>
#include <valhalla/baldr/graphreader.h>

namespace valhalla {
namespace loki {

/**
 * Find nodes within the given bounding box in the route network.
 *
 * @param  bbox   bounding box in which to look for nodes.
 * @param  reader graph reader object to use for loading tiles.
 * @return nodes  a collection of nodes which are in the bounding box.
 */
std::vector<baldr::GraphId> nodes_in_bbox(const midgard::AABB2<midgard::PointLL>& bbox,
                                          baldr::GraphReader& reader);

} // namespace loki
} // namespace valhalla

#endif // VALHALLA_LOKI_NODE_SEARCH_H_
