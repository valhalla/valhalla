#ifndef VALHALLA_LOKI_POLYGON_SEARCH_H_
#define VALHALLA_LOKI_POLYGON_SEARCH_H_

#include <valhalla/baldr/graphreader.h>
#include <valhalla/proto/options.pb.h>
#include <valhalla/sif/dynamiccost.h>

namespace valhalla {
namespace loki {

enum SearchStrategy { AVOID, CHINESE };

/**
 * Finds all edge IDs which are intersected by or within the single ring
 *
 * @param ring The (optionally closed) ring to intersect edges with
 * @param reader GraphReader instance
 * @param max_length The maximum length of the total perimeters of all rings
 * @param mode Determines the search stragegy, either for avoids or chinese polygon.
 *
 */
std::unordered_set<valhalla::baldr::GraphId>
edges_in_rings(const valhalla::Ring& ring,
               baldr::GraphReader& reader,
               const std::shared_ptr<sif::DynamicCost>& costing,
               float max_length,
               const SearchStrategy strategy);

/**
 * Finds all edge IDs which are intersected by or within the rings
 *
 * @param rings The (optionally closed) rings to intersect edges with
 * @param reader GraphReader instance
 * @param max_length The maximum length of the total perimeters of all rings
 * @param strategy Determines the search stragegy, either for avoids or chinese polygon.
 *
 */
std::unordered_set<valhalla::baldr::GraphId>
edges_in_rings(const google::protobuf::RepeatedPtrField<valhalla::Ring>& rings,
               baldr::GraphReader& reader,
               const std::shared_ptr<sif::DynamicCost>& costing,
               float max_length,
               const SearchStrategy strategy);

} // namespace loki
} // namespace valhalla

#endif // VALHALLA_LOKI_POLYGON_SEARCH_H_
