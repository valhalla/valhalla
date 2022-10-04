#ifndef VALHALLA_LOKI_POLYGON_SEARCH_H_
#define VALHALLA_LOKI_POLYGON_SEARCH_H_

#include <valhalla/baldr/graphreader.h>
#include <valhalla/proto/options.pb.h>
#include <valhalla/sif/dynamiccost.h>

namespace valhalla {
namespace loki {

enum Purpose { AVOID, CHINESE };

/**
 * Finds all edge IDs which are intersected by or within the single ring
 *
 * @param ring The (optionally closed) ring to intersect edges with
 * @param reader GraphReader instance
 * @param max_length The maximum length of the total perimeters of all rings
 * @param mode The method to extract the edges and the exception thrown. It is either `avoid_polygons`
 * (using intersect) or `chinese_postman` (using within)
 *
 */
std::unordered_set<valhalla::baldr::GraphId>
edges_in_rings(const valhalla::Ring& ring,
               baldr::GraphReader& reader,
               const std::shared_ptr<sif::DynamicCost>& costing,
               float max_length,
               const Purpose purpose);

/**
 * Finds all edge IDs which are intersected by or within the rings
 *
 * @param rings The (optionally closed) rings to intersect edges with
 * @param reader GraphReader instance
 * @param max_length The maximum length of the total perimeters of all rings
 * @param mode The method to extract the edges and the exception thrown. It is either `avoid_polygons`
 * (using intersect) or `chinese_postman` (using within)
 *
 */
std::unordered_set<valhalla::baldr::GraphId>
edges_in_rings(const google::protobuf::RepeatedPtrField<valhalla::Ring>& rings,
               baldr::GraphReader& reader,
               const std::shared_ptr<sif::DynamicCost>& costing,
               float max_length,
               const Purpose purpose);

} // namespace loki
} // namespace valhalla

#endif // VALHALLA_LOKI_POLYGON_SEARCH_H_
