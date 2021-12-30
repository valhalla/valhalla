#ifndef VALHALLA_LOKI_POLYGON_SEARCH_H_
#define VALHALLA_LOKI_POLYGON_SEARCH_H_

#include <valhalla/baldr/graphreader.h>
#include <valhalla/proto/options.pb.h>
#include <valhalla/sif/dynamiccost.h>

namespace valhalla {
namespace loki {

/**
 * Finds all edge IDs which are intersected by or within the ring
 *
 * @param rings The (optionally closed) rings to intersect edges with
 * @param reader GraphReader instance
 * @param max_length The maximum length of the total perimeters of all rings
 * @param mode The method to extract the edges and the exception thrown. It is either `avoid_polygons`
 * (using intersect) or `chinese_postman` (using within)
 *
 */
std::unordered_set<valhalla::baldr::GraphId>
edges_in_rings(const google::protobuf::RepeatedPtrField<valhalla::Options_Ring>& rings,
               baldr::GraphReader& reader,
               const std::shared_ptr<sif::DynamicCost>& costing,
               float max_length,
               std::string mode = "avoid_polygons");

} // namespace loki
} // namespace valhalla

#endif // VALHALLA_LOKI_POLYGON_SEARCH_H_
