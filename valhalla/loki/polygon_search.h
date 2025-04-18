#ifndef VALHALLA_LOKI_POLYGON_SEARCH_H_
#define VALHALLA_LOKI_POLYGON_SEARCH_H_

#include <memory>
#include <unordered_set>
namespace google::protobuf {
template <typename Element> class RepeatedPtrField;
}
namespace valhalla {
class Ring;
namespace baldr {
class GraphId;
class GraphReader;
} // namespace baldr
namespace sif {
class DynamicCost;
}
namespace loki {

/**
 * Finds all edge IDs which are intersected by the ring
 *
 * @param rings The (optionally closed) rings to intersect edges with
 * @param reader GraphReader instance
 *
 */
std::unordered_set<valhalla::baldr::GraphId>
edges_in_rings(const google::protobuf::RepeatedPtrField<valhalla::Ring>& rings,
               baldr::GraphReader& reader,
               const std::shared_ptr<sif::DynamicCost>& costing,
               float max_length);

} // namespace loki
} // namespace valhalla

#endif // VALHALLA_LOKI_POLYGON_SEARCH_H_
