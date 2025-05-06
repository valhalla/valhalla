#ifndef VALHALLA_LOKI_POLYGON_SEARCH_H_
#define VALHALLA_LOKI_POLYGON_SEARCH_H_

#include <geos_c.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/proto/options.pb.h>
#include <valhalla/sif/dynamiccost.h>

namespace valhalla {
namespace loki {

struct geos_t {
  geos_t();
  ~geos_t();
  GEOSGeometry* linestring_from_shape(const std::vector<PointLL>& shape) const;
  GEOSGeometry* polygon_from_pbf(const valhalla::Ring& ring) const;

  GEOSContextHandle_t ctx;
};

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

/**
 * Finds all edge IDs which are intersected by the ring
 *
 * @param rings The (optionally closed) rings to intersect edges with
 * @param reader GraphReader instance
 *
 */
std::unordered_set<valhalla::baldr::GraphId>
edges_in_polygons(const google::protobuf::RepeatedPtrField<valhalla::Ring>& rings,
                  baldr::GraphReader& reader,
                  const std::shared_ptr<sif::DynamicCost>& costing,
                  float max_length,
                  const geos_t& geos_helper);
} // namespace loki
} // namespace valhalla

#endif // VALHALLA_LOKI_POLYGON_SEARCH_H_
