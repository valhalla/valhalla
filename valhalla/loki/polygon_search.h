#ifndef VALHALLA_LOKI_POLYGON_SEARCH_H_
#define VALHALLA_LOKI_POLYGON_SEARCH_H_

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/geometries/register/ring.hpp>

#include <valhalla/baldr/graphreader.h>
#include <valhalla/midgard/constants.h>
#include <valhalla/midgard/pointll.h>
#include <valhalla/proto/options.pb.h>
#include <valhalla/sif/dynamiccost.h>
#include <valhalla/worker.h>

#include <functional>

// TODO: register PointLL in its own header, so it's available for bg across valhalla?
BOOST_GEOMETRY_REGISTER_POINT_2D(valhalla::midgard::PointLL,
                                 double,
                                 boost::geometry::cs::geographic<boost::geometry::degree>,
                                 first,
                                 second)
BOOST_GEOMETRY_REGISTER_RING(std::vector<valhalla::midgard::PointLL>)
static const auto Haversine = [] {
  return boost::geometry::strategy::distance::haversine<float>(valhalla::midgard::kRadEarthMeters);
};

namespace valhalla {
namespace loki {

using line_bg_t = boost::geometry::model::linestring<midgard::PointLL>;
using ring_bg_t = std::vector<midgard::PointLL>;
// map of tile for map of bin ids & their ring ids
using bins_collector =
    std::unordered_map<uint32_t, std::unordered_map<unsigned short, std::vector<size_t>>>;

/**
 * Finds all edge IDs which are intersected by the ring
 *
 * @param rings The (optionally closed) rings to intersect edges with
 * @param reader GraphReader instance
 *
 */
std::unordered_set<valhalla::baldr::GraphId>
edges_in_rings(const google::protobuf::RepeatedPtrField<valhalla::Options_Ring>& rings,
               baldr::GraphReader& reader,
               const std::shared_ptr<sif::DynamicCost>& costing,
               float max_length);

} // namespace loki
} // namespace valhalla

#endif // VALHALLA_LOKI_POLYGON_SEARCH_H_
