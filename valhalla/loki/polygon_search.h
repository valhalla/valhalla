#ifndef VALHALLA_LOKI_POLYGON_SEARCH_H_
#define VALHALLA_LOKI_POLYGON_SEARCH_H_

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/register/box.hpp>
#include <boost/geometry/geometries/register/linestring.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/geometries/register/ring.hpp>

#include <valhalla/baldr/graphreader.h>
#include <valhalla/midgard/aabb2.h>
#include <valhalla/midgard/constants.h>
#include <valhalla/midgard/pointll.h>
#include <valhalla/proto/options.pb.h>

#include <functional>

// Register custom geom types with boost
BOOST_GEOMETRY_REGISTER_POINT_2D(valhalla::midgard::PointLL,
                                 double,
                                 boost::geometry::cs::geographic<boost::geometry::degree>,
                                 first,
                                 second)
BOOST_GEOMETRY_REGISTER_BOX(valhalla::midgard::AABB2<valhalla::midgard::PointLL>,
                            valhalla::midgard::PointLL,
                            ll,
                            ur)
BOOST_GEOMETRY_REGISTER_RING(std::vector<valhalla::midgard::PointLL>)
static const auto Haversine = [] {
  return boost::geometry::strategy::distance::haversine<float>(valhalla::midgard::kRadEarthMeters);
};

namespace valhalla {
namespace loki {

using ring_bg_t = std::vector<midgard::PointLL>;
using multi_ring_t = std::vector<ring_bg_t>;

std::set<valhalla::baldr::GraphId> edges_in_rings(const multi_ring_t& rings,
                                                  baldr::GraphReader& reader);

multi_ring_t PBFToRings(const google::protobuf::RepeatedPtrField<Options::AvoidPolygon>& rings_pbf);

// double GetAvoidArea(const multi_ring_t& rings);
float GetRingLength(const multi_ring_t& rings);

} // namespace loki
} // namespace valhalla

#endif // VALHALLA_LOKI_POLYGON_SEARCH_H_
