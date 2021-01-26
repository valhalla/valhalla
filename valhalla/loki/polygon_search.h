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
#include <valhalla/sif/dynamiccost.h>

#include <functional>

// Register custom geom types with boost
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

struct ring_bin {
  std::vector<size_t> ring_ids;
  bool within;

  ring_bin() : within(false) {
  }
};

using bins_collector = std::unordered_map<unsigned short, ring_bin>;

/**
 * Finds all edge IDs which are intersected by the ring
 *
 * @param rings The (optionally closed) rings to intersect edges with
 * @param reader GraphReader instance
 *
 */
std::set<valhalla::baldr::GraphId> edges_in_rings(const std::vector<ring_bg_t>& rings,
                                                  baldr::GraphReader& reader,
                                                  const std::shared_ptr<sif::DynamicCost>& costing);

/**
 * Convert PBF Polygon to boost geometry ring.
 *
 * @param ring_pbf The Options::Polygon ring
 *
 */
ring_bg_t PBFToRing(const Options::Polygon& ring_pbf);

/**
 * Computes the length of a ring's circumference.
 *
 * @param ring The boost geometry ring.
 *
 */
double GetRingLength(const ring_bg_t& ring);

} // namespace loki
} // namespace valhalla

#endif // VALHALLA_LOKI_POLYGON_SEARCH_H_
