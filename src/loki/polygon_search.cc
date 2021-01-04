#include <boost/geometry.hpp>
#include <boost/geometry/geometries/register/box.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <iostream>
#include <math.h>

#include <midgard/aabb2.h>
#include <midgard/pointll.h>
#include <proto/options.pb.h>

namespace bg = boost::geometry;
namespace vm = valhalla::midgard;

// Register custom geom types with boost
BOOST_GEOMETRY_REGISTER_POINT_2D(vm::PointLL, double, bg::cs::geographic<bg::degree>, first, second)
BOOST_GEOMETRY_REGISTER_BOX(vm::AABB2<vm::PointLL>, vm::PointLL, ll, ur)

namespace {

using ring_bg_t = bg::model::ring<vm::PointLL>;

} // namespace

namespace valhalla {
namespace loki {

double GetRingsArea(const google::protobuf::RepeatedPtrField<Options::AvoidPolygon>& rings_pbf) {
  double area;
  for (const auto& ring_pbf : rings_pbf) {
    ring_bg_t new_ring;
    for (const auto& coord : ring_pbf.coords()) {
      new_ring.push_back({coord.ll().lng(), coord.ll().lat()});
    }
    area += bg::area(new_ring);
  }

  return area;
}
} // namespace loki
} // namespace valhalla
