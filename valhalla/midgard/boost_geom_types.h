#ifndef VALHALLA_MIDGARD_BOOST_GEOM_HELPER_H_
#define VALHALLA_MIDGARD_BOOST_GEOM_HELPER_H_

#include "midgard/aabb2.h"
#include "midgard/point2.h"
#include "midgard/pointll.h"

#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/multi_linestring.hpp>
#include <boost/geometry/geometries/multi_polygon.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/register/box.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/geometries/ring.hpp>

namespace valhalla::midgard::bg {

// Point2d
using linestring_2d_t = boost::geometry::model::linestring<valhalla::midgard::Point2d>;
using multilinestring_2d_t = boost::geometry::model::multi_linestring<linestring_2d_t>;
using ring_2d_t = boost::geometry::model::ring<valhalla::midgard::Point2d>;
using polygon_2d_t = boost::geometry::model::polygon<valhalla::midgard::Point2d>;
using multipolygon_2d_t = boost::geometry::model::multi_polygon<polygon_2d_t>;

// PointLL
using linestring_ll_t = boost::geometry::model::linestring<valhalla::midgard::PointLL>;
using multilinestring_ll_t = boost::geometry::model::multi_linestring<linestring_ll_t>;
using ring_ll_t = boost::geometry::model::ring<valhalla::midgard::PointLL>;
using polygon_ll_t = boost::geometry::model::polygon<valhalla::midgard::PointLL>;
using multipolygon_ll_t = boost::geometry::model::multi_polygon<polygon_ll_t>;

// Point with integer coords
using point_2i_t = boost::geometry::model::point<int32_t, 2, boost::geometry::cs::cartesian>;
using box_2i_t = boost::geometry::model::box<point_2i_t>;

} // namespace valhalla::midgard::bg

// register a few common types
BOOST_GEOMETRY_REGISTER_POINT_2D(valhalla::midgard::PointLL,
                                 double,
                                 boost::geometry::cs::geographic<boost::geometry::degree>,
                                 first,
                                 second)
BOOST_GEOMETRY_REGISTER_POINT_2D(valhalla::midgard::Point2d,
                                 double,
                                 boost::geometry::cs::cartesian,
                                 first,
                                 second)
BOOST_GEOMETRY_REGISTER_BOX(valhalla::midgard::AABB2<valhalla::midgard::PointLL>,
                            valhalla::midgard::PointLL,
                            minpt(),
                            maxpt())
BOOST_GEOMETRY_REGISTER_BOX(valhalla::midgard::AABB2<valhalla::midgard::Point2d>,
                            valhalla::midgard::Point2d,
                            minpt(),
                            maxpt())

#endif // VALHALLA_MIDGARD_BOOST_GEOM_HELPER_H_