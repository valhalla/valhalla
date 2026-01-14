#ifndef VALHALLA_MIDGARD_BOOST_GEOM_HELPER_H_
#define VALHALLA_MIDGARD_BOOST_GEOM_HELPER_H_
#include "midgard/aabb2.h"
#include "midgard/point2.h"
#include "midgard/pointll.h"

#include <boost/geometry/geometries/register/box.hpp>
#include <boost/geometry/geometries/register/linestring.hpp>
#include <boost/geometry/geometries/register/multi_linestring.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/geometries/register/ring.hpp>

// TODO: if we need to, use wrappers to distinguish between e.g. LineString & Ring (both
// vector<point>)

BOOST_GEOMETRY_REGISTER_POINT_2D(valhalla::midgard::PointLL,
                                 double,
                                 boost::geometry::cs::geographic<boost::geometry::degree>,
                                 first,
                                 second)
BOOST_GEOMETRY_REGISTER_RING(std::vector<valhalla::midgard::PointLL>)

// for MVT
BOOST_GEOMETRY_REGISTER_POINT_2D(valhalla::midgard::Point2i,
                                 int32_t,
                                 boost::geometry::cs::cartesian,
                                 first,
                                 second)
BOOST_GEOMETRY_REGISTER_BOX(valhalla::midgard::AABB2<valhalla::midgard::Point2i>,
                            valhalla::midgard::Point2i,
                            minpt(),
                            maxpt())
BOOST_GEOMETRY_REGISTER_LINESTRING(std::vector<valhalla::midgard::Point2i>)
BOOST_GEOMETRY_REGISTER_MULTI_LINESTRING(std::vector<std::vector<valhalla::midgard::Point2i>>)

#endif // VALHALLA_MIDGARD_BOOST_GEOM_HELPER_H_
