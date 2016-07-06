#include "midgard/pointll.h"
#include "midgard/constants.h"
#include "midgard/util.h"
#include <cmath>

#include "test.h"

using namespace std;
using namespace valhalla::midgard;

namespace {
bool equal(const PointLL& a, const PointLL& b){
  return valhalla::midgard::equal(a.first, b.first) && valhalla::midgard::equal(a.second, b.second);
}

void test_invalid() {
  PointLL ll;
  if (ll.IsValid())
    throw std::logic_error(
        "PointLL default initialization should not be valid");
  ll.Set(0, 0);
  if (!ll.IsValid())
    throw std::logic_error("0,0 is a valid coordinate");
  ll.Invalidate();
  if (ll.IsValid())
    throw std::logic_error("Invalidation produced valid coordinates");
}

void test_constructor() {
  PointLL ll { 1, 2 };
  if (ll.x() != 1 || ll.y() != 2)
    throw std::runtime_error("PointLL object should be set");
}

void test_along(const std::vector<PointLL> l, float d, float a) {
  auto r = PointLL::HeadingAlongPolyline(l, d);
  if(!valhalla::midgard::equal(r, a, 1.f))
    throw std::logic_error("Invalid polyline begin heading was " + std::to_string(r) + " but should be " + std::to_string(a));
}

void test_end(const std::vector<PointLL> l, float d, float a) {
  auto r = PointLL::HeadingAtEndOfPolyline(l, d);
  if(!valhalla::midgard::equal(r, a, 1.f))
    throw std::logic_error("Invalid polyline end heading was " + std::to_string(r) + " but should be " + std::to_string(a));
}

void TestHeadingAlongPolyline() {
  test_along({{ -73.986392, 40.755800 }, { -73.986438, 40.755819 } }, 30.0f, 303);
  test_along({{ -73.986438, 40.755819 }, { -73.986484, 40.755681 } }, 30.0f, 194);
  test_along({{ -73.985777, 40.755539 },{ -73.986440, 40.755820 },{ -73.986617, 40.755254 } }, 30.0f,299);

  // Partial roundabout
  test_along({
    { -76.316360,39.494102 },
    { -76.316360,39.494129 },
    { -76.316376,39.494152 },
    { -76.316391,39.494175 },
    { -76.316422,39.494194 },
    { -76.316444,39.494209 },
    { -76.316483,39.494221 },
    { -76.316521,39.494228 }
      }, 30.0f,314);

  // north (0)
  test_along({
    { -76.612682, 39.294540 },
    { -76.612681, 39.294897 },
    { -76.612708, 39.295208 }
      }, 30.0f, 0);

  // east (90)
  test_along({
    { -76.612682, 39.294540 },
    { -76.612508, 39.294535 },
    { -76.612359, 39.294541 },
    { -76.612151, 39.294545 }
      }, 30.0f,90);

  // south (176)
  test_along({
    { -76.612682, 39.294540 },
    { -76.612670, 39.294447 },
    { -76.612666, 39.294378 },
    { -76.612659, 39.294280 }
      }, 30.0f,176);

  // west (266)
  test_along({
    { -76.612682, 39.294540 },
    { -76.612789, 39.294527 },
    { -76.612898, 39.294525 },
    { -76.613033, 39.294523 },
      }, 30.0f,266);

}

void TestHeadingAtEndOfPolyline() {
  test_end( {{ -73.986392, 40.755800 },{ -73.986438, 40.755819 } }, 30.0f, 303);
  test_end( {{ -73.986438, 40.755819 },{ -73.986484, 40.755681 } }, 30.0f, 194);
  test_end( {{ -73.985777, 40.755539 },{ -73.986440, 40.755820 },{ -73.986617, 40.755254 } }, 30.0f, 194);

  // Partial roundabout
  test_end( {
    { -76.316360,39.494102 },
    { -76.316360,39.494129 },
    { -76.316376,39.494152 },
    { -76.316391,39.494175 },
    { -76.316422,39.494194 },
    { -76.316444,39.494209 },
    { -76.316483,39.494221 },
    { -76.316521,39.494228 }
      }, 30.0f, 314);

  // north (356)
  test_end({
    { -76.612682, 39.294540 },
    { -76.612681, 39.294897 },
    { -76.612708, 39.295208 }
      }, 30.0f, 356);

  // east (89)
  test_end({
    { -76.612682, 39.294540 },
    { -76.612508, 39.294535 },
    { -76.612359, 39.294541 },
    { -76.612151, 39.294545 }
      }, 30.0f, 89);

  // south (176)
  test_end({
    { -76.612682, 39.294540 },
    { -76.612670, 39.294447 },
    { -76.612666, 39.294378 },
    { -76.612659, 39.294280 }
      }, 30.0f, 176);

  // west (266)
  test_end({
    { -76.612682, 39.294540 },
    { -76.612789, 39.294527 },
    { -76.612898, 39.294525 },
    { -76.613033, 39.294523 },
      }, 30.0f, 266);

}

void TryClosestPoint(const std::vector<PointLL>& pts, const PointLL&p,
                     const PointLL& c, const int idx) {
  auto result = p.ClosestPoint(pts);
  if (idx != std::get<2>(result) )
      throw runtime_error("ClosestPoint test failed -index of closest segment is wrong");
  if (fabs(c.x() - std::get<0>(result).lng()) > kEpsilon ||
      fabs(c.y() - std::get<0>(result).lat()) > kEpsilon)
    throw runtime_error("ClosestPoint test failed - closest point is wrong");
}
void TestClosestPoint() {
  // Construct a simple polyline
  std::vector<PointLL> pts = {
      { 0.0f, 0.0f },
      { 2.0f, 2.0f },
      { 4.0f, 2.0f },
      { 4.0f, 0.0f },
      { 12.0f, 0.0f }
  };

  // Closest to the first point
  TryClosestPoint(pts, PointLL(-4.0f, 0.0f), PointLL(0.0f, 0.0f), 0);

  // Closest along the last segment
  TryClosestPoint(pts, PointLL(10.0f, -4.0f), PointLL(10.0f, 0.0f), 3);

  // Closest to the last point
  TryClosestPoint(pts, PointLL(15.0f, 4.0f), PointLL(12.0f, 0.0f), 3);
}

void TryWithinConvexPolygon(const std::vector<PointLL>& pts, const PointLL&p,
                            const bool res) {
  if (p.WithinConvexPolygon(pts) != res)
    throw runtime_error("TryWithinConvexPolygon test failed");
}

void TestWithinConvexPolygon() {
  // Construct a convex polygon
  std::vector<PointLL> pts = {
      {   2.0f,  2.0f },
      {   0.0f,  4.0f },
      { -10.0f,  0.0f },
      {   0.0f, -4.0f },
      {   2.0f, -2.0f }
  };

  // Inside
  TryWithinConvexPolygon(pts, PointLL(0.0f, 0.0f), true);

  // Check a vertex - should be inside
  TryWithinConvexPolygon(pts, PointLL( 0.0f, -4.0f), true);
  TryWithinConvexPolygon(pts, PointLL( 2.0f, -2.0f), true);

  // Outside
  TryWithinConvexPolygon(pts, PointLL(15.0f, 4.0f), false);
  TryWithinConvexPolygon(pts, PointLL( 2.5f, 0.0f), false);
  TryWithinConvexPolygon(pts, PointLL(-3.0f, 3.0f), false);
  TryWithinConvexPolygon(pts, PointLL( 1.0f,-3.5f), false);
}

void TestMidPoint() {
  //lines of longitude are geodesics so the mid point of points
  //on the same line of longitude should still be at the same longitude
  auto mid = PointLL(0, 90).MidPoint({0, 0});
  if(!equal(mid, PointLL(0, 45)))
    throw std::logic_error("Wrong mid point");
  mid = PointLL(0, 90).MidPoint({0, -66});
  if(!equal(mid, PointLL(0,12)))
    throw std::logic_error("Wrong mid point");

  //lines of latitude are not geodesics so if we put them 180 degrees apart
  //the shortest path between them is actually the geodesic that intersects
  //the pole. longitude is meaningless then
  mid = PointLL(-23, 45).MidPoint({157, 45});
  if(mid.second != 90)
    throw std::logic_error("Wrong mid point");

  //in the northern hemisphere we should expect midpoints on
  //geodesics between point of the same latitude to have higher latitude
  mid = PointLL(-15, 45).MidPoint({15, 45});
  if(mid.second <= 45.1)
    throw std::logic_error("Wrong mid point");
  mid = PointLL(-80, 1).MidPoint({80, 1});
  if(mid.second <= 1.1)
    throw std::logic_error("Wrong mid point");

  //conversely in the southern hemisphere we should expect them lower
  mid = PointLL(-15, -45).MidPoint({15, -45});
  if(mid.second >= -45.1)
    throw std::logic_error("Wrong mid point");
  mid = PointLL(-80, -1).MidPoint({80, -1});
  if(mid.second >= -1.1)
    throw std::logic_error("Wrong mid point");

  //the equator is the only line of latitude that is also a geodesic
  mid = PointLL(-15, 0).MidPoint({15, 0});
  if(!equal(mid, PointLL(0, 0)))
    throw std::logic_error("Wrong mid point");
  mid = PointLL(-170, 0).MidPoint({160, 0});
  if(!equal(mid, PointLL(175, 0)))
    throw std::logic_error("Wrong mid point");
}

}

int main(void) {
  test::suite suite("pointll");

  suite.test(TEST_CASE(test_invalid));
  suite.test(TEST_CASE(test_constructor));

  // HeadingAlongPolyline
  suite.test(TEST_CASE(TestHeadingAlongPolyline));

  // HeadingAtEndOfPolyline
  suite.test(TEST_CASE(TestHeadingAtEndOfPolyline));

  suite.test(TEST_CASE(TestClosestPoint));

  // Test if within polygon
  suite.test(TEST_CASE(TestWithinConvexPolygon));

  // Test midpoint
  suite.test(TEST_CASE(TestMidPoint));

  //TODO: many more!

  return suite.tear_down();
}
