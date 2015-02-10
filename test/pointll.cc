#include "midgard/pointll.h"
#include "midgard/constants.h"
#include <cmath>

#include "test.h"

using namespace std;
using namespace valhalla::midgard;

namespace {
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

void TestHeadingAlongPolyline() {
  if (((int)(PointLL::HeadingAlongPolyline( {
    { -73.986392, 40.755800 },
    { -73.986438, 40.755819 } }, 30.0f) + 0.5f)) != 303)
    throw std::runtime_error("Invalid polyline begin heading");
  if (((int)(PointLL::HeadingAlongPolyline( {
    { -73.986438, 40.755819 },
    { -73.986484, 40.755681 } }, 30.0f) + 0.5f)) != 194)
    throw std::runtime_error("Invalid polyline begin heading");
  if (((int)(PointLL::HeadingAlongPolyline( {
    { -73.985777, 40.755539 },
    { -73.986440, 40.755820 },
    { -73.986617, 40.755254 } }, 30.0f) + 0.5f)) != 299)
    throw std::runtime_error("Invalid polyline begin heading");

  // TODO - test larger distance
//  std::cout << PointLL::HeadingAlongPolyline( {
//    { -73.985777, 40.755539 },
//    { -73.986440, 40.755820 },
//    { -73.986617, 40.755254 } }, 0.03f)
//    << std::endl;
}

void TestHeadingAtEndOfPolyline() {
  if (((int)(PointLL::HeadingAtEndOfPolyline( {
    { -73.986392, 40.755800 },
    { -73.986438, 40.755819 } }, 30.0f) + 0.5f)) != 303)
    throw std::runtime_error("Invalid polyline end heading");
  if (((int)(PointLL::HeadingAtEndOfPolyline( {
    { -73.986438, 40.755819 },
    { -73.986484, 40.755681 } }, 30.0f) + 0.5f)) != 194)
    throw std::runtime_error("Invalid polyline end heading");
  if (((int)(PointLL::HeadingAtEndOfPolyline( {
    { -73.985777, 40.755539 },
    { -73.986440, 40.755820 },
    { -73.986617, 40.755254 } }, 30.0f) + 0.5f)) != 194)
    throw std::runtime_error("Invalid polyline end heading");

  // TODO - test larger distance
//  std::cout << PointLL::HeadingAtEndOfPolyline( {
//    { -73.985777, 40.755539 },
//    { -73.986440, 40.755820 },
//    { -73.986617, 40.755254 } }, 0.03f)
//    << std::endl;
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

  //TODO: many more!

  return suite.tear_down();
}
