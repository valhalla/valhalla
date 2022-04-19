#include "midgard/pointll.h"
#include "midgard/constants.h"
#include "midgard/util.h"
#include <algorithm>
#include <cmath>

#include "test.h"

using namespace std;
using namespace valhalla::midgard;

namespace {

#define ASSERT_POINTLL_EQUAL(a, b)                                                                   \
  do {                                                                                               \
    constexpr double kPrecision = 1e-7;                                                              \
    EXPECT_NEAR(a.first, b.first, kPrecision);                                                       \
    EXPECT_NEAR(a.second, b.second, kPrecision);                                                     \
  } while (0)

TEST(PointLL, test_invalid) {
  PointLL ll;
  EXPECT_FALSE(ll.IsValid()) << "PointLL default initialization should not be valid";

  ll.Set(0, 0);
  EXPECT_TRUE(ll.IsValid()) << "0,0 is a valid coordinate";

  ll.Invalidate();
  EXPECT_FALSE(ll.IsValid()) << "Invalidation produced valid coordinates";
}

TEST(PointLL, test_constructor) {
  PointLL ll{1, 2};
  EXPECT_EQ(ll.x(), 1);
  EXPECT_EQ(ll.y(), 2);
}

void test_along(const std::vector<PointLL>& l, double d, double a) {
  auto r = PointLL::HeadingAlongPolyline(l, d);
  EXPECT_NEAR(r, a, 1.) << "Invalid polyline begin heading was " + std::to_string(r) +
                               " but should be " + std::to_string(a);
}

void test_end(const std::vector<PointLL>& l, double d, double a) {
  auto r = PointLL::HeadingAtEndOfPolyline(l, d);
  EXPECT_NEAR(r, a, 1.) << "Invalid polyline end heading was " + std::to_string(r) +
                               " but should be " + std::to_string(a);
}

TEST(PointLL, TestHeadingAlongPolyline) {
  // Test with empty (or 1 point) polyline
  std::vector<PointLL> empty;
  EXPECT_NEAR(PointLL::HeadingAlongPolyline(empty, 30.0), 0.0, 1.0);

  std::vector<PointLL> withOnePoint;
  withOnePoint.emplace_back(-70.0, 30.0);
  EXPECT_NEAR(PointLL::HeadingAlongPolyline(withOnePoint, 30.0), 0.0, 1.0);

  test_along({{-73.986392, 40.755800}, {-73.986438, 40.755819}}, 30.0, 299);
  test_along({{-73.986438, 40.755819}, {-73.986484, 40.755681}}, 30.0, 194);
  test_along({{-73.985777, 40.755539}, {-73.986440, 40.755820}, {-73.986617, 40.755254}}, 30.0, 299);

  // Partial roundabout
  test_along({{-76.316360, 39.494102},
              {-76.316360, 39.494129},
              {-76.316376, 39.494152},
              {-76.316391, 39.494175},
              {-76.316422, 39.494194},
              {-76.316444, 39.494209},
              {-76.316483, 39.494221},
              {-76.316521, 39.494228}},
             30.0, 315);

  // north (0)
  test_along({{-76.612682, 39.294540}, {-76.612681, 39.294897}, {-76.612708, 39.295208}}, 30.0, 0);

  // east (90)
  test_along({{-76.612682, 39.294540},
              {-76.612508, 39.294535},
              {-76.612359, 39.294541},
              {-76.612151, 39.294545}},
             30.0, 90);

  // south (176)
  test_along({{-76.612682, 39.294540},
              {-76.612670, 39.294447},
              {-76.612666, 39.294378},
              {-76.612659, 39.294280}},
             30.0, 176);

  // west (266)
  test_along(
      {
          {-76.612682, 39.294540},
          {-76.612789, 39.294527},
          {-76.612898, 39.294525},
          {-76.613033, 39.294523},
      },
      30.0, 266);
}

TEST(PointLL, TestHeadingAtEndOfPolyline) {
  // Test with empty (or 1 point) polyline
  std::vector<PointLL> empty;
  EXPECT_NEAR(PointLL::HeadingAtEndOfPolyline(empty, 30.0), 0.0, 1.0);

  std::vector<PointLL> withOnePoint;
  withOnePoint.emplace_back(-70.0, 30.0);
  EXPECT_NEAR(PointLL::HeadingAtEndOfPolyline(withOnePoint, 30.0), 0.0, 1.0);

  test_end({{-73.986392, 40.755800}, {-73.986438, 40.755819}}, 30.0, 299);
  test_end({{-73.986438, 40.755819}, {-73.986484, 40.755681}}, 30.0, 194);
  test_end({{-73.985777, 40.755539}, {-73.986440, 40.755820}, {-73.986617, 40.755254}}, 30.0, 194);

  // Partial roundabout
  test_end({{-76.316360, 39.494102},
            {-76.316360, 39.494129},
            {-76.316376, 39.494152},
            {-76.316391, 39.494175},
            {-76.316422, 39.494194},
            {-76.316444, 39.494209},
            {-76.316483, 39.494221},
            {-76.316521, 39.494228}},
           30.0, 315);

  // north (356)
  test_end({{-76.612682, 39.294540}, {-76.612681, 39.294897}, {-76.612708, 39.295208}}, 30.0, 356);

  // east (88)
  test_end({{-76.612682, 39.294540},
            {-76.612508, 39.294535},
            {-76.612359, 39.294541},
            {-76.612151, 39.294545}},
           30.0, 88);

  // south (176)
  test_end({{-76.612682, 39.294540},
            {-76.612670, 39.294447},
            {-76.612666, 39.294378},
            {-76.612659, 39.294280}},
           30.0, 176);

  // west (266)
  test_end(
      {
          {-76.612682, 39.294540},
          {-76.612789, 39.294527},
          {-76.612898, 39.294525},
          {-76.613033, 39.294523},
      },
      30.0, 266);
}

TEST(PointLL, TestHeadingPrecision) {
  auto actual = PointLL(11.6057196, 48.1032867).Heading(PointLL(11.6056538, 48.1035118));
  double expected = 348.954542;
  EXPECT_NEAR(actual, expected, 0.000001);
}

void TryClosestPoint(std::vector<PointLL> pts,
                     const PointLL& pt,
                     const PointLL& expected_pt,
                     double expected_dist,
                     int expected_idx,
                     int expected_inverse_idx,
                     int begin_idx = 0) {

  // do forwards and backwards searches
  for (int reverse = 0; reverse < 2; ++reverse) {

    // if we are doing a backwards search we will just flip
    // the direction of the line string and invert the indices
    // inverting them gets tricky at the boundaries :o(
    double forward = std::numeric_limits<double>::infinity();
    double backward = 0;
    if (reverse) {
      std::reverse(pts.begin(), pts.end());
      if (pts.size() > 1) {
        begin_idx = (pts.size() - 1) - begin_idx;
      }
      std::swap(forward, backward);
      expected_idx = expected_inverse_idx;
    }

    // look for the closest point
    auto result = pt.ClosestPoint(pts, begin_idx, forward, backward);
    PointLL result_pt = std::get<0>(result);

    // Test expected closest point
    EXPECT_TRUE(result_pt.ApproximatelyEqual(expected_pt))
        << "TryClosestPoint point test failed - found: " + std::to_string(result_pt.lat()) + "," +
               std::to_string(result_pt.lng()) + " | expected: " + std::to_string(expected_pt.lat()) +
               "," + std::to_string(expected_pt.lng());

    // Test expected distance
    EXPECT_NEAR(std::get<1>(result), expected_dist, 0.5);

    // Test expected index
    EXPECT_EQ(std::get<2>(result), expected_idx);
  }
}

void TryClosestPointNoDistance(const std::vector<PointLL>& pts,
                               const PointLL& pt,
                               const PointLL& expected_pt,
                               int expected_idx) {
  auto result = pt.ClosestPoint(pts);
  PointLL result_pt = std::get<0>(result);

  // Test expected closest point
  EXPECT_TRUE(result_pt.ApproximatelyEqual(expected_pt))
      << "TryClosestPointNoDistance point test failed - found: " + std::to_string(result_pt.lat()) +
             "," + std::to_string(result_pt.lng()) +
             " | expected: " + std::to_string(expected_pt.lat()) + "," +
             std::to_string(expected_pt.lng());

  // Test expected index
  EXPECT_EQ(std::get<2>(result), expected_idx)
      << "TryClosestPointNoDistance index test failed - found: " +
             std::to_string(std::get<2>(result)) + " | expected: " + std::to_string(expected_idx);
}

TEST(PointLL, TestClosestPoint) {
  // Test no points
  std::vector<PointLL> pts0;
  TryClosestPoint(pts0, PointLL(-76.299179, 40.042572), PointLL(), std::numeric_limits<double>::max(),
                  -1, -1);

  // Test one point - should fail then update code to do sqrt
  std::vector<PointLL> pts1 = {{-76.299171, 40.042519}};
  TryClosestPoint(pts1, PointLL(-76.299179, 40.042572), PointLL(-76.299171, 40.042519), 5.933, 0, 0);

  // Construct a simple polyline
  std::vector<PointLL> pts = {{-76.299171, 40.042519},
                              {-76.298851, 40.042549},
                              {-76.297806, 40.042671},
                              {-76.297691, 40.042015},
                              {-76.296837, 40.042099}};

  // Closest to the 1st point
  TryClosestPoint(pts, PointLL(-76.299189, 40.042572), PointLL(-76.299171, 40.042519), 5.933, 0, 4);

  // Closest along the 2nd segment
  TryClosestPoint(pts, PointLL(-76.298477, 40.042645), PointLL(-76.298470, 40.042595), 5.592, 1, 2);

  // Closest to third shape point
  TryClosestPoint(pts, PointLL(-76.297806, 40.042671), PointLL(-76.297806, 40.042671), 0., 2, 2);

  // Closest along the 3rd segment
  TryClosestPoint(pts, PointLL(-76.297752, 40.042183), PointLL(-76.297722, 40.042187), 2.592, 2, 1);

  // Closest along the 3rd segment with begin_index = 2
  TryClosestPoint(pts, PointLL(-76.297752, 40.042183), PointLL(-76.297722, 40.042187), 2.592, 2, 1,
                  2);

  // Closest along the 4th segment
  TryClosestPoint(pts, PointLL(-76.297020, 40.042133), PointLL(-76.297012, 40.042084), 5.491, 3, 0);

  // Closest to the last point
  TryClosestPoint(pts, PointLL(-76.296700, 40.042114), PointLL(-76.296837, 40.042099), 11.78, 4, 0);

  // Closest to the last point with begin_index = 4 - therefore, special case of one point
  TryClosestPoint(pts, PointLL(-76.296700, 40.042114), PointLL(-76.296837, 40.042099), 11.78, 4, 0,
                  4);

  // Invalid begin_index of 5
  TryClosestPoint(pts, PointLL(-76.299179, 40.042572), PointLL(), std::numeric_limits<double>::max(),
                  -1, -1, 5);

  // Try at high latitude where we need to properly project the closest point
  std::vector<PointLL> shape = {{-97.2987, 50.4072}, {-97.3208, 50.4265}};
  TryClosestPointNoDistance(shape, PointLL(-97.3014, 50.4212), PointLL(-97.310097, 50.417156), 0);
}

void TryWithinConvexPolygon(const std::vector<PointLL>& pts, const PointLL& p, const bool res) {
  EXPECT_EQ(p.WithinPolygon(pts), res);
}

TEST(PointLL, TestWithinConvexPolygon) {
  // Construct a convex polygon
  std::vector<PointLL> pts = {{2.0, 2.0}, {0.0, 4.0}, {-10.0, 0.0}, {0.0, -4.0}, {2.0, -2.0}};

  // Inside
  TryWithinConvexPolygon(pts, PointLL(0.0, 0.0), true);

  // Check a vertex - should be inside
  TryWithinConvexPolygon(pts, PointLL(0.0, -3.99), true);
  TryWithinConvexPolygon(pts, PointLL(1.99, -2.0), true);

  // Outside
  TryWithinConvexPolygon(pts, PointLL(15.0, 4.0), false);
  TryWithinConvexPolygon(pts, PointLL(2.5, 0.0), false);
  TryWithinConvexPolygon(pts, PointLL(-3.0, 3.0), false);
  TryWithinConvexPolygon(pts, PointLL(1.0, -3.5), false);
}

TEST(PointLL, TestMidPoint) {
  // lines of longitude are geodesics so the mid point of points
  // on the same line of longitude should still be at the same longitude
  auto mid = PointLL(0, 90).PointAlongSegment({0, 0});
  ASSERT_POINTLL_EQUAL(mid, PointLL(0, 45));
  mid = PointLL(0, 90).PointAlongSegment({0, -66});
  ASSERT_POINTLL_EQUAL(mid, PointLL(0, 12));

  // lines of latitude are not geodesics so if we put them 180 degrees apart
  // the shortest path between them is actually the geodesic that intersects
  // the pole. longitude is meaningless then
  mid = PointLL(-23, 45).PointAlongSegment({157, 45});
  EXPECT_EQ(mid.second, 90);

  // in the northern hemisphere we should expect midpoints on
  // geodesics between point of the same latitude to have higher latitude
  mid = PointLL(-15, 45).PointAlongSegment({15, 45});
  EXPECT_GT(mid.second, 45.1);
  mid = PointLL(-80, 1).PointAlongSegment({80, 1});
  EXPECT_GT(mid.second, 1.1);

  // conversely in the southern hemisphere we should expect them lower
  mid = PointLL(-15, -45).PointAlongSegment({15, -45});
  EXPECT_LT(mid.second, -45.1);
  mid = PointLL(-80, -1).PointAlongSegment({80, -1});
  EXPECT_LT(mid.second, -1.1);

  // the equator is the only line of latitude that is also a geodesic
  mid = PointLL(-15, 0).PointAlongSegment({15, 0});
  ASSERT_POINTLL_EQUAL(mid, PointLL(0, 0));
  mid = PointLL(-170, 0).PointAlongSegment({160, 0});
  ASSERT_POINTLL_EQUAL(mid, PointLL(175, 0));

  // take a random geodesic and see if the midpoint is the correct distance along it
  mid = PointLL(-12.5, 4).PointAlongSegment(PointLL(7.123, -18.945), .33);
  double mdist = PointLL(-12.5, 4).Distance(mid);
  double dist = PointLL(-12.5, 4).Distance(PointLL(7.123, -18.945));
  ASSERT_NEAR(mdist, .33 * dist, 1e-8);
  mid = PointLL(81.2366, -34.54987).PointAlongSegment(PointLL(-176.123, 81.945), .66);
  mdist = PointLL(81.2366, -34.54987).Distance(mid);
  dist = PointLL(81.2366, -34.54987).Distance(PointLL(-176.123, 81.945));
  ASSERT_NEAR(mdist, .66 * dist, 1e-8);
}

TEST(PointLL, TestDistance) {
  auto d = PointLL(-90.0, 0.0).Distance({90.0, 0.0});
  EXPECT_EQ(d, kPi * kRadEarthMeters) << "Distance 180 from each other should be PI * earth radius";

  d = PointLL(-90.0, 0.0).Distance({-90.0, 0.0});
  EXPECT_EQ(d, 0.0) << "Distance between same points should be 0";

  d = PointLL(45.0, 45.0).Distance({45.0, 40.0});
  EXPECT_NEAR(d, 556599.5, 1.0) << "Distance d = " + std::to_string(d) +
                                       " between points should be approx 556599.5 meters";
}

} // namespace

// todo: add many more tests!

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
