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
    constexpr float kPrecision = .00001;                                                             \
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

void test_along(const std::vector<PointLL>& l, float d, float a) {
  auto r = PointLL::HeadingAlongPolyline(l, d);
  EXPECT_NEAR(r, a, 1.f) << "Invalid polyline begin heading was " + std::to_string(r) +
                                " but should be " + std::to_string(a);
}

void test_end(const std::vector<PointLL>& l, float d, float a) {
  auto r = PointLL::HeadingAtEndOfPolyline(l, d);
  EXPECT_NEAR(r, a, 1.f) << "Invalid polyline end heading was " + std::to_string(r) +
                                " but should be " + std::to_string(a);
}

TEST(PointLL, TestHeadingAlongPolyline) {
  // Test with empty (or 1 point) polyline
  std::vector<PointLL> empty;
  EXPECT_NEAR(PointLL::HeadingAlongPolyline(empty, 30.0f), 0.0f, 1.0f);

  std::vector<PointLL> withOnePoint;
  withOnePoint.emplace_back(-70.0f, 30.0f);
  EXPECT_NEAR(PointLL::HeadingAlongPolyline(withOnePoint, 30.0f), 0.0f, 1.0f);

  test_along({{-73.986392, 40.755800}, {-73.986438, 40.755819}}, 30.0f, 299);
  test_along({{-73.986438, 40.755819}, {-73.986484, 40.755681}}, 30.0f, 194);
  test_along({{-73.985777, 40.755539}, {-73.986440, 40.755820}, {-73.986617, 40.755254}}, 30.0f, 299);

  // Partial roundabout
  test_along({{-76.316360, 39.494102},
              {-76.316360, 39.494129},
              {-76.316376, 39.494152},
              {-76.316391, 39.494175},
              {-76.316422, 39.494194},
              {-76.316444, 39.494209},
              {-76.316483, 39.494221},
              {-76.316521, 39.494228}},
             30.0f, 315);

  // north (0)
  test_along({{-76.612682, 39.294540}, {-76.612681, 39.294897}, {-76.612708, 39.295208}}, 30.0f, 0);

  // east (90)
  test_along({{-76.612682, 39.294540},
              {-76.612508, 39.294535},
              {-76.612359, 39.294541},
              {-76.612151, 39.294545}},
             30.0f, 90);

  // south (176)
  test_along({{-76.612682, 39.294540},
              {-76.612670, 39.294447},
              {-76.612666, 39.294378},
              {-76.612659, 39.294280}},
             30.0f, 176);

  // west (266)
  test_along(
      {
          {-76.612682, 39.294540},
          {-76.612789, 39.294527},
          {-76.612898, 39.294525},
          {-76.613033, 39.294523},
      },
      30.0f, 266);
}

TEST(PointLL, TestHeadingAtEndOfPolyline) {
  // Test with empty (or 1 point) polyline
  std::vector<PointLL> empty;
  EXPECT_NEAR(PointLL::HeadingAtEndOfPolyline(empty, 30.0f), 0.0f, 1.0f);

  std::vector<PointLL> withOnePoint;
  withOnePoint.emplace_back(-70.0f, 30.0f);
  EXPECT_NEAR(PointLL::HeadingAtEndOfPolyline(withOnePoint, 30.0f), 0.0f, 1.0f);

  test_end({{-73.986392, 40.755800}, {-73.986438, 40.755819}}, 30.0f, 299);
  test_end({{-73.986438, 40.755819}, {-73.986484, 40.755681}}, 30.0f, 194);
  test_end({{-73.985777, 40.755539}, {-73.986440, 40.755820}, {-73.986617, 40.755254}}, 30.0f, 194);

  // Partial roundabout
  test_end({{-76.316360, 39.494102},
            {-76.316360, 39.494129},
            {-76.316376, 39.494152},
            {-76.316391, 39.494175},
            {-76.316422, 39.494194},
            {-76.316444, 39.494209},
            {-76.316483, 39.494221},
            {-76.316521, 39.494228}},
           30.0f, 315);

  // north (356)
  test_end({{-76.612682, 39.294540}, {-76.612681, 39.294897}, {-76.612708, 39.295208}}, 30.0f, 356);

  // east (89)
  test_end({{-76.612682, 39.294540},
            {-76.612508, 39.294535},
            {-76.612359, 39.294541},
            {-76.612151, 39.294545}},
           30.0f, 89);

  // south (176)
  test_end({{-76.612682, 39.294540},
            {-76.612670, 39.294447},
            {-76.612666, 39.294378},
            {-76.612659, 39.294280}},
           30.0f, 176);

  // west (266)
  test_end(
      {
          {-76.612682, 39.294540},
          {-76.612789, 39.294527},
          {-76.612898, 39.294525},
          {-76.613033, 39.294523},
      },
      30.0f, 266);
}

TEST(PointLL, TestHeadingPrecision) {
  float actual = PointLL(11.6057196, 48.1032867).Heading(PointLL(11.6056538, 48.1035118));
  float expected = 348.952393;
  EXPECT_NEAR(actual, expected, 0.000001);
}

void TryClosestPoint(std::vector<PointLL> pts,
                     const PointLL& pt,
                     const PointLL& expected_pt,
                     float expected_dist,
                     int expected_idx,
                     int expected_inverse_idx,
                     int begin_idx = 0) {

  // do forwards and backwards searches
  for (int reverse = 0; reverse < 2; ++reverse) {

    // if we are doing a backwards search we will just flip
    // the direction of the line string and invert the indices
    // inverting them gets tricky at the boundaries :o(
    float forward = std::numeric_limits<float>::infinity();
    float backward = 0;
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
    EXPECT_NEAR(std::get<1>(result), expected_dist, 0.5f);

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
  TryClosestPoint(pts0, PointLL(-76.299179f, 40.042572f), PointLL(),
                  std::numeric_limits<float>::max(), -1, -1);

  // Test one point - should fail then update code to do sqrt
  std::vector<PointLL> pts1 = {{-76.299171f, 40.042519f}};
  TryClosestPoint(pts1, PointLL(-76.299179f, 40.042572f), PointLL(-76.299171f, 40.042519f), 5.933f, 0,
                  0);

  // Construct a simple polyline
  std::vector<PointLL> pts = {{-76.299171f, 40.042519f},
                              {-76.298851f, 40.042549f},
                              {-76.297806f, 40.042671f},
                              {-76.297691f, 40.042015f},
                              {-76.296837f, 40.042099f}};

  // Closest to the 1st point
  TryClosestPoint(pts, PointLL(-76.299189f, 40.042572f), PointLL(-76.299171f, 40.042519f), 5.933f, 0,
                  4);

  // Closest along the 2nd segment
  TryClosestPoint(pts, PointLL(-76.298477f, 40.042645f), PointLL(-76.298470f, 40.042595f), 5.592f, 1,
                  2);

  // Closest to third shape point
  TryClosestPoint(pts, PointLL(-76.297806f, 40.042671f), PointLL(-76.297806f, 40.042671f), 0.f, 2, 2);

  // Closest along the 3rd segment
  TryClosestPoint(pts, PointLL(-76.297752f, 40.042183f), PointLL(-76.297722f, 40.042187f), 2.592f, 2,
                  1);

  // Closest along the 3rd segment with begin_index = 2
  TryClosestPoint(pts, PointLL(-76.297752f, 40.042183f), PointLL(-76.297722f, 40.042187f), 2.592f, 2,
                  1, 2);

  // Closest along the 4th segment
  TryClosestPoint(pts, PointLL(-76.297020f, 40.042133f), PointLL(-76.297012f, 40.042084f), 5.491f, 3,
                  0);

  // Closest to the last point
  TryClosestPoint(pts, PointLL(-76.296700f, 40.042114f), PointLL(-76.296837f, 40.042099f), 11.78f, 4,
                  0);

  // Closest to the last point with begin_index = 4 - therefore, special case of one point
  TryClosestPoint(pts, PointLL(-76.296700f, 40.042114f), PointLL(-76.296837f, 40.042099f), 11.78f, 4,
                  0, 4);

  // Invalid begin_index of 5
  TryClosestPoint(pts, PointLL(-76.299179f, 40.042572f), PointLL(), std::numeric_limits<float>::max(),
                  -1, -1, 5);

  // Try at high latitude where we need to properly project the closest point
  std::vector<PointLL> shape = {{-97.2987f, 50.4072f}, {-97.3208f, 50.4265f}};
  TryClosestPointNoDistance(shape, PointLL(-97.3014f, 50.4212f), PointLL(-97.310097f, 50.417156f), 0);
}

void TryWithinConvexPolygon(const std::vector<PointLL>& pts, const PointLL& p, const bool res) {
  EXPECT_EQ(p.WithinPolygon(pts), res);
}

TEST(PointLL, TestWithinConvexPolygon) {
  // Construct a convex polygon
  std::vector<PointLL> pts = {{2.0f, 2.0f},
                              {0.0f, 4.0f},
                              {-10.0f, 0.0f},
                              {0.0f, -4.0f},
                              {2.0f, -2.0f}};

  // Inside
  TryWithinConvexPolygon(pts, PointLL(0.0f, 0.0f), true);

  // Check a vertex - should be inside
  TryWithinConvexPolygon(pts, PointLL(0.0f, -3.99f), true);
  TryWithinConvexPolygon(pts, PointLL(1.99f, -2.0f), true);

  // Outside
  TryWithinConvexPolygon(pts, PointLL(15.0f, 4.0f), false);
  TryWithinConvexPolygon(pts, PointLL(2.5f, 0.0f), false);
  TryWithinConvexPolygon(pts, PointLL(-3.0f, 3.0f), false);
  TryWithinConvexPolygon(pts, PointLL(1.0f, -3.5f), false);
}

TEST(PointLL, TestMidPoint) {
  // lines of longitude are geodesics so the mid point of points
  // on the same line of longitude should still be at the same longitude
  auto mid = PointLL(0, 90).MidPoint({0, 0});
  ASSERT_POINTLL_EQUAL(mid, PointLL(0, 45));
  mid = PointLL(0, 90).MidPoint({0, -66});
  ASSERT_POINTLL_EQUAL(mid, PointLL(0, 12));

  // lines of latitude are not geodesics so if we put them 180 degrees apart
  // the shortest path between them is actually the geodesic that intersects
  // the pole. longitude is meaningless then
  mid = PointLL(-23, 45).MidPoint({157, 45});
  EXPECT_EQ(mid.second, 90);

  // in the northern hemisphere we should expect midpoints on
  // geodesics between point of the same latitude to have higher latitude
  mid = PointLL(-15, 45).MidPoint({15, 45});
  EXPECT_GT(mid.second, 45.1);
  mid = PointLL(-80, 1).MidPoint({80, 1});
  EXPECT_GT(mid.second, 1.1);

  // conversely in the southern hemisphere we should expect them lower
  mid = PointLL(-15, -45).MidPoint({15, -45});
  EXPECT_LT(mid.second, -45.1);
  mid = PointLL(-80, -1).MidPoint({80, -1});
  EXPECT_LT(mid.second, -1.1);

  // the equator is the only line of latitude that is also a geodesic
  mid = PointLL(-15, 0).MidPoint({15, 0});
  ASSERT_POINTLL_EQUAL(mid, PointLL(0, 0));
  mid = PointLL(-170, 0).MidPoint({160, 0});
  ASSERT_POINTLL_EQUAL(mid, PointLL(175, 0));
}

TEST(PointLL, TestDistance) {
  float d = PointLL(-90.0f, 0.0f).Distance({90.0f, 0.0f});
  EXPECT_EQ(d, kPi * kRadEarthMeters) << "Distance 180 from each other should be PI * earth radius";

  d = PointLL(-90.0f, 0.0f).Distance({-90.0f, 0.0f});
  EXPECT_EQ(d, 0.0f) << "Distance between same points should be 0";

  d = PointLL(45.0f, 45.0f).Distance({45.0f, 40.0f});
  EXPECT_NEAR(d, 556599.5f, 1.0f) << "Distance d = " + std::to_string(d) +
                                         " between points should be approx 556599.5 meters";
}

} // namespace

// todo: add many more tests!

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
