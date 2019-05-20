#include "midgard/distanceapproximator.h"

#include "midgard/constants.h"
#include "midgard/pointll.h"
#include "test.h"

using namespace std;
using namespace valhalla::midgard;

namespace {

void TryMetersPerDegreeLongitude(const PointLL& p, const float d2) {
  float d = DistanceApproximator::MetersPerLngDegree(p.lat());
  if (fabs(d - d2) > kEpsilon)
    throw runtime_error("KmPerDegreeLongitude test failed");
}

void TestMetersPerDegreeLongitude() {
  TryMetersPerDegreeLongitude(PointLL(-80.0f, 0.0f), kMetersPerDegreeLat);
}

void TryDistanceSquaredFromTestPt(const PointLL& testpt, const PointLL& p, const float d2) {
  // Test if distance is within 2% of the spherical distance
  DistanceApproximator approx(testpt);
  float d = sqrtf(approx.DistanceSquared(p));
  // std::cout << " d = " << d << " ArcDistance = " << d2 << std::endl;
  if (fabs(d - d2) / d2 > 0.02f)
    throw runtime_error("DistanceSquared from point test failed");
}
void TestDistanceSquaredFromTestPt() {
  PointLL p1(-80.0f, 42.0f);
  PointLL p2(-78.0f, 40.0f);
  TryDistanceSquaredFromTestPt(p2, p1, p1.Distance(p2));
  TryDistanceSquaredFromTestPt(p1, p2, p1.Distance(p2));
}

void TryDistanceSquared(const PointLL& a, const PointLL& b, const float d2) {
  // Test if distance is > 2% the spherical distance
  float d = sqrtf(DistanceApproximator::DistanceSquared(a, b));
  // std::cout << " d = " << d << " ArcDistance = " << d2 << std::endl;
  if (fabs(d - d2) / d2 > 2.0f)
    throw runtime_error("DistanceSquared between 2 points test failed");
}

void TestDistanceSquared() {
  PointLL a(-80.0f, 42.0f);
  PointLL b(-78.0f, 40.0f);
  auto d = a.Distance(b);
  TryDistanceSquared(a, b, d * d);
}

} // namespace

int main() {
  test::suite suite("distanceapproximator");

  // Test meters per degree longitude at a specified latitude
  suite.test(TEST_CASE(TestMetersPerDegreeLongitude));

  // Test distance squared from a test point
  suite.test(TEST_CASE(TestDistanceSquaredFromTestPt));

  // Test distance squared between 2 points
  suite.test(TEST_CASE(TestDistanceSquared));

  return suite.tear_down();
}
