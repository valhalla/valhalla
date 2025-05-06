#include "midgard/distanceapproximator.h"
#include "midgard/constants.h"
#include "midgard/pointll.h"
#include "test.h"

using namespace std;
using namespace valhalla::midgard;

namespace {

void TryMetersPerDegreeLongitude(const PointLL& p, const float d2) {
  EXPECT_NEAR(DistanceApproximator<PointLL>::MetersPerLngDegree(p.lat()), d2, kEpsilon);
}

TEST(DistanceApproximator, TestMetersPerDegreeLongitude) {
  TryMetersPerDegreeLongitude(PointLL(-80.0f, 0.0f), kMetersPerDegreeLat);
}

void TryDistanceSquaredFromTestPt(const PointLL& testpt, const PointLL& p, const float d2) {
  // Test if distance is within 2% of the spherical distance
  DistanceApproximator<PointLL> approx(testpt);
  float d = sqrtf(approx.DistanceSquared(p));
  // std::cout << " d = " << d << " ArcDistance = " << d2 << std::endl;
  EXPECT_NEAR(d / d2, 1, 0.02f);
}

TEST(DistanceApproximator, TestDistanceSquaredFromTestPt) {
  PointLL p1(-80.0f, 42.0f);
  PointLL p2(-78.0f, 40.0f);
  TryDistanceSquaredFromTestPt(p2, p1, p1.Distance(p2));
  TryDistanceSquaredFromTestPt(p1, p2, p1.Distance(p2));
}

void TryDistanceSquared(const PointLL& a, const PointLL& b, const float d2) {
  // Test if distance is > 2% the spherical distance
  float d = sqrtf(DistanceApproximator<PointLL>::DistanceSquared(a, b));
  // std::cout << " d = " << d << " ArcDistance = " << d2 << std::endl;
  EXPECT_NEAR(d / d2, 1, 2.0f) << "DistanceSquared between 2 points test failed";
}

TEST(DistanceApproximator, TestDistanceSquared) {
  PointLL a(-80.0f, 42.0f);
  PointLL b(-78.0f, 40.0f);
  auto d = a.Distance(b);
  TryDistanceSquared(a, b, d * d);
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}