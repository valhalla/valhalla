#include "test.h"

#include "include/config.h"
#include <vector>
#include "geo/point2.h"
#include "geo/polyline2.h"

using namespace std;
using namespace valhalla::geo;

namespace {

void TryGeneralizeAndLength(Polyline2& pl, const float& gen, const float& res) {
  unsigned int size = pl.Generalize(gen);

  std::vector<Point2> pts = pl.pts();

  if (pl.pts().size() != 2 )
    throw runtime_error("Generalize #1 test failed.");

  if ((pts[0] != Point2(25.0f,25.0f)) || (pts[1] != Point2(50.0f,100.0f))) {
    throw runtime_error("Generalize #2 test failed.");
  }

  Polyline2 pl2;
  pl2 = pl.GeneralizedPolyline(gen);

  if (pl2.pts().size() != 2 )
    throw runtime_error("Generalize #3 test failed.");

  if ((pl2.pts().at(0) != Point2(25.0f,25.0f)) || (pl2.pts().at(1) != Point2(50.0f,100.0f))) {
    throw runtime_error("Generalize #2 test failed.");
  }

  if (!(fabs(pl2.Length() - res) > kEpsilon))
    throw runtime_error("Length test failed.");
}

void TestGeneralizeAndLength() {
  Point2 a(25.0f, 25.0f);
  Point2 b(50.0f, 50.0f);
  Point2 c(25.0f, 75.0f);
  Point2 d(50.0f, 100.0f);

  std::vector<Point2> pts;

  pts.push_back(a);
  pts.push_back(b);
  pts.push_back(c);
  pts.push_back(d);

  Polyline2 pl(pts);

  TryGeneralizeAndLength(pl, 100.0f, 79.0569f);
}

void TryClosestPoint(const Polyline2& pl, const Point2& a, const Point2& b) {

  Point2 c;
  int x;
  pl.ClosestPoint(a, c, x);

  if (c != b)
    throw runtime_error("ClosestPoint test failed.");
}

void TestClosestPoint() {
   Point2 a(25.0f, 25.0f);
   Point2 b(50.0f, 50.0f);
   Point2 c(25.0f, 75.0f);
   Point2 d(50.0f, 100.0f);

   Polyline2 pl;

   pl.Add(a);
   pl.Add(b);
   pl.Add(c);
   pl.Add(d);

   Point2 beg(0.0f, 0.0f);
   TryClosestPoint(pl,beg,a);

   Point2 mid(60.0f, 50.0f);
   TryClosestPoint(pl,mid,b);

   Point2 end(50.0f, 125.0f);
   TryClosestPoint(pl,end,d);
}

void TryClip(Polyline2& pl, const AABB2& a) {

  unsigned int x = pl.Clip(a);

  if (x != 2 )
    throw runtime_error("Clip #1 test failed.");

  if ((pl.pts().at(0) != Point2(25.0f,25.0f)) || (pl.pts().at(1) != Point2(50.0f,50.0f))) {
     throw runtime_error("Clip #2 test failed.");
   }
}

void TestClip() {
  Point2 a(25.0f, 25.0f);
  Point2 b(50.0f, 50.0f);
  Point2 c(25.0f, 75.0f);
  Point2 d(50.0f, 100.0f);

  std::vector<Point2> pts;

  pts.push_back(a);
  pts.push_back(b);
  pts.push_back(c);
  pts.push_back(d);

  Polyline2 pl(pts);

  Point2 minpt(0.0f, 0.0f);
  Point2 maxpt(75.0f, 50.0f);

  TryClip(pl, AABB2(minpt, maxpt));

}


void TryClippedPolyline(Polyline2& pl, const AABB2& a) {

  Polyline2 pl2 = pl.ClippedPolyline(a);

  unsigned int x = pl2.pts().size();

  if (x != 2 )
    throw runtime_error("ClippedPolyline #1 test failed.");

  if ((pl2.pts().at(0) != Point2(25.0f,25.0f)) || (pl2.pts().at(1) != Point2(50.0f,50.0f))) {
     throw runtime_error("ClippedPolyline #2 test failed.");
   }
}

void TestClippedPolyline() {
  Point2 a(25.0f, 25.0f);
  Point2 b(50.0f, 50.0f);
  Point2 c(25.0f, 75.0f);
  Point2 d(50.0f, 100.0f);

  std::vector<Point2> pts;

  pts.push_back(a);
  pts.push_back(b);
  pts.push_back(c);
  pts.push_back(d);

  Polyline2 pl(pts);

  Point2 minpt(0.0f, 0.0f);
  Point2 maxpt(75.0f, 50.0f);

  TryClippedPolyline(pl, AABB2(minpt, maxpt));

}

}

int main() {
  test::suite suite("polyline2");

  // Test Generalize
  suite.test(TEST_CASE(TestGeneralizeAndLength));

  // Test distance of a point to a line segment
  suite.test(TEST_CASE(TestClosestPoint));

  // Test clip
  suite.test(TEST_CASE(TestClip));

  // Test clipped polyline
  suite.test(TEST_CASE(TestClippedPolyline));

  return suite.tear_down();
}
