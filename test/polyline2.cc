#include "midgard/polyline2.h"
#include <cstdint>

#include <algorithm>
#include <vector>

#include "midgard/point2.h"

#include "test.h"

using namespace std;
using namespace valhalla::midgard;

namespace {

template <typename PrecisionT>
void TryGeneralizeAndLength(Polyline2<PointXY<PrecisionT>>& pl, const float& gen, const float& res) {
  uint32_t size = pl.Generalize(gen);

  std::vector<PointXY<PrecisionT>> pts = pl.pts();

  EXPECT_EQ(pl.pts().size(), 2);
  EXPECT_EQ(pts[0], PointXY<PrecisionT>(25.0, 25.0));
  EXPECT_EQ(pts[1], PointXY<PrecisionT>(50.0, 100.0));

  Polyline2<PointXY<PrecisionT>> pl2;
  pl2 = pl.GeneralizedPolyline(gen);

  EXPECT_EQ(pl2.pts().size(), 2);
  EXPECT_EQ(pl2.pts().at(0), PointXY<PrecisionT>(25.0, 25.0));
  EXPECT_EQ(pl2.pts().at(1), PointXY<PrecisionT>(50.0, 100.0));

  // TODO: there was a bug in test and it never ran
  // TODO: currently it doesn't compare up to kEpsilon precision
  EXPECT_NEAR(pl2.Length(), res, 1e-4);
}

TEST(Polyline2, TestGeneralizeAndLength) {
  std::vector<Point2> pts = {Point2(25.0f, 25.0f), Point2(50.0f, 50.0f), Point2(25.0f, 75.0f),
                             Point2(50.0f, 100.0f)};
  Polyline2<Point2> pl(pts);
  TryGeneralizeAndLength(pl, 100.0f, 79.0569f);
}

TEST(Polyline2, TestGeneralizeAndLengthWithDoubles) {
  std::vector<Point2d> pts = {Point2d(25.0, 25.0), Point2d(50.0, 50.0), Point2d(25.0, 75.0),
                              Point2d(50.0, 100.0)};
  Polyline2<Point2d> pl(pts);
  TryGeneralizeAndLength(pl, 100.0, 79.0569);
}

TEST(Polyline2, TestGeneralizeSimplification) {
  Polyline2<Point2> line{{{17, 0}, {17, 1}, {17, 2}, {17, 3}, {17, 4}, {17, 5}}};

  line.Generalize(1, std::unordered_set<size_t>{2, 4});
  EXPECT_EQ(line, (Polyline2<Point2>{{{17, 0}, {17, 2}, {17, 4}, {17, 5}}}))
      << "Should have removed all but the first, last and marked points";

  line.Generalize(1, std::unordered_set<size_t>{2});
  EXPECT_EQ(line, (Polyline2<Point2>{{{17, 0}, {17, 4}, {17, 5}}}))
      << "Should have removed all but the first, last and marked points";

  {
    Polyline2<PointLL> line{
        {{-76.58489, 40.31402}, {-76.58496, 40.31411}, {-76.58506, 40.31416}, {-76.58521, 40.31414},
         {-76.58586, 40.31383}, {-76.58596, 40.31379}, {-76.58658, 40.31349}, {-76.58723, 40.31319},
         {-76.58787, 40.31286}, {-76.58842, 40.31362}, {-76.58865, 40.31427}, {-76.58895, 40.31514},
         {-76.58921, 40.31579}, {-76.58923, 40.31582}, {-76.58924, 40.31586}, {-76.58924, 40.31589},
         {-76.58994, 40.31779}, {-76.59043, 40.31924}, {-76.59077, 40.32019}, {-76.5922, 40.32265},
         {-76.5927, 40.32239},  {-76.59319, 40.32216}, {-76.59346, 40.32202}, {-76.59371, 40.32189},
         {-76.59594, 40.32079}, {-76.59701, 40.32033}, {-76.59809, 40.31994}, {-76.59971, 40.31932},
         {-76.60005, 40.31922}, {-76.60037, 40.31917}, {-76.6011, 40.31905}}};
    line.Generalize(10, std::unordered_set<size_t>{3, 7, 11, 15, 19, 23, 27});
    std::vector<PointLL> remaining = {{-76.58489, 40.31402}, {-76.58521, 40.31414},
                                      {-76.58723, 40.31319}, {-76.58895, 40.31514},
                                      {-76.58924, 40.31589}, {-76.5922, 40.32265},
                                      {-76.59371, 40.32189}, {-76.59971, 40.31932},
                                      {-76.6011, 40.31905}};
    for (const auto& p : remaining) {
      ASSERT_NE(std::find(line.pts().cbegin(), line.pts().cend(), p), line.pts().cend())
          << "Should still have at least the first, last and marked points";
    }
  }

  {
    Polyline2<GeoPoint<double>> line{
        {{-76.58489, 40.31402}, {-76.58496, 40.31411}, {-76.58506, 40.31416}, {-76.58521, 40.31414},
         {-76.58586, 40.31383}, {-76.58596, 40.31379}, {-76.58658, 40.31349}, {-76.58723, 40.31319},
         {-76.58787, 40.31286}, {-76.58842, 40.31362}, {-76.58865, 40.31427}, {-76.58895, 40.31514},
         {-76.58921, 40.31579}, {-76.58923, 40.31582}, {-76.58924, 40.31586}, {-76.58924, 40.31589},
         {-76.58994, 40.31779}, {-76.59043, 40.31924}, {-76.59077, 40.32019}, {-76.5922, 40.32265},
         {-76.5927, 40.32239},  {-76.59319, 40.32216}, {-76.59346, 40.32202}, {-76.59371, 40.32189},
         {-76.59594, 40.32079}, {-76.59701, 40.32033}, {-76.59809, 40.31994}, {-76.59971, 40.31932},
         {-76.60005, 40.31922}, {-76.60037, 40.31917}, {-76.6011, 40.31905}}};
    line.Generalize(10, std::unordered_set<size_t>{3, 7, 11, 15, 19, 23, 27});
    std::vector<GeoPoint<double>> remaining = {{-76.58489, 40.31402}, {-76.58521, 40.31414},
                                               {-76.58723, 40.31319}, {-76.58895, 40.31514},
                                               {-76.58924, 40.31589}, {-76.5922, 40.32265},
                                               {-76.59371, 40.32189}, {-76.59971, 40.31932},
                                               {-76.6011, 40.31905}};
    for (const auto& p : remaining) {
      ASSERT_NE(std::find(line.pts().cbegin(), line.pts().cend(), p), line.pts().cend())
          << "Should still have at least the first, last and marked points";
    }
  }

  {

    Polyline2<Point2> line{{{-79.3837, 43.6481},
                            {-79.3839, 43.6485},
                            {-79.3839, 43.6485},
                            {-79.3839, 43.6486},
                            {-79.3842, 43.6491},
                            {-79.3842, 43.6492},
                            {-79.3842, 43.6492},
                            {-79.3841, 43.6493},
                            {-79.3841, 43.6493},
                            {-79.384, 43.6493},
                            {-79.3841, 43.6496},
                            {-79.384, 43.6496},
                            {-79.384, 43.6496},
                            {-79.3839, 43.6496},
                            {-79.3839, 43.6496},
                            {-79.3838, 43.6496}}};
    line.Generalize(2.6f, std::unordered_set<size_t>{15, 14, 13, 0, 10, 6, 9});

    EXPECT_EQ(line, (Polyline2<Point2>{{{-79.3837, 43.6481},
                                        {-79.3842, 43.6492},
                                        {-79.384, 43.6493},
                                        {-79.3841, 43.6496},
                                        {-79.3839, 43.6496},
                                        {-79.3839, 43.6496},
                                        {-79.3838, 43.6496}}}))
        << "Wrong points removed.";
  }

  {
    Polyline2<Point2> line{{{-79.3837, 43.6481},
                            {-79.3839, 43.6485},
                            {-79.3839, 43.6485},
                            {-79.3839, 43.6486},
                            {-79.3842, 43.6491},
                            {-79.3842, 43.6492},
                            {-79.3842, 43.6492},
                            {-79.3841, 43.6493},
                            {-79.3841, 43.6493},
                            {-79.384, 43.6493},
                            {-79.3841, 43.6496},
                            {-79.384, 43.6496},
                            {-79.384, 43.6496},
                            {-79.3839, 43.6496},
                            {-79.3839, 43.6496},
                            {-79.3838, 43.6496}}};
    line.Generalize(0.f);

    EXPECT_EQ(line.pts().size(), 16) << "No points should be removed.";
  }
}

void TryClosestPoint(const Polyline2<Point2>& pl, const Point2& a, const Point2& b) {
  auto result = pl.ClosestPoint(a);
  EXPECT_EQ(std::get<0>(result), b);
}

TEST(Polyline2, TestClosestPoint) {
  Point2 a(25.0f, 25.0f);
  Point2 b(50.0f, 50.0f);
  Point2 c(25.0f, 75.0f);
  Point2 d(50.0f, 100.0f);

  // Test adding points to polyline
  Polyline2<Point2> pl;
  pl.Add(a);
  pl.Add(b);
  pl.Add(c);
  pl.Add(d);

  Point2 beg(0.0f, 0.0f);
  TryClosestPoint(pl, beg, a);

  Point2 mid(60.0f, 50.0f);
  TryClosestPoint(pl, mid, b);

  Point2 end(50.0f, 125.0f);
  TryClosestPoint(pl, end, d);
}

void TryClip(Polyline2<Point2>& pl, const AABB2<Point2>& a, const uint32_t exp) {
  // Clip and check vertex count and 1st 2 points
  uint32_t x = pl.Clip(a);
  EXPECT_EQ(x, exp) << "Clip test failed: count not correct";

  EXPECT_EQ(pl.pts().at(0), Point2(25.0f, 25.0f));
  EXPECT_EQ(pl.pts().at(1), Point2(50.0f, 50.0f));
}

void TryClipOutside(Polyline2<Point2>& pl, const AABB2<Point2>& a) {
  uint32_t x = pl.Clip(a);
  EXPECT_EQ(x, 0) << "Clip test failed: all vertices outside so count should be 0";
}

TEST(Polyline2, TestClip) {
  std::vector<Point2> pts = {Point2(25.0f, 25.0f), Point2(50.0f, 50.0f), Point2(25.0f, 75.0f),
                             Point2(50.0f, 100.0f)};
  Polyline2<Point2> pl(pts);
  TryClip(pl, AABB2<Point2>(Point2(0.0f, 0.0f), Point2(75.0f, 50.0f)), 2);

  // Test with vertices on edges
  Polyline2<Point2> pl2(pts);
  TryClip(pl2, AABB2<Point2>(Point2(25.0f, 25.0f), Point2(50.0f, 100.0f)), 4);

  // Test Clip with all vertices above top of AABB
  Polyline2<Point2> pl3(pts);
  TryClipOutside(pl3, AABB2<Point2>(Point2(0.0f, 0.0f), Point2(50.0f, 20.0f)));

  // Test Clip with all vertices left of AABB
  Polyline2<Point2> pl4(pts);
  TryClipOutside(pl4, AABB2<Point2>(Point2(50.0f, 25.0f), Point2(100.0f, 100.0f)));

  // Test Clip with all vertices right of AABB
  Polyline2<Point2> pl5(pts);
  TryClipOutside(pl5, AABB2<Point2>(Point2(0.0f, 25.0f), Point2(10.0f, 100.0f)));

  // Test Clip with all vertices below bottom of AABB
  Polyline2<Point2> pl6(pts);
  TryClipOutside(pl6, AABB2<Point2>(Point2(25.0f, 100.0f), Point2(50.0f, 200.0f)));
}

void TryClippedPolyline(Polyline2<Point2>& pl, const AABB2<Point2>& a) {
  Polyline2<Point2> pl2 = pl.ClippedPolyline(a);
  uint32_t x = pl2.pts().size();
  EXPECT_EQ(x, 2) << "count not correct";

  EXPECT_EQ(pl2.pts().at(0), Point2(25.0f, 25.0f));
  EXPECT_EQ(pl2.pts().at(1), Point2(50.0f, 50.0f));
}

TEST(Polyline2, TestClippedPolyline) {
  std::vector<Point2> pts = {Point2(25.0f, 25.0f), Point2(50.0f, 50.0f), Point2(25.0f, 75.0f),
                             Point2(50.0f, 100.0f)};
  Polyline2<Point2> pl(pts);
  TryClippedPolyline(pl, AABB2<Point2>(Point2(0.0f, 0.0f), Point2(75.0f, 50.0f)));
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
