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

TEST(Polyline2, PeuckerSelfIntersectionTest1) {
  // These are real-world coordinates pulled off an isochrone polygon with gen_factor=0.
  // Using the raw Douglas-Peucker algorithm results in a self-intersection (using a
  // gen_factor=5). The modified Douglas-Peucker algorithm avoids the self-intersection.
  std::vector<PointLL> points =
      {{-117.20467966, 33.77518033}, {-117.20394301, 33.77518757}, {-117.20303785, 33.77482215},
       {-117.20251280, 33.77391699}, {-117.20232287, 33.77353715}, {-117.20194304, 33.77334723},
       {-117.20100573, 33.77297971}, {-117.20086145, 33.77299859}, {-117.20082284, 33.77279683},
       {-117.20026941, 33.77191702}, {-117.20016060, 33.77169937}, {-117.19994294, 33.77159056},
       {-117.19962097, 33.77159503}, {-117.19822555, 33.77163442}, {-117.19794297, 33.77163847},
       {-117.19749885, 33.77147289}, {-117.19604794, 33.77181206}, {-117.19596388, 33.76993787},
       {-117.19595160, 33.76991698}, {-117.19594873, 33.76991125}, {-117.19594299, 33.76990838},
       {-117.19593012, 33.76990411}, {-117.19587159, 33.76991698}, {-117.19593906, 33.76992091},
       {-117.19592176, 33.77189578}, {-117.19593198, 33.77191702}, {-117.19592806, 33.77193195},
       {-117.19594299, 33.77196341}, {-117.19599274, 33.77196676}, {-117.19768004, 33.77217994},
       {-117.19794297, 33.77219556}, {-117.19859647, 33.77257053}, {-117.19924840, 33.77261156},
       {-117.19916533, 33.77313940}, {-117.19948144, 33.77391699}, {-117.19963527, 33.77422466},
       {-117.19994294, 33.77437851}, {-117.20090806, 33.77495196}, {-117.20132555, 33.77591702},
       {-117.20153140, 33.77632866}, {-117.20194304, 33.77653447}, {-117.20258894, 33.77656294}};

  constexpr double gen_factor = 5.0;

  {
    // Allow self-intersections, see them occur.
    Polyline2<PointLL> polyline(points);
    polyline.Generalize(gen_factor, {}, /* avoid self-intersections? */ false);
    std::vector<PointLL> intersections = polyline.GetSelfIntersections();
    ASSERT_EQ(intersections.size(), 1);
  }

  {
    // Avoid self-intersections, see none.
    Polyline2<PointLL> polyline(points);
    polyline.Generalize(gen_factor, {}, /* avoid self-intersections? */ true);
    std::vector<PointLL> intersections = polyline.GetSelfIntersections();
    ASSERT_EQ(intersections.size(), 0);
  }
}

TEST(Polyline2, PeuckerSelfIntersectionTest2) {
  // These are real-world coordinates pulled off an isochrone polygon with gen_factor=0.
  // Using the raw Douglas-Peucker algorithm results in a self-intersection (using a
  // gen_factor=50). The modified Douglas-Peucker algorithm avoids the self-intersection.
  std::vector<PointLL> points =
      {{-118.17329133, 33.78885961}, {-118.17410177, 33.78965699}, {-118.17407460, 33.79007635},
       {-118.17379829, 33.79096132}, {-118.17377209, 33.79165699}, {-118.17404231, 33.79210863},
       {-118.17449396, 33.79235270}, {-118.17518239, 33.79234540}, {-118.17601166, 33.79213932},
       {-118.17649399, 33.79213106}, {-118.17725704, 33.79242005}, {-118.17841540, 33.79173556},
       {-118.17774024, 33.79290326}, {-118.17803809, 33.79365699}, {-118.17807143, 33.79407953},
       {-118.17849397, 33.79548576}, {-118.17936408, 33.79452708}, {-118.18010898, 33.79365699},
       {-118.17945666, 33.79269433}, {-118.17891198, 33.79207499}, {-118.17855126, 33.79165699},
       {-118.17852006, 33.79163090}, {-118.17849397, 33.79161013}, {-118.17825732, 33.79142034},
       {-118.17741529, 33.79073567}, {-118.17649399, 33.79007552}, {-118.17627859, 33.78987239},
       {-118.17597193, 33.78965699}, {-118.17626466, 33.78942765}, {-118.17649399, 33.78910775},
       {-118.17736539, 33.78852840}, {-118.17778081, 33.78837014}, {-118.17849397, 33.78803414},
       {-118.17871682, 33.78787982}, {-118.17900272, 33.78765698}, {-118.17996996, 33.78713294},
       {-118.18049400, 33.78653054}, {-118.18094331, 33.78720766}, {-118.18082302, 33.78765698},
       {-118.18159441, 33.78855655}, {-118.18249397, 33.78881215}, {-118.18324743, 33.78841046}};

  constexpr double gen_factor = 50.0;

  {
    // Allow self-intersections, see them occur.
    Polyline2<PointLL> polyline(points);
    polyline.Generalize(gen_factor, {}, /* avoid self-intersections? */ false);
    std::vector<PointLL> intersections = polyline.GetSelfIntersections();
    ASSERT_EQ(intersections.size(), 2);
  }

  {
    // Avoid self-intersections, see none.
    Polyline2<PointLL> polyline(points);
    polyline.Generalize(gen_factor, {}, /* avoid self-intersections? */ true);
    std::vector<PointLL> intersections = polyline.GetSelfIntersections();
    ASSERT_EQ(intersections.size(), 0);
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
