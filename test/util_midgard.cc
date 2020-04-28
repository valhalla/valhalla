#include "midgard/constants.h"
#include "midgard/distanceapproximator.h"
#include "midgard/encoded.h"
#include "midgard/polyline2.h"
#include "midgard/util.h"
#include <cmath>
#include <random>

#include <list>

#include "test.h"

using namespace valhalla::midgard;

namespace {

TEST(UtilMidgard, TestRangedDefaultT) {
  // arbitrary values
  constexpr float lower = -50;
  constexpr float upper = 70;
  constexpr unsigned seed = 0;
  std::mt19937 generator(seed);
  std::uniform_real_distribution<float> defaultDistributor(lower, upper);
  std::uniform_real_distribution<float> testDistributor(lower - 40, upper + 40);

  for (unsigned i = 0; i < 100; ++i) {
    ranged_default_t<float> testRange{lower, defaultDistributor(generator), upper};
    float defaultVal = testRange.def;
    float testVal = testDistributor(generator);

    float finalVal = testRange(testVal);

    if (testVal < testRange.min || testVal > testRange.max) {
      // Was outside of range so finalVal should now be snapped to default
      EXPECT_EQ(finalVal, testRange.def) << "Final value did not snap to the range default value";
    } else {
      // Was inside of range so finalVal should still be the same number
      EXPECT_EQ(finalVal, testVal) << "Final value moved invalidly";
    }

    // Test Edge cases because random distribution is unlikely to land exactly on boundaries
    finalVal = testRange(testRange.min);
    EXPECT_EQ(finalVal, testRange.min) << "Final value invalidly moves on lower bound";

    finalVal = testRange(testRange.max);
    EXPECT_EQ(finalVal, testRange.max) << "Final value invalidly moves on upper bound";
  }
}

TEST(UtilMidgard, TestGetTurnDegree) {
  // Slight Right
  EXPECT_EQ(GetTurnDegree(315, 335), 20) << "Invalid turn degree";
  // Right
  EXPECT_EQ(GetTurnDegree(0, 90), 90) << "Invalid turn degree";
  // Right
  EXPECT_EQ(GetTurnDegree(90, 180), 90) << "Invalid turn degree";
  // Sharp Right
  EXPECT_EQ(GetTurnDegree(180, 340), 160) << "Invalid turn degree";
  // Sharp Right
  EXPECT_EQ(GetTurnDegree(180, 352), 172) << "Invalid turn degree";
  // Sharp Left
  EXPECT_EQ(GetTurnDegree(180, 40), 220) << "Invalid turn degree";
  // Sharp Left
  EXPECT_EQ(GetTurnDegree(180, 10), 190) << "Invalid turn degree";
  // Left
  EXPECT_EQ(GetTurnDegree(0, 180), 180) << "Invalid turn degree";
  // Left
  EXPECT_EQ(GetTurnDegree(270, 180), 270) << "Invalid turn degree";
  // Slight Left
  EXPECT_EQ(GetTurnDegree(90, 70), 340) << "Invalid turn degree";
  // Continue
  EXPECT_EQ(GetTurnDegree(358, 2), 4) << "Invalid turn degree";
}

TEST(UtilMidgard, TestGetTime) {
  EXPECT_EQ(GetTime(100, 100), 3600) << "Invalid time";
  EXPECT_EQ(GetTime(5, 20), 900) << "Invalid time";
  EXPECT_EQ(GetTime(5, 0), 0) << "Invalid time";
}

TEST(UtilMidgard, AppxEqual) {
  EXPECT_TRUE(equal<float>(-136.170790, -136.170800, .00002f));
  EXPECT_TRUE(equal<float>(-136.170800, -136.170790, .00002f));
  EXPECT_TRUE(equal<float>(16.645590, 16.645580, .00002f));
  EXPECT_TRUE(equal<float>(76.627980, 76.627970, .00002f));
  EXPECT_TRUE(equal<int>(0, 0));
  EXPECT_TRUE(equal<float>(1, 1, 0));
}

TEST(UtilMidgard, MemoryStatus) {
  // only check this if the os supports it (system must have /proc/self/status)
  if (memory_status::supported()) {
    memory_status status({"VmSize", "VmSwap", "VmPeak"});

    // should have each of these
    for (const auto& key : {"VmSize", "VmSwap", "VmPeak"}) {
      auto value = status.metrics.find(key);
      ASSERT_NE(value, status.metrics.end()) << "Missing memory statistic for " + std::string(key);
      EXPECT_GE(value->second.first, 0.) << "Negative memory usage values are not allowed";
      EXPECT_EQ(value->second.second.back(), 'B') << "Units should be some magnitude of bytes";
    }
  }
}

TEST(UtilMidgard, TestClamp) {
  EXPECT_TRUE(equal<float>(circular_range_clamp<float>(467, -90, 90), -73));
  EXPECT_TRUE(equal<float>(circular_range_clamp<float>(-467, -90, 90), 73));
  EXPECT_TRUE(equal<float>(circular_range_clamp<float>(7, -90, 90), 7));
  EXPECT_TRUE(equal<float>(circular_range_clamp<float>(-67, -90, 90), -67));
  EXPECT_TRUE(equal<float>(circular_range_clamp<float>(-97, -90, 90), 83));
  EXPECT_TRUE(equal<float>(circular_range_clamp<float>(-97.2, -90, 90), 82.8));
  EXPECT_TRUE(equal<float>(circular_range_clamp<float>(-180, -90, 90), 0));
  EXPECT_TRUE(equal<float>(circular_range_clamp<float>(270, -90, 90), -90));
  EXPECT_TRUE(equal<float>(circular_range_clamp<float>(369, 0, 360), 9));
  EXPECT_TRUE(equal<float>(circular_range_clamp<float>(-369, 0, 360), 351));
  EXPECT_TRUE(equal<float>(circular_range_clamp<float>(739, -45, -8), -38));

  // Test invalid range - should throw an exception
  EXPECT_THROW(circular_range_clamp<float>(739, -8, -45);, std::runtime_error)
      << "should throw exception (lower > upper)";

  // Make sure get_turn_degree180 throws an exception if inbound and outbound
  // degrees are not clamped to 0,360
  EXPECT_THROW(get_turn_degree180(420, 580);, std::invalid_argument)
      << "get_turn_degree should throw exception (inputs not in 0,360 range)";
}

TEST(UtilMidgard, TestResample) {
  for (const auto& example : std::vector<std::pair<float, std::string>>{
           {100.f,
            "cfcglAlj_~pCsiAdOaeAvN}_@|ImTyBiW}I}TsQ}d@}^cUyWcGaHoNcPc`@oh@ykAw`BuTeZkt@emAquAk}"
            "BucAelBwXqg@o|@{~@oSiBuOkAiCtDw`AxMaHxBmUpGcFzAe_Atm@_w@ju@wb@hWu_@~Ied@}@wb@uO}_@"
            "uDmTrFwb@~g@wNfw@jGrxBiClJy\\yBsvC_hAwN|@wNl^qBvYiH`fAjGxwGn|@vpFiC|eD?z@cK|"
            "pCcAhWsGjj@qQju@iWtOwXrF}s@?m}Agw@g`BmhAycC{rB}gAgw@ceAiw@_|AofAcPyByHsQy[kUgh@"
            "qHyvDjV_cApGa{@vm@iWfYuD?{AhLwXn^m@hL_I|@slClcEwb@fNalAoH{mA}^kdBk`Aso@iLm^z@cBzAsK|"
            "JvD~|@vXv`Am@rQwDpHso@e[_xDu~Cy}GqvEoT{Ked@oHen@z@gTzKwX|^egBnbEuh@dmAwdCdrF_eC~uF"},
           {60.f,
            "etgflAbin|pCjFxWvNls@vMhl@xM~]|JtYzEb\\dF~Rp\\ngAzObo@jQnh@xMfYfIlJvHxWtJb["
            "tJvWjLxWlExMpVvl@hRbe@hRhb@zKp\\bGtYtJhMfNdPfI`RvHtOhN~SvHbQvMhV|Pnh@bPxWnTxWfMzUdKpR`"
            "CpGTpGcAbGqBfD_^l_@{PbPeFbRuJdYkPlJwNjAgIbGsApH~Cx`@dFdPbFjKt@nJ?"
            "lJyGfCcGiBuJkAoCfCOrGrFzU|DvCdFdFbAvCoC|IsFvDoJrFmYdPqBnHLrGdA~"
            "HpGnHfNnJpRvCvStOlTbQhM|TtEhMdOx`@jLlUdK~HlNvCxM|@vNpGdJMln@un@`XqRvRoSlPuOpGwCrFdDlU|"
            "_@cBtOaGzUaSre@aRfX}T~]mUz`@sZfc@cGtD_XtZ`BvCh\\_S|ErFxQjU`DvO`BrF{@zJyCbGd@"
            "xBpCiBnDaIpBeEvCzApCpH`BzJe@rQmEfYeEfCwDxMqB~R}D`SeEnHoDvCwDvDaBjK_@|"
            "JrBnIlDjAdFrFtDlJNbG\\lJbBrPt^vw@`]tx@`BnJtFfMtDhClEzUfDbQNtOeAlKaBpG_@lJqB~"
            "HMxMsFbFaD`HqB~HjAvDd@nIgCtEsFfCqHl@cFiBgEwDoCiCqB^qCxByLhWcGnIsAhMuD`HePdY}JjLsF|"
            "IuNfOyHbFeJ{AuJzAcGpG]nJu@`GqBj`@aC`I?xLtE`HvHbQrGdOfIxMvI~RrFjV|"
            "DtEhGdEpCLdOnIxCrGlDdc@NzK}DhMeAlJ^tF~BrEbB`Id@~HcAzKaCpGcGhWiGnTyMjUuI`"
            "HcLnIuJbGyMfCgS|@oCm@mF{AyG{AkFjAeEzBiCrFwCjAkB_@aGeOe@yBsA{@sA{AsA{AsBkAs@z@oDgCeK}@"
            "iBiB{@kAmAcGe@]aB}@yB{@kB?uIyCyCm@gIkAiBMe@L}@?aBMcBLiB?u@NwD?UkA{A{AsA}@m@wCkAiBkA{"
            "AsA{AqB{@yB_@yB|@iC?yBzAyBrF{@zAmAjAk@hBsBNqLpGsFjByBz@iCiBoC?qMLej@{J{y@rF_DxAgSfEcp@"
            "fN}EjAcVz@sExByBhCgIlJcB|@cBLoCdF{Al@aBz@g@xB{@\\cB^{@vCGhB\\zAFfDGxBe@l@aCjAFhC{"
            "AjAgChBu@NcBhBqB]qBl@qBjAqBjA}@|@aCzA{AhB_C\\aC^sBxAyBl@yAjBcBxA{AzA_D|@u@xBcAhB{"
            "AxBqBzAyBhBaCzAyBzAgChByBjAyBjB{BzA_Cz@"
            "yBzAiCzAyBzAyBxAqBjAkBzAyBjAiBjAcBlAaBjAkBjAiBzAkBjAqBzAiBjAu@z@cBjAiBzAyBjBcBxAiB|@"
            "aCNyB\\iC\\gCl@yBNaC?iC?gCLoDNqBLsB^aBjAkBhByB|@qBjA{@l@kBz@yBlAyBjA_DxByBjAxBvC?zAmJ}"
            "@iCbGFfDaMxMcFtD"},
       }) {

    // try it
    auto input_shape = decode<std::vector<PointLL>>(example.second.c_str(), example.second.length());
    auto resampled = resample_spherical_polyline(input_shape, example.first, false);

    // check that nothing is too far apart
    for (auto p = std::next(resampled.cbegin()); p != resampled.cend(); ++p) {
      auto dist = p->Distance(*std::prev(p));
      EXPECT_LE(dist, example.first + 1)
          << "Distance between any two points on the resampled line cannot be "
          << "further than resample distance";
    }

    // all the points better be within a meter or so of the original line
    for (const auto& p : resampled) {
      auto cp = p.ClosestPoint(input_shape);
      auto dist = std::get<1>(cp);
      if (!equal(dist, 0.f, 1.2f)) {
        throw std::runtime_error("Sampled point was not found on original line");
      }
    }

    // all the original points should be in this one
    resampled = resample_spherical_polyline(input_shape, example.first, true);
    auto current = resampled.cbegin();
    for (const auto& p : input_shape) {
      while (current != resampled.cend() && *current != p)
        ++current;
      EXPECT_NE(current, resampled.cend())
          << "All original points should be found in resampled polyline";
    }
    EXPECT_EQ(current + 1, resampled.cend())
        << "Last found point should be last point in resampled polyline";

    // Test resample_polyline
    Polyline2<PointLL> pl(input_shape);
    float resolution = 100.0f;
    auto length = pl.Length();
    resampled = resample_polyline(input_shape, length, resolution);
    size_t n = std::round(length / resolution);
    float sample_distance = length / n;
    EXPECT_EQ(resampled.size(), n + 1)
        << "resample_polyline - Sampled polyline is not the expected length";
  }
}

TEST(UtilMidgard, TestResampleDuplicate) {
  std::vector<PointLL> polyline = {{-75.589897, 39.776615},
                                   {-75.589996, 39.777287},
                                   {-75.589996, 39.777287},
                                   {-75.590080, 39.777798},
                                   {-75.590103, 39.777969}};
  double interval = 30.0;
  auto resampled = resample_spherical_polyline(polyline, interval, false);

  // check that nothing is too far apart
  for (auto p = std::next(resampled.cbegin()); p != resampled.cend(); ++p) {
    auto dist = p->Distance(*std::prev(p));
    EXPECT_LE(dist, interval + 1)
        << "Distance between any two points on the resampled line cannot be "
        << "further than resample distance";
  }

  // all the points better be within a meter or so of the original line
  for (const auto& p : resampled) {
    auto cp = p.ClosestPoint(polyline);
    auto dist = std::get<1>(cp);
    if (!equal(dist, 0.f, 1.2f)) {
      throw std::runtime_error("Sampled point was not found on original line");
    }
  }

  // Test whether the last resampled vertex is close to the last polyline vertex
  if (resampled.back().Distance(polyline.back()) > interval) {
    throw std::runtime_error("Last resampled point too far from original polyline end");
  }
}

TEST(UtilMidgard, TestIterable) {
  int a[] = {1, 2, 3, 4, 5};
  char b[] = {'a', 'b', 'c', 'd', 'e'};
  std::string c[] = {"one", "two", "three", "four", "five"};
  const size_t d[] = {11, 12, 13, 14, 15};

  int sum = 0;
  for (const auto& i : iterable_t<int>(a, 5))
    sum += i;
  EXPECT_EQ(sum, 15) << "integer array sum failed";

  std::string concatinated;
  for (const auto& i : iterable_t<char>(b, 5))
    concatinated.push_back(i);
  EXPECT_EQ(concatinated, "abcde") << "char concatenation failed";

  concatinated = "";
  for (const auto& i : iterable_t<std::string>(c, 5))
    concatinated.append(i);
  EXPECT_EQ(concatinated, "onetwothreefourfive") << "string concatenation failed";

  size_t cumulative_product = 1;
  iterable_t<const size_t> iterable(d, 5);
  for (iterable_t<const size_t>::iterator i = iterable.begin(); i != iterable.end(); ++i)
    cumulative_product *= *i;
  EXPECT_EQ(cumulative_product, 360360) << "cumulative product failed";
}

TEST(UtilMidgard, TestTrimPolyline) {
  using Point = valhalla::midgard::Point2;

  std::vector<Point> line{{0, 0}, {0, 0}, {20, 20}, {31, 1}, {31, 1}, {12, 23}, {7, 2}, {7, 2}};

  auto clip = trim_polyline(line.begin(), line.end(), 0.f, 1.f);
  EXPECT_EQ(length(clip.begin(), clip.end()), length(line.begin(), line.end()))
      << "Should not clip anything if range is [0, 1]";

  clip = trim_polyline(line.begin(), line.end(), 0.f, 0.1f);
  EXPECT_NEAR(length(clip.begin(), clip.end()), length(line.begin(), line.end()) * 0.1f, 1e-5)
      << "10% portion should be clipped";

  clip = trim_polyline(line.begin(), line.end(), 0.5f, 1.f);
  EXPECT_NEAR(length(clip.begin(), clip.end()), length(line.begin(), line.end()) * 0.5f, 1e-5)
      << "50% portion should be clipped";

  clip = trim_polyline(line.begin(), line.end(), 0.5f, 0.7f);
  EXPECT_NEAR(length(clip.begin(), clip.end()), length(line.begin(), line.end()) * 0.2f, 1e-5)
      << "0.2 portion should be clipped";

  clip = trim_polyline(line.begin(), line.end(), 0.65f, 0.7f);
  EXPECT_NEAR(length(clip.begin(), clip.end()), length(line.begin(), line.end()) * 0.05f, 1e-5)
      << "5% portion should be clipped";

  clip = trim_polyline(line.begin(), line.end(), 0.4999f, 0.5f);
  EXPECT_NEAR(length(clip.begin(), clip.end()), length(line.begin(), line.end()) * 0.0001f, 1e-5)
      << "0.1% portion should be clipped";

  EXPECT_TRUE(trim_polyline(line.begin(), line.end(), 0.65f, 0.5f).empty())
      << "nothing should be clipped since [0.65, 0.5]";

  EXPECT_TRUE(trim_polyline(line.begin(), line.end(), -2.f, -1.f).empty())
      << "nothing should be clipped since negative [-2, -1]";

  EXPECT_EQ(trim_polyline(line.begin(), line.end(), 0.f, 0.f).back(), Point(0, 0))
      << "nothing should be clipped since empty set [0, 0]";

  EXPECT_EQ(trim_polyline(line.begin(), line.end(), -1.f, 0.f).back(), Point(0, 0))
      << "nothing should be clipped since out of range [-1, 0]";

  EXPECT_EQ(trim_polyline(line.begin(), line.end(), 1.f, 1.f).front(), Point(7, 2))
      << "nothing should be clipped since [1, 1]";

  EXPECT_EQ(trim_polyline(line.begin(), line.end(), 1.f, 2.f).front(), Point(7, 2))
      << "nothing should be clipped since out of range [1, 2]";

  EXPECT_TRUE(trim_polyline(line.begin(), line.end(), 1.001f, 2.f).empty())
      << "nothing should be clipped since out of range [1.001, 2]";

  EXPECT_TRUE(trim_polyline(line.begin(), line.end(), 0.5f, 0.1f).empty())
      << "nothing should be clipped since empty set [0.5, 0.1]";

  // Make sure length returns 0 when iterator is equal
  EXPECT_EQ(length(clip.begin(), clip.begin()), 0.0f) << "incorrect length when iterators are equal";
}

TEST(UtilMidgard, TestTrimFront) {
  std::vector<Point2> pts = {{-1.0f, -1.0f}, {-1.0f, 1.0f}, {0.0f, 1.0f},
                             {1.0f, 1.0f},   {4.0f, 5.0f},  {5.0f, 5.0f}};

  constexpr float tolerance = 0.0001f;
  float l = length(pts);
  auto trim = trim_front(pts, 9.0f);
  EXPECT_NEAR(length(trim), 9.0f, tolerance) << "incorrect length of trimmed polyline";
  EXPECT_LE(std::abs(l - length(pts) - 9.0f), tolerance)
      << "length of remaining polyline not correct - " << l << " - " << length(pts);

  EXPECT_EQ(pts.size(), 2) << "number of remaining points not correct";

  std::list<Point2> pts2 = {{-81.0f, -45.0f}, {-18.0f, 17.0f}, {8.0f, 8.0f},
                            {6.0f, 19.0f},    {49.0f, -5.0f},  {75.0f, 45.0f}};
  l = length(pts2);
  float d = l * 0.75f;
  auto trim2 = trim_front(pts2, d);
  EXPECT_NEAR(length(trim2), d, tolerance) << "incorrect length of trimmed polyline 2";

  float d2 = l * 0.25f;
  float l2 = length(pts2);
  EXPECT_NEAR(d2, l2, tolerance) << "length of remaining polyline 2 does not match";

  // Make sure if trim distance exceeds polyline length that the entire
  // polyline is returned and none remains
  std::list<Point2> pts3 = {{-81.0f, -45.0f}, {-18.0f, 17.0f}, {8.0f, 8.0f},
                            {6.0f, 19.0f},    {49.0f, -5.0f},  {75.0f, 45.0f}};
  size_t n = pts3.size();
  l = length(pts3);
  auto trim3 = trim_front(pts3, l + 1.0f);
  EXPECT_NEAR(length(trim3), l, tolerance)
      << "length of trimmed polyline not equal to original length when trim; distance exceeds length";

  EXPECT_EQ(trim3.size(), n)
      << "trimmed polyline not equal size of original when trim distance exceeds length";
  EXPECT_LE(pts3.size(), 0) << "some of original polyline remains when trim distance exceeds length";
}

TEST(UtilMidgard, TestLengthWithEmptyVector) {
  std::vector<PointLL> empty;
  EXPECT_EQ(length(empty), 0.0f) << "empty polyline returns non-zero length";
  // Test with only 1 point, should still return 0
  empty.emplace_back(-70.0f, 30.0f);
  EXPECT_EQ(length(empty), 0.0f) << "one point polyline returns non-zero length";
}

TEST(UtilMidgard, TestTangentAngle) {
  PointLL point{-122.839554f, 38.3990479f};
  std::vector<PointLL> shape{{-122.839104f, 38.3988266f},
                             {-122.839539f, 38.3988342f},
                             {-122.839546f, 38.3990479f}};
  constexpr float kTestDistance = 24.0f; // Use the maximum distance from GetOffsetForHeading
  float expected = shape[1].Heading(shape[2]);
  float tang = tangent_angle(1, point, shape, kTestDistance, true);
  EXPECT_NEAR(tang, expected, 5.0f) << "tangent_angle outside expected tolerance";

  PointLL point2{-122.839125f, 38.3988266f};
  expected = shape[1].Heading(shape[0]);
  tang = tangent_angle(0, point2, shape, kTestDistance, false);

  EXPECT_NEAR(tang, expected, 5.0f) << "tangent_angle outside expected tolerance";
}

TEST(UtilMidgard, TestExpandLocation) {
  // Expand to create a box approx 200x200 meters
  PointLL loc(-77.0f, 39.0f);
  AABB2<PointLL> box = ExpandMeters(loc, 100);
  float area = (box.Height() * kMetersPerDegreeLat) * box.Width() *
               DistanceApproximator::MetersPerLngDegree(loc.lat());
  EXPECT_LE(area, 201.0f * 201.0f);
  EXPECT_GE(area, 199.0f * 199.0f);

  // Should throw an exception if negative value is sent
  EXPECT_THROW(AABB2<PointLL> box = ExpandMeters(loc, -10.0f);, std::invalid_argument)
      << "ExpandLocation: should throw exception with negative meters supplied";
}

TEST(UtilMidgard, TestSimilarAndEqual) {
  // Make sure no negative epsilons are allowed in equal
  EXPECT_THROW(equal<float>(10.0f, 10.0f, -0.0001f);, std::logic_error);

  // Test the equality case
  EXPECT_TRUE(similar<float>(45.0f, 45.0f, 0.0001f)) << "Similar test fails for equal values";

  // Test case where signs are different - if opposing signs the values should not
  // be similar regardless of difference
  EXPECT_FALSE(similar<float>(0.00001f, -0.00001f, 0.0001f))
      << "Similar test fails for values with opposing signs";
}

// Fixes case in https://github.com/valhalla/valhalla/issues/2201#issuecomment-582656499
TEST(UtilMidgard, TrimShapeAsan) {
  float start = 42.2698097;
  float end = 47;
  std::vector<PointLL> shape = {
      PointLL{8.5468483, 47.3655319},
      PointLL{8.54691314, 47.365448},
      PointLL{8.54711914, 47.3651543},
  };
  trim_shape(start, shape.front(), end, shape.back(), shape);
  ASSERT_EQ(shape.size(), 2);
  ASSERT_FLOAT_EQ(shape.at(0).lat(), 47.365532);
  ASSERT_FLOAT_EQ(shape.at(0).lng(), 8.5468483);
  ASSERT_FLOAT_EQ(shape.at(1).lat(), 47.365154);
  ASSERT_FLOAT_EQ(shape.at(1).lng(), 8.54712);
}

// Check for empty shape
TEST(UtilMidgard, TrimShapeEmpty) {
  PointLL start_vertex;
  PointLL end_vertex;
  std::vector<PointLL> shape;
  trim_shape(0, start_vertex, 0, end_vertex, shape);
  ASSERT_EQ(shape.size(), 0);
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
