#include "midgard/constants.h"
#include "midgard/distanceapproximator.h"
#include "midgard/encoded.h"
#include "midgard/polyline2.h"
#include "midgard/util.h"
#include "test.h"
#include <cmath>
#include <random>

#include <list>

using namespace valhalla::midgard;

namespace {

void TestRangedDefaultT() {
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
      if (finalVal != testRange.def) {
        throw std::runtime_error("Final value did not snap to the range default value");
      }
    } else {
      // Was inside of range so finalVal should still be the same number
      if (finalVal != testVal) {
        throw std::runtime_error("Final value moved invalidly");
      }
    }

    // Test Edge cases because random distribution is unlikely to land exactly on boundaries
    finalVal = testRange(testRange.min);
    if (finalVal != testRange.min) {
      throw std::runtime_error("Final value invalidly moves on lower bound");
    }

    finalVal = testRange(testRange.max);
    if (finalVal != testRange.max) {
      throw std::runtime_error("Final value invalidly moves on upper bound");
    }
  }
}

void TestGetTurnDegree() {
  // Slight Right
  if (GetTurnDegree(315, 335) != 20)
    throw std::runtime_error("Invalid turn degree");
  // Right
  if (GetTurnDegree(0, 90) != 90)
    throw std::runtime_error("Invalid turn degree");
  // Right
  if (GetTurnDegree(90, 180) != 90)
    throw std::runtime_error("Invalid turn degree");
  // Sharp Right
  if (GetTurnDegree(180, 340) != 160)
    throw std::runtime_error("Invalid turn degree");
  // Sharp Right
  if (GetTurnDegree(180, 352) != 172)
    throw std::runtime_error("Invalid turn degree");
  // Sharp Left
  if (GetTurnDegree(180, 40) != 220)
    throw std::runtime_error("Invalid turn degree");
  // Sharp Left
  if (GetTurnDegree(180, 10) != 190)
    throw std::runtime_error("Invalid turn degree");
  // Left
  if (GetTurnDegree(0, 180) != 180)
    throw std::runtime_error("Invalid turn degree");
  // Left
  if (GetTurnDegree(270, 180) != 270)
    throw std::runtime_error("Invalid turn degree");
  // Slight Left
  if (GetTurnDegree(90, 70) != 340)
    throw std::runtime_error("Invalid turn degree");
  // Continue
  if (GetTurnDegree(358, 2) != 4)
    throw std::runtime_error("Invalid turn degree");
}

void TestGetTime() {
  if (GetTime(100, 100) != 3600)
    throw std::runtime_error("Invalid time");
  if (GetTime(5, 20) != 900)
    throw std::runtime_error("Invalid time");
  if (GetTime(5, 0) != 0)
    throw std::runtime_error("Invalid time");
}

void AppxEqual() {
  if (!equal<float>(-136.170790, -136.170800, .00002f))
    throw std::runtime_error("Should be equal");
  if (!equal<float>(-136.170800, -136.170790, .00002f))
    throw std::runtime_error("Should be equal");
  if (!equal<float>(16.645590, 16.645580, .00002f))
    throw std::runtime_error("Should be equal");
  if (!equal<float>(76.627980, 76.627970, .00002f))
    throw std::runtime_error("Should be equal");
  if (!equal<int>(0, 0))
    throw std::runtime_error("Should be equal");
  if (!equal<float>(1, 1, 0))
    throw std::runtime_error("Should be equal");
}

void MemoryStatus() {
  // only check this if the os supports it (system must have /proc/self/status)
  if (memory_status::supported()) {
    memory_status status({"VmSize", "VmSwap", "VmPeak"});

    // should have each of these
    for (const auto& key : {"VmSize", "VmSwap", "VmPeak"}) {
      auto value = status.metrics.find(key);
      if (value == status.metrics.end())
        throw std::runtime_error("Missing memory statistic for " + std::string(key));
      if (value->second.first < 0.)
        throw std::runtime_error("Negative memory usage values are not allowed");
      if (value->second.second.back() != 'B')
        throw std::runtime_error("Units should be some magnitude of bytes");
    }
  }
}

void TestClamp() {
  if (!equal<float>(circular_range_clamp<float>(467, -90, 90), -73))
    throw std::runtime_error("Wrong clamp value");
  if (!equal<float>(circular_range_clamp<float>(-467, -90, 90), 73))
    throw std::runtime_error("Wrong clamp value");
  if (!equal<float>(circular_range_clamp<float>(7, -90, 90), 7))
    throw std::runtime_error("Wrong clamp value");
  if (!equal<float>(circular_range_clamp<float>(-67, -90, 90), -67))
    throw std::runtime_error("Wrong clamp value");
  if (!equal<float>(circular_range_clamp<float>(-97, -90, 90), 83))
    throw std::runtime_error("Wrong clamp value");
  if (!equal<float>(circular_range_clamp<float>(-97.2, -90, 90), 82.8))
    throw std::runtime_error("Wrong clamp value");
  if (!equal<float>(circular_range_clamp<float>(-180, -90, 90), 0))
    throw std::runtime_error("Wrong clamp value");
  if (!equal<float>(circular_range_clamp<float>(270, -90, 90), -90))
    throw std::runtime_error("Wrong clamp value");
  if (!equal<float>(circular_range_clamp<float>(369, 0, 360), 9))
    throw std::runtime_error("wrong clamp value");
  if (!equal<float>(circular_range_clamp<float>(-369, 0, 360), 351))
    throw std::runtime_error("wrong clamp value");
  if (!equal<float>(circular_range_clamp<float>(739, -45, -8), -38))
    throw std::runtime_error("wrong clamp value");

  // Test invalid range - should throw an exception
  try {
    circular_range_clamp<float>(739, -8, -45);
    throw std::runtime_error("should throw exception (lower > upper)");
  } catch (...) {}

  // Make sure get_turn_degree180 throws an exception if inbound and outbound
  // degrees are not clamped to 0,360
  try {
    get_turn_degree180(420, 580);
    throw std::runtime_error("get_turn_degree should throw exception (inputs not in 0,360 range)");
  } catch (...) {}
}

void TestResample() {
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
      if (dist > example.first + 1)
        throw std::runtime_error("Distance between any two points on the resampled line cannot be "
                                 "further than resample distance");
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
      if (current == resampled.cend())
        throw std::runtime_error("All original points should be found in resampled polyline");
    }
    if (current + 1 != resampled.cend())
      throw std::runtime_error("Last found point should be last point in resampled polyline");

    // Test resample_polyline
    Polyline2<PointLL> pl(input_shape);
    float resolution = 100.0f;
    auto length = pl.Length();
    resampled = resample_polyline(input_shape, length, resolution);
    size_t n = std::round(length / resolution);
    float sample_distance = length / n;
    if (resampled.size() != n + 1) {
      throw std::runtime_error("resample_polyline - Sampled polyline is not the expected length");
    }
  }
}

void TestIterable() {
  int a[] = {1, 2, 3, 4, 5};
  char b[] = {'a', 'b', 'c', 'd', 'e'};
  std::string c[] = {"one", "two", "three", "four", "five"};
  const size_t d[] = {11, 12, 13, 14, 15};

  int sum = 0;
  for (const auto& i : iterable_t<int>(a, 5))
    sum += i;
  if (sum != 15)
    throw std::logic_error("integer array sum failed");

  std::string concatinated;
  for (const auto& i : iterable_t<char>(b, 5))
    concatinated.push_back(i);
  if (concatinated != "abcde")
    throw std::logic_error("char concatenation failed");

  concatinated = "";
  for (const auto& i : iterable_t<std::string>(c, 5))
    concatinated.append(i);
  if (concatinated != "onetwothreefourfive")
    throw std::logic_error("string concatenation failed");

  size_t cumulative_product = 1;
  iterable_t<const size_t> iterable(d, 5);
  for (iterable_t<const size_t>::iterator i = iterable.begin(); i != iterable.end(); ++i)
    cumulative_product *= *i;
  if (cumulative_product != 360360)
    throw std::logic_error("cumulative product failed");
}

void TestTrimPolyline() {
  using Point = valhalla::midgard::Point2;

  std::vector<Point> line{{0, 0}, {0, 0}, {20, 20}, {31, 1}, {31, 1}, {12, 23}, {7, 2}, {7, 2}};
  auto clip = trim_polyline(line.begin(), line.end(), 0.f, 1.f);
  test::assert_bool(length(clip.begin(), clip.end()) == length(line.begin(), line.end()),
                    "Should not clip anything if range is [0, 1]");

  clip = trim_polyline(line.begin(), line.end(), 0.f, 0.1f);
  test::assert_bool(equal(length(clip.begin(), clip.end()), length(line.begin(), line.end()) * 0.1f),
                    "10% portion should be clipped");

  clip = trim_polyline(line.begin(), line.end(), 0.5f, 1.f);
  test::assert_bool(equal(length(clip.begin(), clip.end()), length(line.begin(), line.end()) * 0.5f),
                    "50% portion should be clipped");

  clip = trim_polyline(line.begin(), line.end(), 0.5f, 0.7f);
  test::assert_bool(equal(length(clip.begin(), clip.end()), length(line.begin(), line.end()) * 0.2f),
                    "0.2 portion should be clipped");

  clip = trim_polyline(line.begin(), line.end(), 0.65f, 0.7f);
  test::assert_bool(equal(length(clip.begin(), clip.end()), length(line.begin(), line.end()) * 0.05f),
                    "5% portion should be clipped");

  clip = trim_polyline(line.begin(), line.end(), 0.4999f, 0.5f);
  test::assert_bool(equal(length(clip.begin(), clip.end()),
                          length(line.begin(), line.end()) * 0.0001f),
                    "0.1% portion should be clipped");

  test::assert_bool(trim_polyline(line.begin(), line.end(), 0.65f, 0.5f).empty(),
                    "nothing should be clipped since [0.65, 0.5]");

  test::assert_bool(trim_polyline(line.begin(), line.end(), -2.f, -1.f).empty(),
                    "nothing should be clipped since negative [-2, -1]");

  test::assert_bool(trim_polyline(line.begin(), line.end(), 0.f, 0.f).back() == Point(0, 0),
                    "nothing should be clipped since empty set [0, 0]");

  test::assert_bool(trim_polyline(line.begin(), line.end(), -1.f, 0.f).back() == Point(0, 0),
                    "nothing should be clipped since out of range [-1, 0]");

  test::assert_bool(trim_polyline(line.begin(), line.end(), 1.f, 1.f).front() == Point(7, 2),
                    "nothing should be clipped since [1, 1]");

  test::assert_bool(trim_polyline(line.begin(), line.end(), 1.f, 2.f).front() == Point(7, 2),
                    "nothing should be clipped since out of range [1, 2]");

  test::assert_bool(trim_polyline(line.begin(), line.end(), 1.001f, 2.f).empty(),
                    "nothing should be clipped since out of range [1.001, 2]");

  test::assert_bool(trim_polyline(line.begin(), line.end(), 0.5f, 0.1f).empty(),
                    "nothing should be clipped since empty set [0.5, 0.1]");

  // Make sure length returns 0 when iterator is equal
  if (length(clip.begin(), clip.begin()) != 0.0f) {
    throw std::logic_error("incorrect length when iterators are equal");
  }
}

void TestTrimFront() {
  std::vector<Point2> pts = {{-1.0f, -1.0f}, {-1.0f, 1.0f}, {0.0f, 1.0f},
                             {1.0f, 1.0f},   {4.0f, 5.0f},  {5.0f, 5.0f}};

  constexpr float tolerance = 0.0001f;
  float l = length(pts);
  auto trim = trim_front(pts, 9.0f);
  if (std::abs(length(trim) - 9.0f) > tolerance) {
    throw std::logic_error("incorrect length of trimmed polyline");
  }
  if (std::abs(l - length(pts) - 9.0f) > tolerance) {
    throw std::logic_error("length of remaining polyline not correct");
  }
  if (pts.size() != 2) {
    throw std::logic_error("number of remaining points not correct");
  }

  std::list<Point2> pts2 = {{-81.0f, -45.0f}, {-18.0f, 17.0f}, {8.0f, 8.0f},
                            {6.0f, 19.0f},    {49.0f, -5.0f},  {75.0f, 45.0f}};
  l = length(pts2);
  float d = l * 0.75f;
  auto trim2 = trim_front(pts2, d);
  if (std::abs(length(trim2) - d) > tolerance) {
    throw std::logic_error("incorrect length of trimmed polyline 2");
  }
  float d2 = l * 0.25f;
  float l2 = length(pts2);
  if (std::abs(d2 - l2) > tolerance) {
    throw std::logic_error("length of remaining polyline 2 does not match");
  }

  // Make sure if trim distance exceeds polyline length that the entire
  // polyline is returned and none remains
  std::list<Point2> pts3 = {{-81.0f, -45.0f}, {-18.0f, 17.0f}, {8.0f, 8.0f},
                            {6.0f, 19.0f},    {49.0f, -5.0f},  {75.0f, 45.0f}};
  size_t n = pts3.size();
  l = length(pts3);
  auto trim3 = trim_front(pts3, l + 1.0f);
  if (std::abs(length(trim3) - l) > tolerance) {
    throw std::logic_error("length of trimmed polyline not equal to original length when trim "
                           "distance exceeds length");
  }
  if (trim3.size() != n) {
    throw std::logic_error(
        "trimmed polyline not equal size of original when trim distance exceeds length");
  }
  if (pts3.size() > 0) {
    throw std::logic_error("some of original polyline remains when trim distance exceeds length");
  }
}

void TestLengthWithEmptyVector() {
  std::vector<PointLL> empty;
  if (length(empty) != 0.0f) {
    throw std::logic_error("empty polyline returns non-zero length");
  }
  // Test with only 1 point, should still return 0
  empty.emplace_back(-70.0f, 30.0f);
  if (length(empty) != 0.0f) {
    throw std::logic_error("one point polyline returns non-zero length");
  }
}

void TestTangentAngle() {
  PointLL point{-122.839554f, 38.3990479f};
  std::vector<PointLL> shape{{-122.839104f, 38.3988266f},
                             {-122.839539f, 38.3988342f},
                             {-122.839546f, 38.3990479f}};
  constexpr float kTestDistance = 24.0f; // Use the maximum distance from GetOffsetForHeading
  float expected = shape[1].Heading(shape[2]);
  float tang = tangent_angle(1, point, shape, kTestDistance, true);
  if (std::abs(tang - expected) > 5.0f) {
    throw std::logic_error("tangent_angle outside expected tolerance: expected " +
                           std::to_string(expected) + " but tangent = " + std::to_string(tang));
  }

  PointLL point2{-122.839125f, 38.3988266f};
  expected = shape[1].Heading(shape[0]);
  tang = tangent_angle(0, point2, shape, kTestDistance, false);
  if (std::abs(tang - expected) > 5.0f) {
    throw std::logic_error("tangent_angle outside expected tolerance: expected " +
                           std::to_string(expected) + " but tangent = " + std::to_string(tang));
  }
}

void TestExpandLocation() {
  // Expand to create a box approx 200x200 meters
  PointLL loc(-77.0f, 39.0f);
  AABB2<PointLL> box = ExpandMeters(loc, 100);
  float area = (box.Height() * kMetersPerDegreeLat) * box.Width() *
               DistanceApproximator::MetersPerLngDegree(loc.lat());
  if (area < 199.0f * 199.0f || area > 201.0f * 201.0f) {
    throw std::logic_error("ExpandLocation: area of the bounding box is incorrect " +
                           std::to_string(area));
  }

  // Should throw an exception if negative value is sent
  try {
    AABB2<PointLL> box = ExpandMeters(loc, -10.0f);
    throw std::logic_error("ExpandLocation: should throw exception with negative meters supplied");
  } catch (...) {}
}

void TestSimilarAndEqual() {
  // Make sure no negative epsilons are allowed in equal
  try {
    equal<float>(10.0f, 10.0f, -0.0001f);
    throw std::logic_error("Equal test fails to throw exception for negative epsilon");
  } catch (...) {}

  // Test the equality case
  if (!similar<float>(45.0f, 45.0f, 0.0001f)) {
    throw std::logic_error("Similar test fails for equal values");
  }

  // Test case where signs are different - if opposing signs the values should not
  // be similar regardless of difference
  if (similar<float>(0.00001f, -0.00001f, 0.0001f)) {
    throw std::logic_error("Similar test fails for values with opposing signs");
  }
}

} // namespace

int main() {
  test::suite suite("util");

  suite.test(TEST_CASE(TestRangedDefaultT));

  // GetTurnDegree
  suite.test(TEST_CASE(TestGetTurnDegree));

  // GetTime
  suite.test(TEST_CASE(TestGetTime));

  suite.test(TEST_CASE(AppxEqual));

  suite.test(TEST_CASE(MemoryStatus));

  suite.test(TEST_CASE(TestClamp));

  suite.test(TEST_CASE(TestResample));

  suite.test(TEST_CASE(TestIterable));

  suite.test(TEST_CASE(TestTrimPolyline));

  suite.test(TEST_CASE(TestExpandLocation));

  // trim_front of a polyline
  suite.test(TEST_CASE(TestTrimFront));

  // Test that length with empty container (or only 1 point) returns 0
  suite.test(TEST_CASE(TestLengthWithEmptyVector));

  // tangent angle
  suite.test(TEST_CASE(TestTangentAngle));

  // Test similar and equal edge cases
  suite.test(TEST_CASE(TestSimilarAndEqual));

  return suite.tear_down();
}
