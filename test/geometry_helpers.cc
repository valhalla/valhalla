// -*- mode: c++ -*-
#include <cmath>

#include <valhalla/midgard/point2.h>
#include <valhalla/midgard/util.h>

#include "test.h"
#include "meili/geometry_helpers.h"

using namespace valhalla::midgard;
using namespace valhalla::meili::helpers;

void TestClipLineString()
{
  using Point = valhalla::midgard::Point2;

  std::vector<Point> line{{0,0}, {0, 0}, {20, 20}, {31, 1}, {31,1}, {12, 23}, {7, 2}, {7,2}};
  auto clip = ClipLineString(line.begin(), line.end(), 0.f, 1.f);
  test::assert_bool(LineStringLength(clip.begin(), clip.end()) == LineStringLength(line.begin(), line.end()),
                    "Should not clip anything if range is [0, 1]");

  clip = ClipLineString(line.begin(), line.end(), 0.f, 0.1f);
  test::assert_bool(equal(LineStringLength(clip.begin(), clip.end()), LineStringLength(line.begin(), line.end()) * 0.1f),
                    "10% portion should be clipped");

  clip = ClipLineString(line.begin(), line.end(), 0.5f, 1.f);
  test::assert_bool(equal(LineStringLength(clip.begin(), clip.end()), LineStringLength(line.begin(), line.end()) * 0.5f),
                    "50% portion should be clipped");

  clip = ClipLineString(line.begin(), line.end(), 0.5f, 0.7f);
  test::assert_bool(equal(LineStringLength(clip.begin(), clip.end()), LineStringLength(line.begin(), line.end()) * 0.2f),
                    "0.2 portion should be clipped");

  clip = ClipLineString(line.begin(), line.end(), 0.65f, 0.7f);
  test::assert_bool(equal(LineStringLength(clip.begin(), clip.end()), LineStringLength(line.begin(), line.end()) * 0.05f),
                    "5% portion should be clipped");

  clip = ClipLineString(line.begin(), line.end(), 0.4999f, 0.5f);
  test::assert_bool(equal(LineStringLength(clip.begin(), clip.end()), LineStringLength(line.begin(), line.end()) * 0.0001f),
                    "0.1% portion should be clipped");

  test::assert_bool(ClipLineString(line.begin(), line.end(), 0.65f, 0.5f).empty(),
                    "nothing should be clipped since [0.65, 0.5]");

  test::assert_bool(ClipLineString(line.begin(), line.end(), -2.f, -1.f).empty(),
                    "nothing should be clipped since negative [-2, -1]");

  test::assert_bool(ClipLineString(line.begin(), line.end(), 0.f, 0.f).back() == Point(0, 0),
                    "nothing should be clipped since empty set [0, 0]");

  test::assert_bool(ClipLineString(line.begin(), line.end(), -1.f, 0.f).back() == Point(0, 0),
                    "nothing should be clipped since out of range [-1, 0]");

  test::assert_bool(ClipLineString(line.begin(), line.end(), 1.f, 1.f).front() == Point(7, 2),
                    "nothing should be clipped since [1, 1]");

  test::assert_bool(ClipLineString(line.begin(), line.end(), 1.f, 2.f).front() == Point(7, 2),
                    "nothing should be clipped since out of range [1, 2]");

  test::assert_bool(ClipLineString(line.begin(), line.end(), 1.001f, 2.f).empty(),
                    "nothing should be clipped since out of range [1.001, 2]");

  test::assert_bool(ClipLineString(line.begin(), line.end(), 0.5f, 0.1f).empty(),
                    "nothing should be clipped since empty set [0.5, 0.1]");
}

int main(int argc, char *argv[])
{
  test::suite suite("geometry helpers");

  suite.test(TEST_CASE(TestClipLineString));

  return suite.tear_down();
}
