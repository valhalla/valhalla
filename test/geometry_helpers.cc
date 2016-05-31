// -*- mode: c++ -*-

//enable asserts for this test
#undef NDEBUG

#include <cmath>
#include <cassert>
#include <iostream>

#include <valhalla/midgard/point2.h>

#include "meili/geometry_helpers.h"

using namespace valhalla::meili::helpers;


template <typename T>
bool approximate(T a, T b, T r = 0.00001)
{
  return std::abs(a - b) <= r;
}


void TestClipLineString()
{
  using Point = valhalla::midgard::Point2;

  std::vector<Point> line{{0,0}, {0, 0}, {20, 20}, {31, 1}, {31,1}, {12, 23}, {7, 2}, {7,2}};
  auto clip = ClipLineString(line.begin(), line.end(), 0.f, 1.f);
  assert(LineStringLength(clip.begin(), clip.end()) == LineStringLength(line.begin(), line.end()));

  clip = ClipLineString(line.begin(), line.end(), 0.f, 0.1f);
  assert(approximate(LineStringLength(clip.begin(), clip.end()), LineStringLength(line.begin(), line.end()) * 0.1f));

  clip = ClipLineString(line.begin(), line.end(), 0.5f, 1.f);
  assert(LineStringLength(clip.begin(), clip.end()) == LineStringLength(line.begin(), line.end()) * 0.5f);

  clip = ClipLineString(line.begin(), line.end(), 0.5f, 0.7f);
  assert(approximate(LineStringLength(clip.begin(), clip.end()), LineStringLength(line.begin(), line.end()) * 0.2f));

  clip = ClipLineString(line.begin(), line.end(), 0.65f, 0.7f);
  assert(approximate(LineStringLength(clip.begin(), clip.end()), LineStringLength(line.begin(), line.end()) * 0.05f));

  clip = ClipLineString(line.begin(), line.end(), 0.4999f, 0.5f);
  assert(approximate(LineStringLength(clip.begin(), clip.end()), LineStringLength(line.begin(), line.end()) * 0.0001f));

  assert(ClipLineString(line.begin(), line.end(), 0.65f, 0.5f).empty());
  assert(ClipLineString(line.begin(), line.end(), -2.f, -1.f).empty());
  assert(ClipLineString(line.begin(), line.end(), 0.f, 0.f).back() == Point(0, 0));
  assert(ClipLineString(line.begin(), line.end(), -1.f, 0.f).back() == Point(0, 0));
  assert(ClipLineString(line.begin(), line.end(), 1.f, 1.f).front() == Point(7, 2));
  assert(ClipLineString(line.begin(), line.end(), 1.f, 2.f).front() == Point(7, 2));
  assert(ClipLineString(line.begin(), line.end(), 1.001f, 2.f).empty());
  assert(ClipLineString(line.begin(), line.end(), 0.5f, 0.1f).empty());
}


int main(int argc, char *argv[])
{
  TestClipLineString();

  std::cout << "all tests passed" << std::endl;
  return 0;
}
