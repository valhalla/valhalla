#include "test.h"

#include "include/config.h"
#include "geo/point2.h"
#include "geo/aabbll.h"
#include "geo/constants.h"


using namespace std;
using namespace valhalla::geo;

namespace {

void TryMinLat(const AABBLL& a, float res) {
  if (fabs(a.minlat() - res) > kEpsilon)
    throw runtime_error("TestMinLat test failed");
}

void TestMinLat() {
  TryMinLat(AABBLL(39.8249f, -76.8013f, 40.2559f, -75.8997f), 39.8249f);
}

void TryMinLng(const AABBLL& a, float res) {
  if (fabs(a.minlng() - res) > kEpsilon)
    throw runtime_error("TestMinLng test failed");
}

void TestMinLng() {
  TryMinLng(AABBLL(39.8249f, -76.8013f, 40.2559f, -75.8997f), -76.8013f);
}

void TryMaxLat(const AABBLL& a, float res) {
  if (fabs(a.maxlat() - res) > kEpsilon)
    throw runtime_error("TestMaxLat test failed");
}

void TestMaxLat() {
  TryMaxLat(AABBLL(39.8249f, -76.8013f, 40.2559f, -75.8997f), 40.2559f);
}

void TryMaxLng(const AABBLL& a, float res) {
  if (fabs(a.maxlng() - res) > kEpsilon)
    throw runtime_error("TestMaxLng test failed");
}

void TestMaxLng() {
  TryMaxLng(AABBLL(39.8249f, -76.8013f, 40.2559f, -75.8997f), -75.8997f);
}

void TryGetUpperLeft(const AABBLL& a, const PointLL& pt) {
  if (!(a.GetUpperLeft() == pt))
    throw runtime_error("GetUpperLeft test failed");
}

void TestGetUpperLeft() {
  TryGetUpperLeft(AABBLL(39.8249f, -76.8013f, 40.2559f, -75.8997f), PointLL(40.2559f, -76.8013f));
}

void TryGetUpperRight(const AABBLL& a, const PointLL& pt) {
  if (!(a.GetUpperRight() == pt))
    throw runtime_error("GetUpperRight test failed");
}

void TestGetUpperRight() {
  TryGetUpperRight(AABBLL(39.8249f, -76.8013f, 40.2559f, -75.8997f), PointLL(40.2559f, -75.8997f));
}

void TryGetLowerLeft(const AABBLL& a, const PointLL& pt) {
  if (!(a.GetLowerLeft() == pt))
    throw runtime_error("GetLowerLeft test failed");
}

void TestGetLowerLeft() {
  TryGetLowerLeft(AABBLL(39.8249f, -76.8013f, 40.2559f, -75.8997f), PointLL(39.8249f, -76.8013f));
}

void TryGetLowerRight(const AABBLL& a, const PointLL& pt) {
  if (!(a.GetLowerRight() == pt))
    throw runtime_error("GetLowerRight test failed");
}

void TestGetLowerRight() {
  TryGetLowerRight(AABBLL(39.8249f, -76.8013f, 40.2559f, -75.8997f), PointLL(39.8249f, -75.8997f));
}

}

int main() {
  test::suite suite("aabbll");

  suite.test(TEST_CASE(TestMinLat));

  suite.test(TEST_CASE(TestMinLng));

  suite.test(TEST_CASE(TestMaxLat));

  suite.test(TEST_CASE(TestMaxLng));

  suite.test(TEST_CASE(TestGetUpperLeft));

  suite.test(TEST_CASE(TestGetUpperRight));

  suite.test(TEST_CASE(TestGetLowerLeft));

  suite.test(TEST_CASE(TestGetLowerRight));

  return suite.tear_down();
}
