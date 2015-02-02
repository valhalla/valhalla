#include "test.h"
#include "valhalla/midgard/util.h"

using namespace valhalla::midgard;

namespace {

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
  // Sharp Left
  if (GetTurnDegree(180, 40) != 220)
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
}

void AppxEqual() {
  if(!equal<float>(-136.170790, -136.170800, .00002f))
    throw std::runtime_error("Should be equal");
  if(!equal<float>(-136.170800, -136.170790, .00002f))
    throw std::runtime_error("Should be equal");
  if(!equal<float>(16.645590, 16.645580, .00002f))
    throw std::runtime_error("Should be equal");
  if(!equal<float>(76.627980, 76.627970, .00002f))
    throw std::runtime_error("Should be equal");
  if(!equal<int>(0, 0))
    throw std::runtime_error("Should be equal");
  if(!equal<float>(1, 1, 0))
    throw std::runtime_error("Should be equal");
}

}

int main() {
  test::suite suite("util");

  // GetTurnDegree
  suite.test(TEST_CASE(TestGetTurnDegree));

  // GetTime
  suite.test(TEST_CASE(TestGetTime));

  suite.test(TEST_CASE(AppxEqual));

  return suite.tear_down();
}
