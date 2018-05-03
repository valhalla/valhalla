#include "test.h"

#include "midgard/openlr.h"

#include <iostream>
#include <string>

using namespace std;
using namespace valhalla::midgard::OpenLR;

// Maximum deviation from expected decoded value
constexpr double kPrecisionThreshold = 0.00001;

struct testfixture {
    std::string descriptor;
    double expectedFirstCoordinateLongitude;
    double expectedFirstCoordinateLatitude;
    double expectedFirstCoordinateBearing;

    double expectedLastCoordinateLongitude;
    double expectedLastCoordinateLatitude;
    double expectedLastCoordinateBearing;
};

const testfixture testfixtures [] = {
    { "CgOa9yUQACODBQEqAL4jEw==", 5.069987, 52.119130, 3, 5.072967, 52.121030, 19 },
    { "CwOiYCUMoBNWAv9P/+MSBg==", 5.110692, 52.100590, 2, 5.108922, 52.100300, 6 },
    { "CxWj2OogyxJBDhDSAvwSUL4=", 30.431259, -30.757352, 14, 30.474319, -30.749712, 16 }
};

namespace {

  void test_decode() {

      for (auto & fixture : testfixtures) {
        auto locRef = TwoPointLinearReference::fromBase64(fixture.descriptor);

        if (std::abs(locRef.getFirstCoordinate().lng() - fixture.expectedFirstCoordinateLongitude) > kPrecisionThreshold) {
           throw runtime_error("First coordinate longitude incorrect.  Was " + std::to_string(locRef.getFirstCoordinate().lng()) + ", expected " + std::to_string(fixture.expectedFirstCoordinateLongitude));
        }
        if (std::abs(locRef.getFirstCoordinate().lat() - fixture.expectedFirstCoordinateLatitude) > kPrecisionThreshold) {
           throw runtime_error("First coordinate latitude incorrect.  Was " + std::to_string(locRef.getFirstCoordinate().lat()) + ", expected " + std::to_string(fixture.expectedFirstCoordinateLatitude));
        }

        if (std::abs(locRef.getLastCoordinate().lng() - fixture.expectedLastCoordinateLongitude) > kPrecisionThreshold) {
           throw runtime_error("Last coordinate longitude incorrect.  Was " + std::to_string(locRef.getLastCoordinate().lng()) + ", expected " + std::to_string(fixture.expectedLastCoordinateLongitude));
        }
        if (std::abs(locRef.getLastCoordinate().lat() - fixture.expectedLastCoordinateLatitude) > kPrecisionThreshold) {
           throw runtime_error("Last coordinate latitude incorrect.  Was " + std::to_string(locRef.getLastCoordinate().lat()) + ", expected " + std::to_string(fixture.expectedLastCoordinateLatitude));
        }
      }

  }
}

int main(void)
{
  test::suite suite("openlr");

  suite.test(TEST_CASE(test_decode));

  return suite.tear_down();
}