#include "midgard/openlr.h"
#include "test.h"

#include <boost/archive/iterators/base64_from_binary.hpp>
#include <boost/archive/iterators/binary_from_base64.hpp>
#include <boost/archive/iterators/transform_width.hpp>

#include <string>

using namespace std;
using namespace valhalla::midgard::OpenLR;

std::string decode64(const std::string& val) {
  using namespace boost::archive::iterators;
  using It = transform_width<binary_from_base64<std::string::const_iterator>, 8, 6>;
  return std::string(It(std::begin(val)), It(std::end(val)));
}

std::string encode64(const std::string& val) {
  using namespace boost::archive::iterators;
  using It = base64_from_binary<transform_width<std::string::const_iterator, 6, 8>>;
  auto tmp = std::string(It(std::begin(val)), It(std::end(val)));
  return tmp.append((3 - val.size() % 3) % 3, '=');
}

struct testfixture {
  std::string descriptor;
  double expectedFirstCoordinateLongitude;
  double expectedFirstCoordinateLatitude;
  double expectedFirstCoordinateBearing;

  double expectedLastCoordinateLongitude;
  double expectedLastCoordinateLatitude;
  double expectedLastCoordinateBearing;

  double expectedDistance;
  double expectedPoff;
  double expectedNoff;
};

const testfixture testfixtures[] = {{"CwOa9yUQACODBQEqAL4jEw==", 5.069987, 52.119130, 39.375,
                                     5.072967, 52.121030, 219.375, 293, 0, 0},
                                    {"CwOa9yUQACODqgEqAL4jEw==", 5.069987, 52.119130, 39.375,
                                     5.072967, 52.121030, 219.375, 9962, 0, 0},
                                    {"CwOiYCUMoBNWAv9P/+MSBg==", 5.110692, 52.100590, 253.125,
                                     5.108922, 52.100300, 73.125, 117.2, 0, 0},
                                    {"CxWj2OogyxJBDhDSAvwSUL4=", 30.431259, -30.757352, 16.875,
                                     30.474319, -30.749712, 185.625, 820.4, 608.890625, 0},
                                    {"CxWj2OogyxJBDhDSAvwSMA0=", 30.431259, -30.757352, 16.875,
                                     30.474319, -30.749712, 185.625, 820.4, 0, 41.6609},
                                    {"CxWj2OogyxJBDhDSAvwScL4N", 30.431259, -30.757352, 16.875,
                                     30.474319, -30.749712, 185.625, 820.4, 608.890625, 41.6609},
                                    {"C6i8rRtM3BjgAAAAAAUYAA==", -122.713562, 38.390942, 5.625,
                                     -122.713562, 38.390991, 5.625, 0, 0, 0}};

namespace {

bool check_close(double value, double expected, const std::string& message, double abstol = 1e-5) {
  if (std::abs(value - expected) > abstol)
    throw runtime_error(message + "  Was " + std::to_string(value) + ", expected " +
                        std::to_string(expected));
}

void test_decode() {

  for (auto& fixture : testfixtures) {
    auto locRef = LineLocation(decode64(fixture.descriptor));

    check_close(locRef.getFirstCoordinate().lng(), fixture.expectedFirstCoordinateLongitude,
                "First coordinate longitude incorrect.");
    check_close(locRef.getFirstCoordinate().lat(), fixture.expectedFirstCoordinateLatitude,
                "First coordinate latitude incorrect.");
    check_close(locRef.first.bearing, fixture.expectedFirstCoordinateBearing,
                "First bearing incorrect.");
    check_close(locRef.getLastCoordinate().lng(), fixture.expectedLastCoordinateLongitude,
                "Last coordinate longitude incorrect.");
    check_close(locRef.getLastCoordinate().lat(), fixture.expectedLastCoordinateLatitude,
                "Last coordinate latitude incorrect.");
    check_close(locRef.last.bearing, fixture.expectedLastCoordinateBearing,
                "Last bearing incorrect.");

    check_close(locRef.first.distance, fixture.expectedDistance, "Distance incorrect.", 1e-3);
    check_close(locRef.poff, fixture.expectedPoff, "Positive offset incorrect.", 1e-3);
    check_close(locRef.noff, fixture.expectedNoff, "Negative offset incorrect.", 1e-3);

    if (encode64(locRef.to_binary()) != fixture.descriptor) {
      throw runtime_error("Incorrectly encoded reference " + encode64(locRef.to_binary()) +
                          ", expected " + fixture.descriptor);
    }
  }
}

} // namespace

int main(void) {
  test::suite suite("openlr");

  suite.test(TEST_CASE(test_decode));

  return suite.tear_down();
}
