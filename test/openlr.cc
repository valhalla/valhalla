#include "midgard/openlr.h"
#include "test.h"

#include <boost/archive/iterators/base64_from_binary.hpp>
#include <boost/archive/iterators/binary_from_base64.hpp>
#include <boost/archive/iterators/transform_width.hpp>

#include <iomanip>
#include <sstream>
#include <string>

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

std::string to_hex(const std::string& value) {
  std::stringstream s;
  for (auto c : value)
    s << " " << std::setw(2) << std::setfill('0') << std::hex << (int)((unsigned char)c);
  return s.str();
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
  if (std::abs(value - expected) > abstol) {
    throw std::runtime_error(message + "  Was " + std::to_string(value) + ", expected " +
                             std::to_string(expected));
  }
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

    // This test can be faulty - we check if the coordinates and bearing are close (but not
    // exact) above. If they are not exact then this test will fail! Try again with expected
    // values. TODO - this currently fails for 32 bit
#if _WIN64 || __amd64__
    if (encode64(locRef.toBinary()) != fixture.descriptor) {
      locRef.first.latitude = fixture.expectedFirstCoordinateLatitude;
      locRef.first.longitude = fixture.expectedFirstCoordinateLongitude;
      locRef.first.bearing = fixture.expectedFirstCoordinateBearing;
      locRef.last.latitude = fixture.expectedLastCoordinateLatitude;
      locRef.last.longitude = fixture.expectedLastCoordinateLongitude;
      locRef.last.bearing = fixture.expectedLastCoordinateBearing;
      locRef.first.distance = fixture.expectedDistance;
      locRef.poff = fixture.expectedPoff;
      locRef.noff = fixture.expectedNoff;
      if (encode64(locRef.toBinary()) != fixture.descriptor) {
        throw std::runtime_error("Incorrectly encoded reference " + encode64(locRef.toBinary()) +
                                 ", expected " + fixture.descriptor);
      }
    }
#endif
  }
}

void test_internal_reference_points() {
  auto location = "CwG1ASK3PhD82sz0CIAQ89r83hRxEAM=";

  auto locRef = LineLocation(decode64(location));

  if (encode64(locRef.toBinary()) != location) {
    throw std::runtime_error("Incorrectly encoded reference " + encode64(locRef.toBinary()) +
                             ", expected " + location);
  }

  if (locRef.intermediate.size() != 1) {
    throw std::runtime_error("Incorrectly number of intermediate LRP");
  }

  check_close(locRef.getFirstCoordinate().lng(), 2.400523, "First coordinate longitude incorrect.");
  check_close(locRef.getFirstCoordinate().lat(), 48.819069, "First coordinate latitude incorrect.");
  check_close(locRef.first.bearing, 320.625, "First bearing incorrect.");
  check_close(locRef.first.distance, 12774.8, "Distance incorrect.", 1e-3);

  check_close(locRef.intermediate[0].longitude, 2.269843, "First coordinate longitude incorrect.");
  check_close(locRef.intermediate[0].latitude, 48.840829, "First coordinate latitude incorrect.");
  check_close(locRef.intermediate[0].bearing, 219.375, "First bearing incorrect.");
  check_close(locRef.intermediate[0].distance, 12774.8, "Distance incorrect.", 1e-3);

  check_close(locRef.getLastCoordinate().lng(), 2.261823, "Last coordinate longitude incorrect.");
  check_close(locRef.getLastCoordinate().lat(), 48.893158, "Last coordinate latitude incorrect.");
  check_close(locRef.last.bearing, 39.375, "Last bearing incorrect.");

  check_close(locRef.getLength(), 2 * 12774.8, "Distance incorrect.", 1e-3);
  check_close(locRef.poff, 0, "Positive offset incorrect.", 1e-3);
  check_close(locRef.noff, 0, "Negative offset incorrect.", 1e-3);

  for (float poff = 0; poff < locRef.first.distance; poff += locRef.first.distance / 3) {
    for (float noff = 0; noff < locRef.intermediate[0].distance;
         noff += locRef.intermediate[0].distance / 3) {
      locRef.poff = poff;
      locRef.noff = noff;
      LineLocation tryRef(locRef.toBinary());

      check_close(tryRef.getLength(), 2 * 12774.8, "Distance incorrect.", 1e-3);
      check_close(tryRef.poff, locRef.poff, "Positive offset incorrect.", 58.6);
      check_close(tryRef.noff, locRef.noff, "Negative offset incorrect.", 58.6);
    }
  }

  locRef.intermediate.push_back(locRef.intermediate.front());
  locRef.intermediate.push_back(locRef.intermediate.front());
  locRef.intermediate.push_back(locRef.intermediate.front());
  check_close(locRef.getLength(), 5 * 12774.8, "Distance incorrect.", 1e-3);

  auto hex = " 0b 01 b5 01 22 b7 3e 10 fc da cc f4 08 80 10 f3 da "
             "00 00 00 00 10 f3 da 00 00 00 00 10 f3 da 00 00 00 00 10 f3 da fc de 14 71 10 63 aa aa";
  if (to_hex(locRef.toBinary()) != hex) {
    throw std::runtime_error("Incorrectly encoded reference " + to_hex(locRef.toBinary()) +
                             ", expected " + hex);
  }
}

void test_offsets_overrun() {
  auto location = "CwG1ASK3PhD82sz0CIAQ89oAAAAAEPPa/N4UcRBj";
  auto locRef = LineLocation(decode64(location));
  if (locRef.poff != 0.f || locRef.noff != 0.f) {
    throw std::runtime_error("Non-zero positive " + std::to_string(locRef.poff) +
                             " or negative offset " + std::to_string(locRef.noff));
  }
}

void test_too_small_reference() {
  auto location = "CwG1ASK3PhD82=";
  try {
    auto locRef = LineLocation(decode64(location));
  } catch (const std::invalid_argument& e) { return; }
  throw std::runtime_error("No error returned");
}

} // namespace

int main(void) {
  test::suite suite("openlr");

  suite.test(TEST_CASE(test_decode));
  suite.test(TEST_CASE(test_internal_reference_points));
  suite.test(TEST_CASE(test_offsets_overrun));
  suite.test(TEST_CASE(test_too_small_reference));

  return suite.tear_down();
}
