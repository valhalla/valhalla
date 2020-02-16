#include "midgard/openlr.h"

#include <boost/archive/iterators/base64_from_binary.hpp>
#include <boost/archive/iterators/binary_from_base64.hpp>
#include <boost/archive/iterators/transform_width.hpp>

#include <iomanip>
#include <sstream>
#include <string>

#include "test.h"

namespace {

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

TEST(OpenLR, Decode) {

  for (auto& fixture : testfixtures) {
    auto locRef = LineLocation(decode64(fixture.descriptor));

    EXPECT_NEAR(locRef.getFirstCoordinate().lng(), fixture.expectedFirstCoordinateLongitude, 1e-5);
    EXPECT_NEAR(locRef.getFirstCoordinate().lat(), fixture.expectedFirstCoordinateLatitude, 1e-5);
    EXPECT_NEAR(locRef.first.bearing, fixture.expectedFirstCoordinateBearing, 1e-5);
    EXPECT_NEAR(locRef.getLastCoordinate().lng(), fixture.expectedLastCoordinateLongitude, 1e-5);
    EXPECT_NEAR(locRef.getLastCoordinate().lat(), fixture.expectedLastCoordinateLatitude, 1e-5);
    EXPECT_NEAR(locRef.last.bearing, fixture.expectedLastCoordinateBearing, 1e-5);

    EXPECT_NEAR(locRef.first.distance, fixture.expectedDistance, 1e-3);
    EXPECT_NEAR(locRef.poff, fixture.expectedPoff, 1e-3);
    EXPECT_NEAR(locRef.noff, fixture.expectedNoff, 1e-3);

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
      EXPECT_EQ(encode64(locRef.toBinary()), fixture.descriptor);
    }
#endif
  }
}

TEST(OpenLR, InternalReferencePoints) {
  auto location = "CwG1ASK3PhD82sz0CIAQ89r83hRxEAM=";

  auto locRef = LineLocation(decode64(location));

  EXPECT_EQ(encode64(locRef.toBinary()), location);
  EXPECT_EQ(locRef.intermediate.size(), 1) << "Incorrectly number of intermediate LRP";

  EXPECT_NEAR(locRef.getFirstCoordinate().lng(), 2.400523, 1e-5);
  EXPECT_NEAR(locRef.getFirstCoordinate().lat(), 48.819069, 1e-5);
  EXPECT_NEAR(locRef.first.bearing, 320.625, 1e-5);
  EXPECT_NEAR(locRef.first.distance, 12774.8, 1e-3);

  EXPECT_NEAR(locRef.intermediate[0].longitude, 2.269843, 1e-5);
  EXPECT_NEAR(locRef.intermediate[0].latitude, 48.840829, 1e-5);
  EXPECT_NEAR(locRef.intermediate[0].bearing, 219.375, 1e-5);
  EXPECT_NEAR(locRef.intermediate[0].distance, 12774.8, 1e-3);

  EXPECT_NEAR(locRef.getLastCoordinate().lng(), 2.261823, 1e-5);
  EXPECT_NEAR(locRef.getLastCoordinate().lat(), 48.893158, 1e-5);
  EXPECT_NEAR(locRef.last.bearing, 39.375, 1e-5);

  EXPECT_NEAR(locRef.getLength(), 2 * 12774.8, 1e-3);
  EXPECT_NEAR(locRef.poff, 0, 1e-3);
  EXPECT_NEAR(locRef.noff, 0, 1e-3);

  for (float poff = 0; poff < locRef.first.distance; poff += locRef.first.distance / 3) {
    for (float noff = 0; noff < locRef.intermediate[0].distance;
         noff += locRef.intermediate[0].distance / 3) {
      locRef.poff = poff;
      locRef.noff = noff;
      LineLocation tryRef(locRef.toBinary());

      EXPECT_NEAR(tryRef.getLength(), 2 * 12774.8, 1e-3) << "Distance incorrect.";
      EXPECT_NEAR(tryRef.poff, locRef.poff, 58.6) << "Positive offset incorrect.";
      EXPECT_NEAR(tryRef.noff, locRef.noff, 58.6) << "Negative offset incorrect.";
    }
  }

  locRef.intermediate.push_back(locRef.intermediate.front());
  locRef.intermediate.push_back(locRef.intermediate.front());
  locRef.intermediate.push_back(locRef.intermediate.front());
  EXPECT_NEAR(locRef.getLength(), 5 * 12774.8, 1e-3) << "Distance incorrect.";

  auto hex = " 0b 01 b5 01 22 b7 3e 10 fc da cc f4 08 80 10 f3 da "
             "00 00 00 00 10 f3 da 00 00 00 00 10 f3 da 00 00 00 00 10 f3 da fc de 14 71 10 63 aa aa";
  EXPECT_EQ(to_hex(locRef.toBinary()), hex) << "Incorrectly encoded reference";
}

TEST(OpenLR, OffsetsOverrun) {
  auto location = "CwG1ASK3PhD82sz0CIAQ89oAAAAAEPPa/N4UcRBj";
  auto locRef = LineLocation(decode64(location));
  EXPECT_EQ(locRef.poff, 0.f);
  EXPECT_EQ(locRef.noff, 0.f);
}

TEST(OpenLR, TooSmallReference) {
  auto location = "CwG1ASK3PhD82=";
  EXPECT_THROW(auto locRef = LineLocation(decode64(location)), std::invalid_argument);
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
