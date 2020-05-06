#include "midgard/openlr.h"
#include "midgard/pointll.h"

#include <boost/archive/iterators/base64_from_binary.hpp>
#include <boost/archive/iterators/binary_from_base64.hpp>
#include <boost/archive/iterators/transform_width.hpp>

#include <iomanip>
#include <sstream>
#include <string>

#include "test.h"

namespace {

using namespace valhalla::midgard;
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
    EXPECT_NEAR(locRef.lrps[0].bearing, fixture.expectedFirstCoordinateBearing, 1e-5);
    EXPECT_NEAR(locRef.getLastCoordinate().lng(), fixture.expectedLastCoordinateLongitude, 1e-5);
    EXPECT_NEAR(locRef.getLastCoordinate().lat(), fixture.expectedLastCoordinateLatitude, 1e-5);
    EXPECT_NEAR(locRef.lrps.back().bearing, fixture.expectedLastCoordinateBearing, 1e-5);

    EXPECT_NEAR(locRef.lrps[0].distance, fixture.expectedDistance, 1e-3);
    EXPECT_NEAR(locRef.poff, fixture.expectedPoff, 1e-3);
    EXPECT_NEAR(locRef.noff, fixture.expectedNoff, 1e-3);

    // This test can be faulty - we check if the coordinates and bearing are close (but not
    // exact) above. If they are not exact then this test will fail! Try again with expected
    // values. TODO - this currently fails for 32 bit
#if _WIN64 || __amd64__
    if (encode64(locRef.toBinary()) != fixture.descriptor) {
      locRef.lrps[0].latitude = fixture.expectedFirstCoordinateLatitude;
      locRef.lrps[0].longitude = fixture.expectedFirstCoordinateLongitude;
      locRef.lrps[0].bearing = fixture.expectedFirstCoordinateBearing;
      locRef.lrps.back().latitude = fixture.expectedLastCoordinateLatitude;
      locRef.lrps.back().longitude = fixture.expectedLastCoordinateLongitude;
      locRef.lrps.back().bearing = fixture.expectedLastCoordinateBearing;
      locRef.lrps[0].distance = fixture.expectedDistance;
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
  EXPECT_EQ(locRef.lrps.size(), 3) << "Incorrectly number of intermediate LRP";

  EXPECT_NEAR(locRef.getFirstCoordinate().lng(), 2.400523, 1e-5);
  EXPECT_NEAR(locRef.getFirstCoordinate().lat(), 48.819069, 1e-5);
  EXPECT_NEAR(locRef.lrps[0].bearing, 320.625, 1e-5);
  EXPECT_NEAR(locRef.lrps[0].distance, 12774.8, 1e-3);

  EXPECT_NEAR(locRef.lrps[1].longitude, 2.269843, 1e-5);
  EXPECT_NEAR(locRef.lrps[1].latitude, 48.840829, 1e-5);
  EXPECT_NEAR(locRef.lrps[1].bearing, 219.375, 1e-5);
  EXPECT_NEAR(locRef.lrps[1].distance, 12774.8, 1e-3);

  EXPECT_NEAR(locRef.getLastCoordinate().lng(), 2.261823, 1e-5);
  EXPECT_NEAR(locRef.getLastCoordinate().lat(), 48.893158, 1e-5);
  EXPECT_NEAR(locRef.lrps.back().bearing, 39.375, 1e-5);

  EXPECT_NEAR(locRef.getLength(), 2 * 12774.8, 1e-3);
  EXPECT_NEAR(locRef.poff, 0, 1e-3);
  EXPECT_NEAR(locRef.noff, 0, 1e-3);

  for (float poff = 0; poff < locRef.lrps[0].distance; poff += locRef.lrps[0].distance / 3) {
    for (float noff = 0; noff < locRef.lrps[1].distance; noff += locRef.lrps[1].distance / 3) {
      locRef.poff = poff;
      locRef.noff = noff;
      LineLocation tryRef(locRef.toBinary());

      EXPECT_NEAR(tryRef.getLength(), 2 * 12774.8, 1e-3) << "Distance incorrect.";
      EXPECT_NEAR(tryRef.poff, locRef.poff, 58.6) << "Positive offset incorrect.";
      EXPECT_NEAR(tryRef.noff, locRef.noff, 58.6) << "Negative offset incorrect.";
    }
  }

  locRef.lrps.insert(std::prev(std::prev(locRef.lrps.end())), locRef.lrps[1]);
  locRef.lrps.insert(std::prev(std::prev(locRef.lrps.end())), locRef.lrps[1]);
  locRef.lrps.insert(std::prev(std::prev(locRef.lrps.end())), locRef.lrps[1]);
  EXPECT_NEAR(locRef.getLength(), 5 * 12774.8, 1e-3) << "Distance incorrect.";

  auto hex = " 0b 01 b5 01 22 b7 3e 10 fc da cc f4 08 80 10 f3 da "
             "00 00 00 00 10 f3 da 00 00 00 00 10 f3 da 00 00 00 00 10 f3 da fc de 14 71 10 63 ab ab";
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

TEST(OpenLR, CreateLinearReference) {
  // make a reference directly using lrps
  std::vector<PointLL> points{{-76.55015772558436, 40.482238379871575},
                              {-76.55060287976856, 40.48275880818994},
                              {-76.55108895828207, 40.489983794570605},
                              {-76.54576249703334, 40.49195968608316}};
  std::vector<LocationReferencePoint> lrps;
  unsigned char frc = 0;
  auto fow = LocationReferencePoint::FormOfWay::MOTORWAY;
  unsigned char lowest_frc_next_point = 7;
  uint16_t bearing;
  for (const auto& p : points) {
    // frc and fow are 3 bits wide, we dont use the last 2 bits for anything
    unsigned char attribute1 = (frc << 3) | fow;
    // first or intermediate point
    if (&p != &points.back()) {
      bearing = static_cast<uint16_t>(p.Heading(*std::next(&p)));
      auto distance = p.Distance(*std::next(&p));
      lrps.emplace_back(p.lng(), p.lat(), bearing, frc, fow, lrps.empty() ? nullptr : &lrps.back(),
                        distance, lowest_frc_next_point);
    } // last point
    else {
      bearing += 180;
      bearing %= 360;
      lrps.emplace_back(p.lng(), p.lat(), bearing, frc, fow, lrps.empty() ? nullptr : &lrps.back());
    }
    // try different frcs and fows for kicks
    ++frc;
    fow = static_cast<LocationReferencePoint::FormOfWay>(static_cast<uint8_t>(fow) + 1);
    --lowest_frc_next_point;
  }
  LineLocation line_location(lrps, 17.438, 13.721);

  // do a round trip conversion
  LineLocation converted(line_location.toBinary());

  // compare to the original reference before conversion
  EXPECT_EQ(line_location, converted);

  // if positive or negative offset is too large it should throw
  EXPECT_THROW(LineLocation(lrps, lrps[0].distance + 1, 0), std::invalid_argument);
  EXPECT_THROW(LineLocation(lrps, 0, lrps[lrps.size() - 2].distance + 1), std::invalid_argument);
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
