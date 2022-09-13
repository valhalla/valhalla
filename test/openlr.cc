#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>

#include "baldr/openlr.h"
#include "midgard/encoded.h"
#include "midgard/pointll.h"
#include "proto/common.pb.h"
#include "proto/trip.pb.h"
#include "tyr/serializers.h"

#include "test.h"

namespace {

using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::baldr::OpenLR;

using FormOfWay = OpenLR::LocationReferencePoint::FormOfWay;

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

  Orientation expectedOrientation;
  SideOfTheRoad expectedSideOfRoad;
};

const testfixture testfixtures[] =
    {{"CwOa9yUQACODBQEqAL4jEw==", 5.069987, 52.119130, 39.375, 5.072967, 52.121030, 219.375, 293, 0,
      0, Orientation::NoOrientation, SideOfTheRoad::DirectlyOnRoadOrNotApplicable},
     {"CwOa9yUQACODqgEqAL4jEw==", 5.069987, 52.119130, 39.375, 5.072967, 52.121030, 219.375, 9962, 0,
      0, Orientation::NoOrientation, SideOfTheRoad::DirectlyOnRoadOrNotApplicable},
     {"CwOiYCUMoBNWAv9P/+MSBg==", 5.110692, 52.100590, 253.125, 5.108922, 52.100300, 73.125, 117.2, 0,
      0, Orientation::NoOrientation, SideOfTheRoad::DirectlyOnRoadOrNotApplicable},
     {"CxWj2OogyxJBDhDSAvwSUL4=", 30.431259, -30.757352, 16.875, 30.474319, -30.749712, 185.625,
      820.4, 190, 0, Orientation::NoOrientation, SideOfTheRoad::DirectlyOnRoadOrNotApplicable},
     {"CxWj2OogyxJBDhDSAvwSMA0=", 30.431259, -30.757352, 16.875, 30.474319, -30.749712, 185.625,
      820.4, 0, 13, Orientation::NoOrientation, SideOfTheRoad::DirectlyOnRoadOrNotApplicable},
     {"CxWj2OogyxJBDhDSAvwScL4N", 30.431259, -30.757352, 16.875, 30.474319, -30.749712, 185.625,
      820.4, 190, 13, Orientation::NoOrientation, SideOfTheRoad::DirectlyOnRoadOrNotApplicable},
     {"C6i8rRtM3BjgAAAAAAUYAA==", -122.713562, 38.390942, 5.625, -122.713562, 38.390991, 5.625, 0, 0,
      0, Orientation::NoOrientation, SideOfTheRoad::DirectlyOnRoadOrNotApplicable},
     {"CwRbWyNG9RpsCQCb/jsbtAT/6/+jK1lE", 6.1268198, 49.6085178, 140.625, 6.1281598, 49.6030578,
      286.875, 527.4, 68, 0, Orientation::NoOrientation,
      SideOfTheRoad::DirectlyOnRoadOrNotApplicable},
     {"CwB67CGukRxiCACyAbwaMXU=", 0.6752192, 47.3651611, 28.125, 0.6769992, 47.3696011, 196.875,
      468.8, 0, 117, Orientation::NoOrientation, SideOfTheRoad::DirectlyOnRoadOrNotApplicable},
     {"CwcX6CItqAs6AQAAAAALGg==", 9.9750602, 48.0632865, 298.125, 9.9750602, 48.0632865, 298.125,
      58.6, 0, 0, Orientation::NoOrientation, SideOfTheRoad::DirectlyOnRoadOrNotApplicable},
     {"CwRbWyNG9BpgAACa/jsboAD/6/+kKwA=", 6.1268198, 49.6084964, 5.625, 6.1281498, 49.6030464, 5.625,
      0, 0, 0, Orientation::NoOrientation, SideOfTheRoad::DirectlyOnRoadOrNotApplicable},
     // PointAlongLine
     {"K6m3URtxwCOLAwDF/5MjW+g=", -121.33676, 38.59359, 129.375, -121.33478546, 38.592496, 309.375,
      175.8, 232, 0, Orientation::NoOrientation, SideOfTheRoad::DirectlyOnRoadOrNotApplicable},
     {"KwRbWyNG9dpsCQCb/jvbVEQ=", 6.1268198, 49.6085178, 140.625, 6.1283698, 49.6039878, 230.625,
      527.4, 68, 0, Orientation::BothDirections, SideOfTheRoad::BothSidesOfRoad}};

TEST(OpenLR, Decode) {

  for (auto& fixture : testfixtures) {
    EXPECT_NO_THROW(OpenLr(fixture.descriptor, true));
    auto locRef = OpenLr(fixture.descriptor, true);
    EXPECT_NEAR(locRef.getFirstCoordinate().lng(), fixture.expectedFirstCoordinateLongitude, 1e-5);
    EXPECT_NEAR(locRef.getFirstCoordinate().lat(), fixture.expectedFirstCoordinateLatitude, 1e-5);
    EXPECT_NEAR(locRef.lrps[0].bearing, fixture.expectedFirstCoordinateBearing, 1e-5);
    EXPECT_NEAR(locRef.getLastCoordinate().lng(), fixture.expectedLastCoordinateLongitude, 1e-5);
    EXPECT_NEAR(locRef.getLastCoordinate().lat(), fixture.expectedLastCoordinateLatitude, 1e-5);
    EXPECT_NEAR(locRef.lrps.back().bearing, fixture.expectedLastCoordinateBearing, 1e-5);

    EXPECT_NEAR(locRef.lrps[0].distance, fixture.expectedDistance, 1e-3);
    EXPECT_NEAR(locRef.poff, fixture.expectedPoff, 1e-3);
    EXPECT_NEAR(locRef.noff, fixture.expectedNoff, 1e-3);

    EXPECT_EQ(locRef.orientation, fixture.expectedOrientation);
    EXPECT_EQ(locRef.sideOfTheRoad, fixture.expectedSideOfRoad);

    // This test can be faulty - we check if the coordinates and bearing are close (but not
    // exact) above. If they are not exact then this test will fail! Try again with expected
    // values. TODO - this currently fails for 32 bit
#if _WIN64 || __amd64__
    if (locRef.toBase64() != fixture.descriptor) {
      locRef.lrps[0].latitude = fixture.expectedFirstCoordinateLatitude;
      locRef.lrps[0].longitude = fixture.expectedFirstCoordinateLongitude;
      locRef.lrps[0].bearing = fixture.expectedFirstCoordinateBearing;
      locRef.lrps.back().latitude = fixture.expectedLastCoordinateLatitude;
      locRef.lrps.back().longitude = fixture.expectedLastCoordinateLongitude;
      locRef.lrps.back().bearing = fixture.expectedLastCoordinateBearing;
      locRef.lrps[0].distance = fixture.expectedDistance;
      locRef.poff = fixture.expectedPoff;
      locRef.noff = fixture.expectedNoff;
      locRef.orientation = fixture.expectedOrientation;
      locRef.sideOfTheRoad = fixture.expectedSideOfRoad;
      EXPECT_EQ(locRef.toBase64(), fixture.descriptor);
    }
#endif
  }
}

TEST(OpenLR, InternalReferencePoints) {
  auto location = "CwG1ASK3PhD82sz0CIAQ89r83hRxEAM=";

  auto locRef = OpenLr(location, true);

  EXPECT_EQ(locRef.toBase64(), location);
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

  for (double poff = 0; poff < locRef.lrps[0].distance;
       poff += locRef.lrps[0].distance / 3) { // NOLINT
    for (double noff = 0; noff < locRef.lrps[1].distance;
         noff += locRef.lrps[1].distance / 3) { // NOLINT
      locRef.poff = 256 * poff / locRef.lrps[0].distance;
      locRef.noff = 256 * noff / locRef.lrps[1].distance;
      OpenLr tryRef(locRef.toBinary());

      EXPECT_NEAR(tryRef.getLength(), 2 * 12774.8, 1e-3) << "Distance incorrect.";
      EXPECT_EQ(tryRef.poff, locRef.poff) << "Positive offset incorrect.";
      EXPECT_EQ(tryRef.noff, locRef.noff) << "Negative offset incorrect.";
    }
  }

  locRef.lrps.insert(std::prev(std::prev(locRef.lrps.end())), locRef.lrps[1]);
  locRef.lrps.insert(std::prev(std::prev(locRef.lrps.end())), locRef.lrps[1]);
  locRef.lrps.insert(std::prev(std::prev(locRef.lrps.end())), locRef.lrps[1]);
  EXPECT_NEAR(locRef.getLength(), 5 * 12774.8, 1e-3) << "Distance incorrect.";

  auto hex =
      " 0b 01 b5 01 22 b7 3e 10 fc da cc f4 08 80 10 f3 da 00 00 00 00 10 f3 da 00 00 00 00 10 f3 da 00 00 00 00 10 f3 da fc de 14 71 10 63 aa aa";
  EXPECT_EQ(to_hex(locRef.toBinary()), hex) << "Incorrectly encoded reference";
}

TEST(OpenLR, OffsetsOverrun) {
  auto location = "CwG1ASK3PhD82sz0CIAQ89oAAAAAEPPa/N4UcRBj";
  auto locRef = OpenLr(location, true);
  EXPECT_EQ(locRef.poff, 0.f);
  EXPECT_EQ(locRef.noff, 0.f);
}

TEST(OpenLR, TooSmallReference) {
  // This location is a substring of the OpenLR segment in InternalReferencePoints above.
  auto location = "83hRxEAM=";
  EXPECT_THROW(auto locRef = OpenLr(location, true), std::invalid_argument);
}

TEST(OpenLR, CreateLinearReference) {
  // make a reference directly using lrps
  std::vector<PointLL> points{{-76.550157, 40.482238},
                              {-76.550602, 40.482758},
                              {-76.551088, 40.489983},
                              {-76.545762, 40.491959}};
  std::vector<LocationReferencePoint> lrps;
  unsigned char frc = 0;
  auto fow = LocationReferencePoint::FormOfWay::MOTORWAY;
  unsigned char lowest_frc_next_point = 7;
  uint16_t bearing = 0;
  for (const auto& p : points) {
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
    EXPECT_NEAR(lrps.back().longitude, p.lng(), .00002);
    EXPECT_NEAR(lrps.back().latitude, p.lat(), .00002);
    // try different frcs and fows for kicks
    ++frc;
    fow = static_cast<LocationReferencePoint::FormOfWay>(static_cast<uint8_t>(fow) + 1);
    --lowest_frc_next_point;
  }
  OpenLr line_location(lrps, 12, 234);

  // do a round trip conversion
  OpenLr converted(line_location.toBinary());

  // compare to the original reference before conversion
  EXPECT_EQ(line_location, converted);

  // If only one LRP, should error
  EXPECT_THROW(OpenLr({lrps.front()}, 0, 0), std::invalid_argument);
  // If we only have 2 LRPs, and the pos/neg offsets would overlap, should throw
  EXPECT_THROW(OpenLr({lrps.front(), lrps.back()}, 0.6 * 255, 0.6 * 255), std::invalid_argument);

  // make a short line location so that poff and noff must be 0
  lrps.clear();
  lrps.emplace_back(0, 0, 90, frc, fow, nullptr, 5, lowest_frc_next_point);
  lrps.emplace_back(.000005, 0, 270, frc, fow, &lrps.back());
  OpenLr short_location(lrps, 0, 0);
  EXPECT_EQ(short_location.poff, 0);
  EXPECT_EQ(short_location.noff, 0);
}

TEST(OpenLR, EncodeDecodeDistancePrecision) {
  for (double dist = 0; dist < 15000; dist += 58.6) {
    // construct location reference points (truncate precision)
    LocationReferencePoint first(-76.550157, 40.482238, 0.f, 0,
                                 LocationReferencePoint::FormOfWay::MOTORWAY, nullptr, dist);
    LocationReferencePoint last(-76.550602, 40.482758, 0.f, 0,
                                LocationReferencePoint::FormOfWay::MOTORWAY, nullptr, 0.);
    OpenLr record({first, last}, 0, 0);
    // encode-decode openlr record
    OpenLr decoded(record.toBinary());
    // compare distance value after decoding
    ASSERT_NEAR(decoded.lrps.front().distance, first.distance, 1e-3);
  }
}

TEST(OpenLR, EncodeDecodeBearingPrecision) {
  for (double bearing = 0; bearing < 360; bearing += 11.25) {
    // construct location reference points (truncate precision)
    LocationReferencePoint first(-76.550157, 40.482238, bearing, 0,
                                 LocationReferencePoint::FormOfWay::MOTORWAY, nullptr);
    auto last_bearing = bearing < 180 ? (bearing + 180) : (bearing - 180);
    LocationReferencePoint last(-76.550602, 40.482758, last_bearing, 0,
                                LocationReferencePoint::FormOfWay::MOTORWAY, nullptr);
    OpenLr record({first, last}, 0, 0);
    // encode-decode openlr record
    OpenLr decoded(record.toBinary());
    // compare bearing values after decoding
    ASSERT_NEAR(decoded.lrps.front().bearing, first.bearing, 1e-3);
    ASSERT_NEAR(decoded.lrps.back().bearing, last.bearing, 1e-3);
  }
}

TripLeg CreateLeg(const std::vector<PointLL>& points) {
  TripLeg path;
  TripLeg::Node* node;
  TripLeg::Edge* edge;
  node = path.add_node();
  path.set_shape(encode(points));
  edge = node->mutable_edge();
  edge->set_begin_shape_index(0);
  edge->set_end_shape_index(1);
  edge->set_use(TripLeg::kRoadUse);
  edge->set_road_class(valhalla::RoadClass::kPrimary);
  return path;
}

std::vector<OpenLR::OpenLr> LegToOpenLrs(TripLeg&& leg) {
  // gin up a route/request for it
  valhalla::Options options;
  options.set_action(Options::route);
  options.set_linear_references(true);
  valhalla::TripRoute route;
  route.mutable_legs()->Add()->Swap(&leg);
  baldr::json::MapPtr container = baldr::json::map({});

  // serialize some b64 encoded openlrs and get them back out as openlr objects
  tyr::route_references(container, route, options);
  auto references = boost::get<baldr::json::ArrayPtr>(container->find("linear_references")->second);
  std::vector<OpenLR::OpenLr> openlrs;
  for (const auto& reference : *references) {
    auto b64 = boost::get<std::string>(reference);
    openlrs.emplace_back(b64, true);
  }

  return openlrs;
}

TEST(OpenLR, road_class_to_fow) {
  // make a leg to test basic behavior
  auto leg = CreateLeg({{0, 0}, {1, 1}});
  auto* edge = leg.mutable_node(0)->mutable_edge();
  edge->set_roundabout(true);
  edge->set_use(TripLeg::kRoadUse);
  edge->set_road_class(valhalla::RoadClass::kPrimary);
  auto openlrs = LegToOpenLrs(std::move(leg));

  // parse them out and check whats in them
  EXPECT_EQ(openlrs.front().lrps[0].fow, FormOfWay::ROUNDABOUT);
}

// TripPath to OpenLR tests: Spot check that some two point line segments (on a primary road
// segment). Note that the validation of OpenLR serialization is done in the tests fixtures above.

TEST(OpenLR, openlr_edges) {
  // Check encoding
  std::vector<PointLL> points = {
      PointLL(5.08531221, 52.0938563),
      PointLL(5.0865867, 52.0930211),
  };
  auto openlrs = LegToOpenLrs(CreateLeg(points));
  EXPECT_EQ(openlrs.size(), 1);
  const auto& decoded = openlrs[0];
  EXPECT_EQ(decoded.lrps.size(), 2);
  PointLL first = decoded.getFirstCoordinate();
  EXPECT_NEAR(first.lat(), points.at(0).lat(), 0.0001);
  EXPECT_NEAR(first.lng(), points.at(0).lng(), 0.0001);
  PointLL last = decoded.getLastCoordinate();
  EXPECT_NEAR(last.lat(), points.at(1).lat(), 0.0001);
  EXPECT_NEAR(last.lng(), points.at(1).lng(), 0.0001);
}

TEST(OpenLR, openlr_edges_duplicate) {
  // Check encoding
  std::vector<PointLL> points = {
      PointLL(5.08531221, 52.0938563),
      PointLL(5.08531221, 52.0938563),

  };
  auto openlrs = LegToOpenLrs(CreateLeg(points));
  EXPECT_EQ(openlrs.size(), 1);
  const auto& decoded = openlrs[0];
  EXPECT_EQ(decoded.lrps.size(), 2);
  PointLL first = decoded.getFirstCoordinate();
  EXPECT_NEAR(first.lat(), points.at(0).lat(), 0.0001);
  EXPECT_NEAR(first.lng(), points.at(0).lng(), 0.0001);
  PointLL last = decoded.getLastCoordinate();
  EXPECT_NEAR(last.lat(), points.at(1).lat(), 0.0001);
  EXPECT_NEAR(last.lng(), points.at(1).lng(), 0.0001);
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
