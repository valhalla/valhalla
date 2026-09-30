#include "gurka.h"
#include "test.h"

#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

namespace {
const std::array<std::string, 3> kDateTimeTypes = {"1", "2", "3"};
const std::array<std::string, 6> kMotorVehicleCostingModels = {"auto",  "bus",
                                                               "taxi",  "motor_scooter",
                                                               "truck", "motorcycle"};
} // namespace

using namespace valhalla;

/*
 * Tests the timeAllowed and timeDenied conditional restrictions for auto, bicycle and pedestrian
 * costing. NOTE: To test restrictions, the map must contain more than 1 edge.  If you only have 1
 * edge, then it will never set the date_time info because its traversing a trivial path (see
 * https://github.com/valhalla/valhalla/blob/master/src/thor/triplegbuilder.cc#L1216)
 */
class ConditionalRestrictions : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double grid_size_meters = 100;

    const std::string ascii_map = R"(
         A----B
         |    |
         D    C
          \  /
           E

         F----G----H----I----J----K----L
                             |    |
                             M----N

          R---S---T---U
    )";

    const std::string condition =
        "(Mar 00:00-07:00;Mar 17:30-24:00;Apr 00:00-07:00;Apr 19:00-24:00;Aug 00:00-07:00;Aug 19:00-24:00)";
    const std::string timeDenied = "no @ " + condition;
    const std::string yesAllowed = "yes @ " + condition;
    const std::string privateAllowed = "private @ " + condition;
    const std::string deliveryAllowed = "delivery @ " + condition;
    const std::string designatedAllowed = "designated @ " + condition;
    const std::string destinationAllowed = "destination @ " + condition;
    const gurka::ways ways = {
        {"AD",
         {{"highway", "service"},
          {"motorcar", "no"},
          {"bicycle", "yes"},
          {"foot", "yes"},
          {"bicycle:conditional", yesAllowed},
          {"foot:conditional", privateAllowed}}},
        {"AB",
         {{"highway", "service"},
          {"motorcar:conditional", deliveryAllowed},
          {"motorcycle:conditional", deliveryAllowed},
          {"moped:conditional", deliveryAllowed},
          {"psv:conditional", deliveryAllowed},
          {"taxi:conditional", deliveryAllowed},
          {"bus:conditional", deliveryAllowed},
          {"hov:conditional", deliveryAllowed},
          {"emergency:conditional", deliveryAllowed},
          {"bicycle:conditional", timeDenied},
          {"foot:conditional", designatedAllowed}}},
        {"BC", {{"highway", "service"}, {"motorcar:conditional", destinationAllowed}}},
        {"CD",
         {{"highway", "service"},
          {"motorcar:conditional", yesAllowed},
          {"bicycle:conditional", timeDenied},
          {"foot:conditional", privateAllowed}}},
        {"DE",
         {{"highway", "service"},
          {"motorcar:conditional", deliveryAllowed},
          {"bicycle:conditional", timeDenied},
          {"foot:conditional", designatedAllowed}}},
        {"CE",
         {{"highway", "service"},
          {"motorcar:conditional", destinationAllowed},
          {"bicycle:conditional", timeDenied},
          {"foot:conditional", yesAllowed}}},

        {"FG", {{"highway", "residential"}}},
        {"GH", {{"highway", "residential"}}},
        {"HI", {{"highway", "residential"}, {"motor_vehicle:conditional", destinationAllowed}}},
        {"IJ", {{"highway", "residential"}}},
        {"JK", {{"highway", "residential"}, {"motor_vehicle:conditional", destinationAllowed}}},
        {"KL", {{"highway", "residential"}}},
        {"JM", {{"highway", "residential"}}},
        {"MN", {{"highway", "residential"}}},
        {"NK", {{"highway", "residential"}}},

        {"RS", {{"highway", "primary"}}},
        {"ST", {{"highway", "primary"}, {"access:conditional", timeDenied}}},
        {"TU", {{"highway", "primary"}}},
    };

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, grid_size_meters);
    map = gurka::buildtiles(layout, ways, {}, {},
                            VALHALLA_BUILD_DIR "test/data/conditional_restrictions",
                            {{"mjolnir.timezone", {VALHALLA_BUILD_DIR "test/data/tz.sqlite"}}});
  }
};

gurka::map ConditionalRestrictions::map = {};

/*************************************************************/

TEST_F(ConditionalRestrictions, NoRestrictionAutoNoDate) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "E"}, "auto");
  gurka::assert::osrm::expect_steps(result, {"AB", "BC", "CE"});
  gurka::assert::raw::expect_path(result, {"AB", "BC", "CE"});
}

TEST_F(ConditionalRestrictions, NoRestrictionAuto) {
  auto result =
      gurka::do_action(valhalla::Options::route, map, {"A", "E"}, "auto",
                       {{"/date_time/type", "1"}, {"/date_time/value", "2020-04-15T06:00"}});
  gurka::assert::osrm::expect_steps(result, {"AB", "BC", "CE"});
  gurka::assert::raw::expect_path(result, {"AB", "BC", "CE"});
}

TEST_F(ConditionalRestrictions, RestrictionAuto) {
  auto result =
      gurka::do_action(valhalla::Options::route, map, {"A", "E"}, "auto",
                       {{"/date_time/type", "1"}, {"/date_time/value", "2020-04-02T12:00"}});
  gurka::assert::osrm::expect_steps(result, {"AB", "BC", "CE"});
  gurka::assert::raw::expect_path(result, {"AB", "BC", "CE"});
}

TEST_F(ConditionalRestrictions, NoRestrictionBikeNoDate) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "E"}, "bicycle");
  gurka::assert::osrm::expect_steps(result, {"AD", "DE"});
  gurka::assert::raw::expect_path(result, {"AD", "DE"});
}

TEST_F(ConditionalRestrictions, NoRestrictionBike) {
  auto result =
      gurka::do_action(valhalla::Options::route, map, {"A", "E"}, "bicycle",
                       {{"/date_time/type", "1"}, {"/date_time/value", "2020-04-02T12:00"}});
  gurka::assert::osrm::expect_steps(result, {"AD", "DE"});
  gurka::assert::raw::expect_path(result, {"AD", "DE"});
}

TEST_F(ConditionalRestrictions, RestrictionBike) {
  // this tests that the expected exception is thrown
  EXPECT_THROW(
      {
        try {
          [[maybe_unused]] auto result =
              gurka::do_action(valhalla::Options::route, map, {"A", "E"}, "bicycle",
                               {{"/date_time/type", "1"}, {"/date_time/value", "2020-04-02T20:00"}});
        } catch (const std::exception& e) {
          // and this tests that it has the correct message
          EXPECT_STREQ("No path could be found for input", e.what());
          throw;
        }
      },
      std::exception);
}

TEST_F(ConditionalRestrictions, NoRestrictionPedestrianNoDate) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "E"}, "pedestrian");
  gurka::assert::osrm::expect_steps(result, {"AD", "DE"});
  gurka::assert::raw::expect_path(result, {"AD", "DE"});
}

TEST_F(ConditionalRestrictions, NoRestrictionPedestrian) {
  auto result =
      gurka::do_action(valhalla::Options::route, map, {"A", "E"}, "pedestrian",
                       {{"/date_time/type", "1"}, {"/date_time/value", "2020-04-02T20:00"}});
  gurka::assert::osrm::expect_steps(result, {"AD", "DE"});
  gurka::assert::raw::expect_path(result, {"AD", "DE"});
}

TEST_F(ConditionalRestrictions, RestrictionPedestrian) {
  // this tests that the expected exception is thrown
  EXPECT_THROW(
      {
        try {
          [[maybe_unused]] auto result =
              gurka::do_action(valhalla::Options::route, map, {"A", "E"}, "pedestrian",
                               {{"/date_time/type", "1"}, {"/date_time/value", "2020-04-02T12:00"}});
        } catch (const std::exception& e) {
          // and this tests that it has the correct message
          EXPECT_STREQ("No path could be found for input", e.what());
          throw;
        }
      },
      std::exception);
}

TEST_F(ConditionalRestrictions, DestinationRestrictionOnLastEdgeIsValid) {
  for (auto const& date_time_type : kDateTimeTypes) {
    for (auto const& costing : kMotorVehicleCostingModels) {
      auto result = gurka::do_action(valhalla::Options::route, map, {"F", "I"}, costing,
                                     {{"/date_time/type", date_time_type},
                                      {"/date_time/value", "2020-04-04T20:00"}});
      gurka::assert::raw::expect_path(result, {"FG", "GH", "HI"},
                                      "Date time type: " + date_time_type +
                                          ", costing type: " + costing);
    }
  }
}

TEST_F(ConditionalRestrictions, DestinationRestrictionOnLastEdgeIsNotValid) {
  for (auto const& date_time_type : kDateTimeTypes) {
    for (auto const& costing : kMotorVehicleCostingModels) {
      auto result = gurka::do_action(valhalla::Options::route, map, {"F", "I"}, costing,
                                     {{"/date_time/type", date_time_type},
                                      {"/date_time/value", "2020-04-04T12:00"}});
      gurka::assert::raw::expect_path(result, {"FG", "GH", "HI"},
                                      "Date time type: " + date_time_type +
                                          ", costing type: " + costing);
    }
  }
}

TEST_F(ConditionalRestrictions, DestinationRestrictionOnMidEdgeIsValid_OnlyAlternative) {
  for (auto const& date_time_type : kDateTimeTypes) {
    for (auto const& costing : kMotorVehicleCostingModels) {
      auto result = gurka::do_action(valhalla::Options::route, map, {"F", "J"}, costing,
                                     {{"/date_time/type", date_time_type},
                                      {"/date_time/value", "2020-04-04T20:00"}});
      gurka::assert::raw::expect_path(result, {"FG", "GH", "HI", "IJ"},
                                      "Date time type: " + date_time_type +
                                          ", costing type: " + costing);
    }
  }
}

TEST_F(ConditionalRestrictions, DestinationRestrictionOnMidEdgeIsNotValid_OnlyAlternative) {
  for (auto const& date_time_type : kDateTimeTypes) {
    for (auto const& costing : kMotorVehicleCostingModels) {
      auto result = gurka::do_action(valhalla::Options::route, map, {"F", "J"}, costing,
                                     {{"/date_time/type", date_time_type},
                                      {"/date_time/value", "2020-04-04T12:00"}});
      gurka::assert::raw::expect_path(result, {"FG", "GH", "HI", "IJ"},
                                      "Date time type: " + date_time_type +
                                          ", costing type: " + costing);
    }
  }
}

TEST_F(ConditionalRestrictions, DestinationRestrictionOnMidEdgeIsValid_ManyAlternatives) {
  for (auto const& date_time_type : kDateTimeTypes) {
    for (auto const& costing : kMotorVehicleCostingModels) {
      auto result = gurka::do_action(valhalla::Options::route, map, {"I", "L"}, costing,
                                     {{"/date_time/type", date_time_type},
                                      {"/date_time/value", "2020-04-04T20:00"}});
      gurka::assert::raw::expect_path(result, {"IJ", "JM", "MN", "NK", "KL"},
                                      "Date time type: " + date_time_type +
                                          ", costing type: " + costing);
    }
  }
}

TEST_F(ConditionalRestrictions, DestinationRestrictionOnMidEdgeIsNotValid_ManyAlternatives) {
  for (auto const& date_time_type : kDateTimeTypes) {
    for (auto const& costing : kMotorVehicleCostingModels) {
      auto result = gurka::do_action(valhalla::Options::route, map, {"I", "L"}, costing,
                                     {{"/date_time/type", date_time_type},
                                      {"/date_time/value", "2020-04-04T12:00"}});
      gurka::assert::raw::expect_path(result, {"IJ", "JK", "KL"},
                                      "Date time type: " + date_time_type +
                                          ", costing type: " + costing);
    }
  }
}

TEST_F(ConditionalRestrictions, AccessConditional) {
  std::vector<std::string> all_costings(kMotorVehicleCostingModels.begin(),
                                        kMotorVehicleCostingModels.end());
  all_costings.push_back("bicycle");
  all_costings.push_back("pedestrian");

  const std::vector<std::string> wypoints = {"R", "U"};
  const std::vector<std::string> expected_path = {"RS", "ST", "TU"};
  const std::string good_time = "2020-04-02T12:00";
  const std::string bad_time = "2020-04-02T20:00";

  for (auto const& costing : all_costings) {
    // no time - all good
    auto result = gurka::do_action(valhalla::Options::route, map, wypoints, costing);
    gurka::assert::raw::expect_path(result, expected_path, costing);

    // time outside of restriction - all good
    result = gurka::do_action(valhalla::Options::route, map, wypoints, costing,
                              {{"/date_time/type", "1"}, {"/date_time/value", good_time}});
    gurka::assert::raw::expect_path(result, expected_path, costing);

    // time inside of restriction - no path
    EXPECT_THROW(
        {
          try {
            [[maybe_unused]] auto result =
                gurka::do_action(valhalla::Options::route, map, wypoints, costing,
                                 {{"/date_time/type", "1"}, {"/date_time/value", bad_time}});
          } catch (const std::exception& e) {
            // and this tests that it has the correct message
            EXPECT_STREQ("No path could be found for input", e.what());
            throw;
          }
        },
        std::exception)
        << costing;

    // time inside of restriction - all good if restrictions are ignored
    result = gurka::do_action(valhalla::Options::route, map, wypoints, costing,
                              {{"/date_time/type", "1"},
                               {"/date_time/value", bad_time},
                               {"/costing_options/" + costing + "/ignore_restrictions", "1"}});
    gurka::assert::raw::expect_path(result, expected_path, costing);

    // 'access' vs 'restriction' might be difficult to articulate. As per
    // https://wiki.openstreetmap.org/wiki/Key:access#Access_time_and_other_conditional_restrictions,
    // the `access:conditional` falls under the 'conditional restrictions' category, so
    // `ignore_access` doesn't apply (and also because it is implemented via conditional
    // restrictions).
    EXPECT_THROW(
        {
          try {
            [[maybe_unused]] auto result =
                gurka::do_action(valhalla::Options::route, map, wypoints, costing,
                                 {{"/date_time/type", "1"},
                                  {"/date_time/value", bad_time},
                                  {"/costing_options/" + costing + "/ignore_access", "1"}});
          } catch (const std::exception& e) {
            // and this tests that it has the correct message
            EXPECT_STREQ("No path could be found for input", e.what());
            throw;
          }
        },
        std::exception)
        << costing;
  }
}
class DestinationOnlyZones : public ::testing::Test {
protected:
  // A contiguous run of dest-only edges is a zone, static and conditional parts alike: traffic
  // with an endpoint inside may cross all of it, through traffic may not.
  static constexpr const char* kZoneCondition = "destination @ (Mo-Su 09:00-17:00)";
  static constexpr const char* kRestricted = "2020-04-02T12:00";
  static constexpr const char* kUnrestricted = "2020-04-02T20:00";

  static gurka::map map;

  static void SetUpTestSuite() {
    const std::string ascii_map = R"(
       A----B----C----D----E----F----G----H----I
            |                   |    |         |
            J-------------------K    +----L----+
                                     |         |
                                     +----M----+
    )";
    const std::pair<std::string, std::string> stat = {"motor_vehicle", "destination"};
    const std::pair<std::string, std::string> cond = {"motor_vehicle:conditional", kZoneCondition};
    const gurka::ways ways = {
        {"AB", {{"highway", "residential"}}},       {"BC", {{"highway", "residential"}, stat}},
        {"CD", {{"highway", "residential"}, stat}}, {"DE", {{"highway", "residential"}, cond}},
        {"EF", {{"highway", "residential"}, cond}}, {"FG", {{"highway", "residential"}}},
        {"GH", {{"highway", "residential"}, cond}}, {"HI", {{"highway", "residential"}}},
        {"GL", {{"highway", "residential"}, stat}}, {"LI", {{"highway", "residential"}}},
        {"GM", {{"highway", "residential"}}},       {"MI", {{"highway", "residential"}}},
        {"BJ", {{"highway", "residential"}}},       {"JK", {{"highway", "residential"}}},
        {"KF", {{"highway", "residential"}}},
    };
    const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
    map =
        gurka::buildtiles(layout, ways, {}, {}, VALHALLA_BUILD_DIR "test/data/destination_only_zones",
                          {{"mjolnir.timezone", {VALHALLA_BUILD_DIR "test/data/tz.sqlite"}},
                           // loki drops date_time from a matrix request beyond this
                           {"service_limits.max_timedep_distance_matrix", "50000"}});
  }

  // warning 401 means pass 0 failed and dest-only was dropped globally for the request
  static bool used_relaxed_pass(const valhalla::Api& result) {
    for (const auto& w : result.info().warnings())
      if (w.code() == 401)
        return true;
    return false;
  }
};
gurka::map DestinationOnlyZones::map = {};

// Origin F rather than G: an origin edge is seeded without an access check, so a restricted
// edge has to be one hop in to be evaluated at all.
TEST_F(DestinationOnlyZones, ThroughTrafficExcluded) {
  for (const auto& dt : kDateTimeTypes) {
    auto result = gurka::do_action(valhalla::Options::route, map, {"F", "I"}, "auto",
                                   {{"/date_time/type", dt}, {"/date_time/value", kRestricted}});
    gurka::assert::raw::expect_path(result, {"FG", "GM", "MI"}, "date_time type " + dt);
    EXPECT_FALSE(used_relaxed_pass(result)) << "date_time type " << dt;
  }
}

// With the penalty at zero nothing separates an open conditional zone from a static one, so
// the shortest branch wins for the time-dependent trees and bidirectional still refuses both.
TEST_F(DestinationOnlyZones, ZeroPenaltyTreatsConditionalLikeStatic) {
  for (const auto& dt : {std::string("1"), std::string("2")}) {
    auto result = gurka::do_action(valhalla::Options::route, map, {"F", "I"}, "auto",
                                   {{"/date_time/type", dt},
                                    {"/date_time/value", kRestricted},
                                    {"/costing_options/auto/destination_only_penalty", "0"}});
    gurka::assert::raw::expect_path(result, {"FG", "GH", "HI"}, "date_time type " + dt);
  }

  auto bidirectional = gurka::do_action(valhalla::Options::route, map, {"F", "I"}, "auto",
                                        {{"/date_time/type", "3"},
                                         {"/date_time/value", kRestricted},
                                         {"/costing_options/auto/destination_only_penalty", "0"}});
  gurka::assert::raw::expect_path(bidirectional, {"FG", "GM", "MI"});
}

TEST_F(DestinationOnlyZones, ThroughTrafficAllowedOutsideWindow) {
  for (const auto& dt : kDateTimeTypes) {
    auto result = gurka::do_action(valhalla::Options::route, map, {"F", "I"}, "auto",
                                   {{"/date_time/type", dt}, {"/date_time/value", kUnrestricted}});
    gurka::assert::raw::expect_path(result, {"FG", "GH", "HI"}, "date_time type " + dt);
  }
}

TEST_F(DestinationOnlyZones, EnterOneEdge) {
  for (const auto& dt : kDateTimeTypes) {
    auto result = gurka::do_action(valhalla::Options::route, map, {"G", "E"}, "auto",
                                   {{"/date_time/type", dt},
                                    {"/date_time/value", kRestricted},
                                    {"/costing_options/auto/destination_only_penalty", "0"}});
    gurka::assert::raw::expect_path(result, {"FG", "EF"}, "date_time type " + dt);
    EXPECT_FALSE(used_relaxed_pass(result)) << "date_time type " << dt;
  }
}

TEST_F(DestinationOnlyZones, LeaveOneEdge) {
  for (const auto& dt : kDateTimeTypes) {
    auto result = gurka::do_action(valhalla::Options::route, map, {"E", "G"}, "auto",
                                   {{"/date_time/type", dt},
                                    {"/date_time/value", kRestricted},
                                    {"/costing_options/auto/destination_only_penalty", "0"}});
    gurka::assert::raw::expect_path(result, {"EF", "FG"}, "date_time type " + dt);
    EXPECT_FALSE(used_relaxed_pass(result)) << "date_time type " << dt;
  }
}

TEST_F(DestinationOnlyZones, EnterTwoEdges) {
  for (const auto& dt : kDateTimeTypes) {
    auto result = gurka::do_action(valhalla::Options::route, map, {"G", "D"}, "auto",
                                   {{"/date_time/type", dt},
                                    {"/date_time/value", kRestricted},
                                    {"/costing_options/auto/destination_only_penalty", "0"}});
    gurka::assert::raw::expect_path(result, {"FG", "EF", "DE"}, "date_time type " + dt);
    EXPECT_FALSE(used_relaxed_pass(result)) << "date_time type " << dt;
  }
}

TEST_F(DestinationOnlyZones, LeaveTwoEdges) {
  for (const auto& dt : kDateTimeTypes) {
    auto result = gurka::do_action(valhalla::Options::route, map, {"D", "G"}, "auto",
                                   {{"/date_time/type", dt},
                                    {"/date_time/value", kRestricted},
                                    {"/costing_options/auto/destination_only_penalty", "0"}});
    gurka::assert::raw::expect_path(result, {"DE", "EF", "FG"}, "date_time type " + dt);
    EXPECT_FALSE(used_relaxed_pass(result)) << "date_time type " << dt;
  }
}

// `pred.destonly()` is false on a conditional edge, so the seam trips the static check too.
TEST_F(DestinationOnlyZones, StaticOriginThroughConditional) {
  for (const auto& dt : {std::string("1"), std::string("3")}) {
    auto result = gurka::do_action(valhalla::Options::route, map, {"C", "G"}, "auto",
                                   {{"/date_time/type", dt},
                                    {"/date_time/value", kRestricted},
                                    {"/costing_options/auto/destination_only_penalty", "0"}});
    gurka::assert::raw::expect_path(result, {"CD", "DE", "EF", "FG"}, "date_time type " + dt);
    EXPECT_FALSE(used_relaxed_pass(result)) << "date_time type " << dt;
  }
}

TEST_F(DestinationOnlyZones, StaticDestinationThroughConditional) {
  for (const auto& dt : {std::string("2"), std::string("3")}) {
    auto result = gurka::do_action(valhalla::Options::route, map, {"G", "C"}, "auto",
                                   {{"/date_time/type", dt},
                                    {"/date_time/value", kRestricted},
                                    {"/costing_options/auto/destination_only_penalty", "0"}});
    gurka::assert::raw::expect_path(result, {"FG", "EF", "DE", "CD"}, "date_time type " + dt);
    EXPECT_FALSE(used_relaxed_pass(result)) << "date_time type " << dt;
  }
}

TEST_F(DestinationOnlyZones, ConditionalOriginThroughStatic) {
  for (const auto& dt : kDateTimeTypes) {
    auto result = gurka::do_action(valhalla::Options::route, map, {"E", "A"}, "auto",
                                   {{"/date_time/type", dt},
                                    {"/date_time/value", kRestricted},
                                    {"/costing_options/auto/destination_only_penalty", "0"}});
    gurka::assert::raw::expect_path(result, {"DE", "CD", "BC", "AB"}, "date_time type " + dt);
    EXPECT_FALSE(used_relaxed_pass(result)) << "date_time type " << dt;
  }
}

TEST_F(DestinationOnlyZones, ConditionalDestinationThroughStatic) {
  for (const auto& dt : kDateTimeTypes) {
    auto result = gurka::do_action(valhalla::Options::route, map, {"A", "E"}, "auto",
                                   {{"/date_time/type", dt},
                                    {"/date_time/value", kRestricted},
                                    {"/costing_options/auto/destination_only_penalty", "0"}});
    gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DE"}, "date_time type " + dt);
    EXPECT_FALSE(used_relaxed_pass(result)) << "date_time type " << dt;
  }
}

// Leaving a zone must not switch dest-only off for the rest of the route; either downstream
// shortcut being taken means the exemption leaked past the zone boundary.
TEST_F(DestinationOnlyZones, StillAppliesAfterLeavingZone) {
  for (const auto& dt : kDateTimeTypes) {
    auto result = gurka::do_action(valhalla::Options::route, map, {"E", "I"}, "auto",
                                   {{"/date_time/type", dt}, {"/date_time/value", kRestricted}});
    gurka::assert::raw::expect_path(result, {"EF", "FG", "GM", "MI"}, "date_time type " + dt);
    EXPECT_FALSE(used_relaxed_pass(result)) << "date_time type " << dt;
  }
}

TEST_F(DestinationOnlyZones, DestinationInDownstreamZone) {
  for (const auto& dt : kDateTimeTypes) {
    auto result = gurka::do_action(valhalla::Options::route, map, {"E", "H"}, "auto",
                                   {{"/date_time/type", dt},
                                    {"/date_time/value", kRestricted},
                                    {"/costing_options/auto/destination_only_penalty", "0"}});
    gurka::assert::raw::expect_path(result, {"EF", "FG", "GH"}, "date_time type " + dt);
    EXPECT_FALSE(used_relaxed_pass(result)) << "date_time type " << dt;
  }
}

// TimeDistanceMatrix seeds only its sources, so D's label has to carry the zone from DE into
// EF; the way around through BJ, JK and KF is 3900m.
TEST_F(DestinationOnlyZones, TimeDistanceMatrixCrossesZoneToEndpoint) {
  auto result = gurka::do_action(valhalla::Options::sources_to_targets, map, {"D"}, {"G"}, "auto",
                                 {{"/date_time/type", "1"}, {"/date_time/value", kRestricted}});
  EXPECT_EQ(result.matrix().algorithm(), Matrix::TimeDistanceMatrix);
  EXPECT_NEAR(result.matrix().distances(0), 1500, 10);
}

// CostMatrix grows a tree from each end, and only conditional edges meet at E, so both trees
// have to cross EF to pair E with G at all.
TEST_F(DestinationOnlyZones, CostMatrixCrossesZoneToEndpoint) {
  auto result =
      gurka::do_action(valhalla::Options::sources_to_targets, map, {"E", "G"}, {"G", "E"}, "auto",
                       {{"/prioritize_bidirectional", "1"},
                        {"/date_time/type", "1"},
                        {"/date_time/value", kRestricted}});
  EXPECT_EQ(result.matrix().algorithm(), Matrix::CostMatrix);
  EXPECT_NEAR(result.matrix().distances(0), 1000, 10) << "forward tree from E";
  EXPECT_NEAR(result.matrix().distances(3), 1000, 10) << "reverse tree from E";
}

// Dijkstras prices dest-only rather than refusing it, and a time contour bounds seconds while
// the penalty only moves cost -- so the charge is what /expansion reports, not a smaller polygon.
TEST_F(DestinationOnlyZones, IsochroneChargesConditionalLikeStatic) {
  auto reader = test::make_clean_graphreader(map.config.get_child("mjolnir"));
  auto gh = static_cast<uint64_t>(std::get<0>(gurka::findEdgeByNodes(*reader, map.nodes, "G", "H")));
  auto gl = static_cast<uint64_t>(std::get<0>(gurka::findEdgeByNodes(*reader, map.nodes, "G", "L")));

  auto edge_costs = [](const char* date_time) {
    auto result = gurka::do_action(valhalla::Options::expansion, map, {"F"}, "auto",
                                   {{"/action", "isochrone"},
                                    {"/expansion_properties/0", "edge_id"},
                                    {"/expansion_properties/1", "cost"},
                                    {"/contours/0/time", "60"},
                                    {"/date_time/type", "1"},
                                    {"/date_time/value", date_time}});
    const auto& e = result.expansion();
    std::unordered_map<uint64_t, uint32_t> costs;
    for (int i = 0; i < e.edge_id_size(); ++i)
      costs.emplace(e.edge_id(i), e.costs(i));
    return costs;
  };

  auto restricted = edge_costs(kRestricted);
  auto unrestricted = edge_costs(kUnrestricted);
  constexpr uint32_t kDefaultDestOnlyPenalty = 600;

  EXPECT_NEAR(restricted.at(gh) - unrestricted.at(gh), kDefaultDestOnlyPenalty, 2)
      << "an open window costs the same as the static tag";
  EXPECT_EQ(restricted.at(gl), unrestricted.at(gl)) << "the static tag is priced either way";
}
