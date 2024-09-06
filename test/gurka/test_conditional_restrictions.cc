#include "gurka.h"
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
          auto result =
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
          auto result =
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
