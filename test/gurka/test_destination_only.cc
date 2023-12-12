#include "gurka.h"
#include <gtest/gtest.h>

using namespace valhalla;

TEST(Standalone, DestinationOnly) {
  const std::string ascii_map = R"(
      A----B----C----D----E----J---L
           |         |    |    |
           |         |    |    |
           |         |    |    |
           I----H----G----F----K
  )";

  const gurka::ways ways = {
      {"AB", {{"highway", "residential"}}},
      {"BC", {{"highway", "residential"}}},
      {"CD", {{"highway", "residential"}}},
      {"DE", {{"highway", "residential"}}},
      {"EF", {{"highway", "residential"}}},
      {"FG", {{"highway", "residential"}}},
      {"GH", {{"highway", "residential"}}},
      {"HI", {{"highway", "residential"}}},
      {"DG", {{"highway", "residential"}, {"motor_vehicle", "destination"}}},
      {"BI", {{"highway", "residential"}, {"access", "private"}}},
      {"JK", {{"highway", "residential"}, {"hgv", "destination"}}},
      {"JL", {{"highway", "residential"}}},
      {"EJ", {{"highway", "residential"}}},
      {"FK", {{"highway", "residential"}}},
  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_destination");
  baldr::GraphReader reader(map.config.get_child("mjolnir"));

  EXPECT_TRUE(std::get<1>(gurka::findEdge(reader, layout, "JK", "K"))->destonly_hgv());
  EXPECT_TRUE(std::get<3>(gurka::findEdge(reader, layout, "JK", "K"))->destonly_hgv());

  // auto
  {
    auto result = gurka::do_action(valhalla::Options::route, map, {"A", "I"}, "auto");
    ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
    auto leg = result.trip().routes(0).legs(0);
    gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DE", "EF", "FG", "GH", "HI"});

    result = gurka::do_action(valhalla::Options::sources_to_targets, map, {"A"}, {"I"}, "auto");
    ASSERT_EQ(result.matrix().distances(0), 3900);
  }

  // hgv
  {
    auto result = gurka::do_action(valhalla::Options::route, map, {"K", "L"}, "truck");
    ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
    auto leg = result.trip().routes(0).legs(0);
    gurka::assert::raw::expect_path(result, {"JL", "EJ", "EF", "FK"});

    result = gurka::do_action(valhalla::Options::sources_to_targets, map, {"K"}, {"L"}, "truck");
    ASSERT_EQ(result.matrix().distances(0), 1700);
  }
}

TEST(Standalone, DestinationOnlyDrivewaysMidNoAlternatives) {
  const std::string ascii_map = R"(
      A----B----C----D----E
  )";

  for (const auto& dest_tag :
       std::map<std::string, std::string>{{"access", "private"}, {"motor_vehicle", "destination"}}) {
    const gurka::ways ways = {
        {"AB", {{"highway", "service"}}},
        {"BC", {{"highway", "service"}}},
        {"CD", {{"highway", "service"}, {"service", "driveway"}, dest_tag}},
        {"DE", {{"highway", "service"}}},

    };
    const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
    // Building tiles with the option mjolnir.include_driveways setting to true.
    {
      const auto map =
          gurka::buildtiles(layout, ways, {}, {},
                            "test/data/gurka_destination_only_driveways_mid_no_alternatives_1",
                            {{"mjolnir.include_driveways", "true"}});
      const auto result = gurka::do_action(valhalla::Options::route, map, {"A", "E"}, "auto");
      gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DE"});
    }
    // Building tiles with the option mjolnir.include_driveways setting to false.
    {
      const auto map =
          gurka::buildtiles(layout, ways, {}, {},
                            "test/data/gurka_destination_only_driveways_mid_no_alternatives_2",
                            {{"mjolnir.include_driveways", "false"}});
      EXPECT_THROW(gurka::do_action(valhalla::Options::route, map, {"A", "E"}, "auto"),
                   valhalla_exception_t);
    }
  }
}

TEST(Standalone, DestinationOnlyDrivewaysMidWithAlternatives) {
  const std::string ascii_map = R"(
      A----B----C----D----E
           |         |
           |         |
           I---------F
  )";

  for (const auto& dest_tag :
       std::map<std::string, std::string>{{"access", "private"}, {"motor_vehicle", "destination"}}) {
    const gurka::ways ways = {
        {"AB", {{"highway", "service"}}},
        {"BC", {{"highway", "service"}}},
        {"CD", {{"highway", "service"}, {"service", "driveway"}, dest_tag}},
        {"DE", {{"highway", "service"}}},
        {"BI", {{"highway", "service"}}},
        {"IF", {{"highway", "service"}}},
        {"FD", {{"highway", "service"}}},
    };
    const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
    // Building tiles with the option mjolnir.include_driveways setting to true.
    {
      const auto map =
          gurka::buildtiles(layout, ways, {}, {},
                            "test/data/gurka_destination_only_driveways_mid_with_alternatives_1",
                            {{"mjolnir.include_driveways", "true"}});
      const auto result = gurka::do_action(valhalla::Options::route, map, {"A", "E"}, "auto");
      gurka::assert::raw::expect_path(result, {"AB", "BI", "IF", "FD", "DE"});
    }
    // Building tiles with the option mjolnir.include_driveways setting to false.
    {
      const auto map =
          gurka::buildtiles(layout, ways, {}, {},
                            "test/data/gurka_destination_only_driveways_mid_with_alternatives_2",
                            {{"mjolnir.include_driveways", "false"}});
      const auto result = gurka::do_action(valhalla::Options::route, map, {"A", "E"}, "auto");
      gurka::assert::raw::expect_path(result, {"AB", "BI", "IF", "FD", "DE"});
    }
  }
}

TEST(Standalone, DestinationOnlyDrivewaysBeginEnd) {
  const std::string ascii_map = R"(
      A----B----C----D----E
  )";

  for (const auto& dest_tag :
       std::map<std::string, std::string>{{"access", "private"}, {"motor_vehicle", "destination"}}) {
    const gurka::ways ways = {
        {"AB", {{"highway", "service"}, {"service", "driveway"}, dest_tag}},
        {"BC", {{"highway", "service"}}},
        {"CD", {{"highway", "service"}}},
        {"DE", {{"highway", "service"}, {"service", "driveway"}, dest_tag}},

    };
    const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
    // Building tiles with the option mjolnir.include_driveways setting to true.
    {
      const auto map = gurka::buildtiles(layout, ways, {}, {},
                                         "test/data/gurka_destination_only_driveways_begin_end_1",
                                         {{"mjolnir.include_driveways", "true"}});
      const auto result = gurka::do_action(valhalla::Options::route, map, {"A", "E"}, "auto");
      gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DE"});
    }
    // Building tiles with the option mjolnir.include_driveways setting to false.
    {
      const auto map = gurka::buildtiles(layout, ways, {}, {},
                                         "test/data/gurka_destination_only_driveways_begin_end_2",
                                         {{"mjolnir.include_driveways", "false"}});
      const auto result = gurka::do_action(valhalla::Options::route, map, {"A", "E"}, "auto");
      gurka::assert::raw::expect_path(result, {"BC", "CD"});
    }
  }
}
